#include <larm/model/RobotModel.h>
#include <larm/runtime/LocalRuntime.h>
#include <larm/sim/Simulation.h>

#include <gtest/gtest.h>

#include <cstdlib>
#include <future>
#include <thread>

namespace larm::runtime {
namespace {

JointVector clearancePose() {
    auto q = JointVector{7};
    q << 0.0, 0.7, 1.1, 0.0, 0.0, 0.0, 0.02;
    return q;
}

JointVector armTarget(double const base) {
    auto q = JointVector{6};
    q << base, 1.0, 1.4, -0.5, 0.4, 1.0;
    return q;
}

// Runs a sender to completion: the value, nullopt when stopped, or rethrows its error.
template <class Sender> auto run(Sender &&sender) { return lexec::sync_wait(std::forward<Sender>(sender)); }

MotionError motionError(std::exception_ptr const &error) {
    try {
        std::rethrow_exception(error);
    } catch (MotionError const &motionError) {
        return motionError;
    }
}

template <class Sender> MotionError expectMotionError(Sender &&sender) {
    try {
        run(std::forward<Sender>(sender));
    } catch (MotionError const &error) {
        return error;
    }
    ADD_FAILURE() << "expected a MotionError";
    return MotionError{MotionFailure::Fault, control::FaultCode::None, "none"};
}

struct LocalRuntimeTest : testing::Test {
    void SetUp() override {
        auto const *const path = std::getenv("LARM_ROBOT_PROFILE");
        ASSERT_NE(path, nullptr);
        auto loaded = loadRobotProfile(path);
        ASSERT_TRUE(loaded) << loaded.error().message;
        profile = *loaded;
        auto simulation =
            sim::makeSimulation(profile, {.pacing = sim::Pacing::RealTime, .realTimeFactor = 4.0});
        ASSERT_TRUE(simulation) << simulation.error().message;
        (*simulation)->reset(clearancePose());
        auto started = startLocalRuntime(profile, std::move(*simulation), {});
        ASSERT_TRUE(started) << started.error().message;
        session = std::move(*started);
        ASSERT_TRUE(run(session->enable()));
        arm = session->motion("arm");
        hand = session->gripper("gripper");
        ASSERT_NE(arm, nullptr);
        ASSERT_NE(hand, nullptr);
    }

    RobotProfile profile;
    std::unique_ptr<RobotSession> session;
    MotionApi *arm{};
    GripperApi *hand{};
};

TEST_F(LocalRuntimeTest, ExposesGroupsByCapability) {
    EXPECT_EQ(session->motion("gripper"), nullptr);
    EXPECT_EQ(session->gripper("arm"), nullptr);
    EXPECT_EQ(session->motion("legs"), nullptr);
    EXPECT_EQ(session->latest().power, hal::DrivePower::Enabled);
}

TEST_F(LocalRuntimeTest, MovesToJoints) {
    auto const result = run(arm->moveToJoints({.position = armTarget(0.8)}));
    ASSERT_TRUE(result);
    auto const &[motion] = *result;
    EXPECT_LT((motion.position - armTarget(0.8)).cwiseAbs().maxCoeff(), 0.02);
}

TEST_F(LocalRuntimeTest, MovesToAPose) {
    auto model = model::loadRobotModel(profile);
    ASSERT_TRUE(model);
    auto kinematics = (*model)->makeKinematics();
    auto const tool = kinematics->findFrame("gripper_end");
    auto goal = clearancePose();
    goal.head(6) = armTarget(-0.6);
    kinematics->update(goal);
    auto const target = kinematics->framePose(*tool);

    auto const result = run(arm->moveToPose({.target = target}));
    ASSERT_TRUE(result);
    auto reached = session->latest().state.joints.position;
    kinematics->update(reached);
    EXPECT_LT(poseError(kinematics->framePose(*tool), target).head<3>().norm(), 0.01);
}

TEST_F(LocalRuntimeTest, RejectsUnreachablePosesAndInvalidGoals) {
    auto const far = Pose3{.translation = Eigen::Vector3d{3.0, 0.0, 0.0}};
    EXPECT_EQ(expectMotionError(arm->moveToPose({.target = far})).reason, MotionFailure::Unreachable);
    auto beyond = armTarget(0.0);
    beyond[1] = 5.0;
    EXPECT_EQ(expectMotionError(arm->moveToJoints({.position = beyond})).reason, MotionFailure::InvalidGoal);
    EXPECT_EQ(expectMotionError(arm->moveToJoints({.position = JointVector::Zero(2)})).reason,
              MotionFailure::InvalidGoal);
}

TEST_F(LocalRuntimeTest, FollowsATimedPathFromTheCurrentPosition) {
    auto path = JointPath{};
    auto middle = armTarget(0.3);
    path.waypoints.push_back({.time = std::chrono::seconds{1}, .position = middle});
    path.waypoints.push_back({.time = std::chrono::seconds{2}, .position = armTarget(0.6)});
    auto const result = run(arm->followPath(std::move(path)));
    ASSERT_TRUE(result);
    auto const &[motion] = *result;
    EXPECT_LT((motion.position - armTarget(0.6)).cwiseAbs().maxCoeff(), 0.02);
}

TEST_F(LocalRuntimeTest, GripsToAPosition) {
    auto const result = run(hand->grip({.position = 0.045, .maxEffort = 10.0}));
    ASSERT_TRUE(result);
    auto const &[grip] = *result;
    EXPECT_TRUE(grip.reachedGoal);
    EXPECT_NEAR(grip.position, 0.045, 0.002);
}

TEST_F(LocalRuntimeTest, StopRequestDeceleratesAndCompletesStopped) {
    auto source = lexec::inplace_stop_source{};
    auto canceller = std::jthread{[&] {
        std::this_thread::sleep_for(std::chrono::milliseconds{200});
        source.request_stop();
    }};
    auto const result = run(lexec::write_env(arm->moveToJoints({.position = armTarget(2.0)}),
                                             lexec::prop{lexec::get_stop_token, source.get_token()}));
    EXPECT_FALSE(result);
    std::this_thread::sleep_for(std::chrono::milliseconds{50});
    auto const snapshot = session->latest();
    EXPECT_EQ(snapshot.activeCount, 0u);
    EXPECT_LT(snapshot.state.joints.velocity.head(6).cwiseAbs().maxCoeff(), 0.05);
    EXPECT_LT(snapshot.state.joints.position[0], 1.9);
}

TEST_F(LocalRuntimeTest, NewGoalPreemptsTheRunningOne) {
    auto first =
        std::async(std::launch::async, [&] { return run(arm->moveToJoints({.position = armTarget(2.0)})); });
    std::this_thread::sleep_for(std::chrono::milliseconds{200});
    auto const second = run(arm->moveToJoints({.position = armTarget(0.2), .speed = 0.5}));
    EXPECT_FALSE(first.get());
    ASSERT_TRUE(second);
    auto const &[motion] = *second;
    EXPECT_LT((motion.position - armTarget(0.2)).cwiseAbs().maxCoeff(), 0.02);
}

TEST_F(LocalRuntimeTest, GripperAndArmMoveIndependently) {
    auto gripping =
        std::async(std::launch::async, [&] { return run(hand->grip({.position = 0.0, .maxEffort = 5.0})); });
    auto const moved = run(arm->moveToJoints({.position = armTarget(0.5)}));
    EXPECT_TRUE(moved);
    EXPECT_TRUE(gripping.get());
}

TEST_F(LocalRuntimeTest, EmergencyStopFailsTheMotionUntilReset) {
    auto moving = std::async(std::launch::async, [&] {
        try {
            run(arm->moveToJoints({.position = armTarget(2.0)}));
        } catch (...) {
            return std::current_exception();
        }
        return std::exception_ptr{};
    });
    std::this_thread::sleep_for(std::chrono::milliseconds{200});
    session->emergencyStop();
    auto const error = moving.get();
    ASSERT_TRUE(error);
    EXPECT_EQ(motionError(error).fault, control::FaultCode::EmergencyStop);

    EXPECT_EQ(expectMotionError(arm->moveToJoints({.position = armTarget(0.0)})).fault,
              control::FaultCode::EmergencyStop);
    EXPECT_TRUE(run(session->resetFault()));
    EXPECT_TRUE(run(arm->moveToJoints({.position = armTarget(0.0), .speed = 0.5})));
}

TEST_F(LocalRuntimeTest, DisableRequiresTheRestPoseUnlessForced) {
    EXPECT_EQ(expectMotionError(session->disable({})).reason, MotionFailure::InvalidGoal);
    EXPECT_TRUE(run(session->park()));
    EXPECT_TRUE(run(session->disable({})));
    EXPECT_EQ(session->latest().power, hal::DrivePower::Disabled);
    EXPECT_EQ(expectMotionError(arm->moveToJoints({.position = armTarget(0.0)})).fault,
              control::FaultCode::NotEnabled);
}

TEST_F(LocalRuntimeTest, ShutdownFailsOperationsStillRunning) {
    auto moving = std::async(std::launch::async, [&] {
        try {
            run(arm->moveToJoints({.position = armTarget(2.0), .speed = 0.2}));
        } catch (...) {
            return std::current_exception();
        }
        return std::exception_ptr{};
    });
    std::this_thread::sleep_for(std::chrono::milliseconds{200});
    session.reset();
    auto const error = moving.get();
    ASSERT_TRUE(error);
    EXPECT_EQ(motionError(error).reason, MotionFailure::Shutdown);
}

} // namespace
} // namespace larm::runtime
