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

TEST_F(LocalRuntimeTest, TakesTargetsJustPastALimitToBeOnIt) {
    ASSERT_TRUE(run(session->park()));
    // A joint resting on its limit may measure slightly past it, and a path may start there.
    JointVector onLimit = profile.safety.restPose.head(6);
    onLimit[1] = -0.01;
    auto path = JointPath{};
    path.waypoints.push_back({.time = std::chrono::seconds{1}, .position = onLimit});
    path.waypoints.push_back({.time = std::chrono::seconds{3}, .position = armTarget(0.0)});
    EXPECT_TRUE(run(arm->followPath(std::move(path))));

    auto const result = run(arm->moveToJoints({.position = onLimit}));
    ASSERT_TRUE(result);
    EXPECT_NEAR(std::get<0>(*result).position[1], 0.0, 0.02);
}

TEST_F(LocalRuntimeTest, RejectsTargetsFarPastALimitSayingWhichAndWhere) {
    auto beyond = armTarget(0.0);
    beyond[1] = -0.2;
    auto const error = expectMotionError(arm->moveToJoints({.position = beyond}));
    EXPECT_EQ(error.reason, MotionFailure::InvalidGoal);
    EXPECT_NE(std::string{error.what()}.find("'joint2' target -0.200 is outside its limits [0.000, 3.140]"),
              std::string::npos)
        << error.what();
}

TEST_F(LocalRuntimeTest, RefusesToPressTheWristIntoTheArm) {
    ASSERT_TRUE(run(session->park()));
    JointVector lowered = profile.safety.restPose.head(6);
    lowered[3] = -0.85;
    auto const error = expectMotionError(arm->moveToJoints({.position = lowered}));
    EXPECT_EQ(error.reason, MotionFailure::PlanningFailed);
    EXPECT_NE(std::string{error.what()}.find("self-collision"), std::string::npos) << error.what();
    EXPECT_NE(std::string{error.what()}.find("link5"), std::string::npos) << error.what();
    std::this_thread::sleep_for(std::chrono::milliseconds{200});
    EXPECT_NEAR(session->latest().state.joints.position[3], 0.0, 0.02);
}

TEST_F(LocalRuntimeTest, ParksWithTheWristTurnedCloseToTheFoldedArm) {
    auto near = JointVector{6};
    near << -0.212, 0.0, 0.343, 0.142, -0.542, -0.416;
    ASSERT_TRUE(run(arm->moveToJoints({.position = near})));
    EXPECT_TRUE(run(session->park()));
}

struct ToolFrameTest : LocalRuntimeTest {
    void SetUp() override {
        LocalRuntimeTest::SetUp();
        auto loaded = model::loadRobotModel(profile);
        ASSERT_TRUE(loaded);
        robotModel = std::move(*loaded);
        kinematics = robotModel->makeKinematics();
        auto const &spec = profile.groups[*profile.findGroup("arm")];
        tool = *kinematics->findFrame(spec.toolFrame);
        tcp = spec.tcp;
    }

    Pose3 tcpAt(JointVector const &q) {
        kinematics->update(q);
        return kinematics->framePose(tool) * tcp;
    }

    Pose3 measuredTcp() { return tcpAt(session->latest().state.joints.position); }
    Pose3 commandedTcp() { return tcpAt(session->latest().command.position); }

    static Pose3 shift(double const x, double const y, double const z) {
        return Pose3{.translation = Eigen::Vector3d{x, y, z}};
    }

    std::unique_ptr<model::RobotModel> robotModel;
    std::unique_ptr<model::Kinematics> kinematics;
    model::FrameId tool;
    Pose3 tcp;
};

// Steps start from the commanded TCP, so tracking errors do not accumulate.
TEST_F(ToolFrameTest, ToolFrameStepsAddUp) {
    auto const start = commandedTcp();
    for (int step = 0; step < 2; ++step) {
        ASSERT_TRUE(run(arm->moveToPose(
            {.target = shift(0.0, 0.0, -0.02), .frame = Frame::Tool, .path = PathShape::Linear})));
    }
    auto const expected = start * shift(0.0, 0.0, -0.04);
    auto const reached = commandedTcp();
    EXPECT_LT((reached.translation - expected.translation).norm(), 5e-4);
    EXPECT_LT(reached.rotation.angularDistance(expected.rotation), 2e-3);
    EXPECT_LT((measuredTcp().translation - expected.translation).norm(), 0.005);
}

TEST_F(ToolFrameTest, TurnsAboutTheToolCenterPoint) {
    auto const start = measuredTcp();
    auto const turn = Pose3{.rotation = Eigen::Quaterniond{Eigen::AngleAxisd{0.3, Eigen::Vector3d::UnitX()}}};
    ASSERT_TRUE(run(arm->moveToPose({.target = turn, .frame = Frame::Tool, .path = PathShape::Linear})));
    auto const reached = measuredTcp();
    EXPECT_LT((reached.translation - start.translation).norm(), 0.003);
    EXPECT_NEAR(reached.rotation.angularDistance(start.rotation), 0.3, 0.01);
}

TEST_F(ToolFrameTest, MovesAlongABaseFrameLine) {
    kinematics->update(session->latest().command.position);
    auto const base = kinematics->framePose(*kinematics->findFrame("base_link"));
    auto target = base.inverse() * measuredTcp();
    target.translation += Eigen::Vector3d{0.0, 0.05, -0.05};
    ASSERT_TRUE(run(arm->moveToPose({.target = target, .path = PathShape::Linear, .speed = 0.5})));
    auto const reached = base.inverse() * measuredTcp();
    EXPECT_LT((reached.translation - target.translation).norm(), 0.003);
}

TEST_F(ToolFrameTest, RejectsAToolFrameMoveOutOfReach) {
    auto const error = expectMotionError(
        arm->moveToPose({.target = shift(0.6, 0.0, 0.0), .frame = Frame::Tool, .path = PathShape::Linear}));
    EXPECT_EQ(error.reason, MotionFailure::Unreachable) << error.what();
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
