#include <larm/drivers/robstride/Backend.h>
#include <larm/model/RobotModel.h>
#include <larm/runtime/LocalRuntime.h>

#include <gtest/gtest.h>

#include <cstdlib>
#include <thread>

namespace larm::drivers::robstride {
namespace {

using runtime::RobotSession;

template <class Sender> auto run(Sender &&sender) { return lexec::sync_wait(std::forward<Sender>(sender)); }

JointVector armTarget(double const base) {
    auto q = JointVector{6};
    q << base, 0.7, 1.1, -0.3, 0.3, 0.5;
    return q;
}

// Waits up to two seconds for `done` to hold on the session's latest snapshot.
template <class Predicate> bool eventually(RobotSession &session, Predicate done) {
    for (int i = 0; i < 200; ++i) {
        if (done(session.latest())) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }
    return false;
}

// The runtime over the RobStride driver and simulated motors that carry the arm's gravity.
struct RobStrideRuntimeTest : testing::Test {
    void SetUp() override {
        auto loaded = loadRobotProfile(std::getenv("LARM_ROBOT_PROFILE"));
        ASSERT_TRUE(loaded) << loaded.error().message;
        profile = std::move(*loaded);
        auto parsed = parseDriverConfig(profile);
        ASSERT_TRUE(parsed) << parsed.error().message;
        config = *parsed;
    }

    void start() {
        auto model = model::loadRobotModel(profile);
        ASSERT_TRUE(model) << model.error().message;
        auto const dynamics = std::shared_ptr<model::Dynamics>{(*model)->makeDynamics()};
        auto backend = makeSimulatedRobStrideBackend(
            profile,
            {.load = jointSpaceLoad(config, [dynamics](JointVector const &position, JointVector &torque) {
                 dynamics->gravity(position, torque);
                 torque *= -1.0;
             })});
        ASSERT_TRUE(backend) << backend.error().message;
        motors = (*backend)->simulatedMotors();
        auto started = runtime::startLocalRuntime(profile, std::move(*backend), {});
        ASSERT_TRUE(started) << started.error().message;
        session = std::move(*started);
    }

    RobotProfile profile;
    DriverConfig config;
    SimulatedMotors *motors{};
    std::unique_ptr<RobotSession> session;
};

TEST_F(RobStrideRuntimeTest, EnablesMovesGripsParksAndDisables) {
    start();
    ASSERT_TRUE(run(session->enable()));
    auto const moved = run(session->motion("arm")->moveToJoints({.position = armTarget(0.4), .speed = 1.0}));
    ASSERT_TRUE(moved);
    EXPECT_LT((std::get<0>(*moved).position - armTarget(0.4)).cwiseAbs().maxCoeff(), 0.02);

    auto const gripped = run(session->gripper("gripper")->grip({.position = 0.03, .maxEffort = 10.0}));
    ASSERT_TRUE(gripped);
    EXPECT_NEAR(std::get<0>(*gripped).position, 0.03, 2e-3);
    EXPECT_NEAR(motors->position(6), 0.03 / 0.007353, 0.3);

    ASSERT_TRUE(run(session->park()));
    ASSERT_TRUE(run(session->disable({})));
    for (std::size_t i = 0; i < profile.dof(); ++i) {
        EXPECT_FALSE(motors->enabled(i)) << i;
    }
}

TEST_F(RobStrideRuntimeTest, AFaultedActuatorLeavesTheOthersHolding) {
    start();
    ASSERT_TRUE(run(session->enable()));
    ASSERT_TRUE(run(session->motion("arm")->moveToJoints({.position = armTarget(0.0), .speed = 1.0})));

    motors->injectFault(3, status_flag::kOvercurrent, 1U << 16U);
    ASSERT_TRUE(eventually(*session, [](control::RobotSnapshot const &snapshot) {
        return snapshot.fault == control::FaultCode::ActuatorFault;
    }));
    std::this_thread::sleep_for(std::chrono::milliseconds{300});
    auto const held = session->latest();
    EXPECT_EQ(held.safety, control::SafetyState::Faulted);
    EXPECT_FALSE(held.state.actuators[3].enabled);
    for (std::size_t i : {0U, 1U, 2U, 4U, 5U, 6U}) {
        EXPECT_TRUE(motors->enabled(i)) << i;
    }
    EXPECT_NEAR(held.state.joints.position[1], 0.7, 0.02);
    EXPECT_NEAR(held.state.joints.position[2], 1.1, 0.02);

    // Without brakes a faulted arm is disabled where it is, then cleared by the next enable.
    ASSERT_TRUE(run(session->disable({.force = true})));
    ASSERT_TRUE(run(session->resetFault()));
    ASSERT_TRUE(run(session->enable()));
    EXPECT_TRUE(motors->enabled(3));
    ASSERT_TRUE(run(session->park()));
    ASSERT_TRUE(run(session->disable({})));
}

TEST_F(RobStrideRuntimeTest, RejectsAPeriodShorterThanTheBusTraffic) {
    profile.controlPeriod = std::chrono::milliseconds{2};
    auto backend = makeSimulatedRobStrideBackend(profile, {});
    ASSERT_TRUE(backend);
    auto const started = runtime::startLocalRuntime(profile, std::move(*backend), {});
    ASSERT_FALSE(started);
    EXPECT_NE(started.error().message.find("shorter than the driver's minimum"), std::string::npos)
        << started.error().message;
}

} // namespace
} // namespace larm::drivers::robstride
