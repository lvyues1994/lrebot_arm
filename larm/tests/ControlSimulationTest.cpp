#include <larm/control/ControlCycle.h>
#include <larm/control/JointTrajectoryController.h>
#include <larm/sim/Simulation.h>

#include <gtest/gtest.h>

#include <cstdlib>
#include <optional>

namespace larm {
namespace {

using control::ControlStatus;
using control::GoalFinished;
using control::GoalId;

JointVector clearancePose() {
    auto q = JointVector{7};
    q << 0.0, 0.7, 1.1, 0.0, 0.0, 0.0, 0.02;
    return q;
}

struct ControlSimulation : testing::Test {
    void SetUp() override {
        auto const *const path = std::getenv("LARM_ROBOT_PROFILE");
        ASSERT_NE(path, nullptr);
        auto loaded = loadRobotProfile(path);
        ASSERT_TRUE(loaded) << loaded.error().message;
        profile = std::move(*loaded);
        auto robot = model::loadRobotModel(profile);
        ASSERT_TRUE(robot) << robot.error().message;
        robotModel = std::move(*robot);
        auto made = sim::makeSimulation(profile, {});
        ASSERT_TRUE(made) << made.error().message;
        simulation = std::move(*made);
        simulation->reset(clearancePose());
        channels = std::make_unique<control::RuntimeChannels>(profile.dof());
        auto cycleMade = control::makeControlCycle(
            profile,
            {.driver = &simulation->driver(), .model = robotModel.get(), .channels = channels.get()});
        ASSERT_TRUE(cycleMade) << cycleMade.error().message;
        cycle = std::move(*cycleMade);
    }

    void step() {
        cycle->tick();
        simulation->timeline().advance();
        while (auto event = channels->events.tryPop()) {
            if (auto const *const goal = std::get_if<GoalFinished>(&*event)) {
                results.push_back(*goal);
            }
            if (auto const *const power = std::get_if<control::PowerChanged>(&*event)) {
                powered = power->power == hal::DrivePower::Enabled;
            }
        }
        while (channels->retired.tryPop()) {
        }
    }

    std::optional<GoalFinished> result(GoalId const goal) const {
        for (auto const &finished : results) {
            if (finished.goal == goal) {
                return finished;
            }
        }
        return std::nullopt;
    }

    control::RobotSnapshot const &snapshot() {
        channels->snapshot.refresh();
        return channels->snapshot.current();
    }

    std::unique_ptr<control::Controller> trajectory(JointMask const joints, JointVector const &target) {
        auto start = motion::JointSample::zero(profile.dof());
        start.position = snapshot().state.joints.position;
        auto planned = motion::planPointToPoint(
            {.start = start, .target = target, .limits = motion::motionLimits(profile), .joints = joints});
        EXPECT_TRUE(planned) << planned.error().message;
        return control::makeJointTrajectoryController({
            .trajectory = *planned,
            .joints = joints,
            .impedance = control::profileImpedance(profile),
            .trackingTolerance = control::profileTrackingTolerance(profile),
            .goalTolerance = control::profileTrackingTolerance(profile) * 0.05,
            .goalTimeout = std::chrono::seconds{1},
            .stopLimits = motion::motionLimits(profile),
        });
    }

    RobotProfile profile;
    std::unique_ptr<model::RobotModel> robotModel;
    std::unique_ptr<sim::Simulation> simulation;
    std::unique_ptr<control::RuntimeChannels> channels;
    std::unique_ptr<control::ControlCycle> cycle;
    std::vector<GoalFinished> results;
    bool powered{};
};

TEST_F(ControlSimulation, ArmAndGripperReachTheirTargetsTogether) {
    ASSERT_TRUE(channels->requests.tryPush(control::SetDrivePower{.power = hal::DrivePower::Enabled}));
    for (int i = 0; i < 300 and not powered; ++i) {
        step();
    }
    ASSERT_TRUE(powered);

    auto arm = JointMask{};
    auto gripper = JointMask{};
    for (std::size_t i = 0; i < 6; ++i) {
        arm.set(i);
    }
    gripper.set(6);
    auto target = JointVector{7};
    target << 0.8, 1.0, 1.4, -0.5, 0.4, 1.0, 0.04;
    ASSERT_TRUE(channels->requests.tryPush(
        control::ActivateController{.goal = GoalId{1}, .controller = trajectory(arm, target)}));
    ASSERT_TRUE(channels->requests.tryPush(
        control::ActivateController{.goal = GoalId{2}, .controller = trajectory(gripper, target)}));

    auto worstTracking = 0.0;
    for (int i = 0; i < 2000 and not(result(GoalId{1}) and result(GoalId{2})); ++i) {
        step();
        auto const &current = snapshot();
        auto const error =
            (current.command.position - current.state.joints.position).head(6).cwiseAbs().maxCoeff();
        worstTracking = std::max(worstTracking, error);
    }
    ASSERT_TRUE(result(GoalId{1}));
    ASSERT_TRUE(result(GoalId{2}));
    EXPECT_EQ(result(GoalId{1})->status, ControlStatus::Succeeded);
    EXPECT_EQ(result(GoalId{2})->status, ControlStatus::Succeeded);
    EXPECT_LT(worstTracking, 0.05);

    for (int i = 0; i < 250; ++i) {
        step();
    }
    auto const &settled = snapshot().state.joints.position;
    EXPECT_LT((settled - target).head(6).cwiseAbs().maxCoeff(), 0.01);
    EXPECT_NEAR(settled[6], target[6], 0.002);
}

} // namespace
} // namespace larm
