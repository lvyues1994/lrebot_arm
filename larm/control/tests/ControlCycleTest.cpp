#include <larm/control/ControlCycle.h>
#include <larm/control/JointTrajectoryController.h>
#include <larm/hal/IdealBackend.h>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <limits>
#include <optional>
#include <vector>

namespace larm::control {
namespace {

using hal::DrivePower;

// Writes a fixed command to its joints; used to exercise the safety filter.
struct FixedCommandController final : Controller {
    FixedCommandController(JointMask joints_, double position_, double effort_)
        : joints{joints_}, position{position_}, effort{effort_} {}

    JointMask claims() const noexcept override { return joints; }
    ControlStep start(ControlContext const &) noexcept override { return {}; }
    ControlStep update(ControlContext const &, JointCommand &out) noexcept override {
        for (std::size_t i = 0; i < kMaxDof; ++i) {
            if (joints.test(i)) {
                out.position[idx(i)] = position;
                out.velocity[idx(i)] = 0.0;
                out.effort[idx(i)] = effort;
                out.stiffness[idx(i)] = 10.0;
                out.damping[idx(i)] = 1.0;
            }
        }
        return {};
    }
    void requestStop() noexcept override {}

  private:
    JointMask joints;
    double position;
    double effort;
};

JointVector clearancePose() {
    auto q = JointVector{7};
    q << 0.0, 0.7, 1.1, 0.0, 0.0, 0.0, 0.02;
    return q;
}

struct ControlCycleTest : testing::Test {
    void SetUp() override {
        auto const *const path = std::getenv("LARM_ROBOT_PROFILE");
        ASSERT_NE(path, nullptr);
        auto loaded = loadRobotProfile(path);
        ASSERT_TRUE(loaded) << loaded.error().message;
        profile = std::move(*loaded);
        auto robot = model::loadRobotModel(profile);
        ASSERT_TRUE(robot) << robot.error().message;
        robotModel = std::move(*robot);
        backend =
            hal::makeIdealBackend({.initialPosition = clearancePose(), .period = profile.controlPeriod});
        channels = std::make_unique<RuntimeChannels>(profile.dof());
        auto made = makeControlCycle(
            profile, {.driver = &backend->driver(), .model = robotModel.get(), .channels = channels.get()});
        ASSERT_TRUE(made) << made.error().message;
        cycle = std::move(*made);
        for (auto const joint : profile.groups[*profile.findGroup("arm")].joints) {
            arm.set(joint);
        }
    }

    void run(int const cycles) {
        for (int i = 0; i < cycles; ++i) {
            cycle->tick();
            backend->timeline().advance();
            drainEvents();
        }
    }

    void push(ControlRequest request) { ASSERT_TRUE(channels->requests.tryPush(std::move(request))); }

    void drainEvents() {
        while (auto event = channels->events.tryPop()) {
            events.push_back(std::move(*event));
        }
        while (channels->retired.tryPop()) {
            ++retiredCount;
        }
    }

    template <class Event, class Predicate> std::optional<Event> findEvent(Predicate predicate) const {
        for (auto const &event : events) {
            if (auto const *const typed = std::get_if<Event>(&event);
                typed != nullptr and predicate(*typed)) {
                return *typed;
            }
        }
        return std::nullopt;
    }

    std::optional<GoalFinished> finished(GoalId const goal) const {
        return findEvent<GoalFinished>([&](GoalFinished const &event) { return event.goal == goal; });
    }

    RobotSnapshot const &snapshot() {
        channels->snapshot.refresh();
        return channels->snapshot.current();
    }

    void enable() {
        push(SetDrivePower{.power = DrivePower::Enabled});
        for (int i = 0; i < 300 and not powered(); ++i) {
            run(1);
        }
        ASSERT_TRUE(powered());
    }

    bool powered() const {
        return findEvent<PowerChanged>(
                   [](PowerChanged const &event) { return event.power == DrivePower::Enabled; })
            .has_value();
    }

    std::unique_ptr<Controller> trajectoryTo(JointVector const &target, double const scale = 1.0) {
        auto start = motion::JointSample::zero(profile.dof());
        start.position = snapshot().state.joints.position;
        auto planned = motion::planPointToPoint({.start = start,
                                                 .target = target,
                                                 .limits = motion::motionLimits(profile, scale),
                                                 .joints = arm});
        EXPECT_TRUE(planned) << planned.error().message;
        return makeJointTrajectoryController({
            .trajectory = *planned,
            .joints = arm,
            .impedance = profileImpedance(profile),
            .trackingTolerance = profileTrackingTolerance(profile),
            .goalTolerance = JointVector::Constant(idx(profile.dof()), 0.01),
            .goalTimeout = std::chrono::milliseconds{500},
            .stopLimits = motion::motionLimits(profile),
        });
    }

    RobotProfile profile;
    std::unique_ptr<model::RobotModel> robotModel;
    std::unique_ptr<hal::IdealBackend> backend;
    std::unique_ptr<RuntimeChannels> channels;
    std::unique_ptr<ControlCycle> cycle;
    JointMask arm;
    std::vector<ControlEvent> events;
    int retiredCount{};
};

TEST_F(ControlCycleTest, StaysPassiveUntilEnabledThenRampsStiffness) {
    run(5);
    EXPECT_EQ(backend->lastRequestedPower(), DrivePower::Disabled);
    EXPECT_EQ(backend->lastCommand().stiffness.norm(), 0.0);

    push(SetDrivePower{.power = DrivePower::Enabled});
    run(60);
    EXPECT_FALSE(powered());
    auto const halfway = backend->lastCommand().stiffness[1];
    EXPECT_GT(halfway, 0.0);
    EXPECT_LT(halfway, profile.joints[1].gains.stiffness);

    enable();
    EXPECT_DOUBLE_EQ(backend->lastCommand().stiffness[1], profile.joints[1].gains.stiffness);
    EXPECT_TRUE(backend->lastCommand().position.isApprox(clearancePose()));
    EXPECT_EQ(snapshot().power, DrivePower::Enabled);
}

TEST_F(ControlCycleTest, RejectsControllersWhileDisabled) {
    run(2);
    push(ActivateController{.goal = GoalId{1}, .controller = trajectoryTo(clearancePose())});
    run(1);
    auto const result = finished(GoalId{1});
    ASSERT_TRUE(result);
    EXPECT_EQ(result->status, ControlStatus::Failed);
    EXPECT_EQ(result->fault, FaultCode::NotEnabled);
    EXPECT_EQ(retiredCount, 1);
}

TEST_F(ControlCycleTest, RunsATrajectoryToItsGoal) {
    enable();
    auto target = clearancePose();
    target[0] = 0.6;
    target[3] = -0.4;
    push(ActivateController{.goal = GoalId{7}, .controller = trajectoryTo(target)});
    run(2);
    EXPECT_EQ(snapshot().activeCount, 1u);
    for (int i = 0; i < 1000 and not finished(GoalId{7}); ++i) {
        run(1);
    }
    auto const result = finished(GoalId{7});
    ASSERT_TRUE(result);
    EXPECT_EQ(result->status, ControlStatus::Succeeded);
    run(3);
    EXPECT_EQ(snapshot().activeCount, 0u);
    EXPECT_EQ(retiredCount, 1);
    EXPECT_LT((backend->lastCommand().position - target).cwiseAbs().maxCoeff(), 1e-6);
}

TEST_F(ControlCycleTest, CancelDeceleratesWithoutJumps) {
    enable();
    auto target = clearancePose();
    target[0] = 2.0;
    push(ActivateController{.goal = GoalId{3}, .controller = trajectoryTo(target)});
    run(100);
    push(CancelGoal{.goal = GoalId{3}});
    auto previous = backend->lastCommand().position;
    auto const maxStep = 2.0 * toSeconds(profile.controlPeriod) * 1.01;
    for (int i = 0; i < 500 and not finished(GoalId{3}); ++i) {
        run(1);
        auto const current = backend->lastCommand().position;
        EXPECT_LE((current - previous).cwiseAbs().maxCoeff(), maxStep);
        previous = current;
    }
    auto const result = finished(GoalId{3});
    ASSERT_TRUE(result);
    EXPECT_EQ(result->status, ControlStatus::Stopped);
    EXPECT_LT(backend->lastCommand().velocity.norm(), 1e-9);
    EXPECT_LT(backend->lastCommand().position[0], 2.0);
}

TEST_F(ControlCycleTest, RejectsOverlappingClaims) {
    enable();
    auto target = clearancePose();
    target[0] = 1.0;
    push(ActivateController{.goal = GoalId{1}, .controller = trajectoryTo(target, 0.2)});
    push(ActivateController{.goal = GoalId{2}, .controller = trajectoryTo(target)});
    run(1);
    auto const second = finished(GoalId{2});
    ASSERT_TRUE(second);
    EXPECT_EQ(second->fault, FaultCode::ClaimConflict);
    EXPECT_FALSE(finished(GoalId{1}));
}

TEST_F(ControlCycleTest, RejectsATrajectoryThatStartsAwayFromTheArm) {
    enable();
    auto start = motion::JointSample::zero(profile.dof());
    start.position = clearancePose();
    start.position[0] = 1.0;
    auto planned = motion::planPointToPoint(
        {.start = start, .target = start.position, .limits = motion::motionLimits(profile), .joints = arm});
    ASSERT_TRUE(planned);
    push(ActivateController{.goal = GoalId{4},
                            .controller = makeJointTrajectoryController({
                                .trajectory = *planned,
                                .joints = arm,
                                .impedance = profileImpedance(profile),
                                .trackingTolerance = profileTrackingTolerance(profile),
                                .goalTolerance = JointVector::Constant(7, 0.01),
                                .stopLimits = motion::motionLimits(profile),
                            })});
    run(1);
    auto const result = finished(GoalId{4});
    ASSERT_TRUE(result);
    EXPECT_EQ(result->fault, FaultCode::StartRejected);
}

TEST_F(ControlCycleTest, FeedbackLossFaultsAndLatchesUntilReset) {
    enable();
    auto target = clearancePose();
    target[0] = 1.5;
    push(ActivateController{.goal = GoalId{5}, .controller = trajectoryTo(target)});
    run(20);
    backend->setFeedbackLost(true);
    run(static_cast<int>(profile.safety.feedbackTimeoutCycles));
    auto const result = finished(GoalId{5});
    ASSERT_TRUE(result);
    EXPECT_EQ(result->fault, FaultCode::FeedbackLost);
    EXPECT_EQ(snapshot().safety, SafetyState::Faulted);

    push(ActivateController{.goal = GoalId{6}, .controller = trajectoryTo(target)});
    push(ResetFault{});
    run(1);
    EXPECT_EQ(finished(GoalId{6})->fault, FaultCode::FeedbackLost);
    EXPECT_EQ(snapshot().safety, SafetyState::Faulted);

    backend->setFeedbackLost(false);
    push(ResetFault{});
    run(1);
    EXPECT_EQ(snapshot().safety, SafetyState::Normal);
    EXPECT_TRUE(findEvent<SafetyChanged>(
        [](SafetyChanged const &event) { return event.state == SafetyState::Normal; }));
}

TEST_F(ControlCycleTest, EmergencyStopAbortsAndHolds) {
    enable();
    auto target = clearancePose();
    target[0] = 1.5;
    push(ActivateController{.goal = GoalId{8}, .controller = trajectoryTo(target)});
    run(50);
    push(EmergencyStop{});
    run(1);
    EXPECT_EQ(finished(GoalId{8})->fault, FaultCode::EmergencyStop);
    auto const held = backend->lastCommand().position;
    run(10);
    EXPECT_TRUE(backend->lastCommand().position.isApprox(held));
    EXPECT_EQ(snapshot().safety, SafetyState::EmergencyStopped);
    EXPECT_EQ(backend->lastRequestedPower(), DrivePower::Enabled);
}

TEST_F(ControlCycleTest, SafetyFilterClampsCommands) {
    enable();
    auto joint = JointMask{};
    joint.set(0);
    push(ActivateController{.goal = GoalId{9},
                            .controller = std::make_unique<FixedCommandController>(joint, 10.0, 1e3)});
    run(1);
    EXPECT_DOUBLE_EQ(backend->lastCommand().position[0], profile.joints[0].limits.upper);
    EXPECT_DOUBLE_EQ(backend->lastCommand().effort[0], profile.joints[0].limits.effort);
}

TEST_F(ControlCycleTest, NonFiniteCommandsFault) {
    enable();
    auto joint = JointMask{};
    joint.set(2);
    push(ActivateController{.goal = GoalId{10},
                            .controller = std::make_unique<FixedCommandController>(
                                joint, std::numeric_limits<double>::quiet_NaN(), 0.0)});
    run(1);
    EXPECT_EQ(snapshot().safety, SafetyState::Faulted);
    EXPECT_EQ(snapshot().fault, FaultCode::InvalidCommand);
    EXPECT_TRUE(std::isfinite(backend->lastCommand().position[2]));
}

TEST_F(ControlCycleTest, DisablingStopsGoalsAndReportsPowerOff) {
    enable();
    auto target = clearancePose();
    target[0] = 1.0;
    push(ActivateController{.goal = GoalId{11}, .controller = trajectoryTo(target)});
    run(10);
    push(SetDrivePower{.power = DrivePower::Disabled});
    run(3);
    EXPECT_EQ(finished(GoalId{11})->status, ControlStatus::Stopped);
    EXPECT_TRUE(findEvent<PowerChanged>(
        [](PowerChanged const &event) { return event.power == DrivePower::Disabled; }));
    EXPECT_EQ(backend->lastCommand().stiffness.norm(), 0.0);
    EXPECT_EQ(snapshot().safety, SafetyState::Normal);
}

} // namespace
} // namespace larm::control
