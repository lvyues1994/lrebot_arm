#include <larm/control/ControlCycle.h>
#include <larm/control/GravityCompController.h>
#include <larm/control/GripperController.h>
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

// Forwards to another driver and, when the cycle writes (before it publishes its snapshot), takes the
// events the cycle has made visible so far.
struct PeekingDriver final : hal::RobotDriver, hal::RealtimeIo {
    PeekingDriver(hal::RobotDriver &inner_, RuntimeChannels &channels_)
        : inner{&inner_}, channels{&channels_} {}

    hal::DriverCapabilities capabilities() const override { return inner->capabilities(); }
    Expected<void> connect() override { return inner->connect(); }
    void disconnect() noexcept override { inner->disconnect(); }
    hal::RealtimeIo &io() override { return *this; }
    void read(RobotState &out) noexcept override { inner->io().read(out); }
    void write(JointCommand const &command, hal::PowerRequest const &power) noexcept override {
        inner->io().write(command, power);
        while (channels->events.tryPop()) {
            ++earlyEvents;
        }
    }

    hal::RobotDriver *inner;
    RuntimeChannels *channels;
    int earlyEvents{};
};

TEST_F(ControlCycleTest, EventsArriveAfterTheSnapshotOfTheirCycle) {
    auto peeking = PeekingDriver{backend->driver(), *channels};
    auto made = makeControlCycle(profile,
                                 {.driver = &peeking, .model = robotModel.get(), .channels = channels.get()});
    ASSERT_TRUE(made) << made.error().message;
    auto &own = **made;
    own.tick();
    backend->timeline().advance();
    // Rejected while disabled, the goal finishes in the tick that takes it.
    push(ActivateController{.goal = GoalId{21},
                            .controller = std::make_unique<FixedCommandController>(arm, 0.0, 0.0)});
    own.tick();
    EXPECT_EQ(peeking.earlyEvents, 0);
    auto const event = channels->events.tryPop();
    ASSERT_TRUE(event);
    ASSERT_TRUE(std::holds_alternative<GoalFinished>(*event));
    EXPECT_EQ(std::get<GoalFinished>(*event).goal, GoalId{21});
}

TEST_F(ControlCycleTest, StaysPassiveUntilEnabledThenRampsStiffness) {
    run(5);
    EXPECT_EQ(backend->lastRequestedPower(), DrivePower::Disabled);
    EXPECT_EQ(backend->lastCommand().stiffness.norm(), 0.0);

    push(SetDrivePower{.power = DrivePower::Enabled});
    run(60);
    EXPECT_FALSE(powered());
    auto const halfway = backend->lastCommand().stiffness[1];
    auto const halfwayEffort = backend->lastCommand().effort[1];
    EXPECT_GT(halfway, 0.0);
    EXPECT_LT(halfway, profile.joints[1].gains.stiffness);

    enable();
    EXPECT_DOUBLE_EQ(backend->lastCommand().stiffness[1], profile.joints[1].gains.stiffness);
    EXPECT_TRUE(backend->lastCommand().position.isApprox(clearancePose()));
    // The gravity feedforward ramps with the stiffness.
    auto const fullEffort = backend->lastCommand().effort[1];
    ASSERT_GT(std::abs(fullEffort), 0.1);
    EXPECT_NEAR(halfwayEffort / fullEffort, halfway / profile.joints[1].gains.stiffness, 1e-3);
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

TEST_F(ControlCycleTest, StaleFeedbackWhileDisabledOnlyKeepsTheArmFromEnabling) {
    enable();
    push(SetDrivePower{.power = DrivePower::Disabled});
    run(3);
    ASSERT_EQ(snapshot().power, DrivePower::Disabled);
    auto const timeout = static_cast<int>(profile.safety.feedbackTimeoutCycles);
    backend->setFeedbackLost(true);
    run(5 * timeout);
    EXPECT_FALSE(snapshot().state.isFresh);
    EXPECT_EQ(snapshot().safety, SafetyState::Normal);

    push(SetDrivePower{.power = DrivePower::Enabled});
    run(timeout + 1);
    EXPECT_EQ(snapshot().power, DrivePower::Disabled);
    EXPECT_EQ(snapshot().safety, SafetyState::Faulted);
    EXPECT_EQ(snapshot().fault, FaultCode::FeedbackLost);
}

TEST_F(ControlCycleTest, ActuatorsFallingSilentAsTheyAreDisabledDoNotFault) {
    enable();
    auto const timeout = static_cast<int>(profile.safety.feedbackTimeoutCycles);
    // The actuators still report themselves on while their feedback is stale.
    backend->setFeedbackLost(true);
    push(SetDrivePower{.power = DrivePower::Disabled});
    run(5 * timeout);
    EXPECT_EQ(snapshot().safety, SafetyState::Normal);
    backend->setFeedbackLost(false);
    run(2);
    EXPECT_EQ(snapshot().power, DrivePower::Disabled);
    EXPECT_EQ(snapshot().safety, SafetyState::Normal);
}

TEST_F(ControlCycleTest, ResetReArmsADroppedActuatorWhileTheOthersHold) {
    enable();
    run(10);
    backend->dropActuator(2);
    run(2);
    EXPECT_EQ(snapshot().safety, SafetyState::Faulted);
    EXPECT_EQ(snapshot().fault, FaultCode::ActuatorFault);
    EXPECT_EQ(snapshot().power, DrivePower::Disabled);
    EXPECT_DOUBLE_EQ(backend->lastCommand().stiffness[1], profile.joints[1].gains.stiffness);
    EXPECT_EQ(backend->lastCommand().stiffness[2], 0.0);
    run(20);
    EXPECT_EQ(snapshot().safety, SafetyState::Faulted);

    events.clear();
    push(ResetFault{});
    run(2);
    EXPECT_EQ(snapshot().safety, SafetyState::Normal);
    EXPECT_LT(backend->lastCommand().stiffness[2], profile.joints[2].gains.stiffness);
    EXPECT_DOUBLE_EQ(backend->lastCommand().stiffness[1], profile.joints[1].gains.stiffness);
    EXPECT_FALSE(powered());
    for (int i = 0; i < 300 and not powered(); ++i) {
        run(1);
        EXPECT_DOUBLE_EQ(backend->lastCommand().stiffness[1], profile.joints[1].gains.stiffness);
    }
    ASSERT_TRUE(powered());
    EXPECT_EQ(snapshot().power, DrivePower::Enabled);
    EXPECT_DOUBLE_EQ(backend->lastCommand().stiffness[2], profile.joints[2].gains.stiffness);
    EXPECT_TRUE(backend->lastCommand().position.isApprox(clearancePose(), 1e-9));
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

std::unique_ptr<Controller> gripTo(double const target) {
    return makeGripperController({
        .joint = 6,
        .target = target,
        .maxEffort = 5.0,
        .speed = 0.05,
        .stiffness = 2000.0,
        .damping = 40.0,
        .positionTolerance = 0.001,
        .stallVelocity = 0.005,
        .stallTime = std::chrono::milliseconds{100},
    });
}

TEST_F(ControlCycleTest, GripperReachesItsTarget) {
    enable();
    push(ActivateController{.goal = GoalId{12}, .controller = gripTo(0.04)});
    for (int i = 0; i < 300 and not finished(GoalId{12}); ++i) {
        run(1);
    }
    ASSERT_TRUE(finished(GoalId{12}));
    EXPECT_EQ(finished(GoalId{12})->status, ControlStatus::Succeeded);
    run(1);
    EXPECT_NEAR(backend->lastCommand().position[6], 0.04, 0.001);
}

TEST_F(ControlCycleTest, GripperForceLimitBoundsTheImpedanceForce) {
    enable();
    push(ActivateController{.goal = GoalId{13}, .controller = gripTo(0.0)});
    for (int i = 0; i < 5; ++i) {
        run(1);
        auto const measured = snapshot().state.joints;
        auto const &command = backend->lastCommand();
        auto const force = command.stiffness[6] * (command.position[6] - measured.position[6]) +
                           command.damping[6] * (command.velocity[6] - measured.velocity[6]);
        EXPECT_LE(std::abs(force), 5.0 + 1e-6) << i;
    }
}

TEST_F(ControlCycleTest, GripperStallsWhenItCannotReachTheTarget) {
    enable();
    push(ActivateController{.goal = GoalId{14}, .controller = gripTo(-0.01)});
    for (int i = 0; i < 400 and not finished(GoalId{14}); ++i) {
        run(1);
    }
    ASSERT_TRUE(finished(GoalId{14}));
    EXPECT_EQ(finished(GoalId{14})->status, ControlStatus::Succeeded);
    EXPECT_NEAR(snapshot().state.joints.position[6], 0.0, 1e-9);
}

TEST_F(ControlCycleTest, GravityModeWallsBeginInsideTheLimitsButNotPastRest) {
    auto const config = profileGravityComp(profile, arm);
    EXPECT_NEAR(config.wallLower[0], profile.joints[0].limits.lower + 0.1, 1e-12);
    EXPECT_NEAR(config.wallUpper[0], profile.joints[0].limits.upper - 0.1, 1e-12);
    // joint2 rests on its lower limit.
    EXPECT_NEAR(config.wallLower[1], profile.joints[1].limits.lower, 1e-12);
    EXPECT_NEAR(config.wallUpper[1], profile.joints[1].limits.upper - 0.1, 1e-12);
    EXPECT_NEAR(config.damping[1], 0.2 * profile.joints[1].gains.damping, 1e-12);
}

TEST_F(ControlCycleTest, GravityModeFreesTheJointsExceptInTheWalls) {
    auto controller = makeGravityCompController(profileGravityComp(profile, arm));
    auto state = RobotState::zero(profile.dof());
    state.joints.position = clearancePose();
    state.joints.position[0] = profile.joints[0].limits.upper - 0.05;
    auto const cache =
        ModelCache{.kinematics = nullptr, .gravity = JointVector::Constant(idx(profile.dof()), 1.5)};
    auto const context =
        ControlContext{.now = {}, .period = profile.controlPeriod, .state = state, .model = cache};
    auto out = JointCommand::zero(profile.dof());
    ASSERT_EQ(controller->start(context).status, ControlStatus::Running);

    EXPECT_EQ(controller->update(context, out).status, ControlStatus::Running);
    EXPECT_DOUBLE_EQ(out.stiffness[1], 0.0);
    EXPECT_DOUBLE_EQ(out.position[1], state.joints.position[1]);
    EXPECT_DOUBLE_EQ(out.effort[1], 1.5);
    EXPECT_DOUBLE_EQ(out.damping[1], 0.2 * profile.joints[1].gains.damping);
    // joint1 is in its upper wall and pulled back to where the wall begins.
    EXPECT_DOUBLE_EQ(out.position[0], profile.joints[0].limits.upper - 0.1);
    EXPECT_DOUBLE_EQ(out.stiffness[0], profile.joints[0].gains.stiffness);
    // The gripper is not claimed.
    EXPECT_DOUBLE_EQ(out.effort[6], 0.0);

    state.joints.velocity[3] = 1.1 * profile.joints[3].limits.velocity;
    auto const flung = controller->update(context, out);
    EXPECT_EQ(flung.status, ControlStatus::Failed);
    EXPECT_EQ(flung.fault, FaultCode::TrackingError);

    state.joints.velocity[3] = 0.0;
    controller->requestStop();
    EXPECT_EQ(controller->update(context, out).status, ControlStatus::Stopped);
    EXPECT_DOUBLE_EQ(out.position[0], state.joints.position[0]);
    EXPECT_DOUBLE_EQ(out.stiffness[0], 0.0);
}

TEST_F(ControlCycleTest, GravityModeReleasesTheArmAndHoldsWhereItIsStopped) {
    enable();
    auto const holding = backend->lastCommand();
    push(ActivateController{.goal = GoalId{31},
                            .controller = makeGravityCompController(profileGravityComp(profile, arm))});
    run(3);
    auto const free = backend->lastCommand();
    for (std::size_t i = 0; i < 6; ++i) {
        EXPECT_DOUBLE_EQ(free.stiffness[idx(i)], 0.0) << i;
        EXPECT_NEAR(free.effort[idx(i)], holding.effort[idx(i)], 1e-9) << i;
    }
    EXPECT_DOUBLE_EQ(free.stiffness[6], holding.stiffness[6]);
    EXPECT_FALSE(finished(GoalId{31}));

    push(CancelGoal{.goal = GoalId{31}});
    run(2);
    ASSERT_TRUE(finished(GoalId{31}));
    EXPECT_EQ(finished(GoalId{31})->status, ControlStatus::Stopped);
    auto const &held = backend->lastCommand();
    for (std::size_t i = 0; i < 6; ++i) {
        EXPECT_DOUBLE_EQ(held.stiffness[idx(i)], profile.joints[i].gains.stiffness) << i;
        EXPECT_NEAR(held.position[idx(i)], snapshot().state.joints.position[idx(i)], 1e-12) << i;
    }
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
