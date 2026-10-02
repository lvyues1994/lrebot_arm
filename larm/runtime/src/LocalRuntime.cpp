#include "Coordinator.h"
#include "Operations.h"

#include <larm/control/ControlCycle.h>
#include <larm/control/GripperController.h>
#include <larm/control/JointTrajectoryController.h>
#include <larm/model/RobotModel.h>
#include <larm/motion/Planning.h>
#include <larm/runtime/LocalRuntime.h>

#include <lexec/schedulers/static_thread_pool.hpp>

#include <algorithm>
#include <cmath>

namespace larm::runtime {
namespace {

constexpr double kRestToleranceRadian = 0.05;
constexpr Duration kResetTimeout = std::chrono::milliseconds{200};

[[noreturn]] void reject(MotionFailure const reason, std::string const &message) {
    throw MotionError{reason, control::FaultCode::None, message};
}

JointMask maskOf(JointGroupSpec const &group) {
    auto mask = JointMask{};
    for (auto const joint : group.joints) {
        mask.set(joint);
    }
    return mask;
}

JointVector groupValues(JointGroupSpec const &group, JointVector const &full) {
    auto values = zeroJointVector(group.joints.size());
    for (std::size_t i = 0; i < group.joints.size(); ++i) {
        values[idx(i)] = full[idx(group.joints[i])];
    }
    return values;
}

JointVector withGroupValues(JointGroupSpec const &group, JointVector full, JointVector const &values) {
    for (std::size_t i = 0; i < group.joints.size(); ++i) {
        full[idx(group.joints[i])] = values[idx(i)];
    }
    return full;
}

void requireWithinLimits(RobotProfile const &profile, JointMask const &joints, JointVector const &position) {
    for (std::size_t i = 0; i < profile.dof(); ++i) {
        auto const &limits = profile.joints[i].limits;
        if (joints.test(i) and (not std::isfinite(position[idx(i)]) or position[idx(i)] < limits.lower or
                                position[idx(i)] > limits.upper)) {
            reject(MotionFailure::InvalidGoal,
                   "joint '" + profile.joints[i].name + "' target is outside its limits");
        }
    }
}

double requireSpeed(double const speed) {
    if (not(speed > 0.0 and speed <= 1.0)) {
        reject(MotionFailure::InvalidGoal, "speed must be in (0, 1]");
    }
    return speed;
}

std::unique_ptr<control::Controller>
trajectoryController(RobotProfile const &profile, std::shared_ptr<motion::JointTrajectory const> trajectory,
                     JointMask const &joints) {
    auto const tolerance = control::profileTrackingTolerance(profile);
    return control::makeJointTrajectoryController({
        .trajectory = std::move(trajectory),
        .joints = joints,
        .impedance = control::profileImpedance(profile),
        .trackingTolerance = tolerance,
        .goalTolerance = tolerance * 0.05,
        .goalTimeout = std::chrono::seconds{1},
        .stopLimits = motion::motionLimits(profile),
    });
}

// A rest-to-rest motion of `joints` to `target`, planned from the commanded state at activation.
detail::ControllerFactory pointToPoint(RobotProfile const &profile, JointMask const joints,
                                       JointVector const target, double const speed) {
    return [&profile, joints, target,
            speed](control::RobotSnapshot const &snapshot) -> Expected<std::unique_ptr<control::Controller>> {
        auto start = motion::JointSample::zero(profile.dof());
        start.position = snapshot.command.position;
        auto const trajectory = motion::planPointToPoint({.start = start,
                                                          .target = target,
                                                          .limits = motion::motionLimits(profile, speed),
                                                          .joints = joints});
        if (not trajectory) {
            return tl::make_unexpected(trajectory.error());
        }
        return trajectoryController(profile, *trajectory, joints);
    };
}

struct LocalRuntime;

struct GroupMotion final : MotionApi {
    GroupMotion(LocalRuntime *runtime_, std::size_t const group_) : runtime{runtime_}, group{group_} {}

    Async<MotionResult> moveToJoints(JointGoal goal) override;
    Async<MotionResult> moveToPose(PoseGoal goal) override;
    Async<MotionResult> followPath(JointPath path) override;

  private:
    LocalRuntime *runtime;
    std::size_t group;
};

struct GroupGripper final : GripperApi {
    GroupGripper(LocalRuntime *runtime_, std::size_t const group_) : runtime{runtime_}, group{group_} {}

    Async<GripResult> grip(GripGoal goal) override;

  private:
    LocalRuntime *runtime;
    std::size_t group;
};

struct LocalRuntime final : RobotSession {
    LocalRuntime(RobotProfile profile_, std::unique_ptr<model::RobotModel> model_,
                 std::unique_ptr<hal::Backend> backend_, std::unique_ptr<control::RuntimeChannels> channels_,
                 std::unique_ptr<control::ControlCycle> cycle_, RuntimeOptions const &options_)
        : options{options_}, robot{std::move(profile_)}, model{std::move(model_)},
          backend{std::move(backend_)}, channels{std::move(channels_)}, cycle{std::move(cycle_)},
          coordinator{std::make_unique<detail::Coordinator>(*channels, options.eventPeriod)},
          pool{std::make_unique<lexec::static_thread_pool>(options.planningThreads)},
          runner{std::make_unique<control::RealtimeRunner>(*cycle, backend->timeline(), options.realtime)} {
        for (std::size_t i = 0; i < robot.groups.size(); ++i) {
            auto const &group = robot.groups[i];
            motions.push_back(group.toolFrame.empty() ? nullptr : std::make_unique<GroupMotion>(this, i));
            grippers.push_back(group.joints.size() == 1 ? std::make_unique<GroupGripper>(this, i) : nullptr);
        }
    }

    ~LocalRuntime() override {
        runner.reset();
        coordinator->shutdown();
        pool.reset();
    }

    RobotProfile const &profile() const noexcept override { return robot; }

    Async<> enable() override {
        return detail::WaitSender{
            coordinator.get(),
            {.begin =
                 [this] {
                     return coordinator->send(control::SetDrivePower{.power = hal::DrivePower::Enabled});
                 },
             .check =
                 [](control::RobotSnapshot const &snapshot, std::span<control::ControlEvent const>) {
                     return snapshot.power == hal::DrivePower::Enabled
                                ? detail::WaitVerdict{detail::Reached{}}
                                : detail::WaitVerdict{detail::Waiting{}};
                 },
             .timeout = options.stateTimeout}};
    }

    Async<> disable(DisableOptions const disableOptions) override {
        auto const current = latest();
        if (not disableOptions.force and not atRest(current)) {
            return Async<>{lexec::just_error(std::make_exception_ptr(
                MotionError{MotionFailure::InvalidGoal, control::FaultCode::None,
                            "the arm has no brakes; park it before disabling or force the disable"}))};
        }
        auto const dof = robot.dof();
        return detail::WaitSender{
            coordinator.get(),
            {.begin =
                 [this] {
                     return coordinator->send(control::SetDrivePower{.power = hal::DrivePower::Disabled});
                 },
             .check =
                 [dof](control::RobotSnapshot const &snapshot, std::span<control::ControlEvent const>) {
                     auto const anyOn =
                         std::any_of(snapshot.state.actuators.begin(),
                                     snapshot.state.actuators.begin() + static_cast<long>(dof),
                                     [](ActuatorStatus const &actuator) { return actuator.enabled; });
                     return snapshot.power == hal::DrivePower::Disabled and not anyOn
                                ? detail::WaitVerdict{detail::Reached{}}
                                : detail::WaitVerdict{detail::Waiting{}};
                 },
             .timeout = options.stateTimeout}};
    }

    Async<MotionResult> park() override {
        auto all = JointMask{};
        for (std::size_t i = 0; i < robot.dof(); ++i) {
            all.set(i);
        }
        auto const rest = robot.safety.restPose;
        return planned<MotionResult>([this, all, rest] {
            return detail::GoalPlan<MotionResult>{
                .request = {.joints = all, .makeController = pointToPoint(robot, all, rest, 0.5)},
                .result = [](control::RobotSnapshot const &snapshot) {
                    return MotionResult{.position = snapshot.state.joints.position};
                }};
        });
    }

    Async<> resetFault() override {
        return detail::WaitSender{
            coordinator.get(),
            {.begin = [this] { return coordinator->send(control::ResetFault{}); },
             .check =
                 [](control::RobotSnapshot const &snapshot, std::span<control::ControlEvent const>) {
                     return snapshot.safety == control::SafetyState::Normal
                                ? detail::WaitVerdict{detail::Reached{}}
                                : detail::WaitVerdict{detail::Waiting{}};
                 },
             .timeout = kResetTimeout}};
    }

    void emergencyStop() noexcept override {
        while (not coordinator->send(control::EmergencyStop{})) {
            std::this_thread::yield();
        }
    }

    control::RobotSnapshot latest() const override { return coordinator->latest(); }

    MotionApi *motion(std::string_view const group) override {
        auto const index = robot.findGroup(group);
        return index ? motions[*index].get() : nullptr;
    }

    GripperApi *gripper(std::string_view const group) override {
        auto const index = robot.findGroup(group);
        return index ? grippers[*index].get() : nullptr;
    }

    // Runs `prepare` on the planning pool, then executes the goal it describes. `prepare` may throw
    // MotionError to reject the goal.
    template <class Result, class Prepare> Async<Result> planned(Prepare prepare) {
        return lexec::schedule(pool->get_scheduler()) |
               lexec::let_value([this, prepare = std::move(prepare)]() {
                   return detail::GoalSender<Result>{coordinator.get(), prepare()};
               });
    }

    bool atRest(control::RobotSnapshot const &snapshot) const {
        for (std::size_t i = 0; i < robot.dof(); ++i) {
            if (robot.joints[i].unit == JointUnit::Radian and
                std::abs(snapshot.state.joints.position[idx(i)] - robot.safety.restPose[idx(i)]) >
                    kRestToleranceRadian) {
                return false;
            }
        }
        return true;
    }

    RuntimeOptions options;
    RobotProfile robot;
    std::unique_ptr<model::RobotModel> model;
    std::unique_ptr<hal::Backend> backend;
    std::unique_ptr<control::RuntimeChannels> channels;
    std::unique_ptr<control::ControlCycle> cycle;
    std::unique_ptr<detail::Coordinator> coordinator;
    std::unique_ptr<lexec::static_thread_pool> pool;
    std::unique_ptr<control::RealtimeRunner> runner;
    std::vector<std::unique_ptr<GroupMotion>> motions;
    std::vector<std::unique_ptr<GroupGripper>> grippers;
};

detail::GoalPlan<MotionResult> jointPlan(LocalRuntime const &runtime, std::size_t const group,
                                         JointVector const &target, double const speed) {
    auto const &spec = runtime.robot.groups[group];
    auto const joints = maskOf(spec);
    auto const full = withGroupValues(spec, runtime.latest().command.position, target);
    requireWithinLimits(runtime.robot, joints, full);
    return detail::GoalPlan<MotionResult>{
        .request = {.joints = joints, .makeController = pointToPoint(runtime.robot, joints, full, speed)},
        .result = [spec](control::RobotSnapshot const &snapshot) {
            return MotionResult{.position = groupValues(spec, snapshot.state.joints.position)};
        }};
}

Async<MotionResult> GroupMotion::moveToJoints(JointGoal goal) {
    return runtime->planned<MotionResult>([runtime = runtime, group = group, goal = std::move(goal)] {
        if (dofOf(goal.position) != runtime->robot.groups[group].joints.size()) {
            reject(MotionFailure::InvalidGoal, "joint goal has the wrong number of values");
        }
        return jointPlan(*runtime, group, goal.position, requireSpeed(goal.speed));
    });
}

Async<MotionResult> GroupMotion::moveToPose(PoseGoal goal) {
    return runtime->planned<MotionResult>([runtime = runtime, group = group, goal = std::move(goal)] {
        auto const &spec = runtime->robot.groups[group];
        auto kinematics = runtime->model->makeKinematics();
        auto const base = kinematics->findFrame(spec.baseFrame);
        auto const tool = kinematics->findFrame(spec.toolFrame);
        if (not base or not tool) {
            reject(MotionFailure::InvalidGoal, "group '" + spec.name + "' frames are not in the model");
        }
        auto const seed = runtime->latest().command.position;
        kinematics->update(seed);
        auto const target = kinematics->framePose(*base) * goal.target;
        auto const solution = runtime->model->makeIkSolver()->solve(
            model::IkRequest{.target = target, .frame = *tool, .joints = maskOf(spec)}, seed);
        if (not solution.converged) {
            reject(MotionFailure::Unreachable, "no joint configuration reaches the pose");
        }
        return jointPlan(*runtime, group, groupValues(spec, solution.position), requireSpeed(goal.speed));
    });
}

Async<MotionResult> GroupMotion::followPath(JointPath path) {
    return runtime->planned<MotionResult>([runtime = runtime, group = group, path = std::move(path)] {
        auto const &spec = runtime->robot.groups[group];
        auto const joints = maskOf(spec);
        if (path.waypoints.empty()) {
            reject(MotionFailure::InvalidGoal, "the path has no waypoints");
        }
        auto const width = spec.joints.size();
        for (auto const &waypoint : path.waypoints) {
            if (dofOf(waypoint.position) != width or
                (waypoint.velocity and dofOf(*waypoint.velocity) != width) or
                (waypoint.acceleration and dofOf(*waypoint.acceleration) != width)) {
                reject(MotionFailure::InvalidGoal, "a waypoint has the wrong number of values");
            }
            requireWithinLimits(runtime->robot, joints,
                                withGroupValues(spec, runtime->robot.safety.restPose, waypoint.position));
        }
        auto const &robot = runtime->robot;
        auto factory =
            [&robot, spec, joints,
             path](control::RobotSnapshot const &snapshot) -> Expected<std::unique_ptr<control::Controller>> {
            auto const current = snapshot.command.position;
            auto const expand = [&](JointVector const &values) {
                return withGroupValues(spec, current, values);
            };
            auto const expandRate =
                [&](std::optional<JointVector> const &values) -> std::optional<JointVector> {
                if (not values) {
                    return std::nullopt;
                }
                return withGroupValues(spec, zeroJointVector(dofOf(current)), *values);
            };
            auto waypoints = std::vector<motion::TimedWaypoint>{};
            if (path.waypoints.front().time > Duration{0}) {
                auto const zero = zeroJointVector(dofOf(current));
                waypoints.push_back(
                    {.time = Duration{0}, .position = current, .velocity = zero, .acceleration = zero});
            }
            for (auto const &waypoint : path.waypoints) {
                waypoints.push_back({.time = waypoint.time,
                                     .position = expand(waypoint.position),
                                     .velocity = expandRate(waypoint.velocity),
                                     .acceleration = expandRate(waypoint.acceleration)});
            }
            auto const trajectory = motion::interpolateWaypoints(waypoints);
            if (not trajectory) {
                return tl::make_unexpected(trajectory.error());
            }
            return trajectoryController(robot, *trajectory, joints);
        };
        return detail::GoalPlan<MotionResult>{
            .request = {.joints = joints, .makeController = std::move(factory)},
            .result = [spec](control::RobotSnapshot const &snapshot) {
                return MotionResult{.position = groupValues(spec, snapshot.state.joints.position)};
            }};
    });
}

Async<GripResult> GroupGripper::grip(GripGoal goal) {
    return runtime->planned<GripResult>([runtime = runtime, group = group, goal] {
        auto const joint = runtime->robot.groups[group].joints.front();
        auto const &spec = runtime->robot.joints[joint];
        if (not std::isfinite(goal.position) or not(goal.maxEffort > 0.0)) {
            reject(MotionFailure::InvalidGoal, "grip needs a finite position and a positive effort");
        }
        auto const tolerance = 0.02 * (spec.limits.upper - spec.limits.lower);
        auto const config = control::GripperControllerConfig{
            .joint = joint,
            .target = goal.position,
            .maxEffort = std::min(goal.maxEffort, spec.limits.effort),
            .speed = spec.limits.velocity,
            .stiffness = spec.gains.stiffness,
            .damping = spec.gains.damping,
            .positionTolerance = tolerance,
            .stallVelocity = 0.05 * spec.limits.velocity,
            .stallTime = std::chrono::milliseconds{200},
        };
        auto mask = JointMask{};
        mask.set(joint);
        return detail::GoalPlan<GripResult>{
            .request = {.joints = mask,
                        .makeController = [config](control::RobotSnapshot const &)
                            -> Expected<std::unique_ptr<control::Controller>> {
                            return control::makeGripperController(config);
                        }},
            .result = [joint, target = goal.position, tolerance](control::RobotSnapshot const &snapshot) {
                auto const position = snapshot.state.joints.position[idx(joint)];
                auto const reached = std::abs(position - target) <= tolerance;
                return GripResult{.position = position,
                                  .effort = snapshot.state.joints.effort[idx(joint)],
                                  .reachedGoal = reached,
                                  .stalled = not reached};
            }};
    });
}

} // namespace

std::string_view toString(MotionFailure const failure) noexcept {
    switch (failure) {
    case MotionFailure::InvalidGoal:
        return "invalid_goal";
    case MotionFailure::Unreachable:
        return "unreachable";
    case MotionFailure::PlanningFailed:
        return "planning_failed";
    case MotionFailure::NotEnabled:
        return "not_enabled";
    case MotionFailure::Fault:
        return "fault";
    case MotionFailure::Timeout:
        return "timeout";
    case MotionFailure::Shutdown:
        return "shutdown";
    }
    return "unknown";
}

Expected<std::unique_ptr<RobotSession>> startLocalRuntime(RobotProfile profile,
                                                          std::unique_ptr<hal::Backend> backend,
                                                          RuntimeOptions const &options) {
    if (not backend) {
        return makeError(ErrorCode::InvalidArgument, "the runtime needs a backend");
    }
    auto model = model::loadRobotModel(profile);
    if (not model) {
        return tl::make_unexpected(model.error());
    }
    if (auto connected = backend->driver().connect(); not connected) {
        return tl::make_unexpected(connected.error());
    }
    auto channels = std::make_unique<control::RuntimeChannels>(profile.dof());
    auto cycle = control::makeControlCycle(
        profile, {.driver = &backend->driver(), .model = model->get(), .channels = channels.get()});
    if (not cycle) {
        return tl::make_unexpected(cycle.error());
    }
    return std::unique_ptr<RobotSession>{
        std::make_unique<LocalRuntime>(std::move(profile), std::move(*model), std::move(backend),
                                       std::move(channels), std::move(*cycle), options)};
}

} // namespace larm::runtime
