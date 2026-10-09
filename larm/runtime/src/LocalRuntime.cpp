#include "Coordinator.h"
#include "Operations.h"

#include <larm/control/ControlCycle.h>
#include <larm/control/GripperController.h>
#include <larm/control/JointTrajectoryController.h>
#include <larm/model/RobotModel.h>
#include <larm/motion/CollisionScan.h>
#include <larm/motion/LinearMotion.h>
#include <larm/motion/Planning.h>
#include <larm/runtime/LocalRuntime.h>

#include <lexec/schedulers/static_thread_pool.hpp>

#include <algorithm>
#include <cmath>
#include <format>

namespace larm::runtime {
namespace {

constexpr double kRestToleranceRadian = 0.05;
constexpr Duration kResetTimeout = std::chrono::milliseconds{200};
// Largest joint motion between two self-collision checks of a trajectory.
constexpr double kCollisionStepRadian = 0.02;
constexpr double kCollisionStepMeter = 0.002;

using FactoryResult = tl::expected<std::unique_ptr<control::Controller>, MotionError>;

[[noreturn]] void reject(MotionFailure const reason, std::string const &message) {
    throw MotionError{reason, control::FaultCode::None, message};
}

tl::unexpected<MotionError> failure(MotionFailure const reason, std::string const &message) {
    return tl::make_unexpected(MotionError{reason, control::FaultCode::None, message});
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

// A group target with values past a limit by at most the safety tolerance moved onto the limit: a
// joint resting on its limit may measure slightly past it, and its measured position is a fair target.
JointVector withinLimits(RobotProfile const &profile, JointGroupSpec const &group, JointVector values) {
    auto const tolerance = profile.safety.limitTolerance;
    for (std::size_t i = 0; i < group.joints.size(); ++i) {
        auto const &joint = profile.joints[group.joints[i]];
        auto &value = values[idx(i)];
        if (not std::isfinite(value) or value < joint.limits.lower - tolerance or
            value > joint.limits.upper + tolerance) {
            reject(MotionFailure::InvalidGoal,
                   std::format("joint '{}' target {:.3f} is outside its limits [{:.3f}, {:.3f}]", joint.name,
                               value, joint.limits.lower, joint.limits.upper));
        }
        value = std::clamp(value, joint.limits.lower, joint.limits.upper);
    }
    return values;
}

// Checks planned trajectories for self-collision. Only controller factories use it, and the
// coordinator runs those under its lock.
struct CollisionGuard {
    CollisionGuard(RobotProfile const &profile, model::RobotModel const &model)
        : checker{model.makeCollisionChecker()}, step{zeroJointVector(profile.dof())} {
        for (std::size_t i = 0; i < profile.dof(); ++i) {
            step[idx(i)] =
                profile.joints[i].unit == JointUnit::Meter ? kCollisionStepMeter : kCollisionStepRadian;
        }
    }

    tl::expected<void, MotionError> check(motion::JointTrajectory const &trajectory) {
        auto const collision = motion::findCollision(trajectory, *checker, step);
        if (not collision) {
            return {};
        }
        auto const &contact = collision->contact;
        return failure(
            MotionFailure::PlanningFailed,
            std::format("self-collision: {} and {} would overlap by {:.1f} mm {:.2f} s into the motion",
                        contact.first, contact.second, contact.depth * 1e3,
                        std::chrono::duration<double>{collision->time}.count()));
    }

    std::unique_ptr<model::CollisionChecker> checker;
    JointVector step;
};

struct GroupFrames {
    model::FrameId base;
    model::FrameId tool;
};

// Resolves and plans pose goals. Only controller factories use it, and the coordinator runs those
// under its lock.
struct PosePlanner {
    PosePlanner(RobotProfile const &profile, model::RobotModel const &model)
        : kinematics{model.makeKinematics()}, solver{model.makeIkSolver()} {
        for (auto const &group : profile.groups) {
            auto const base = kinematics->findFrame(group.baseFrame);
            auto const tool = kinematics->findFrame(group.toolFrame);
            frames.push_back(base and tool ? std::optional{GroupFrames{.base = *base, .tool = *tool}}
                                           : std::nullopt);
        }
    }

    std::unique_ptr<model::Kinematics> kinematics;
    std::unique_ptr<model::IkSolver> solver;
    // By group; empty for groups whose frames are not in the model.
    std::vector<std::optional<GroupFrames>> frames;
};

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

// A rest-to-rest motion of `joints` to `target` from the commanded state in `snapshot`.
FactoryResult jointMotion(RobotProfile const &profile, CollisionGuard &collisions,
                          control::RobotSnapshot const &snapshot, JointMask const &joints,
                          JointVector const &target, double const speed) {
    auto start = motion::JointSample::zero(profile.dof());
    start.position = snapshot.command.position;
    auto const trajectory = motion::planPointToPoint(
        {.start = start, .target = target, .limits = motion::motionLimits(profile, speed), .joints = joints});
    if (not trajectory) {
        return failure(MotionFailure::PlanningFailed, trajectory.error().message);
    }
    if (auto checked = collisions.check(**trajectory); not checked) {
        return tl::make_unexpected(checked.error());
    }
    return trajectoryController(profile, *trajectory, joints);
}

// A rest-to-rest motion of `joints` to `target`, planned from the commanded state at activation.
detail::ControllerFactory pointToPoint(RobotProfile const &profile, CollisionGuard &collisions,
                                       JointMask const joints, JointVector const target, double const speed) {
    return [&profile, &collisions, joints, target, speed](control::RobotSnapshot const &snapshot) {
        return jointMotion(profile, collisions, snapshot, joints, target, speed);
    };
}

CartesianLimits scaled(CartesianLimits const &limits, double const speed) {
    return CartesianLimits{.linearVelocity = speed * limits.linearVelocity,
                           .linearAcceleration = speed * limits.linearAcceleration,
                           .angularVelocity = speed * limits.angularVelocity,
                           .angularAcceleration = speed * limits.angularAcceleration};
}

// A motion of the group's TCP to `goal`, resolved against the commanded state at activation.
detail::ControllerFactory poseMotion(RobotProfile const &profile, PosePlanner &planner,
                                     CollisionGuard &collisions, std::size_t const group,
                                     PoseGoal const goal) {
    return [&profile, &planner, &collisions, group,
            goal](control::RobotSnapshot const &snapshot) -> FactoryResult {
        auto const &spec = profile.groups[group];
        auto const &frames = *planner.frames[group];
        auto const joints = maskOf(spec);
        auto const &start = snapshot.command.position;
        planner.kinematics->update(start);
        auto const target = goal.frame == Frame::Tool
                                ? planner.kinematics->framePose(frames.tool) * spec.tcp * goal.target
                                : planner.kinematics->framePose(frames.base) * goal.target;
        if (goal.path == PathShape::Linear) {
            auto const trajectory =
                motion::planLinear({.start = start,
                                    .target = target,
                                    .tool = frames.tool,
                                    .tcp = spec.tcp,
                                    .joints = joints,
                                    .cartesian = scaled(*spec.cartesianLimits, goal.speed),
                                    .limits = motion::motionLimits(profile, goal.speed)},
                                   *planner.kinematics, *planner.solver);
            if (not trajectory) {
                return failure(trajectory.error().code == ErrorCode::SolverFailed
                                   ? MotionFailure::Unreachable
                                   : MotionFailure::PlanningFailed,
                               trajectory.error().message);
            }
            if (auto checked = collisions.check(**trajectory); not checked) {
                return tl::make_unexpected(checked.error());
            }
            return trajectoryController(profile, *trajectory, joints);
        }
        auto const solution = planner.solver->solve(
            model::IkRequest{.target = target * spec.tcp.inverse(), .frame = frames.tool, .joints = joints},
            start);
        if (not solution.converged) {
            return failure(MotionFailure::Unreachable, "no joint configuration reaches the pose");
        }
        return jointMotion(profile, collisions, snapshot, joints, solution.position, goal.speed);
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
        : options{options_}, robot{std::move(profile_)}, model{std::move(model_)}, collisions{robot, *model},
          poses{robot, *model}, backend{std::move(backend_)}, channels{std::move(channels_)},
          cycle{std::move(cycle_)},
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
                .request = {.joints = all, .makeController = pointToPoint(robot, collisions, all, rest, 0.5)},
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
    CollisionGuard collisions;
    PosePlanner poses;
    std::unique_ptr<hal::Backend> backend;
    std::unique_ptr<control::RuntimeChannels> channels;
    std::unique_ptr<control::ControlCycle> cycle;
    std::unique_ptr<detail::Coordinator> coordinator;
    std::unique_ptr<lexec::static_thread_pool> pool;
    std::unique_ptr<control::RealtimeRunner> runner;
    std::vector<std::unique_ptr<GroupMotion>> motions;
    std::vector<std::unique_ptr<GroupGripper>> grippers;
};

detail::GoalPlan<MotionResult> jointPlan(LocalRuntime &runtime, std::size_t const group,
                                         JointVector const &target, double const speed) {
    auto const &spec = runtime.robot.groups[group];
    auto const joints = maskOf(spec);
    auto const full =
        withGroupValues(spec, runtime.latest().command.position, withinLimits(runtime.robot, spec, target));
    return detail::GoalPlan<MotionResult>{
        .request = {.joints = joints,
                    .makeController = pointToPoint(runtime.robot, runtime.collisions, joints, full, speed)},
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
        if (not runtime->poses.frames[group]) {
            reject(MotionFailure::InvalidGoal, "group '" + spec.name + "' frames are not in the model");
        }
        if (goal.path == PathShape::Linear and not spec.cartesianLimits) {
            reject(MotionFailure::InvalidGoal,
                   "group '" + spec.name + "' has no cartesian_limits for straight-line motions");
        }
        if (not goal.target.translation.allFinite() or not goal.target.rotation.coeffs().allFinite() or
            goal.target.rotation.norm() < 0.5) {
            reject(MotionFailure::InvalidGoal, "the target pose is not a finite rigid transform");
        }
        auto normalized = goal;
        normalized.target.rotation.normalize();
        requireSpeed(goal.speed);
        return detail::GoalPlan<MotionResult>{
            .request = {.joints = maskOf(spec),
                        .makeController = poseMotion(runtime->robot, runtime->poses, runtime->collisions,
                                                     group, normalized)},
            .result = [spec](control::RobotSnapshot const &snapshot) {
                return MotionResult{.position = groupValues(spec, snapshot.state.joints.position)};
            }};
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
        auto bounded = path;
        for (auto &waypoint : bounded.waypoints) {
            if (dofOf(waypoint.position) != width or
                (waypoint.velocity and dofOf(*waypoint.velocity) != width) or
                (waypoint.acceleration and dofOf(*waypoint.acceleration) != width)) {
                reject(MotionFailure::InvalidGoal, "a waypoint has the wrong number of values");
            }
            waypoint.position = withinLimits(runtime->robot, spec, waypoint.position);
        }
        auto const &robot = runtime->robot;
        auto factory = [&robot, &collisions = runtime->collisions, spec, joints,
                        path = std::move(bounded)](control::RobotSnapshot const &snapshot) -> FactoryResult {
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
                return failure(MotionFailure::PlanningFailed, trajectory.error().message);
            }
            if (auto checked = collisions.check(**trajectory); not checked) {
                return tl::make_unexpected(checked.error());
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
                        .makeController = [config](control::RobotSnapshot const &) -> FactoryResult {
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

std::optional<MotionFailure> failureFromString(std::string_view const name) noexcept {
    for (auto code = std::underlying_type_t<MotionFailure>{0};
         code <= static_cast<std::underlying_type_t<MotionFailure>>(MotionFailure::Shutdown); ++code) {
        auto const failure = static_cast<MotionFailure>(code);
        if (toString(failure) == name) {
            return failure;
        }
    }
    return std::nullopt;
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
