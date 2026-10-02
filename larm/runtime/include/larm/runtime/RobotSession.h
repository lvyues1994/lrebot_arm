#pragma once

#include <larm/control/Channels.h>
#include <larm/core/Pose.h>
#include <larm/core/RobotProfile.h>

#include <lexec/any_sender_of.hpp>
#include <lexec/execution.hpp>

#include <cstdint>
#include <exception>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace larm::runtime {

template <class... Values>
using Async = lexec::any_sender_of<lexec::set_value_t(Values...), lexec::set_error_t(std::exception_ptr),
                                   lexec::set_stopped_t()>;

enum class MotionFailure : std::uint8_t {
    InvalidGoal,
    Unreachable,
    PlanningFailed,
    NotEnabled,
    Fault,
    Timeout,
    Shutdown,
};

std::string_view toString(MotionFailure failure) noexcept;
std::optional<MotionFailure> failureFromString(std::string_view name) noexcept;

// The error every session operation reports through set_error.
struct MotionError : std::runtime_error {
    MotionError(MotionFailure reason_, control::FaultCode fault_, std::string const &message)
        : std::runtime_error{message}, reason{reason_}, fault{fault_} {}

    MotionFailure reason;
    control::FaultCode fault;
};

// Positions are in the group's joint order.
struct JointGoal {
    JointVector position;
    // Fraction of the profile's motion limits, in (0, 1].
    double speed = 1.0;
};

// Pose of the group's tool frame in its base frame.
struct PoseGoal {
    Pose3 target;
    double speed = 1.0;
};

struct PathWaypoint {
    Duration time{};
    JointVector position;
    std::optional<JointVector> velocity;
    std::optional<JointVector> acceleration;
};

// A timed path in the group's joint order. If the first waypoint is later than time zero, the path
// starts from the current position.
struct JointPath {
    std::vector<PathWaypoint> waypoints;
};

struct MotionResult {
    // Measured group positions when the motion finished.
    JointVector position;
};

struct GripGoal {
    double position{};
    double maxEffort{};
};

struct GripResult {
    double position{};
    double effort{};
    bool reachedGoal{};
    bool stalled{};
};

// Motions of one joint group. Completions arrive on a runtime thread; continue elsewhere with
// lexec::continues_on. Stop requests decelerate the group and complete with set_stopped. A new goal
// on the same joints preempts the running one, which completes with set_stopped.
struct MotionApi {
    virtual ~MotionApi() = default;
    virtual Async<MotionResult> moveToJoints(JointGoal goal) = 0;
    virtual Async<MotionResult> moveToPose(PoseGoal goal) = 0;
    virtual Async<MotionResult> followPath(JointPath path) = 0;
};

struct GripperApi {
    virtual ~GripperApi() = default;
    virtual Async<GripResult> grip(GripGoal goal) = 0;
};

struct DisableOptions {
    // Disable even away from the rest pose. The arm has no brakes and will fall.
    bool force{};
};

struct RobotSession {
    virtual ~RobotSession() = default;
    virtual RobotProfile const &profile() const noexcept = 0;
    // Completes once the actuators are on and the stiffness ramp has finished.
    virtual Async<> enable() = 0;
    virtual Async<> disable(DisableOptions options) = 0;
    // Moves every joint to the profile's rest pose.
    virtual Async<MotionResult> park() = 0;
    virtual Async<> resetFault() = 0;
    virtual void emergencyStop() noexcept = 0;
    virtual control::RobotSnapshot latest() const = 0;
    // Null when the profile has no such group, or the group has no tool frame / is not a single joint.
    virtual MotionApi *motion(std::string_view group) = 0;
    virtual GripperApi *gripper(std::string_view group) = 0;
};

} // namespace larm::runtime
