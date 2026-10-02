#pragma once

#include <larm/core/RobotState.h>
#include <larm/model/RobotModel.h>

#include <cstdint>
#include <string_view>

namespace larm::control {

enum class ControlStatus : std::uint8_t { Running, Succeeded, Stopped, Failed };

enum class FaultCode : std::uint8_t {
    None,
    FeedbackLost,
    ActuatorFault,
    PositionLimit,
    InvalidCommand,
    TrackingError,
    GoalNotReached,
    StartRejected,
    ClaimConflict,
    NotEnabled,
    EmergencyStop,
};

std::string_view toString(ControlStatus status) noexcept;
std::string_view toString(FaultCode fault) noexcept;

struct ControlStep {
    ControlStatus status = ControlStatus::Running;
    FaultCode fault = FaultCode::None;
};

// Computed once per cycle from the measured positions and shared by controllers and safety checks.
struct ModelCache {
    model::Kinematics const *kinematics{};
    JointVector gravity;
};

struct ControlContext {
    TimePoint now;
    Duration period;
    RobotState const &state;
    ModelCache const &model;
};

// Built off the control loop with its goal and all memory it needs. The control loop only calls the
// methods below, which must not allocate, block or throw.
struct Controller {
    virtual ~Controller() = default;
    // Joints this controller commands; controllers active at the same time claim disjoint sets.
    virtual JointMask claims() const noexcept = 0;
    // Takes over from the current state; returning anything but Running rejects the activation.
    virtual ControlStep start(ControlContext const &context) noexcept = 0;
    // Writes the claimed joints of `out` only.
    virtual ControlStep update(ControlContext const &context, JointCommand &out) noexcept = 0;
    // Decelerate to rest, then finish with Stopped.
    virtual void requestStop() noexcept = 0;
};

} // namespace larm::control
