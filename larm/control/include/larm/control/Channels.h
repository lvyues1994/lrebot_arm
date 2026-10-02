#pragma once

#include <larm/control/Controller.h>
#include <larm/core/rt/LatestValue.h>
#include <larm/core/rt/SpscRing.h>
#include <larm/hal/Backend.h>

#include <array>
#include <memory>
#include <variant>

namespace larm::control {

inline constexpr std::size_t kMaxActiveControllers = 8;

struct GoalId {
    std::uint64_t value{};
    friend bool operator==(GoalId, GoalId) = default;
};

enum class SafetyState : std::uint8_t { Normal, Faulted, EmergencyStopped };

// Requests from outside the control loop.
struct ActivateController {
    GoalId goal;
    std::unique_ptr<Controller> controller;
};
struct CancelGoal {
    GoalId goal;
};
struct SetDrivePower {
    hal::DrivePower power{};
};
struct EmergencyStop {};
struct ResetFault {};
using ControlRequest = std::variant<ActivateController, CancelGoal, SetDrivePower, EmergencyStop, ResetFault>;

// Events from the control loop.
struct GoalFinished {
    GoalId goal;
    ControlStatus status{};
    FaultCode fault{};
};
struct SafetyChanged {
    SafetyState state{};
    FaultCode cause{};
};
// Enabled is reported once the actuators are on and the stiffness ramp has finished.
struct PowerChanged {
    hal::DrivePower power{};
};
using ControlEvent = std::variant<GoalFinished, SafetyChanged, PowerChanged>;

struct ActiveGoal {
    GoalId goal;
    JointMask joints;
};

struct RobotSnapshot {
    RobotState state;
    JointCommand command;
    SafetyState safety{};
    FaultCode fault{};
    hal::DrivePower power{};
    std::array<ActiveGoal, kMaxActiveControllers> active{};
    std::size_t activeCount{};
};

struct CycleTelemetry {
    std::uint64_t cycle{};
    TimePoint stamp{};
    // Wall-clock time spent in tick(), excluding the wait for the next period.
    Duration computeTime{};
};

// The only state shared between the control loop and the rest of the process. Each ring has one
// producer and one consumer; controllers come back through `retired` so the loop never frees them.
struct RuntimeChannels {
    explicit RuntimeChannels(std::size_t const dof)
        : snapshot{RobotSnapshot{.state = RobotState::zero(dof), .command = JointCommand::zero(dof)}} {}

    rt::SpscRing<ControlRequest, 64> requests;
    rt::SpscRing<ControlEvent, 256> events;
    rt::SpscRing<std::unique_ptr<Controller>, 64> retired;
    rt::SpscRing<CycleTelemetry, 4096> telemetry;
    rt::LatestValue<RobotSnapshot> snapshot;
};

} // namespace larm::control
