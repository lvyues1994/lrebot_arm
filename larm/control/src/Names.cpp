#include <larm/control/Controller.h>

#include <type_traits>

namespace larm::control {

std::string_view toString(ControlStatus const status) noexcept {
    switch (status) {
    case ControlStatus::Running:
        return "running";
    case ControlStatus::Succeeded:
        return "succeeded";
    case ControlStatus::Stopped:
        return "stopped";
    case ControlStatus::Failed:
        return "failed";
    }
    return "unknown";
}

std::string_view toString(FaultCode const fault) noexcept {
    switch (fault) {
    case FaultCode::None:
        return "none";
    case FaultCode::FeedbackLost:
        return "feedback_lost";
    case FaultCode::ActuatorFault:
        return "actuator_fault";
    case FaultCode::PositionLimit:
        return "position_limit";
    case FaultCode::InvalidCommand:
        return "invalid_command";
    case FaultCode::TrackingError:
        return "tracking_error";
    case FaultCode::GoalNotReached:
        return "goal_not_reached";
    case FaultCode::StartRejected:
        return "start_rejected";
    case FaultCode::ClaimConflict:
        return "claim_conflict";
    case FaultCode::NotEnabled:
        return "not_enabled";
    case FaultCode::EmergencyStop:
        return "emergency_stop";
    }
    return "unknown";
}

std::optional<FaultCode> faultFromString(std::string_view const name) noexcept {
    for (auto code = std::underlying_type_t<FaultCode>{0};
         code <= static_cast<std::underlying_type_t<FaultCode>>(FaultCode::EmergencyStop); ++code) {
        auto const fault = static_cast<FaultCode>(code);
        if (toString(fault) == name) {
            return fault;
        }
    }
    return std::nullopt;
}

} // namespace larm::control
