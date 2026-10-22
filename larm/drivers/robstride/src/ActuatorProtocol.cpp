#include "ActuatorProtocol.h"

#include <sstream>

namespace larm::drivers::robstride::detail {
namespace {

constexpr std::size_t kStackSize = 64 * 1024;
constexpr auto kEnableTimeout = std::chrono::milliseconds{200};
constexpr auto kDisableTimeout = std::chrono::milliseconds{200};

std::string hex(std::uint32_t const value) {
    auto stream = std::ostringstream{};
    stream << "0x" << std::hex << std::uppercase << value;
    return stream.str();
}

} // namespace

ActuatorProtocol::ActuatorProtocol(ActuatorConfig config_, ProtocolSettings const &settings_,
                                   CycleInput const &input_)
    : config{std::move(config_)}, ranges{rangesOf(config.model)}, settings{settings_}, input{&input_} {}

Expected<void> ActuatorProtocol::start() {
    auto created = stackfull::coro::makeCoroutine([this] { body(); },
                                                  stackfull::coro::CoroutineOptions{.stackSize = kStackSize});
    if (not created) {
        return makeError(ErrorCode::Internal,
                         describe() + ": cannot create its protocol coroutine: " + created.error.message());
    }
    // Replacing a previous coroutine unwinds it.
    coroutine = std::move(created.coroutine);
    motor = MotorState{};
    phase = Phase::Connecting;
    outgoing.reset();
    awaiting = nullptr;
    answer.reset();
    timedOut = false;
    isReady = false;
    pinged = false;
    failure.reset();
    resume();
    return {};
}

void ActuatorProtocol::resume() {
    if (coroutine and not coroutine.isDone()) {
        coroutine.resume();
    }
}

bool ActuatorProtocol::offer(CanFrame const &frame) noexcept {
    if (not waiting() or not awaiting->matches(awaiting->context, frame)) {
        return false;
    }
    answer = frame;
    return true;
}

CanFrame ActuatorProtocol::poll() const noexcept {
    return encodeReadParameter(config.id, settings.host, parameter::kMechanicalPosition);
}

void ActuatorProtocol::body() {
    setUp();
    if (failure) {
        return;
    }
    isReady = true;
    stackfull::coro::Coroutine::yield();
    operate();
}

void ActuatorProtocol::setUp() {
    auto const id = config.id;
    auto const host = settings.host;
    auto const ping = request(encodePing(id, host), [id](CanFrame const &frame) {
        auto const reply = decodePing(frame);
        return reply and reply->motor == id;
    });
    if (not ping) {
        return fail(describe() + " does not answer on " + settings.interface);
    }
    pinged = true;

    auto const runMode = readParameter(parameter::kRunMode);
    if (not runMode) {
        return fail(describe() + " did not return parameter " + hex(parameter::kRunMode));
    }
    if (runMode->asInt8() != kRunModeMotion) {
        return fail(describe() + " has run_mode " + std::to_string(runMode->asInt8()) +
                    "; motion control needs 0 (set it with MotorBridge Studio)");
    }
    auto const zeroState = readParameter(parameter::kZeroState);
    if (not zeroState) {
        return fail(describe() + " did not return parameter " + hex(parameter::kZeroState));
    }
    if (zeroState->raw[0] != 1) {
        return fail(describe() + " has zero_sta " + std::to_string(zeroState->raw[0]) +
                    "; positions need 1 (-pi..pi), set and save it with MotorBridge Studio");
    }
    auto const position = readParameter(parameter::kMechanicalPosition);
    if (not position) {
        return fail(describe() + " did not return parameter " + hex(parameter::kMechanicalPosition));
    }
    turnShift = turnCorrection(config, static_cast<double>(position->asFloat()));

    if (settings.disableActiveReport) {
        emit(encodeActiveReport(id, host, false));
    }
    if (settings.canTimeoutTicks) {
        auto const acknowledged = request(
            encodeWriteParameter(id, host, parameter::kCanTimeout, rawUint32(*settings.canTimeoutTicks)),
            [this, id](CanFrame const &frame) {
                auto const status = decodeStatus(frame, ranges);
                return status and status->motor == id;
            });
        if (not acknowledged) {
            return fail(describe() + " did not acknowledge its CAN timeout");
        }
    }
    phase = Phase::Idle;
}

void ActuatorProtocol::operate() {
    for (;;) {
        phase = Phase::Idle;
        while (power() == hal::DrivePower::Disabled) {
            emit(poll());
        }
        auto const enabled = enable();
        if (enabled == Enabled::TimedOut) {
            stayDropped();
            continue;
        }
        if (enabled == Enabled::Yes) {
            phase = Phase::Running;
            while (power() == hal::DrivePower::Enabled and motor.mode == MotorMode::Run) {
                emit(motion());
            }
            // Left run mode by itself.
            if (power() == hal::DrivePower::Enabled) {
                stayDropped();
                continue;
            }
        }
        disable();
    }
}

ActuatorProtocol::Enabled ActuatorProtocol::enable() {
    phase = Phase::Enabling;
    if (motor.flags != 0 or motor.reportedFaults != 0) {
        motor.flags = 0;
        motor.reportedFaults = 0;
        emit(encodeDisable(config.id, settings.host, true));
    }
    auto const deadline = now() + kEnableTimeout;
    for (;;) {
        emit(encodeEnable(config.id, settings.host));
        if (motor.mode == MotorMode::Run) {
            return Enabled::Yes;
        }
        if (power() == hal::DrivePower::Disabled) {
            return Enabled::Withdrawn;
        }
        if (now() >= deadline) {
            return Enabled::TimedOut;
        }
    }
}

void ActuatorProtocol::disable() {
    phase = Phase::Disabling;
    auto const deadline = now() + kDisableTimeout;
    do {
        emit(encodeDisable(config.id, settings.host, false));
    } while (motor.mode == MotorMode::Run and now() < deadline);
}

// Off until power is withdrawn or the control loop asks to re-arm.
void ActuatorProtocol::stayDropped() {
    phase = Phase::Dropped;
    auto const since = rearm();
    while (power() == hal::DrivePower::Enabled and rearm() == since) {
        emit(encodeDisable(config.id, settings.host, false));
    }
}

void ActuatorProtocol::emit(CanFrame const &frame) {
    outgoing = frame;
    stackfull::coro::Coroutine::yield();
}

template <class Matches>
std::optional<CanFrame> ActuatorProtocol::request(CanFrame const &frame, Matches const &matches) {
    auto const matcher =
        Matcher{.context = &matches, .matches = [](void const *context, CanFrame const &reply) {
                    return (*static_cast<Matches const *>(context))(reply);
                }};
    answer.reset();
    timedOut = false;
    awaiting = &matcher;
    outgoing = frame;
    do {
        stackfull::coro::Coroutine::yield();
    } while (not answer and not timedOut);
    awaiting = nullptr;
    return std::exchange(answer, std::nullopt);
}

std::optional<ParameterValue> ActuatorProtocol::readParameter(std::uint16_t const index) {
    auto const id = config.id;
    auto const reply =
        request(encodeReadParameter(id, settings.host, index), [id, index](CanFrame const &frame) {
            auto const value = decodeParameter(frame);
            return value and value->motor == id and value->index == index;
        });
    auto const value = reply ? decodeParameter(*reply) : std::nullopt;
    if (not value or not value->ok) {
        return std::nullopt;
    }
    return value;
}

void ActuatorProtocol::fail(std::string message) { failure = std::move(message); }

CanFrame ActuatorProtocol::motion() const noexcept {
    auto const &[scale, offset] = config.transmission;
    auto const &command = *input->command;
    auto const j = idx(config.joint);
    return encodeMotion(config.id,
                        MotionCommand{
                            .position = (command.position[j] - offset) / scale - turnShift,
                            .velocity = command.velocity[j] / scale,
                            .stiffness = command.stiffness[j] * scale * scale,
                            .damping = command.damping[j] * scale * scale,
                            .torque = command.effort[j] * scale,
                        },
                        ranges);
}

std::string ActuatorProtocol::describe() const {
    return "actuator " + hex(config.id) + " (" + config.jointName + ")";
}

} // namespace larm::drivers::robstride::detail
