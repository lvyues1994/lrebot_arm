#include <larm/drivers/robstride/RobStrideDriver.h>

#include <atomic>
#include <cmath>
#include <functional>
#include <sstream>

namespace larm::drivers::robstride {
namespace {

constexpr std::size_t kReceiveBatch = 64;
constexpr std::uint32_t kEnableAttempts = 50;
constexpr std::uint32_t kDisableAttempts = 50;
constexpr auto kReplyTimeout = std::chrono::milliseconds{100};
// The motor's CAN timeout counts 20000 ticks per second.
constexpr double kCanTimeoutTicksPerSecond = 20000.0;

enum class Phase : std::uint8_t { Idle, Disabling, Clearing, Enabling, Running, Dropped };

// Feedback is kept in motor units.
struct Actuator {
    ActuatorConfig config;
    ModelRanges ranges;
    Phase phase = Phase::Idle;
    std::uint32_t attempts{};
    MotorMode mode = MotorMode::Reset;
    std::uint8_t flags{};
    std::uint32_t reportedFaults{};
    double position{};
    double velocity{};
    double torque{};
    double temperature{};
    bool replied{};
    bool known{};
};

std::string hex(std::uint32_t const value) {
    auto stream = std::ostringstream{};
    stream << "0x" << std::hex << std::uppercase << value;
    return stream.str();
}

struct RobStrideDriverImpl final : RobStrideDriver {
    RobStrideDriverImpl(DriverConfig config_, std::unique_ptr<can::CanTransport> transport_,
                        hal::Timeline const &clock_)
        : config{std::move(config_)}, transport{std::move(transport_)}, clock{&clock_}, realtimeIo{this} {
        motorIndex.fill(kNoActuator);
        for (std::size_t i = 0; i < config.actuators.size(); ++i) {
            auto const &actuator = config.actuators[i];
            actuators.push_back(Actuator{.config = actuator, .ranges = rangesOf(actuator.model)});
            motorIndex[actuator.id] = i;
        }
    }

    hal::DriverCapabilities capabilities() const override {
        auto modes = std::bitset<4>{};
        modes.set(static_cast<std::size_t>(hal::CommandMode::Impedance));
        return hal::DriverCapabilities{.modes = modes, .minPeriod = cycleBusTime(config), .hasBrakes = false};
    }

    Expected<void> connect() override {
        discardPending();
        for (auto &actuator : actuators) {
            auto const id = actuator.config.id;
            auto const answered = request(encodePing(id, config.hostId), [id](CanFrame const &frame) {
                auto const reply = decodePing(frame);
                return reply and reply->motor == id;
            });
            if (not answered) {
                return makeError(ErrorCode::Io,
                                 describe(actuator) + " does not answer on " + config.interface);
            }
        }
        for (auto &actuator : actuators) {
            auto const runMode = readParameter(actuator, parameter::kRunMode);
            if (not runMode) {
                return tl::make_unexpected(runMode.error());
            }
            if (runMode->asInt8() != kRunModeMotion) {
                return makeError(ErrorCode::InvalidConfig,
                                 describe(actuator) + " has run_mode " + std::to_string(runMode->asInt8()) +
                                     "; motion control needs 0 (set it with MotorBridge Studio)");
            }
            auto const zeroState = readParameter(actuator, parameter::kZeroState);
            if (not zeroState) {
                return tl::make_unexpected(zeroState.error());
            }
            if (zeroState->raw[0] != 1) {
                return makeError(ErrorCode::InvalidConfig,
                                 describe(actuator) + " has zero_sta " + std::to_string(zeroState->raw[0]) +
                                     "; positions need 1 (-pi..pi), set and save it with MotorBridge Studio");
            }
        }
        if (config.disableActiveReport) {
            for (auto const &actuator : actuators) {
                auto const frame = encodeActiveReport(actuator.config.id, config.hostId, false);
                transport->send({&frame, 1});
            }
        }
        if (config.canTimeout) {
            auto const ticks = static_cast<std::uint32_t>(
                std::lround(toSeconds(*config.canTimeout) * kCanTimeoutTicksPerSecond));
            for (auto &actuator : actuators) {
                auto const id = actuator.config.id;
                auto const written =
                    request(encodeWriteParameter(id, config.hostId, parameter::kCanTimeout, rawUint32(ticks)),
                            [this, id](CanFrame const &frame) {
                                auto const status = decodeStatus(frame, actuators[motorIndex[id]].ranges);
                                return status and status->motor == id;
                            });
                if (not written) {
                    return makeError(ErrorCode::Io,
                                     describe(actuator) + " did not acknowledge its CAN timeout");
                }
            }
        }
        for (auto &actuator : actuators) {
            actuator.phase = Phase::Idle;
            actuator.attempts = 0;
            actuator.replied = false;
        }
        lastRequested = hal::DrivePower::Disabled;
        return {};
    }

    void disconnect() noexcept override {}

    hal::RealtimeIo &io() override { return realtimeIo; }

    DriverStatistics statistics() const noexcept override {
        return DriverStatistics{
            .cycles = cycles.load(std::memory_order_relaxed),
            .missedReplies = missedReplies.load(std::memory_order_relaxed),
            .foreignFrames = foreignFrames.load(std::memory_order_relaxed),
            .transport = transport->statistics(),
        };
    }

    void read(RobotState &out) noexcept {
        drain();
        auto const dof = actuators.size();
        out.cycle = cycles.fetch_add(1, std::memory_order_relaxed) + 1;
        out.stamp = clock->now();
        out.joints.position.resize(idx(dof));
        out.joints.velocity.resize(idx(dof));
        out.joints.effort.resize(idx(dof));
        auto fresh = true;
        auto missed = std::uint64_t{0};
        for (std::size_t i = 0; i < dof; ++i) {
            auto &actuator = actuators[i];
            auto const &[scale, offset] = actuator.config.transmission;
            out.joints.position[idx(i)] = scale * actuator.position + offset;
            out.joints.velocity[idx(i)] = scale * actuator.velocity;
            out.joints.effort[idx(i)] = actuator.torque / scale;
            out.actuators[i] = ActuatorStatus{
                .enabled = actuator.known and actuator.mode == MotorMode::Run,
                .faultBits = actuator.flags | (actuator.reportedFaults << 8U),
                .temperature = static_cast<float>(actuator.temperature),
            };
            if (not actuator.replied) {
                fresh = false;
                ++missed;
            }
            actuator.replied = false;
        }
        out.isFresh = fresh;
        missedReplies.fetch_add(missed, std::memory_order_relaxed);
    }

    void write(JointCommand const &command, hal::DrivePower const requested) noexcept {
        auto const rising =
            requested == hal::DrivePower::Enabled and lastRequested == hal::DrivePower::Disabled;
        auto const falling =
            requested == hal::DrivePower::Disabled and lastRequested == hal::DrivePower::Enabled;
        lastRequested = requested;
        for (std::size_t i = 0; i < actuators.size(); ++i) {
            auto &actuator = actuators[i];
            if (rising) {
                actuator.phase =
                    actuator.flags != 0 or actuator.reportedFaults != 0 ? Phase::Clearing : Phase::Enabling;
                actuator.attempts = 0;
            } else if (falling) {
                actuator.phase = Phase::Disabling;
                actuator.attempts = 0;
            }
            outgoing[i] = frameFor(actuator, command, i);
        }
        transport->send({outgoing.data(), actuators.size()});
    }

  private:
    static constexpr std::size_t kNoActuator = kMaxDof;

    struct Io final : hal::RealtimeIo {
        explicit Io(RobStrideDriverImpl *driver_) : driver{driver_} {}
        void read(RobotState &out) noexcept override { driver->read(out); }
        void write(JointCommand const &command, hal::DrivePower const requested) noexcept override {
            driver->write(command, requested);
        }

      private:
        RobStrideDriverImpl *driver;
    };

    CanFrame frameFor(Actuator &actuator, JointCommand const &command, std::size_t const joint) noexcept {
        auto const id = actuator.config.id;
        auto const host = config.hostId;
        switch (actuator.phase) {
        case Phase::Idle:
            return encodeReadParameter(id, host, parameter::kMechanicalPosition);
        case Phase::Disabling:
            if ((actuator.attempts > 0 and actuator.mode != MotorMode::Run) or
                actuator.attempts >= kDisableAttempts) {
                actuator.phase = Phase::Idle;
                return encodeReadParameter(id, host, parameter::kMechanicalPosition);
            }
            ++actuator.attempts;
            return encodeDisable(id, host, false);
        case Phase::Clearing:
            actuator.phase = Phase::Enabling;
            actuator.flags = 0;
            actuator.reportedFaults = 0;
            return encodeDisable(id, host, true);
        case Phase::Enabling:
            if (actuator.attempts > 0 and actuator.mode == MotorMode::Run) {
                actuator.phase = Phase::Running;
                return motion(actuator, command, joint);
            }
            if (actuator.attempts >= kEnableAttempts) {
                actuator.phase = Phase::Dropped;
                return encodeDisable(id, host, false);
            }
            ++actuator.attempts;
            return encodeEnable(id, host);
        case Phase::Running:
            if (actuator.mode != MotorMode::Run) {
                actuator.phase = Phase::Dropped;
                return encodeDisable(id, host, false);
            }
            return motion(actuator, command, joint);
        case Phase::Dropped:
            return encodeDisable(id, host, false);
        }
        return encodeReadParameter(id, host, parameter::kMechanicalPosition);
    }

    CanFrame motion(Actuator const &actuator, JointCommand const &command,
                    std::size_t const joint) const noexcept {
        auto const &[scale, offset] = actuator.config.transmission;
        auto const j = idx(joint);
        return encodeMotion(actuator.config.id,
                            MotionCommand{
                                .position = (command.position[j] - offset) / scale,
                                .velocity = command.velocity[j] / scale,
                                .stiffness = command.stiffness[j] * scale * scale,
                                .damping = command.damping[j] * scale * scale,
                                .torque = command.effort[j] * scale,
                            },
                            actuator.ranges);
    }

    void drain() noexcept {
        for (;;) {
            auto const count = transport->receive(incoming);
            for (std::size_t i = 0; i < count; ++i) {
                accept(incoming[i]);
            }
            if (count < incoming.size()) {
                return;
            }
        }
    }

    void discardPending() noexcept {
        while (transport->receive(incoming) == incoming.size()) {
        }
    }

    Actuator *replyingActuator(FrameHeader const &header, bool const toHost) noexcept {
        auto const index = motorIndex[header.data & 0xFFU];
        if (index == kNoActuator or (toHost and header.destination != config.hostId)) {
            foreignFrames.fetch_add(1, std::memory_order_relaxed);
            return nullptr;
        }
        return &actuators[index];
    }

    void accept(CanFrame const &frame) noexcept {
        auto const header = headerOf(frame);
        if (not header) {
            foreignFrames.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        switch (header->type) {
        case FrameType::Status:
        case FrameType::ActiveReport: {
            auto *const actuator = replyingActuator(*header, header->type == FrameType::Status);
            auto const status = actuator ? decodeStatus(frame, actuator->ranges) : std::nullopt;
            if (status) {
                actuator->mode = status->mode;
                actuator->flags = status->flags;
                actuator->position = status->position;
                actuator->velocity = status->velocity;
                actuator->torque = status->torque;
                actuator->temperature = status->temperature;
                actuator->replied = true;
                actuator->known = true;
            }
            return;
        }
        case FrameType::ReadParameter: {
            auto *const actuator = replyingActuator(*header, true);
            auto const value = actuator ? decodeParameter(frame) : std::nullopt;
            if (value and value->ok and value->index == parameter::kMechanicalPosition) {
                actuator->position = static_cast<double>(value->asFloat());
                actuator->velocity = 0.0;
                actuator->torque = 0.0;
                actuator->replied = true;
                actuator->known = true;
            }
            return;
        }
        case FrameType::FaultReport:
            if (auto *const actuator = replyingActuator(*header, true)) {
                if (auto const report = decodeFaultReport(frame)) {
                    actuator->reportedFaults = report->faults;
                }
            }
            return;
        default:
            return;
        }
    }

    // Outside the control loop: sends `frame` and waits for a reply that `matches`, handling the rest.
    std::optional<CanFrame> request(CanFrame const &frame,
                                    std::function<bool(CanFrame const &)> const &matches) {
        if (transport->send({&frame, 1}) != 1) {
            return std::nullopt;
        }
        auto const deadline = std::chrono::steady_clock::now() + kReplyTimeout;
        for (auto now = std::chrono::steady_clock::now(); now < deadline;
             now = std::chrono::steady_clock::now()) {
            if (not transport->waitReadable(deadline - now)) {
                continue;
            }
            auto const count = transport->receive(incoming);
            auto found = std::optional<CanFrame>{};
            for (std::size_t i = 0; i < count; ++i) {
                if (not found and matches(incoming[i])) {
                    found = incoming[i];
                } else {
                    accept(incoming[i]);
                }
            }
            if (found) {
                return found;
            }
        }
        return std::nullopt;
    }

    Expected<ParameterValue> readParameter(Actuator const &actuator, std::uint16_t const index) {
        auto const id = actuator.config.id;
        auto const reply =
            request(encodeReadParameter(id, config.hostId, index), [id, index](CanFrame const &frame) {
                auto const value = decodeParameter(frame);
                return value and value->motor == id and value->index == index;
            });
        auto const value = reply ? decodeParameter(*reply) : std::nullopt;
        if (not value or not value->ok) {
            return makeError(ErrorCode::Io, describe(actuator) + " did not return parameter " + hex(index));
        }
        return *value;
    }

    static std::string describe(Actuator const &actuator) {
        return "actuator " + hex(actuator.config.id) + " (" + actuator.config.jointName + ")";
    }

    DriverConfig config;
    std::unique_ptr<can::CanTransport> transport;
    hal::Timeline const *clock;
    Io realtimeIo;
    std::vector<Actuator> actuators;
    std::array<std::size_t, 256> motorIndex{};
    std::array<CanFrame, kMaxDof> outgoing{};
    std::array<CanFrame, kReceiveBatch> incoming{};
    hal::DrivePower lastRequested = hal::DrivePower::Disabled;
    std::atomic<std::uint64_t> cycles{0};
    std::atomic<std::uint64_t> missedReplies{0};
    std::atomic<std::uint64_t> foreignFrames{0};
};

} // namespace

std::unique_ptr<RobStrideDriver> makeRobStrideDriver(DriverConfig config,
                                                     std::unique_ptr<can::CanTransport> transport,
                                                     hal::Timeline const &clock) {
    return std::make_unique<RobStrideDriverImpl>(std::move(config), std::move(transport), clock);
}

} // namespace larm::drivers::robstride
