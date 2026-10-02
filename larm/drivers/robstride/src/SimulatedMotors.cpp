#include <larm/drivers/robstride/SimulatedMotors.h>

#include <algorithm>
#include <array>
#include <mutex>

namespace larm::drivers::robstride {
namespace {

constexpr int kSubsteps = 16;
constexpr std::uint16_t kZeroState = 0x7029;

struct Motor {
    std::uint8_t id{};
    ModelRanges ranges;
    MotorMode mode = MotorMode::Reset;
    std::uint8_t flags{};
    std::uint32_t faults{};
    std::int8_t runMode = kRunModeMotion;
    bool silent{};
    MotionCommand command;
    double position{};
    double velocity{};
    double torque{};
};

// Replies waiting to be received; full means the oldest unread replies are kept and new ones lost.
struct FrameQueue {
    void push(CanFrame const &frame) noexcept {
        if (count == frames.size()) {
            return;
        }
        frames[(head + count) % frames.size()] = frame;
        ++count;
    }

    std::size_t take(std::span<CanFrame> const out) noexcept {
        auto const taken = std::min(out.size(), count);
        for (std::size_t i = 0; i < taken; ++i) {
            out[i] = frames[head];
            head = (head + 1) % frames.size();
        }
        count -= taken;
        return taken;
    }

    bool empty() const noexcept { return count == 0; }

  private:
    std::array<CanFrame, 512> frames{};
    std::size_t head{};
    std::size_t count{};
};

struct SimulatedMotorsImpl;

struct SimulatedBus final : can::CanTransport {
    explicit SimulatedBus(SimulatedMotorsImpl *motors_) : motors{motors_} {}

    std::size_t send(std::span<CanFrame const> frames) noexcept override;
    std::size_t receive(std::span<CanFrame> frames) noexcept override;
    bool waitReadable(Duration timeout) noexcept override;

    can::TransportStatistics statistics() const noexcept override;

  private:
    SimulatedMotorsImpl *motors;
};

struct SimulatedMotorsImpl final : SimulatedMotors {
    SimulatedMotorsImpl(DriverConfig const &config, SimulatedMotorsOptions options_)
        : host{config.hostId}, options{std::move(options_)} {
        for (std::size_t i = 0; i < config.actuators.size(); ++i) {
            auto const &actuator = config.actuators[i];
            motors.push_back(Motor{
                .id = actuator.id,
                .ranges = rangesOf(actuator.model),
                .position = i < options.position.size() ? options.position[i] : 0.0,
            });
        }
        positions.resize(motors.size());
        load.resize(motors.size());
    }

    std::unique_ptr<can::CanTransport> transport() override { return std::make_unique<SimulatedBus>(this); }

    double position(std::size_t const actuator) const override {
        auto const guard = std::lock_guard{lock};
        return motors.at(actuator).position;
    }

    bool enabled(std::size_t const actuator) const override {
        auto const guard = std::lock_guard{lock};
        return motors.at(actuator).mode == MotorMode::Run;
    }

    void injectFault(std::size_t const actuator, std::uint8_t const statusFlags,
                     std::uint32_t const faults) override {
        auto const guard = std::lock_guard{lock};
        auto &motor = motors.at(actuator);
        motor.mode = MotorMode::Reset;
        motor.flags = statusFlags;
        motor.faults = faults;
        auto report = CanFrame{.id = makeIdentifier(FrameType::FaultReport, motor.id, host),
                               .extended = true,
                               .length = 8,
                               .data = {}};
        auto const raw = rawUint32(faults);
        std::copy(raw.begin(), raw.end(), report.data.begin());
        pending.push(report);
    }

    void setSilent(std::size_t const actuator, bool const silent) override {
        auto const guard = std::lock_guard{lock};
        motors.at(actuator).silent = silent;
    }

    void setRunMode(std::size_t const actuator, std::int8_t const mode) override {
        auto const guard = std::lock_guard{lock};
        motors.at(actuator).runMode = mode;
    }

    // Frames from the driver: commands act, then the motors move one step if any motion frame arrived.
    void deliver(std::span<CanFrame const> const frames) noexcept {
        auto const guard = std::lock_guard{lock};
        for (auto const &frame : frames) {
            handle(frame);
        }
        step();
        counters.sent += frames.size();
    }

    std::size_t take(std::span<CanFrame> const out) noexcept {
        auto const guard = std::lock_guard{lock};
        auto const taken = pending.take(out);
        counters.received += taken;
        return taken;
    }

    can::TransportStatistics busStatistics() const noexcept {
        auto const guard = std::lock_guard{lock};
        return counters;
    }

    bool hasPending() const noexcept {
        auto const guard = std::lock_guard{lock};
        return not pending.empty();
    }

  private:
    void handle(CanFrame const &frame) {
        auto const header = headerOf(frame);
        if (not header) {
            return;
        }
        auto const found = std::find_if(motors.begin(), motors.end(),
                                        [&](Motor const &motor) { return motor.id == header->destination; });
        if (found == motors.end() or found->silent) {
            return;
        }
        auto &motor = *found;
        switch (header->type) {
        case FrameType::Ping:
            pending.push(CanFrame{.id = makeIdentifier(FrameType::Ping, motor.id, 0xFE),
                                  .extended = true,
                                  .length = 8,
                                  .data = {1, 2, 3, 4, 5, 6, 7, motor.id}});
            return;
        case FrameType::Enable:
            if (motor.flags == 0 and motor.faults == 0) {
                motor.mode = MotorMode::Run;
                motor.command = MotionCommand{.position = motor.position};
            }
            return reply(motor);
        case FrameType::Disable:
            motor.mode = MotorMode::Reset;
            if (frame.data[0] == 1) {
                motor.flags = 0;
                motor.faults = 0;
            }
            return reply(motor);
        case FrameType::Motion:
            if (auto const command = decodeMotion(frame, motor.ranges);
                command and motor.mode == MotorMode::Run) {
                motor.command = *command;
                moved = true;
            }
            return reply(motor);
        case FrameType::ReadParameter:
            return replyParameter(motor, frame);
        case FrameType::WriteParameter:
            return reply(motor);
        default:
            return;
        }
    }

    void step() {
        if (not moved) {
            return;
        }
        moved = false;
        auto const dt = toSeconds(options.step) / kSubsteps;
        for (int substep = 0; substep < kSubsteps; ++substep) {
            std::transform(motors.begin(), motors.end(), positions.begin(),
                           [](Motor const &motor) { return motor.position; });
            std::fill(load.begin(), load.end(), 0.0);
            if (options.load) {
                options.load(positions, load);
            }
            for (std::size_t i = 0; i < motors.size(); ++i) {
                auto &motor = motors[i];
                if (motor.mode != MotorMode::Run) {
                    motor.velocity = 0.0;
                    motor.torque = 0.0;
                    continue;
                }
                auto const &command = motor.command;
                motor.torque =
                    std::clamp(command.stiffness * (command.position - motor.position) +
                                   command.damping * (command.velocity - motor.velocity) + command.torque,
                               -motor.ranges.torque, motor.ranges.torque);
                auto const acceleration =
                    (motor.torque + load[i] - options.viscousFriction * motor.velocity) / options.inertia;
                motor.velocity += acceleration * dt;
                motor.position += motor.velocity * dt;
            }
        }
    }

    void reply(Motor const &motor) {
        pending.push(encodeStatus(host,
                                  Status{.motor = motor.id,
                                         .mode = motor.mode,
                                         .flags = motor.flags,
                                         .position = motor.position,
                                         .velocity = motor.velocity,
                                         .torque = motor.torque,
                                         .temperature = 30.0},
                                  motor.ranges));
    }

    void replyParameter(Motor const &motor, CanFrame const &request) {
        auto const index = static_cast<std::uint16_t>(request.data[0] | (request.data[1] << 8U));
        auto value = std::optional<std::array<std::uint8_t, 4>>{};
        switch (index) {
        case parameter::kRunMode:
            value = std::array<std::uint8_t, 4>{static_cast<std::uint8_t>(motor.runMode), 0, 0, 0};
            break;
        case kZeroState:
            value = std::array<std::uint8_t, 4>{1, 0, 0, 0};
            break;
        case parameter::kMechanicalPosition:
            value = rawFloat(static_cast<float>(motor.position));
            break;
        case parameter::kMechanicalVelocity:
            value = rawFloat(static_cast<float>(motor.velocity));
            break;
        case parameter::kBusVoltage:
            value = rawFloat(48.0F);
            break;
        default:
            break;
        }
        auto out = request;
        out.id = makeIdentifier(FrameType::ReadParameter,
                                static_cast<std::uint16_t>(motor.id | (value ? 0U : 1U << 8U)), host);
        std::fill(out.data.begin() + 2, out.data.end(), 0);
        if (value) {
            std::copy(value->begin(), value->end(), out.data.begin() + 4);
        }
        pending.push(out);
    }

    std::uint8_t host;
    SimulatedMotorsOptions options;
    std::vector<Motor> motors;
    std::vector<double> positions;
    std::vector<double> load;
    FrameQueue pending;
    can::TransportStatistics counters;
    bool moved{};
    // The driver's thread and the caller's thread meet here.
    mutable std::mutex lock;
};

std::size_t SimulatedBus::send(std::span<CanFrame const> const frames) noexcept {
    motors->deliver(frames);
    return frames.size();
}

std::size_t SimulatedBus::receive(std::span<CanFrame> const frames) noexcept { return motors->take(frames); }

can::TransportStatistics SimulatedBus::statistics() const noexcept { return motors->busStatistics(); }

bool SimulatedBus::waitReadable(Duration) noexcept { return motors->hasPending(); }

} // namespace

std::unique_ptr<SimulatedMotors> makeSimulatedMotors(DriverConfig const &config,
                                                     SimulatedMotorsOptions options) {
    return std::make_unique<SimulatedMotorsImpl>(config, std::move(options));
}

std::function<void(std::span<double const>, std::span<double>)>
jointSpaceLoad(DriverConfig const &config,
               std::function<void(JointVector const &, JointVector &)> jointTorque) {
    auto transmissions = std::vector<Transmission>{};
    for (auto const &actuator : config.actuators) {
        transmissions.push_back(actuator.transmission);
    }
    auto const dof = transmissions.size();
    return [transmissions = std::move(transmissions), jointTorque = std::move(jointTorque),
            position = zeroJointVector(dof), torque = zeroJointVector(dof)](
               std::span<double const> const motor, std::span<double> const out) mutable {
        for (std::size_t i = 0; i < motor.size(); ++i) {
            position[idx(i)] = transmissions[i].scale * motor[i] + transmissions[i].offset;
        }
        jointTorque(position, torque);
        for (std::size_t i = 0; i < out.size(); ++i) {
            out[i] = torque[idx(i)] * transmissions[i].scale;
        }
    };
}

} // namespace larm::drivers::robstride
