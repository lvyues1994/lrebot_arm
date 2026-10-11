#include "ActuatorProtocol.h"

#include <larm/drivers/robstride/RobStrideDriver.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <vector>

namespace larm::drivers::robstride {
namespace {

using detail::ActuatorProtocol;

constexpr std::size_t kReceiveBatch = 64;
constexpr auto kReplyTimeout = std::chrono::milliseconds{100};
// How long set-up waits for replies before looking at timeouts again.
constexpr auto kSetUpPoll = std::chrono::milliseconds{1};
// The motor's CAN timeout counts 20000 ticks per second.
constexpr double kCanTimeoutTicksPerSecond = 20000.0;

detail::ProtocolSettings settingsOf(DriverConfig const &config) {
    auto settings = detail::ProtocolSettings{.host = config.hostId,
                                             .interface = config.interface,
                                             .disableActiveReport = config.disableActiveReport,
                                             .canTimeoutTicks = std::nullopt};
    if (config.canTimeout) {
        settings.canTimeoutTicks = static_cast<std::uint32_t>(
            std::lround(toSeconds(*config.canTimeout) * kCanTimeoutTicksPerSecond));
    }
    return settings;
}

struct RobStrideDriverImpl final : RobStrideDriver {
    RobStrideDriverImpl(DriverConfig config_, std::unique_ptr<can::CanTransport> transport_,
                        hal::Timeline const &clock_)
        : config{std::move(config_)}, transport{std::move(transport_)}, clock{&clock_}, realtimeIo{this} {
        motorIndex.fill(kNoActuator);
        auto const settings = settingsOf(config);
        for (std::size_t i = 0; i < config.actuators.size(); ++i) {
            protocols.push_back(std::make_unique<ActuatorProtocol>(config.actuators[i], settings, input));
            motorIndex[config.actuators[i].id] = i;
        }
    }

    hal::DriverCapabilities capabilities() const override {
        auto modes = std::bitset<4>{};
        modes.set(static_cast<std::size_t>(hal::CommandMode::Impedance));
        return hal::DriverCapabilities{.modes = modes, .minPeriod = cycleBusTime(config), .hasBrakes = false};
    }

    // Every actuator's set-up runs at once: one batch of requests per round, so a missing actuator
    // costs one reply timeout rather than one per request.
    Expected<void> connect() override {
        if (transport->busState() == can::BusState::Off) {
            return makeError(ErrorCode::Io, "the CAN controller on " + config.interface +
                                                " is bus-off; restart the interface ('sudo ip link set " +
                                                config.interface + " down' and up again)");
        }
        discardPending();
        auto const unacknowledged = transport->statistics().unacknowledged;
        input = detail::CycleInput{};
        for (auto &protocol : protocols) {
            if (auto started = protocol->start(); not started) {
                return tl::make_unexpected(started.error());
            }
        }
        runSetUp();

        auto failures = std::string{};
        for (auto const &protocol : protocols) {
            if (protocol->failed()) {
                failures += (failures.empty() ? "" : "; ") + protocol->failureMessage();
            }
        }
        if (failures.empty()) {
            return {};
        }
        auto const nobodyAnswered =
            std::none_of(protocols.begin(), protocols.end(),
                         [](auto const &protocol) { return protocol->answeredPing(); });
        if (nobodyAnswered and transport->statistics().unacknowledged > unacknowledged) {
            return makeError(ErrorCode::Io, "no node acknowledges frames on " + config.interface +
                                                ": are the motors powered and the bus terminated?");
        }
        return makeError(ErrorCode::Io, failures);
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

    // The coroutine library keeps per-thread state; it is created here rather than in the first cycle.
    void prepareThread() noexcept { static_cast<void>(stackfull::coro::Coroutine::isInsideCoroutine()); }

    void read(RobotState &out) noexcept {
        drain();
        auto const dof = protocols.size();
        out.cycle = cycles.fetch_add(1, std::memory_order_relaxed) + 1;
        out.stamp = clock->now();
        out.joints.position.resize(idx(dof));
        out.joints.velocity.resize(idx(dof));
        out.joints.effort.resize(idx(dof));
        auto fresh = true;
        auto missed = std::uint64_t{0};
        for (std::size_t i = 0; i < dof; ++i) {
            auto &protocol = *protocols[i];
            auto &motor = protocol.motor;
            auto const &[scale, offset] = protocol.config.transmission;
            out.joints.position[idx(i)] = scale * motor.position + offset;
            out.joints.velocity[idx(i)] = scale * motor.velocity;
            out.joints.effort[idx(i)] = motor.torque / scale;
            out.actuators[i] = ActuatorStatus{
                .enabled = motor.known and motor.mode == MotorMode::Run,
                .faultBits = motor.flags | (motor.reportedFaults << 8U),
                .temperature = static_cast<float>(motor.temperature),
            };
            if (not motor.replied) {
                fresh = false;
                ++missed;
            }
            motor.replied = false;
        }
        out.isFresh = fresh;
        missedReplies.fetch_add(missed, std::memory_order_relaxed);
    }

    // Each protocol leaves one frame per cycle; they go out as one batch.
    void write(JointCommand const &command, hal::PowerRequest const &power) noexcept {
        input = detail::CycleInput{.power = power, .command = &command, .now = clock->now()};
        for (std::size_t i = 0; i < protocols.size(); ++i) {
            auto &protocol = *protocols[i];
            if (protocol.ready()) {
                protocol.resume();
            }
            outgoing[i] = protocol.takeFrame().value_or(protocol.poll());
        }
        transport->send({outgoing.data(), protocols.size()});
    }

  private:
    static constexpr std::size_t kNoActuator = kMaxDof;

    struct Io final : hal::RealtimeIo {
        explicit Io(RobStrideDriverImpl *driver_) : driver{driver_} {}
        void prepareThread() noexcept override { driver->prepareThread(); }
        void read(RobotState &out) noexcept override { driver->read(out); }
        void write(JointCommand const &command, hal::PowerRequest const &power) noexcept override {
            driver->write(command, power);
        }

      private:
        RobStrideDriverImpl *driver;
    };

    // Sends what the protocols left, hands replies to the requests waiting for them, times out the
    // rest, and resumes every protocol no longer waiting, until each is ready or failed.
    void runSetUp() {
        using Clock = std::chrono::steady_clock;
        auto deadlines = std::vector<Clock::time_point>(protocols.size());
        auto batch = std::vector<CanFrame>{};
        for (;;) {
            batch.clear();
            auto const sent = Clock::now();
            for (std::size_t i = 0; i < protocols.size(); ++i) {
                if (auto const frame = protocols[i]->takeFrame()) {
                    batch.push_back(*frame);
                    deadlines[i] = sent + kReplyTimeout;
                }
            }
            transport->send(batch);
            auto const settling = [](auto const &protocol) {
                return not protocol->ready() and not protocol->failed();
            };
            if (std::none_of(protocols.begin(), protocols.end(), settling)) {
                return;
            }
            transport->waitReadable(kSetUpPoll);
            drain([this](CanFrame const &frame) {
                return std::any_of(protocols.begin(), protocols.end(),
                                   [&](auto const &protocol) { return protocol->offer(frame); });
            });
            auto const now = Clock::now();
            for (std::size_t i = 0; i < protocols.size(); ++i) {
                auto &protocol = *protocols[i];
                if (protocol.waiting() and now >= deadlines[i]) {
                    protocol.timeOut();
                }
                if (settling(protocols[i]) and not protocol.waiting()) {
                    protocol.resume();
                }
            }
        }
    }

    // Applies every received frame to the motor states; `claim` may take a frame first.
    template <class Claim> void drain(Claim const &claim) noexcept {
        for (;;) {
            auto const count = transport->receive(incoming);
            for (std::size_t i = 0; i < count; ++i) {
                if (not claim(incoming[i])) {
                    accept(incoming[i]);
                }
            }
            if (count < incoming.size()) {
                return;
            }
        }
    }

    void drain() noexcept {
        drain([](CanFrame const &) { return false; });
    }

    void discardPending() noexcept {
        while (transport->receive(incoming) == incoming.size()) {
        }
    }

    ActuatorProtocol *replyingProtocol(FrameHeader const &header, bool const toHost) noexcept {
        auto const index = motorIndex[header.data & 0xFFU];
        if (index == kNoActuator or (toHost and header.destination != config.hostId)) {
            foreignFrames.fetch_add(1, std::memory_order_relaxed);
            return nullptr;
        }
        return protocols[index].get();
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
            auto *const protocol = replyingProtocol(*header, header->type == FrameType::Status);
            auto const status = protocol ? decodeStatus(frame, protocol->ranges) : std::nullopt;
            if (status) {
                auto &motor = protocol->motor;
                motor.mode = status->mode;
                motor.flags = status->flags;
                motor.position = status->position;
                motor.velocity = status->velocity;
                motor.torque = status->torque;
                motor.temperature = status->temperature;
                motor.replied = true;
                motor.known = true;
            }
            return;
        }
        case FrameType::ReadParameter: {
            auto *const protocol = replyingProtocol(*header, true);
            auto const value = protocol ? decodeParameter(frame) : std::nullopt;
            if (value and value->ok and value->index == parameter::kMechanicalPosition) {
                auto &motor = protocol->motor;
                motor.position = static_cast<double>(value->asFloat());
                motor.velocity = 0.0;
                motor.torque = 0.0;
                motor.replied = true;
                motor.known = true;
            }
            return;
        }
        case FrameType::FaultReport:
            if (auto *const protocol = replyingProtocol(*header, true)) {
                if (auto const report = decodeFaultReport(frame)) {
                    protocol->motor.reportedFaults = report->faults;
                }
            }
            return;
        default:
            return;
        }
    }

    DriverConfig config;
    std::unique_ptr<can::CanTransport> transport;
    hal::Timeline const *clock;
    Io realtimeIo;
    // What the protocols see when resumed.
    detail::CycleInput input;
    std::vector<std::unique_ptr<ActuatorProtocol>> protocols;
    std::array<std::size_t, 256> motorIndex{};
    std::array<CanFrame, kMaxDof> outgoing{};
    std::array<CanFrame, kReceiveBatch> incoming{};
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
