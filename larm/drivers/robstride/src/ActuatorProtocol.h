#pragma once

#include <larm/core/Error.h>
#include <larm/drivers/robstride/Codec.h>
#include <larm/drivers/robstride/DriverConfig.h>
#include <larm/hal/Backend.h>

#include <stackfull/coro/Coroutine.h>

#include <optional>
#include <string>
#include <utility>

namespace larm::drivers::robstride::detail {

enum class Phase : std::uint8_t { Connecting, Idle, Enabling, Running, Disabling, Dropped };

// What the motor last reported, in motor units. The driver's receiver writes it.
struct MotorState {
    MotorMode mode = MotorMode::Reset;
    std::uint8_t flags{};
    std::uint32_t reportedFaults{};
    double position{};
    double velocity{};
    double torque{};
    double temperature{};
    // A reply arrived since the last read().
    bool replied{};
    bool known{};
};

// What the control loop asks this cycle; the driver sets it before resuming the protocols.
struct CycleInput {
    hal::PowerRequest power;
    JointCommand const *command{};
    TimePoint now{};
};

struct ProtocolSettings {
    std::uint8_t host{};
    std::string interface;
    bool disableActiveReport{};
    // CAN timeout to write at set-up, in the motor's ticks.
    std::optional<std::uint32_t> canTimeoutTicks;
};

// The bus protocol of one actuator, written as straight-line code in a stackful coroutine.
//
// Set-up (connect()): the driver resumes the protocol whenever it is not waiting, sends the frame it
// left, and hands it the reply it waits for or tells it the wait timed out.
//
// Control loop: the driver resumes it once per cycle, after read() applied the replies; each resume
// leaves exactly one frame. Timeouts run on the control loop's timeline.
//
// The coroutine is created by start(), outside the control loop, and destroyed with the protocol;
// its body never returns once set-up succeeded.
struct ActuatorProtocol {
    ActuatorProtocol(ActuatorConfig config_, ProtocolSettings const &settings_, CycleInput const &input_);
    ActuatorProtocol(ActuatorProtocol const &) = delete;
    ActuatorProtocol &operator=(ActuatorProtocol const &) = delete;

    // Allocates the coroutine's stack and runs set-up to its first request.
    Expected<void> start();
    void resume();

    // Set-up progress.
    bool ready() const noexcept { return isReady; }
    bool failed() const noexcept { return failure.has_value(); }
    std::string const &failureMessage() const noexcept { return *failure; }
    bool answeredPing() const noexcept { return pinged; }
    bool waiting() const noexcept { return awaiting != nullptr and not answer and not timedOut; }
    // Offers a received frame to a waiting request; false if it is not the reply.
    bool offer(CanFrame const &frame) noexcept;
    void timeOut() noexcept { timedOut = true; }

    // The frame left by the last resume, if any.
    std::optional<CanFrame> takeFrame() noexcept { return std::exchange(outgoing, std::nullopt); }
    // A position read, for cycles that do not resume the protocol.
    CanFrame poll() const noexcept;

    ActuatorConfig const config;
    ModelRanges const ranges;
    MotorState motor;
    Phase phase = Phase::Connecting;
    // Whole turns added to the motor's angle, chosen at set-up (turnCorrection).
    double turnShift{};

  private:
    enum class Enabled : std::uint8_t { Yes, Withdrawn, TimedOut };

    // A type-erased reference to a reply predicate living on the coroutine's stack.
    struct Matcher {
        void const *context;
        bool (*matches)(void const *context, CanFrame const &frame);
    };

    void body();
    void setUp();
    void operate();
    Enabled enable();
    void disable();
    void stayDropped();

    void emit(CanFrame const &frame);
    template <class Matches> std::optional<CanFrame> request(CanFrame const &frame, Matches const &matches);
    std::optional<ParameterValue> readParameter(std::uint16_t index);
    void fail(std::string message);

    hal::DrivePower power() const noexcept { return input->power.power; }
    std::uint32_t rearm() const noexcept { return input->power.rearm; }
    TimePoint now() const noexcept { return input->now; }
    CanFrame motion() const noexcept;
    std::string describe() const;

    ProtocolSettings const settings;
    CycleInput const *input;
    stackfull::coro::Coroutine coroutine;
    std::optional<CanFrame> outgoing;
    Matcher const *awaiting{};
    std::optional<CanFrame> answer;
    bool timedOut{};
    bool isReady{};
    bool pinged{};
    std::optional<std::string> failure;
};

} // namespace larm::drivers::robstride::detail
