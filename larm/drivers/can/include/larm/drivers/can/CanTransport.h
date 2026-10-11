#pragma once

#include <larm/core/Error.h>
#include <larm/core/Time.h>

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>

namespace larm::drivers::can {

struct CanFrame {
    // 11- or 29-bit identifier, without flag bits.
    std::uint32_t id{};
    bool extended{};
    std::uint8_t length{};
    std::array<std::uint8_t, 8> data{};

    friend bool operator==(CanFrame const &, CanFrame const &) = default;
};

// The controller's standing on the bus, from its error counters.
enum class BusState : std::uint8_t {
    Active,
    // An error counter passed the warning level.
    Warning,
    // An error counter passed the passive level: the controller no longer signals errors actively.
    Passive,
    // The controller left the bus; it returns when the interface restarts.
    Off,
};

std::string_view toString(BusState state) noexcept;

// What an error frame says about the bus.
struct BusEvent {
    // The controller's new state, when the frame reports one.
    std::optional<BusState> state;
    // No node acknowledged a frame: nothing else is listening on the bus.
    bool unacknowledged{};
};

// Decodes a SocketCAN error frame from its identifier (with the error flag) and payload.
BusEvent decodeErrorFrame(std::uint32_t id, std::array<std::uint8_t, 8> const &data) noexcept;

struct TransportStatistics {
    std::uint64_t sent{};
    std::uint64_t received{};
    // Frames the interface did not accept.
    std::uint64_t sendFailures{};
    // Of those, the ones refused because the transmit queue was full.
    std::uint64_t queueFull{};
    std::uint64_t errorFrames{};
    // Error frames reporting that no node acknowledged a frame.
    std::uint64_t unacknowledged{};
    BusState busState = BusState::Active;
    // The last send failed because the interface is down.
    bool interfaceDown{};
};

// A CAN bus endpoint. send(), receive() and busState() never block and never allocate, so the control
// loop may call them; any thread may read statistics().
struct CanTransport {
    virtual ~CanTransport() = default;
    // Queues frames in order until the interface refuses one; returns how many were queued.
    virtual std::size_t send(std::span<CanFrame const> frames) noexcept = 0;
    // Takes up to frames.size() received data frames; error frames only update the bus state and the
    // statistics.
    virtual std::size_t receive(std::span<CanFrame> frames) noexcept = 0;
    // Outside the control loop: waits until a frame can be received or the timeout passes.
    virtual bool waitReadable(Duration timeout) noexcept = 0;
    virtual BusState busState() const noexcept = 0;
    virtual TransportStatistics statistics() const noexcept = 0;
};

struct SocketCanOptions {
    // Frames sent back to back, e.g. one per actuator each cycle; the interface's transmit queue must
    // hold them. Interfaces without a queue (length 0, such as vcan) always pass.
    std::size_t burst = 1;
};

// A raw SocketCAN socket on a configured, up interface such as "can0" or "vcan0".
Expected<std::unique_ptr<CanTransport>> openSocketCan(std::string const &interface,
                                                      SocketCanOptions const &options = {});

} // namespace larm::drivers::can
