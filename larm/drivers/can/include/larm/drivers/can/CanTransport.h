#pragma once

#include <larm/core/Error.h>
#include <larm/core/Time.h>

#include <array>
#include <cstdint>
#include <memory>
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

struct TransportStatistics {
    std::uint64_t sent{};
    std::uint64_t received{};
    // Frames the interface did not accept, e.g. a full transmit queue.
    std::uint64_t sendFailures{};
    std::uint64_t errorFrames{};
    bool busOff{};
};

// A CAN bus endpoint. send() and receive() never block and never allocate, so the control loop may
// call them; any thread may read statistics().
struct CanTransport {
    virtual ~CanTransport() = default;
    // Queues frames in order until the interface refuses one; returns how many were queued.
    virtual std::size_t send(std::span<CanFrame const> frames) noexcept = 0;
    // Takes up to frames.size() received data frames; error frames only update the statistics.
    virtual std::size_t receive(std::span<CanFrame> frames) noexcept = 0;
    // Outside the control loop: waits until a frame can be received or the timeout passes.
    virtual bool waitReadable(Duration timeout) noexcept = 0;
    virtual TransportStatistics statistics() const noexcept = 0;
};

// A raw SocketCAN socket on a configured, up interface such as "can0" or "vcan0".
Expected<std::unique_ptr<CanTransport>> openSocketCan(std::string const &interface);

} // namespace larm::drivers::can
