#pragma once

#include <array>
#include <atomic>
#include <cstdint>

namespace larm::rt {

// Triple buffer between one writer and one reader. The writer never waits; the reader always
// sees a complete value, the newest one published before its last refresh().
template <class T> struct LatestValue {
    explicit LatestValue(T const &initial = T{}) : buffers{initial, initial, initial} {}

    // Writer only.
    void publish(T const &value) {
        buffers[back] = value;
        auto const previous =
            middle.exchange(static_cast<std::uint8_t>(back | kFresh), std::memory_order_acq_rel);
        back = static_cast<std::uint8_t>(previous & kIndexMask);
    }

    // Reader only. Adopts the newest published value; returns false when nothing new arrived.
    bool refresh() noexcept {
        if ((middle.load(std::memory_order_relaxed) & kFresh) == 0) {
            return false;
        }
        auto const previous = middle.exchange(front, std::memory_order_acq_rel);
        front = static_cast<std::uint8_t>(previous & kIndexMask);
        return true;
    }

    // Reader only.
    T const &current() const noexcept { return buffers[front]; }

  private:
    static constexpr std::uint8_t kIndexMask = 0x3;
    static constexpr std::uint8_t kFresh = 0x4;

    std::array<T, 3> buffers;
    alignas(64) std::atomic<std::uint8_t> middle{2};
    alignas(64) std::uint8_t back = 1;
    alignas(64) std::uint8_t front = 0;
};

} // namespace larm::rt
