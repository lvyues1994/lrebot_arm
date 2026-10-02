#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <optional>
#include <type_traits>
#include <utility>

namespace larm::rt {

// Fixed-capacity single-producer single-consumer queue. Never allocates; both ends are wait-free.
// Elements leave through tryPop() by move, so a move-only payload is destroyed on the consumer side.
template <class T, std::size_t Capacity> struct SpscRing {
    static_assert(Capacity >= 2 and (Capacity & (Capacity - 1)) == 0, "capacity must be a power of two");
    static_assert(std::is_nothrow_move_constructible_v<T>, "elements must be nothrow movable");

    // Producer only. When the ring is full, returns false and leaves `value` untouched.
    bool tryPush(T &&value) noexcept {
        auto const head = writeIndex.load(std::memory_order_relaxed);
        if (head - readIndex.load(std::memory_order_acquire) == Capacity) {
            return false;
        }
        slots[head & kMask].emplace(std::move(value));
        writeIndex.store(head + 1, std::memory_order_release);
        return true;
    }

    // Consumer only.
    std::optional<T> tryPop() noexcept {
        auto const tail = readIndex.load(std::memory_order_relaxed);
        if (tail == writeIndex.load(std::memory_order_acquire)) {
            return std::nullopt;
        }
        auto &slot = slots[tail & kMask];
        auto value = std::optional<T>{std::move(*slot)};
        slot.reset();
        readIndex.store(tail + 1, std::memory_order_release);
        return value;
    }

    // A snapshot; another thread may change it immediately.
    std::size_t sizeApprox() const noexcept {
        return writeIndex.load(std::memory_order_acquire) - readIndex.load(std::memory_order_acquire);
    }

  private:
    static constexpr std::size_t kMask = Capacity - 1;

    alignas(64) std::atomic<std::size_t> writeIndex{0};
    alignas(64) std::atomic<std::size_t> readIndex{0};
    alignas(64) std::array<std::optional<T>, Capacity> slots{};
};

} // namespace larm::rt
