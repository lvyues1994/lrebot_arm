#pragma once

#include <larm/control/ControlCycle.h>
#include <larm/hal/Backend.h>

#include <atomic>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace larm::control {

struct RealtimeRunnerConfig {
    // SCHED_FIFO priority; normal scheduling when empty.
    std::optional<int> priority;
    // CPU to pin the thread to; unpinned when empty.
    std::optional<int> cpu;
};

// Runs tick() then advance() on its own thread from construction until destruction.
struct RealtimeRunner {
    RealtimeRunner(ControlCycle &cycle, hal::Timeline &timeline, RealtimeRunnerConfig const &config);
    ~RealtimeRunner();
    RealtimeRunner(RealtimeRunner const &) = delete;
    RealtimeRunner &operator=(RealtimeRunner const &) = delete;

    std::uint64_t cycles() const noexcept { return completed.load(std::memory_order_relaxed); }
    // Scheduling requests that could not be applied, such as SCHED_FIFO without permission.
    std::vector<std::string> const &warnings() const noexcept { return problems; }

  private:
    std::atomic<std::uint64_t> completed{0};
    std::vector<std::string> problems;
    std::jthread thread;
};

} // namespace larm::control
