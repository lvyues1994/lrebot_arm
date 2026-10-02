#pragma once

#include <larm/hal/Backend.h>

#include <memory>

namespace larm::hal {

struct MonotonicTimeline : Timeline {
    // Periods skipped because a cycle ran past its deadline.
    virtual std::uint64_t missedPeriods() const noexcept = 0;
};

// Sleeps to absolute CLOCK_MONOTONIC deadlines spaced `period` apart.
std::unique_ptr<MonotonicTimeline> makeMonotonicTimeline(Duration period);

} // namespace larm::hal
