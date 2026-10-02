#pragma once

#include <chrono>

namespace larm {

// Time as seen by the control loop. The epoch belongs to the timeline that drives the loop:
// start of the monotonic clock on hardware, simulation start in simulation.
struct TimelineClock {
    using duration = std::chrono::nanoseconds;
    using rep = duration::rep;
    using period = duration::period;
    using time_point = std::chrono::time_point<TimelineClock>;
    static constexpr bool is_steady = true;
};

using Duration = std::chrono::nanoseconds;
using TimePoint = TimelineClock::time_point;

inline double toSeconds(Duration const duration) noexcept {
    return std::chrono::duration<double>(duration).count();
}

inline Duration fromSeconds(double const seconds) noexcept {
    return std::chrono::round<Duration>(std::chrono::duration<double>(seconds));
}

} // namespace larm
