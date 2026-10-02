#include <larm/hal/MonotonicTimeline.h>

#include <cerrno>
#include <ctime>

namespace larm::hal {
namespace {

Duration monotonicNow() noexcept {
    auto spec = timespec{};
    clock_gettime(CLOCK_MONOTONIC, &spec);
    return std::chrono::seconds{spec.tv_sec} + std::chrono::nanoseconds{spec.tv_nsec};
}

void sleepUntil(Duration const deadline) noexcept {
    auto const seconds = std::chrono::duration_cast<std::chrono::seconds>(deadline);
    auto const spec = timespec{
        .tv_sec = static_cast<std::time_t>(seconds.count()),
        .tv_nsec = static_cast<long>((deadline - seconds).count()),
    };
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &spec, nullptr) == EINTR) {
    }
}

struct MonotonicTimelineImpl final : MonotonicTimeline {
    explicit MonotonicTimelineImpl(Duration const period_)
        : period{period_}, epoch{monotonicNow()}, deadline{epoch + period_} {}

    TimePoint now() const noexcept override { return TimePoint{monotonicNow() - epoch}; }

    void advance() noexcept override {
        auto const current = monotonicNow();
        if (current > deadline) {
            auto const late = (current - deadline) / period;
            missed += static_cast<std::uint64_t>(late);
            deadline += late * period;
        }
        sleepUntil(deadline);
        deadline += period;
    }

    std::uint64_t missedPeriods() const noexcept override { return missed; }

  private:
    Duration period;
    Duration epoch;
    Duration deadline;
    std::uint64_t missed{};
};

} // namespace

std::unique_ptr<MonotonicTimeline> makeMonotonicTimeline(Duration const period) {
    return std::make_unique<MonotonicTimelineImpl>(period);
}

} // namespace larm::hal
