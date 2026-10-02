#include <larm/hal/MonotonicTimeline.h>

#include <gtest/gtest.h>

#include <thread>

namespace larm::hal {
namespace {

TEST(MonotonicTimeline, AdvancesInWholePeriods) {
    auto const period = std::chrono::milliseconds{2};
    auto timeline = makeMonotonicTimeline(period);
    auto const start = timeline->now();
    for (int i = 0; i < 10; ++i) {
        timeline->advance();
    }
    auto const elapsed = timeline->now() - start;
    EXPECT_GE(elapsed, 9 * period);
    EXPECT_EQ(timeline->missedPeriods(), 0u);
}

TEST(MonotonicTimeline, SkipsPeriodsMissedByASlowCycle) {
    auto const period = std::chrono::milliseconds{1};
    auto timeline = makeMonotonicTimeline(period);
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
    timeline->advance();
    EXPECT_GE(timeline->missedPeriods(), 5u);
    auto const before = timeline->now();
    timeline->advance();
    EXPECT_LE(timeline->now() - before, std::chrono::milliseconds{50});
}

} // namespace
} // namespace larm::hal
