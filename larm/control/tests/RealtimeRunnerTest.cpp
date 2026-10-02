#include <larm/control/RealtimeRunner.h>
#include <larm/hal/MonotonicTimeline.h>

#include <gtest/gtest.h>

#include <atomic>
#include <thread>

namespace larm::control {
namespace {

struct CountingCycle final : ControlCycle {
    void tick() noexcept override { ticks.fetch_add(1); }
    std::atomic<int> ticks{0};
};

TEST(RealtimeRunner, TicksUntilDestroyed) {
    auto cycle = CountingCycle{};
    auto timeline = hal::makeMonotonicTimeline(std::chrono::milliseconds{1});
    {
        auto runner = RealtimeRunner{cycle, *timeline, {}};
        EXPECT_TRUE(runner.warnings().empty());
        std::this_thread::sleep_for(std::chrono::milliseconds{30});
        EXPECT_GT(runner.cycles(), 10u);
    }
    auto const after = cycle.ticks.load();
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
    EXPECT_EQ(cycle.ticks.load(), after);
}

TEST(RealtimeRunner, ReportsSchedulingItCannotApply) {
    auto cycle = CountingCycle{};
    auto timeline = hal::makeMonotonicTimeline(std::chrono::milliseconds{1});
    auto runner = RealtimeRunner{cycle, *timeline, {.priority = 1000}};
    ASSERT_EQ(runner.warnings().size(), 1u);
    EXPECT_NE(runner.warnings().front().find("SCHED_FIFO"), std::string::npos);
}

} // namespace
} // namespace larm::control
