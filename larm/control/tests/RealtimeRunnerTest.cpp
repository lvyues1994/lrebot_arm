#include <larm/control/RealtimeRunner.h>
#include <larm/hal/MonotonicTimeline.h>

#include <gtest/gtest.h>

#include <atomic>
#include <thread>

namespace larm::control {
namespace {

struct CountingCycle final : ControlCycle {
    void prepareThread() noexcept override {
        preparedOn.store(std::this_thread::get_id());
        prepared.store(ticks.load() == 0);
    }
    void tick() noexcept override {
        tickedOn.store(std::this_thread::get_id());
        ticks.fetch_add(1);
    }
    std::atomic<int> ticks{0};
    std::atomic<bool> prepared{false};
    std::atomic<std::thread::id> preparedOn;
    std::atomic<std::thread::id> tickedOn;
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
    EXPECT_TRUE(cycle.prepared.load());
    EXPECT_EQ(cycle.preparedOn.load(), cycle.tickedOn.load());
    EXPECT_NE(cycle.tickedOn.load(), std::this_thread::get_id());
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
