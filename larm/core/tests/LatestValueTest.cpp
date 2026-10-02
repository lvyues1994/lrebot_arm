#include <larm/core/rt/LatestValue.h>

#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <thread>

namespace larm::rt {
namespace {

struct Pair {
    std::int64_t first{};
    std::int64_t second{};
};

TEST(LatestValue, ReaderSeesInitialUntilPublished) {
    auto value = LatestValue<int>{7};
    EXPECT_EQ(value.current(), 7);
    EXPECT_FALSE(value.refresh());
    value.publish(1);
    value.publish(2);
    EXPECT_TRUE(value.refresh());
    EXPECT_EQ(value.current(), 2);
    EXPECT_FALSE(value.refresh());
    EXPECT_EQ(value.current(), 2);
}

TEST(LatestValue, ReaderNeverSeesTornOrOlderValues) {
    constexpr std::int64_t kCount = 200'000;
    auto value = LatestValue<Pair>{};
    auto done = std::atomic<bool>{false};
    auto writer = std::thread{[&] {
        for (std::int64_t i = 1; i <= kCount; ++i) {
            value.publish(Pair{.first = i, .second = i});
        }
        done.store(true);
    }};
    auto last = std::int64_t{0};
    auto finished = false;
    while (not finished) {
        finished = done.load();
        value.refresh();
        auto const &current = value.current();
        ASSERT_EQ(current.first, current.second);
        ASSERT_GE(current.first, last);
        last = current.first;
    }
    writer.join();
    EXPECT_EQ(value.current().first, kCount);
}

} // namespace
} // namespace larm::rt
