#include <larm/core/rt/SpscRing.h>

#include <gtest/gtest.h>

#include <memory>
#include <thread>

namespace larm::rt {
namespace {

TEST(SpscRing, KeepsFifoOrderAndReportsFull) {
    auto ring = SpscRing<int, 4>{};
    for (int i = 0; i < 4; ++i) {
        EXPECT_TRUE(ring.tryPush(int{i}));
    }
    auto rejected = 99;
    EXPECT_FALSE(ring.tryPush(std::move(rejected)));
    EXPECT_EQ(ring.sizeApprox(), 4u);
    for (int i = 0; i < 4; ++i) {
        EXPECT_EQ(ring.tryPop(), i);
    }
    EXPECT_FALSE(ring.tryPop());
}

TEST(SpscRing, LeavesValueUntouchedWhenFull) {
    auto ring = SpscRing<std::unique_ptr<int>, 2>{};
    EXPECT_TRUE(ring.tryPush(std::make_unique<int>(1)));
    EXPECT_TRUE(ring.tryPush(std::make_unique<int>(2)));
    auto extra = std::make_unique<int>(3);
    EXPECT_FALSE(ring.tryPush(std::move(extra)));
    ASSERT_NE(extra, nullptr);
    EXPECT_EQ(*extra, 3);
    EXPECT_EQ(**ring.tryPop(), 1);
}

TEST(SpscRing, TransfersSequenceAcrossThreads) {
    constexpr auto kCount = 200'000;
    auto ring = SpscRing<int, 64>{};
    auto producer = std::thread{[&] {
        for (int i = 0; i < kCount; ++i) {
            while (not ring.tryPush(int{i})) {
                std::this_thread::yield();
            }
        }
    }};
    auto expected = 0;
    while (expected < kCount) {
        if (auto const value = ring.tryPop()) {
            ASSERT_EQ(*value, expected);
            ++expected;
        }
    }
    producer.join();
    EXPECT_FALSE(ring.tryPop());
}

} // namespace
} // namespace larm::rt
