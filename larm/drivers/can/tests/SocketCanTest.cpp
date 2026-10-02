#include <larm/drivers/can/CanTransport.h>

#include <gtest/gtest.h>

namespace larm::drivers::can {
namespace {

constexpr char const *kVirtualBus = "vcan0";

TEST(SocketCan, RejectsMissingInterfaces) {
    auto const missing = openSocketCan("larm_none0");
    ASSERT_FALSE(missing);
    EXPECT_NE(missing.error().message.find("larm_none0"), std::string::npos);
    EXPECT_FALSE(openSocketCan(""));
}

TEST(SocketCan, ExchangesFramesOnAVirtualBus) {
    auto sender = openSocketCan(kVirtualBus);
    if (not sender) {
        GTEST_SKIP() << "needs " << kVirtualBus
                     << " (sudo ip link add vcan0 type vcan && sudo ip link set vcan0 up): "
                     << sender.error().message;
    }
    auto receiver = openSocketCan(kVirtualBus);
    ASSERT_TRUE(receiver) << receiver.error().message;

    auto empty = std::array<CanFrame, 4>{};
    EXPECT_EQ((*receiver)->receive(empty), 0u);

    auto const frames = std::array<CanFrame, 3>{
        CanFrame{.id = 0x0300FD03, .extended = true, .length = 8, .data = {1, 2, 3, 4, 5, 6, 7, 8}},
        CanFrame{.id = 0x123, .extended = false, .length = 2, .data = {9, 10}},
        CanFrame{.id = 0x1FFFFFFF, .extended = true, .length = 0, .data = {}},
    };
    EXPECT_EQ((*sender)->send(frames), frames.size());
    ASSERT_TRUE((*receiver)->waitReadable(std::chrono::milliseconds{200}));
    auto received = std::array<CanFrame, 8>{};
    auto count = std::size_t{0};
    for (int attempt = 0; attempt < 10 and count < frames.size(); ++attempt) {
        count += (*receiver)->receive(std::span{received}.subspan(count));
        (*receiver)->waitReadable(std::chrono::milliseconds{20});
    }
    ASSERT_EQ(count, frames.size());
    for (std::size_t i = 0; i < frames.size(); ++i) {
        EXPECT_EQ(received[i], frames[i]) << i;
    }
    EXPECT_EQ((*sender)->statistics().sent, frames.size());
    EXPECT_EQ((*receiver)->statistics().received, frames.size());
    EXPECT_FALSE((*receiver)->statistics().busOff);
}

} // namespace
} // namespace larm::drivers::can
