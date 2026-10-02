#include <larm/drivers/robstride/Codec.h>

#include <gtest/gtest.h>

#include <numbers>

namespace larm::drivers::robstride {
namespace {

using Bytes = std::array<std::uint8_t, 8>;

CanFrame extended(std::uint32_t const id, Bytes const &data) {
    return CanFrame{.id = id, .extended = true, .length = 8, .data = data};
}

TEST(RobStrideCodec, IdentifierCarriesTypeDataAndDestination) {
    EXPECT_EQ(makeIdentifier(FrameType::ReadParameter, 0xABCD, 0x7F), 0x11ABCD7Fu);
    auto const header = headerOf(extended(0x11ABCD7F, {}));
    ASSERT_TRUE(header);
    EXPECT_EQ(header->type, FrameType::ReadParameter);
    EXPECT_EQ(header->data, 0xABCD);
    EXPECT_EQ(header->destination, 0x7F);
    EXPECT_FALSE(headerOf(CanFrame{.id = 0x123, .extended = false, .length = 8, .data = {}}));
}

TEST(RobStrideCodec, ControlFramesMatchTheManualLayout) {
    EXPECT_EQ(encodeEnable(0x03, 0xFD), extended(0x0300FD03, {}));
    EXPECT_EQ(encodeDisable(0x03, 0xFD, false), extended(0x0400FD03, {}));
    EXPECT_EQ(encodeDisable(0x03, 0xFD, true), extended(0x0400FD03, {1, 0, 0, 0, 0, 0, 0, 0}));
    EXPECT_EQ(encodePing(0x03, 0xFD), extended(0x0000FD03, {}));
    EXPECT_EQ(encodeReadParameter(0x03, 0xFD, 0x7005), extended(0x1100FD03, {0x05, 0x70, 0, 0, 0, 0, 0, 0}));
    EXPECT_EQ(encodeWriteParameter(0x03, 0xFD, parameter::kCanTimeout, rawUint32(2000)),
              extended(0x1200FD03, {0x28, 0x70, 0, 0, 0xD0, 0x07, 0, 0}));
    EXPECT_EQ(encodeActiveReport(0x03, 0xFD, false), extended(0x1800FD03, {1, 2, 3, 4, 5, 6, 0, 0}));
    EXPECT_EQ(encodeActiveReport(0x03, 0xFD, true), extended(0x1800FD03, {1, 2, 3, 4, 5, 6, 1, 0}));
}

TEST(RobStrideCodec, MotionFrameScalesEachFieldToItsRange) {
    auto const &rs06 = rangesOf(MotorModel::Rs06);
    // Zero maps to the middle of [0, 65535]; kp 150 of 5000 and kd 10 of 100 scale linearly.
    auto const frame = encodeMotion(
        0x02, {.position = 0.0, .velocity = 0.0, .stiffness = 150.0, .damping = 10.0, .torque = 0.0}, rs06);
    EXPECT_EQ(frame, extended(0x01800002, {0x80, 0x00, 0x80, 0x00, 0x07, 0xAE, 0x19, 0x9A}));

    auto const extremes = encodeMotion(
        0x02, {.position = 100.0, .velocity = -100.0, .stiffness = -1.0, .damping = 1e6, .torque = 36.0},
        rs06);
    EXPECT_EQ(extremes, extended(0x01FFFF02, {0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF}));
}

TEST(RobStrideCodec, MotionFrameRoundTripsWithinOneStep) {
    for (auto const model : {MotorModel::Rs00, MotorModel::Rs06}) {
        auto const &ranges = rangesOf(model);
        auto const command = MotionCommand{.position = 1.234,
                                           .velocity = -2.5,
                                           .stiffness = 0.3 * ranges.stiffness,
                                           .damping = 0.7 * ranges.damping,
                                           .torque = -0.4 * ranges.torque};
        auto const decoded = decodeMotion(encodeMotion(0x05, command, ranges), ranges);
        ASSERT_TRUE(decoded);
        EXPECT_NEAR(decoded->position, command.position, 2 * ranges.position / 65535.0);
        EXPECT_NEAR(decoded->velocity, command.velocity, 2 * ranges.velocity / 65535.0);
        EXPECT_NEAR(decoded->stiffness, command.stiffness, ranges.stiffness / 65535.0);
        EXPECT_NEAR(decoded->damping, command.damping, ranges.damping / 65535.0);
        EXPECT_NEAR(decoded->torque, command.torque, 2 * ranges.torque / 65535.0);
    }
}

TEST(RobStrideCodec, StatusCarriesModeFlagsAndMeasurements) {
    auto const &rs00 = rangesOf(MotorModel::Rs00);
    // Run mode, uncalibrated, encoder fault and overcurrent, from motor 0x34 to host 0xFD.
    auto const data =
        static_cast<std::uint16_t>((2U << 14U) | (1U << 13U) | (1U << 11U) | (1U << 9U) | 0x34U);
    auto const status = decodeStatus(extended(makeIdentifier(FrameType::Status, data, 0xFD),
                                              {0xFF, 0xFF, 0x00, 0x00, 0x80, 0x00, 0x01, 0x2C}),
                                     rs00);
    ASSERT_TRUE(status);
    EXPECT_EQ(status->motor, 0x34);
    EXPECT_EQ(status->mode, MotorMode::Run);
    EXPECT_EQ(status->flags,
              status_flag::kUncalibrated | status_flag::kEncoderFault | status_flag::kOvercurrent);
    EXPECT_NEAR(status->position, 4.0 * std::numbers::pi, 1e-12);
    EXPECT_DOUBLE_EQ(status->velocity, -33.0);
    EXPECT_NEAR(status->torque, 0.0, 2 * rs00.torque / 65535.0);
    EXPECT_NEAR(status->temperature, 30.0, 1e-12);

    auto const encoded = encodeStatus(0xFD, *status, rs00);
    EXPECT_EQ(encoded.id, makeIdentifier(FrameType::Status, data, 0xFD));
    EXPECT_EQ(decodeStatus(encoded, rs00)->position, status->position);
}

TEST(RobStrideCodec, ActiveReportsDecodeAsStatus) {
    auto const &rs06 = rangesOf(MotorModel::Rs06);
    auto const frame = extended(makeIdentifier(FrameType::ActiveReport, 0x0002, 0xFD),
                                {0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x01, 0x2C});
    auto const status = decodeStatus(frame, rs06);
    ASSERT_TRUE(status);
    EXPECT_EQ(status->motor, 0x02);
    EXPECT_EQ(status->mode, MotorMode::Reset);
}

TEST(RobStrideCodec, FirmwareVersionRepliesAreNotStatus) {
    // Captured from an RS03 (motor 21, host 0xFD) by the motorbridge project.
    auto const reply = extended(0x020015FD, {0x00, 0xC4, 0x56, 0x00, 0x03, 0x01, 0x29, 0x07});
    EXPECT_FALSE(decodeStatus(reply, rangesOf(MotorModel::Rs06)));
}

TEST(RobStrideCodec, ParameterRepliesCarryIndexValueAndOutcome) {
    auto const value = decodeParameter(extended(makeIdentifier(FrameType::ReadParameter, 0x0005, 0xFD),
                                                {0x19, 0x70, 0, 0, 0, 0, 0xA0, 0x3F}));
    ASSERT_TRUE(value);
    EXPECT_EQ(value->motor, 0x05);
    EXPECT_EQ(value->index, parameter::kMechanicalPosition);
    EXPECT_TRUE(value->ok);
    EXPECT_FLOAT_EQ(value->asFloat(), 1.25F);
    EXPECT_EQ(rawFloat(1.25F), (std::array<std::uint8_t, 4>{0, 0, 0xA0, 0x3F}));

    auto const failed = decodeParameter(
        extended(makeIdentifier(FrameType::ReadParameter, 0x0105, 0xFD), {0x05, 0x70, 0, 0, 0, 0, 0, 0}));
    ASSERT_TRUE(failed);
    EXPECT_FALSE(failed->ok);
    EXPECT_FALSE(decodeParameter(encodeEnable(0x05, 0xFD)));
}

TEST(RobStrideCodec, FaultReportsAndPingReplies) {
    auto const report = decodeFaultReport(extended(makeIdentifier(FrameType::FaultReport, 0x0007, 0xFD),
                                                   {0x10, 0x40, 0x01, 0x00, 0x01, 0, 0, 0}));
    ASSERT_TRUE(report);
    EXPECT_EQ(report->motor, 0x07);
    EXPECT_EQ(report->faults, (1U << 16U) | (1U << 14U) | (1U << 4U));
    EXPECT_EQ(report->warnings, 1U);

    auto const ping =
        decodePing(extended(makeIdentifier(FrameType::Ping, 0x0003, 0xFE), {1, 2, 3, 4, 5, 6, 7, 8}));
    ASSERT_TRUE(ping);
    EXPECT_EQ(ping->motor, 0x03);
    EXPECT_EQ(ping->uid, (Bytes{1, 2, 3, 4, 5, 6, 7, 8}));
}

TEST(RobStrideCodec, ModelNames) {
    EXPECT_EQ(modelFromString("rs-06"), MotorModel::Rs06);
    EXPECT_EQ(modelFromString("rs-00"), MotorModel::Rs00);
    EXPECT_FALSE(modelFromString("rs-99"));
    EXPECT_EQ(toString(MotorModel::Rs06), "rs-06");
}

} // namespace
} // namespace larm::drivers::robstride
