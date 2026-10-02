#include <larm/drivers/robstride/Codec.h>

#include <algorithm>
#include <bit>
#include <cmath>
#include <numbers>

namespace larm::drivers::robstride {
namespace {

constexpr double kFullScale = 65535.0;
constexpr std::array<std::uint8_t, 3> kVersionReplySignature{0x00, 0xC4, 0x56};

// Linear map of [low, high] onto [0, 65535]; non-finite values encode as 0 in the value domain.
std::uint16_t toField(double const value, double const low, double const high) noexcept {
    auto const finite = std::isfinite(value) ? value : std::clamp(0.0, low, high);
    auto const clamped = std::clamp(finite, low, high);
    return static_cast<std::uint16_t>(std::lround((clamped - low) / (high - low) * kFullScale));
}

double fromField(std::uint16_t const field, double const low, double const high) noexcept {
    return low + (high - low) * static_cast<double>(field) / kFullScale;
}

std::uint16_t bigEndian(CanFrame const &frame, std::size_t const offset) noexcept {
    return static_cast<std::uint16_t>((frame.data[offset] << 8U) | frame.data[offset + 1]);
}

void putBigEndian(CanFrame &frame, std::size_t const offset, std::uint16_t const value) noexcept {
    frame.data[offset] = static_cast<std::uint8_t>(value >> 8U);
    frame.data[offset + 1] = static_cast<std::uint8_t>(value & 0xFFU);
}

CanFrame frame(FrameType const type, std::uint16_t const data, std::uint8_t const destination) noexcept {
    return CanFrame{.id = makeIdentifier(type, data, destination), .extended = true, .length = 8, .data = {}};
}

bool isFrame(CanFrame const &frame) noexcept { return frame.extended and frame.length == 8; }

} // namespace

ModelRanges const &rangesOf(MotorModel const model) noexcept {
    static constexpr auto kPosition = 4.0 * std::numbers::pi;
    static constexpr auto kRs00 = ModelRanges{
        .position = kPosition, .velocity = 33.0, .torque = 14.0, .stiffness = 500.0, .damping = 5.0};
    static constexpr auto kRs06 = ModelRanges{
        .position = kPosition, .velocity = 50.0, .torque = 36.0, .stiffness = 5000.0, .damping = 100.0};
    switch (model) {
    case MotorModel::Rs00:
        return kRs00;
    case MotorModel::Rs06:
        return kRs06;
    }
    return kRs00;
}

std::optional<MotorModel> modelFromString(std::string_view const name) noexcept {
    if (name == "rs-00") {
        return MotorModel::Rs00;
    }
    if (name == "rs-06") {
        return MotorModel::Rs06;
    }
    return std::nullopt;
}

std::string_view toString(MotorModel const model) noexcept {
    switch (model) {
    case MotorModel::Rs00:
        return "rs-00";
    case MotorModel::Rs06:
        return "rs-06";
    }
    return "unknown";
}

std::uint32_t makeIdentifier(FrameType const type, std::uint16_t const data,
                             std::uint8_t const destination) noexcept {
    return (static_cast<std::uint32_t>(type) << 24U) | (static_cast<std::uint32_t>(data) << 8U) | destination;
}

std::optional<FrameHeader> headerOf(CanFrame const &frame) noexcept {
    if (not frame.extended) {
        return std::nullopt;
    }
    return FrameHeader{
        .type = static_cast<FrameType>((frame.id >> 24U) & 0x1FU),
        .data = static_cast<std::uint16_t>((frame.id >> 8U) & 0xFFFFU),
        .destination = static_cast<std::uint8_t>(frame.id & 0xFFU),
    };
}

CanFrame encodeMotion(std::uint8_t const motor, MotionCommand const &command,
                      ModelRanges const &ranges) noexcept {
    auto out = frame(FrameType::Motion, toField(command.torque, -ranges.torque, ranges.torque), motor);
    putBigEndian(out, 0, toField(command.position, -ranges.position, ranges.position));
    putBigEndian(out, 2, toField(command.velocity, -ranges.velocity, ranges.velocity));
    putBigEndian(out, 4, toField(command.stiffness, 0.0, ranges.stiffness));
    putBigEndian(out, 6, toField(command.damping, 0.0, ranges.damping));
    return out;
}

CanFrame encodeEnable(std::uint8_t const motor, std::uint8_t const host) noexcept {
    return frame(FrameType::Enable, host, motor);
}

CanFrame encodeDisable(std::uint8_t const motor, std::uint8_t const host, bool const clearFault) noexcept {
    auto out = frame(FrameType::Disable, host, motor);
    out.data[0] = clearFault ? 1 : 0;
    return out;
}

CanFrame encodePing(std::uint8_t const motor, std::uint8_t const host) noexcept {
    return frame(FrameType::Ping, host, motor);
}

CanFrame encodeReadParameter(std::uint8_t const motor, std::uint8_t const host,
                             std::uint16_t const index) noexcept {
    auto out = frame(FrameType::ReadParameter, host, motor);
    out.data[0] = static_cast<std::uint8_t>(index & 0xFFU);
    out.data[1] = static_cast<std::uint8_t>(index >> 8U);
    return out;
}

CanFrame encodeWriteParameter(std::uint8_t const motor, std::uint8_t const host, std::uint16_t const index,
                              std::array<std::uint8_t, 4> const value) noexcept {
    auto out = encodeReadParameter(motor, host, index);
    out.id = makeIdentifier(FrameType::WriteParameter, host, motor);
    std::copy(value.begin(), value.end(), out.data.begin() + 4);
    return out;
}

CanFrame encodeActiveReport(std::uint8_t const motor, std::uint8_t const host, bool const enabled) noexcept {
    auto out = frame(FrameType::ActiveReport, host, motor);
    out.data = {1, 2, 3, 4, 5, 6, static_cast<std::uint8_t>(enabled ? 1 : 0), 0};
    return out;
}

std::optional<Status> decodeStatus(CanFrame const &frame, ModelRanges const &ranges) noexcept {
    auto const header = headerOf(frame);
    if (not header or not isFrame(frame) or
        (header->type != FrameType::Status and header->type != FrameType::ActiveReport) or
        std::equal(kVersionReplySignature.begin(), kVersionReplySignature.end(), frame.data.begin())) {
        return std::nullopt;
    }
    return Status{
        .motor = static_cast<std::uint8_t>(header->data & 0xFFU),
        .mode = static_cast<MotorMode>((header->data >> 14U) & 0x03U),
        .flags = static_cast<std::uint8_t>((header->data >> 8U) & 0x3FU),
        .position = fromField(bigEndian(frame, 0), -ranges.position, ranges.position),
        .velocity = fromField(bigEndian(frame, 2), -ranges.velocity, ranges.velocity),
        .torque = fromField(bigEndian(frame, 4), -ranges.torque, ranges.torque),
        .temperature = 0.1 * static_cast<double>(bigEndian(frame, 6)),
    };
}

CanFrame encodeStatus(std::uint8_t const host, Status const &status, ModelRanges const &ranges) noexcept {
    auto const data = static_cast<unsigned>(status.motor) | ((status.flags & 0x3FU) << 8U) |
                      ((static_cast<unsigned>(status.mode) & 0x03U) << 14U);
    auto out = frame(FrameType::Status, static_cast<std::uint16_t>(data), host);
    putBigEndian(out, 0, toField(status.position, -ranges.position, ranges.position));
    putBigEndian(out, 2, toField(status.velocity, -ranges.velocity, ranges.velocity));
    putBigEndian(out, 4, toField(status.torque, -ranges.torque, ranges.torque));
    putBigEndian(
        out, 6, static_cast<std::uint16_t>(std::lround(std::clamp(status.temperature * 10.0, 0.0, 65535.0))));
    return out;
}

std::optional<MotionCommand> decodeMotion(CanFrame const &frame, ModelRanges const &ranges) noexcept {
    auto const header = headerOf(frame);
    if (not header or not isFrame(frame) or header->type != FrameType::Motion) {
        return std::nullopt;
    }
    return MotionCommand{
        .position = fromField(bigEndian(frame, 0), -ranges.position, ranges.position),
        .velocity = fromField(bigEndian(frame, 2), -ranges.velocity, ranges.velocity),
        .stiffness = fromField(bigEndian(frame, 4), 0.0, ranges.stiffness),
        .damping = fromField(bigEndian(frame, 6), 0.0, ranges.damping),
        .torque = fromField(header->data, -ranges.torque, ranges.torque),
    };
}

float ParameterValue::asFloat() const noexcept { return std::bit_cast<float>(asUint32()); }

std::uint32_t ParameterValue::asUint32() const noexcept {
    return static_cast<std::uint32_t>(raw[0]) | (static_cast<std::uint32_t>(raw[1]) << 8U) |
           (static_cast<std::uint32_t>(raw[2]) << 16U) | (static_cast<std::uint32_t>(raw[3]) << 24U);
}

std::int8_t ParameterValue::asInt8() const noexcept { return static_cast<std::int8_t>(raw[0]); }

std::optional<ParameterValue> decodeParameter(CanFrame const &frame) noexcept {
    auto const header = headerOf(frame);
    if (not header or not isFrame(frame) or header->type != FrameType::ReadParameter) {
        return std::nullopt;
    }
    return ParameterValue{
        .motor = static_cast<std::uint8_t>(header->data & 0xFFU),
        .index = static_cast<std::uint16_t>(frame.data[0] | (frame.data[1] << 8U)),
        .ok = (header->data >> 8U) == 0,
        .raw = {frame.data[4], frame.data[5], frame.data[6], frame.data[7]},
    };
}

std::optional<FaultReport> decodeFaultReport(CanFrame const &frame) noexcept {
    auto const header = headerOf(frame);
    if (not header or not isFrame(frame) or header->type != FrameType::FaultReport) {
        return std::nullopt;
    }
    auto const word = [&](std::size_t const offset) {
        return static_cast<std::uint32_t>(frame.data[offset]) |
               (static_cast<std::uint32_t>(frame.data[offset + 1]) << 8U) |
               (static_cast<std::uint32_t>(frame.data[offset + 2]) << 16U) |
               (static_cast<std::uint32_t>(frame.data[offset + 3]) << 24U);
    };
    return FaultReport{
        .motor = static_cast<std::uint8_t>(header->data & 0xFFU), .faults = word(0), .warnings = word(4)};
}

std::optional<PingReply> decodePing(CanFrame const &frame) noexcept {
    auto const header = headerOf(frame);
    if (not header or not isFrame(frame) or header->type != FrameType::Ping) {
        return std::nullopt;
    }
    return PingReply{.motor = static_cast<std::uint8_t>(header->data & 0xFFU), .uid = frame.data};
}

std::array<std::uint8_t, 4> rawFloat(float const value) noexcept {
    return rawUint32(std::bit_cast<std::uint32_t>(value));
}

std::array<std::uint8_t, 4> rawUint32(std::uint32_t const value) noexcept {
    return {static_cast<std::uint8_t>(value & 0xFFU), static_cast<std::uint8_t>((value >> 8U) & 0xFFU),
            static_cast<std::uint8_t>((value >> 16U) & 0xFFU), static_cast<std::uint8_t>(value >> 24U)};
}

} // namespace larm::drivers::robstride
