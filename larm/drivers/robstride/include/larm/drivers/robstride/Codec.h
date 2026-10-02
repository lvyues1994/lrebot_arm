#pragma once

#include <larm/drivers/can/CanTransport.h>

#include <array>
#include <cstdint>
#include <optional>
#include <string_view>

// The RobStride private protocol on 29-bit identifiers:
//   bits 24..28  frame type
//   bits  8..23  data field (host ID, motor ID with status bits, or the feed-forward torque)
//   bits  0..7   destination (the motor for commands, the host for replies)
// Multi-byte payload values are big-endian for motion and status, little-endian for parameters.
namespace larm::drivers::robstride {

using can::CanFrame;

enum class MotorModel : std::uint8_t { Rs00, Rs06 };

// Full scale of each 16-bit motion field; the encoding maps [-max, max] (or [0, max]) linearly.
struct ModelRanges {
    double position{};
    double velocity{};
    double torque{};
    double stiffness{};
    double damping{};
};

ModelRanges const &rangesOf(MotorModel model) noexcept;
std::optional<MotorModel> modelFromString(std::string_view name) noexcept;
std::string_view toString(MotorModel model) noexcept;

enum class FrameType : std::uint8_t {
    Ping = 0,
    Motion = 1,
    Status = 2,
    Enable = 3,
    Disable = 4,
    ReadParameter = 17,
    WriteParameter = 18,
    FaultReport = 21,
    ActiveReport = 24,
};

namespace parameter {
inline constexpr std::uint16_t kRunMode = 0x7005;
inline constexpr std::uint16_t kTorqueLimit = 0x700B;
inline constexpr std::uint16_t kMechanicalPosition = 0x7019;
inline constexpr std::uint16_t kMechanicalVelocity = 0x701B;
inline constexpr std::uint16_t kBusVoltage = 0x701C;
// 1 reports positions in [-pi, pi) after power-up, 0 in [0, 2pi).
inline constexpr std::uint16_t kZeroState = 0x7029;
// Write-only; 20000 counts per second, 0 disables the timeout.
inline constexpr std::uint16_t kCanTimeout = 0x7028;
} // namespace parameter

// run_mode value for motion (MIT-style impedance) frames.
inline constexpr std::int8_t kRunModeMotion = 0;

struct FrameHeader {
    FrameType type{};
    std::uint16_t data{};
    std::uint8_t destination{};
};

std::uint32_t makeIdentifier(FrameType type, std::uint16_t data, std::uint8_t destination) noexcept;
std::optional<FrameHeader> headerOf(CanFrame const &frame) noexcept;

// Motor-side units: rad, rad/s, N·m, N·m/rad, N·m·s/rad. Values beyond the model's range are clamped.
struct MotionCommand {
    double position{};
    double velocity{};
    double stiffness{};
    double damping{};
    double torque{};
};

CanFrame encodeMotion(std::uint8_t motor, MotionCommand const &command, ModelRanges const &ranges) noexcept;
CanFrame encodeEnable(std::uint8_t motor, std::uint8_t host) noexcept;
// Answered with a status frame, so it also polls a disabled motor. `clearFault` resets latched faults.
CanFrame encodeDisable(std::uint8_t motor, std::uint8_t host, bool clearFault) noexcept;
CanFrame encodePing(std::uint8_t motor, std::uint8_t host) noexcept;
CanFrame encodeReadParameter(std::uint8_t motor, std::uint8_t host, std::uint16_t index) noexcept;
CanFrame encodeWriteParameter(std::uint8_t motor, std::uint8_t host, std::uint16_t index,
                              std::array<std::uint8_t, 4> value) noexcept;
CanFrame encodeActiveReport(std::uint8_t motor, std::uint8_t host, bool enabled) noexcept;

enum class MotorMode : std::uint8_t { Reset = 0, Calibration = 1, Run = 2 };

// Status flag bits as carried in the identifier.
namespace status_flag {
inline constexpr std::uint8_t kUndervoltage = 1U << 0U;
inline constexpr std::uint8_t kOvercurrent = 1U << 1U;
inline constexpr std::uint8_t kOvertemperature = 1U << 2U;
inline constexpr std::uint8_t kEncoderFault = 1U << 3U;
inline constexpr std::uint8_t kStall = 1U << 4U;
inline constexpr std::uint8_t kUncalibrated = 1U << 5U;
} // namespace status_flag

struct Status {
    std::uint8_t motor{};
    MotorMode mode{};
    std::uint8_t flags{};
    double position{};
    double velocity{};
    double torque{};
    double temperature{};
};

// Status and active-report frames; firmware-version replies share the status type and are rejected.
std::optional<Status> decodeStatus(CanFrame const &frame, ModelRanges const &ranges) noexcept;

// The motor's side of the exchange, for simulated motors and tests.
CanFrame encodeStatus(std::uint8_t host, Status const &status, ModelRanges const &ranges) noexcept;
std::optional<MotionCommand> decodeMotion(CanFrame const &frame, ModelRanges const &ranges) noexcept;

struct ParameterValue {
    std::uint8_t motor{};
    std::uint16_t index{};
    // The motor answered the read; on failure `raw` is meaningless.
    bool ok{};
    std::array<std::uint8_t, 4> raw{};

    float asFloat() const noexcept;
    std::uint32_t asUint32() const noexcept;
    std::int8_t asInt8() const noexcept;
};

std::optional<ParameterValue> decodeParameter(CanFrame const &frame) noexcept;

struct FaultReport {
    std::uint8_t motor{};
    std::uint32_t faults{};
    std::uint32_t warnings{};
};

std::optional<FaultReport> decodeFaultReport(CanFrame const &frame) noexcept;

struct PingReply {
    std::uint8_t motor{};
    // The motor's MCU unique identifier.
    std::array<std::uint8_t, 8> uid{};
};

std::optional<PingReply> decodePing(CanFrame const &frame) noexcept;

std::array<std::uint8_t, 4> rawFloat(float value) noexcept;
std::array<std::uint8_t, 4> rawUint32(std::uint32_t value) noexcept;

} // namespace larm::drivers::robstride
