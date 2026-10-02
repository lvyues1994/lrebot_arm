#pragma once

#include <larm/core/RobotProfile.h>
#include <larm/drivers/robstride/Codec.h>

#include <optional>
#include <string>
#include <vector>

namespace larm::drivers::robstride {

// joint = scale * motor + offset; a negative scale reverses the direction. Joint stiffness K maps to
// K * scale^2 at the motor, joint effort F to F * scale.
struct Transmission {
    double scale = 1.0;
    double offset = 0.0;
};

struct ActuatorConfig {
    std::size_t joint{};
    std::string jointName;
    std::uint8_t id{};
    MotorModel model{};
    Transmission transmission;
};

struct DriverConfig {
    std::string interface;
    std::uint8_t hostId{};
    // One per profile joint, in profile order.
    std::vector<ActuatorConfig> actuators;
    // Motors that report on their own load the bus; connect() switches that off.
    bool disableActiveReport = true;
    // Written at connect() when set: the motor drops to reset mode after this long without frames.
    std::optional<Duration> canTimeout;
    std::uint32_t bitrate = 1'000'000;
};

// Reads the profile's `driver` section (type robstride_socketcan) and checks it against the joints:
// every joint has one actuator, and joint limits and gains fit the motor's encoding ranges.
Expected<DriverConfig> parseDriverConfig(RobotProfile const &profile);

// Bus time of one control cycle (a command and a reply per actuator) at the configured bitrate.
Duration cycleBusTime(DriverConfig const &config) noexcept;

} // namespace larm::drivers::robstride
