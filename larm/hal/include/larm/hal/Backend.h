#pragma once

#include <larm/core/Error.h>
#include <larm/core/RobotState.h>
#include <larm/core/Time.h>

#include <bitset>
#include <cstdint>

namespace larm::hal {

enum class CommandMode : std::uint8_t { Position, Velocity, Effort, Impedance };
enum class DrivePower : std::uint8_t { Disabled, Enabled };

struct DriverCapabilities {
    // Indexed by CommandMode.
    std::bitset<4> modes;
    Duration minPeriod{};
    bool hasBrakes{};

    bool supports(CommandMode const mode) const noexcept {
        return modes.test(static_cast<std::size_t>(mode));
    }
};

// Called only from the control loop: no allocation, no locks, bounded time.
struct RealtimeIo {
    virtual ~RealtimeIo() = default;
    virtual void read(RobotState &out) noexcept = 0;
    // Power changes also go through the control loop so the bus has a single writer;
    // the resulting actuator state comes back through read().
    virtual void write(JointCommand const &command, DrivePower requested) noexcept = 0;
};

// Owns time for the control loop: hardware sleeps until the next period, simulation steps physics.
struct Timeline {
    virtual ~Timeline() = default;
    virtual TimePoint now() const noexcept = 0;
    virtual void advance() noexcept = 0;
};

// Lifecycle outside the control loop; calls may block.
struct RobotDriver {
    virtual ~RobotDriver() = default;
    virtual DriverCapabilities capabilities() const = 0;
    // Opens the bus and checks the actuators; they stay disabled.
    virtual Expected<void> connect() = 0;
    virtual void disconnect() noexcept = 0;
    virtual RealtimeIo &io() = 0;
};

// A driver together with the timeline that paces it.
struct Backend {
    virtual ~Backend() = default;
    virtual RobotDriver &driver() = 0;
    virtual Timeline &timeline() = 0;
};

} // namespace larm::hal
