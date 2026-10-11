#pragma once

#include <larm/drivers/can/CanTransport.h>
#include <larm/drivers/robstride/DriverConfig.h>
#include <larm/hal/Backend.h>

#include <memory>

namespace larm::drivers::robstride {

struct DriverStatistics {
    std::uint64_t cycles{};
    // Actuator replies missing at read time, summed over actuators.
    std::uint64_t missedReplies{};
    // Frames from unknown motors or addressed to another host.
    std::uint64_t foreignFrames{};
    can::TransportStatistics transport;
};

// RobStride motors on one CAN bus, driven with motion (MIT-style impedance) frames.
//
// connect() checks every actuator (ping, run_mode, zero_sta) and applies the configured reporting
// and CAN timeout, all actuators at once. Per actuator and cycle the driver then sends one frame and
// expects one reply before the next read(): a position read while disabled, enable frames while
// enabling (200 ms at most), motion frames while running and disable frames while disabling.
// Disabled motors are never sent disable frames, so a motor left running by another program keeps
// its last command until power is requested. A motor that leaves run mode by itself, or does not
// enter it in time, drops out: it is sent disable frames until power is withdrawn or the request
// asks to re-arm, which first clears latched motor faults. While the bus is off nothing changes on
// the motors and the feedback goes stale. Joint feedback goes through each actuator's transmission.
struct RobStrideDriver : hal::RobotDriver {
    virtual DriverStatistics statistics() const noexcept = 0;
};

// `clock` stamps the feedback; it must be the timeline that paces the control loop.
std::unique_ptr<RobStrideDriver> makeRobStrideDriver(DriverConfig config,
                                                     std::unique_ptr<can::CanTransport> transport,
                                                     hal::Timeline const &clock);

} // namespace larm::drivers::robstride
