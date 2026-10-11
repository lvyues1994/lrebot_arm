#pragma once

#include <larm/drivers/robstride/RobStrideDriver.h>
#include <larm/drivers/robstride/SimulatedMotors.h>
#include <larm/hal/MonotonicTimeline.h>

namespace larm::drivers::robstride {

// RobStride motors paced by the monotonic clock at the profile's control period.
struct RobStrideBackend : hal::Backend {
    RobStrideDriver &driver() override = 0;
    hal::MonotonicTimeline &timeline() override = 0;
    // The motors behind a rehearsal backend; null on hardware.
    virtual SimulatedMotors *simulatedMotors() noexcept = 0;
};

// The arm on the SocketCAN interface named in the profile's driver section.
Expected<std::unique_ptr<RobStrideBackend>> makeRobStrideBackend(RobotProfile const &profile);

// The same driver in front of simulated motors, for rehearsing hardware procedures. Unless the options
// give stops, the joints stop at their position limits, where the arm rests on its mechanical stops.
Expected<std::unique_ptr<RobStrideBackend>> makeSimulatedRobStrideBackend(RobotProfile const &profile,
                                                                          SimulatedMotorsOptions options);

} // namespace larm::drivers::robstride
