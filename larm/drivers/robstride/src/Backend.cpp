#include <larm/drivers/robstride/Backend.h>

#include <algorithm>

namespace larm::drivers::robstride {
namespace {

struct RobStrideBackendImpl final : RobStrideBackend {
    RobStrideBackendImpl(Duration const period, DriverConfig config,
                         std::unique_ptr<SimulatedMotors> simulated_,
                         std::unique_ptr<can::CanTransport> transport)
        : clock{hal::makeMonotonicTimeline(period)}, simulated{std::move(simulated_)},
          robstride{makeRobStrideDriver(std::move(config), std::move(transport), *clock)} {}

    RobStrideDriver &driver() override { return *robstride; }
    hal::MonotonicTimeline &timeline() override { return *clock; }
    SimulatedMotors *simulatedMotors() noexcept override { return simulated.get(); }

  private:
    std::unique_ptr<hal::MonotonicTimeline> clock;
    // Before the driver, whose transport may point into it.
    std::unique_ptr<SimulatedMotors> simulated;
    std::unique_ptr<RobStrideDriver> robstride;
};

} // namespace

Expected<std::unique_ptr<RobStrideBackend>> makeRobStrideBackend(RobotProfile const &profile) {
    auto config = parseDriverConfig(profile);
    if (not config) {
        return tl::make_unexpected(config.error());
    }
    // One frame per actuator goes out each cycle.
    auto transport = can::openSocketCan(config->interface, {.burst = config->actuators.size()});
    if (not transport) {
        return tl::make_unexpected(transport.error());
    }
    return std::unique_ptr<RobStrideBackend>{std::make_unique<RobStrideBackendImpl>(
        profile.controlPeriod, std::move(*config), nullptr, std::move(*transport))};
}

Expected<std::unique_ptr<RobStrideBackend>> makeSimulatedRobStrideBackend(RobotProfile const &profile,
                                                                          SimulatedMotorsOptions options) {
    auto config = parseDriverConfig(profile);
    if (not config) {
        return tl::make_unexpected(config.error());
    }
    options.step = profile.controlPeriod;
    if (options.stops.empty()) {
        for (auto const &actuator : config->actuators) {
            auto const &limits = profile.joints[actuator.joint].limits;
            auto const &[scale, offset] = actuator.transmission;
            auto stop = std::array{(limits.lower - offset) / scale, (limits.upper - offset) / scale};
            std::sort(stop.begin(), stop.end());
            options.stops.push_back(stop);
        }
    }
    auto motors = makeSimulatedMotors(*config, std::move(options));
    auto transport = motors->transport();
    return std::unique_ptr<RobStrideBackend>{std::make_unique<RobStrideBackendImpl>(
        profile.controlPeriod, std::move(*config), std::move(motors), std::move(transport))};
}

} // namespace larm::drivers::robstride
