#pragma once

#include <larm/drivers/robstride/DriverConfig.h>

#include <memory>
#include <optional>

namespace larm::probe {

using drivers::robstride::ActuatorConfig;
using drivers::robstride::DriverConfig;

// Pings and parameter reads, one at a time. Sends nothing that changes a motor's state.
struct ReadOnlyBus {
    ReadOnlyBus(DriverConfig config_, std::unique_ptr<drivers::can::CanTransport> transport_);

    bool ping(ActuatorConfig const &actuator);
    std::optional<drivers::robstride::ParameterValue> read(ActuatorConfig const &actuator,
                                                           std::uint16_t index);
    // Status frames nobody asked for, i.e. active reports, seen so far.
    std::uint64_t unsolicitedStatus() const noexcept { return unsolicited; }
    DriverConfig const &driverConfig() const noexcept { return config; }

  private:
    template <class Matches>
    std::optional<drivers::can::CanFrame> request(drivers::can::CanFrame const &frame, Matches matches);

    DriverConfig config;
    std::unique_ptr<drivers::can::CanTransport> transport;
    std::uint64_t unsolicited{};
};

} // namespace larm::probe
