#pragma once

#include <larm/control/Channels.h>
#include <larm/core/RobotProfile.h>
#include <larm/hal/Backend.h>
#include <larm/model/RobotModel.h>

#include <memory>

namespace larm::control {

// One control period: read → requests → model → controllers → hold → safety → write → publish.
// Which thread calls tick() and how time advances in between is decided by the caller.
struct ControlCycle {
    virtual ~ControlCycle() = default;
    virtual void tick() noexcept = 0;
};

struct ControlCycleDeps {
    hal::RobotDriver *driver{};
    model::RobotModel const *model{};
    RuntimeChannels *channels{};
};

// Joints no controller claims hold their last target with the profile gains and gravity feed-forward.
// The driver must support impedance commands.
Expected<std::unique_ptr<ControlCycle>> makeControlCycle(RobotProfile const &profile,
                                                         ControlCycleDeps const &deps);

} // namespace larm::control
