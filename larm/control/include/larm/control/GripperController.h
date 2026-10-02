#pragma once

#include <larm/control/Controller.h>

#include <memory>

namespace larm::control {

struct GripperControllerConfig {
    std::size_t joint{};
    double target{};
    // The commanded offset from the measured position is limited so the impedance force stays below this.
    double maxEffort{};
    // Speed of the reference moving toward the target.
    double speed{};
    double stiffness{};
    double damping{};
    // Within this of the target the motion has reached its goal.
    double positionTolerance{};
    // Slower than this, held for `stallTime` away from the target, the gripper has stalled on an object.
    double stallVelocity{};
    Duration stallTime{};
};

// Moves one joint toward a target with a force limit. Reaching the target or stalling succeeds;
// after a stall the hold keeps squeezing with the limited force.
std::unique_ptr<Controller> makeGripperController(GripperControllerConfig const &config);

} // namespace larm::control
