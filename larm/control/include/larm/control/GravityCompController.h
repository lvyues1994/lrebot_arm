#pragma once

#include <larm/control/Controller.h>
#include <larm/core/RobotProfile.h>

#include <memory>

namespace larm::control {

struct GravityCompControllerConfig {
    JointMask joints;
    // Viscous damping while a joint moves freely.
    JointVector damping;
    // Where the walls begin, inside the joint limits; beyond them a spring pushes the joint back.
    JointVector wallLower;
    JointVector wallUpper;
    JointVector wallStiffness;
    JointVector wallDamping;
    // A joint faster than this fails the goal (the arm was flung).
    JointVector velocityLimit;
};

// For the profile: damping a fifth of the holding damping; walls with the holding gains, beginning
// 0.1 rad (5 mm) inside each limit but not past the rest pose, which may lie on a limit.
GravityCompControllerConfig profileGravityComp(RobotProfile const &profile, JointMask joints);

// Lets the joints be moved by hand: gravity feed-forward, little damping and no stiffness except in the
// walls. Runs until stopped; on a stop request it finishes at once, commanding the measured positions,
// so that holding takes over without a jump.
std::unique_ptr<Controller> makeGravityCompController(GravityCompControllerConfig config);

} // namespace larm::control
