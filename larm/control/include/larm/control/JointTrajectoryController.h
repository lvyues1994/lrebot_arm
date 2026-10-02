#pragma once

#include <larm/control/Controller.h>
#include <larm/core/RobotProfile.h>
#include <larm/motion/JointTrajectory.h>
#include <larm/motion/Planning.h>

#include <memory>

namespace larm::control {

struct JointImpedance {
    JointVector stiffness;
    JointVector damping;
};

// The profile's default gains.
JointImpedance profileImpedance(RobotProfile const &profile);
// The profile's tracking error limit for each joint, by unit.
JointVector profileTrackingTolerance(RobotProfile const &profile);

struct JointTrajectoryControllerConfig {
    std::shared_ptr<motion::JointTrajectory const> trajectory;
    JointMask joints;
    JointImpedance impedance;
    // Largest allowed |reference - measured| while the trajectory runs.
    JointVector trackingTolerance;
    // Largest allowed |final reference - measured| to succeed.
    JointVector goalTolerance;
    // How long after the trajectory ends the goal tolerance may take to be met.
    Duration goalTimeout{};
    // Limits for a requested stop.
    motion::MotionLimits stopLimits;
    bool gravityCompensation = true;
};

std::unique_ptr<Controller> makeJointTrajectoryController(JointTrajectoryControllerConfig config);

} // namespace larm::control
