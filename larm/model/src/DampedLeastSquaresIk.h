#pragma once

#include <larm/model/RobotModel.h>

namespace larm::model {

struct IkLimits {
    JointVector lower;
    JointVector upper;
};

// Damped least-squares IK over any Kinematics backend.
std::unique_ptr<IkSolver> makeDampedLeastSquaresIk(std::unique_ptr<Kinematics> kinematics,
                                                   IkLimits const &limits);

} // namespace larm::model
