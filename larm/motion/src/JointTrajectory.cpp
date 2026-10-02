#include <larm/motion/JointTrajectory.h>

namespace larm::motion {

JointSample JointSample::zero(std::size_t const dof) {
    return JointSample{
        .position = zeroJointVector(dof),
        .velocity = zeroJointVector(dof),
        .acceleration = zeroJointVector(dof),
    };
}

} // namespace larm::motion
