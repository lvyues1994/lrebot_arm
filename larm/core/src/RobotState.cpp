#include <larm/core/RobotState.h>

namespace larm {

JointState JointState::zero(std::size_t const dof) {
    return JointState{
        .position = zeroJointVector(dof),
        .velocity = zeroJointVector(dof),
        .effort = zeroJointVector(dof),
    };
}

JointCommand JointCommand::zero(std::size_t const dof) {
    return JointCommand{
        .position = zeroJointVector(dof),
        .velocity = zeroJointVector(dof),
        .effort = zeroJointVector(dof),
        .stiffness = zeroJointVector(dof),
        .damping = zeroJointVector(dof),
    };
}

RobotState RobotState::zero(std::size_t const dof) {
    auto state = RobotState{};
    state.joints = JointState::zero(dof);
    return state;
}

} // namespace larm
