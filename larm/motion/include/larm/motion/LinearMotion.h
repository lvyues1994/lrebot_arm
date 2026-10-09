#pragma once

#include <larm/core/RobotProfile.h>
#include <larm/model/RobotModel.h>
#include <larm/motion/Planning.h>

namespace larm::motion {

// A rest-to-rest motion of a tool center point (TCP) along a straight line.
struct LinearRequest {
    // The whole robot at the start.
    JointVector start;
    // TCP pose to reach, in the model's world frame.
    Pose3 target;
    model::FrameId tool;
    // The TCP in the tool frame.
    Pose3 tcp;
    // Joints that move; the others hold their start position.
    JointMask joints;
    CartesianLimits cartesian;
    MotionLimits limits;
};

// The TCP moves along the line while its orientation turns at a matching rate (spherical
// interpolation). Timing is jerk-limited and slowed down until every joint keeps its velocity and
// acceleration limits. Fails with SolverFailed when no joint configuration reaches the target or follows
// the line, and with InvalidArgument where the line passes so close to a singularity that the joints
// would jump. Not real-time: the kinematics and the solver are used from the calling thread.
Expected<std::shared_ptr<JointTrajectory const>>
planLinear(LinearRequest const &request, model::Kinematics &kinematics, model::IkSolver &solver);

} // namespace larm::motion
