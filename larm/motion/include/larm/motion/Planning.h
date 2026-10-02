#pragma once

#include <larm/core/Error.h>
#include <larm/core/RobotProfile.h>
#include <larm/motion/JointTrajectory.h>

#include <memory>
#include <optional>
#include <span>

namespace larm::motion {

struct MotionLimits {
    JointVector velocity;
    JointVector acceleration;
    JointVector jerk;
};

// The profile's joint limits, each scaled by `scale` in (0, 1].
MotionLimits motionLimits(RobotProfile const &profile, double scale = 1.0);

struct PointToPointRequest {
    JointSample start;
    JointVector target;
    MotionLimits limits;
    // Joints that move; the others hold their start position.
    JointMask joints;
};

// Time-optimal, jerk-limited motion that ends at rest on the target with all joints arriving together.
Expected<std::shared_ptr<JointTrajectory const>> planPointToPoint(PointToPointRequest const &request);

struct TimedWaypoint {
    Duration time{};
    JointVector position;
    std::optional<JointVector> velocity;
    std::optional<JointVector> acceleration;
};

// Piecewise polynomial through the waypoints: quintic on segments whose ends carry velocities and
// accelerations, cubic Hermite otherwise. Missing velocities come from neighbouring waypoints and are
// zero at both ends. The first waypoint must be at time zero and times must increase.
Expected<std::shared_ptr<JointTrajectory const>>
interpolateWaypoints(std::span<TimedWaypoint const> waypoints);

// Re-plans a stop in place. Construct it off the control loop; plan() and trajectory() never allocate.
struct StopPlanner {
    virtual ~StopPlanner() = default;
    // Brings velocity and acceleration to zero as fast as the limits allow; false if no stop was found.
    virtual bool plan(JointSample const &from) noexcept = 0;
    virtual JointTrajectory const &trajectory() const noexcept = 0;
};

std::unique_ptr<StopPlanner> makeStopPlanner(MotionLimits const &limits);

} // namespace larm::motion
