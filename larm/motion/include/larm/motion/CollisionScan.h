#pragma once

#include <larm/model/RobotModel.h>
#include <larm/motion/JointTrajectory.h>

#include <optional>

namespace larm::motion {

struct TrajectoryCollision {
    Duration time{};
    model::LinkContact contact;
};

// The first self-collision along `trajectory`. Contacts at its start may persist (see
// CollisionChecker::allowContactsAt); it is checked often enough that no joint moves more than its
// `step` between two checks, and at its end.
std::optional<TrajectoryCollision> findCollision(JointTrajectory const &trajectory,
                                                 model::CollisionChecker &checker, JointVector const &step);

} // namespace larm::motion
