#pragma once

#include <larm/core/RobotProfile.h>
#include <larm/runtime/RobotSession.h>

#include <map>
#include <string>
#include <vector>

namespace larm::ros {

// Joint names of one group as messages use them: the description (URDF) names, with the profile
// names accepted as well.
struct GroupJoints {
    std::string group;
    std::vector<std::string> names;
    std::map<std::string, std::size_t> byName;

    static GroupJoints of(RobotProfile const &profile, JointGroupSpec const &spec);

    // For each joint of the group, the index of its value in a message listing `messageNames`.
    // Empty `messageNames` means the group's own order. Throws std::invalid_argument on a mismatch.
    std::vector<std::size_t> order(std::vector<std::string> const &messageNames) const;
};

// Throws std::invalid_argument when a point does not fit the joint order.
runtime::JointPath toJointPath(std::vector<std::size_t> const &order,
                               std::vector<std::vector<double>> const &positions,
                               std::vector<std::vector<double>> const &velocities,
                               std::vector<std::vector<double>> const &accelerations,
                               std::vector<Duration> const &times);

} // namespace larm::ros
