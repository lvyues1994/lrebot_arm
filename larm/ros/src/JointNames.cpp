#include "JointNames.h"

#include <stdexcept>

namespace larm::ros {
namespace {

JointVector pick(std::vector<double> const &values, std::vector<std::size_t> const &order) {
    auto picked = zeroJointVector(order.size());
    for (std::size_t i = 0; i < order.size(); ++i) {
        picked[idx(i)] = values.at(order[i]);
    }
    return picked;
}

} // namespace

GroupJoints GroupJoints::of(RobotProfile const &profile, JointGroupSpec const &spec) {
    auto joints = GroupJoints{.group = spec.name};
    for (std::size_t i = 0; i < spec.joints.size(); ++i) {
        auto const &joint = profile.joints[spec.joints[i]];
        joints.names.push_back(joint.descriptionJoint);
        joints.profileIndices.push_back(spec.joints[i]);
        joints.byName[joint.descriptionJoint] = i;
        joints.byName[joint.name] = i;
    }
    return joints;
}

std::vector<std::size_t> GroupJoints::order(std::vector<std::string> const &messageNames) const {
    auto result = std::vector<std::size_t>(names.size());
    if (messageNames.empty()) {
        for (std::size_t i = 0; i < names.size(); ++i) {
            result[i] = i;
        }
        return result;
    }
    if (messageNames.size() != names.size()) {
        throw std::invalid_argument{"expected the " + std::to_string(names.size()) + " joints of group '" +
                                    group + "'"};
    }
    auto seen = std::vector<bool>(names.size(), false);
    for (std::size_t i = 0; i < messageNames.size(); ++i) {
        auto const found = byName.find(messageNames[i]);
        if (found == byName.end() or seen[found->second]) {
            throw std::invalid_argument{"joint '" + messageNames[i] + "' is not in group '" + group +
                                        "' or repeats"};
        }
        seen[found->second] = true;
        result[found->second] = i;
    }
    return result;
}

runtime::JointPath toJointPath(std::vector<std::size_t> const &order,
                               std::vector<std::vector<double>> const &positions,
                               std::vector<std::vector<double>> const &velocities,
                               std::vector<std::vector<double>> const &accelerations,
                               std::vector<Duration> const &times) {
    auto path = runtime::JointPath{};
    for (std::size_t point = 0; point < positions.size(); ++point) {
        if (positions[point].size() != order.size()) {
            throw std::invalid_argument{"trajectory point " + std::to_string(point) +
                                        " has the wrong number of positions"};
        }
        auto waypoint =
            runtime::PathWaypoint{.time = times.at(point), .position = pick(positions[point], order)};
        if (velocities.at(point).size() == order.size()) {
            waypoint.velocity = pick(velocities[point], order);
        }
        if (accelerations.at(point).size() == order.size()) {
            waypoint.acceleration = pick(accelerations[point], order);
        }
        path.waypoints.push_back(std::move(waypoint));
    }
    return path;
}

} // namespace larm::ros
