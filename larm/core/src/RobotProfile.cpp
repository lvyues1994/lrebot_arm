#include <larm/core/RobotProfile.h>

#include <algorithm>
#include <fstream>
#include <initializer_list>
#include <set>
#include <sstream>

namespace larm {
namespace {

// Thrown inside the parser only; parseRobotProfile() turns it into an Error.
struct ParseFailure {
    std::string message;
};

[[noreturn]] void failAt(std::string const &path, std::string const &what) {
    throw ParseFailure{path + ": " + what};
}

void requireKeys(YAML::Node const &node, std::string const &path,
                 std::initializer_list<std::string_view> allowed) {
    if (not node.IsMap()) {
        failAt(path, "expected a mapping");
    }
    for (auto const &entry : node) {
        auto const key = entry.first.as<std::string>();
        if (std::find(allowed.begin(), allowed.end(), key) == allowed.end()) {
            failAt(path, "unknown key '" + key + "'");
        }
    }
}

std::string childPath(std::string const &path, std::string_view key) {
    return path.empty() ? std::string{key} : path + "." + std::string{key};
}

YAML::Node required(YAML::Node const &node, std::string const &path, char const *key) {
    auto const child = node[key];
    if (not child) {
        failAt(path, std::string{"missing key '"} + key + "'");
    }
    return child;
}

template <class T> T scalar(YAML::Node const &node, std::string const &path) {
    if (not node.IsScalar()) {
        failAt(path, "expected a scalar");
    }
    try {
        return node.as<T>();
    } catch (YAML::Exception const &) {
        failAt(path, "invalid value '" + node.Scalar() + "'");
    }
}

template <class T> T requiredScalar(YAML::Node const &node, std::string const &path, char const *key) {
    return scalar<T>(required(node, path, key), childPath(path, key));
}

double positive(YAML::Node const &node, std::string const &path, char const *key) {
    auto const value = requiredScalar<double>(node, path, key);
    if (not(value > 0.0)) {
        failAt(childPath(path, key), "must be positive");
    }
    return value;
}

double nonNegative(YAML::Node const &node, std::string const &path, char const *key) {
    auto const value = requiredScalar<double>(node, path, key);
    if (not(value >= 0.0)) {
        failAt(childPath(path, key), "must not be negative");
    }
    return value;
}

std::filesystem::path resolvePath(std::filesystem::path const &baseDirectory, std::string const &value) {
    auto const path = std::filesystem::path{value};
    return (path.is_absolute() ? path : baseDirectory / path).lexically_normal();
}

JointUnit parseUnit(YAML::Node const &node, std::string const &path) {
    if (not node) {
        return JointUnit::Radian;
    }
    auto const unit = scalar<std::string>(node, path);
    if (unit == "radian") {
        return JointUnit::Radian;
    }
    if (unit == "meter") {
        return JointUnit::Meter;
    }
    failAt(path, "unit must be 'radian' or 'meter'");
}

JointLimits parseLimits(YAML::Node const &node, std::string const &path) {
    requireKeys(node, path, {"position", "velocity", "acceleration", "jerk", "effort"});
    auto const position = required(node, path, "position");
    auto const positionPath = childPath(path, "position");
    if (not position.IsSequence() or position.size() != 2) {
        failAt(positionPath, "expected [lower, upper]");
    }
    auto limits = JointLimits{
        .lower = scalar<double>(position[0], positionPath),
        .upper = scalar<double>(position[1], positionPath),
        .velocity = positive(node, path, "velocity"),
        .acceleration = positive(node, path, "acceleration"),
        .jerk = positive(node, path, "jerk"),
        .effort = positive(node, path, "effort"),
    };
    if (not(limits.lower < limits.upper)) {
        failAt(positionPath, "lower limit must be below upper limit");
    }
    return limits;
}

JointSpec parseJoint(YAML::Node const &node, std::string const &path) {
    requireKeys(node, path, {"name", "description_joint", "unit", "limits", "gains"});
    auto joint = JointSpec{};
    joint.name = requiredScalar<std::string>(node, path, "name");
    joint.descriptionJoint =
        node["description_joint"]
            ? scalar<std::string>(node["description_joint"], childPath(path, "description_joint"))
            : joint.name;
    joint.unit = parseUnit(node["unit"], childPath(path, "unit"));
    joint.limits = parseLimits(required(node, path, "limits"), childPath(path, "limits"));
    auto const gains = required(node, path, "gains");
    auto const gainsPath = childPath(path, "gains");
    requireKeys(gains, gainsPath, {"stiffness", "damping"});
    joint.gains = JointGains{
        .stiffness = nonNegative(gains, gainsPath, "stiffness"),
        .damping = nonNegative(gains, gainsPath, "damping"),
    };
    return joint;
}

std::vector<JointSpec> parseJoints(YAML::Node const &node, std::string const &path) {
    if (not node.IsSequence() or node.size() == 0 or node.size() > kMaxDof) {
        failAt(path, "expected a list of 1 to " + std::to_string(kMaxDof) + " joints");
    }
    auto joints = std::vector<JointSpec>{};
    auto names = std::set<std::string>{};
    for (std::size_t i = 0; i < node.size(); ++i) {
        auto joint = parseJoint(node[i], path + "[" + std::to_string(i) + "]");
        if (not names.insert(joint.name).second) {
            failAt(path, "duplicate joint '" + joint.name + "'");
        }
        joints.push_back(std::move(joint));
    }
    return joints;
}

std::size_t jointIndexByName(std::vector<JointSpec> const &joints, std::string const &name,
                             std::string const &path) {
    auto const found = std::find_if(joints.begin(), joints.end(),
                                    [&](JointSpec const &joint) { return joint.name == name; });
    if (found == joints.end()) {
        failAt(path, "unknown joint '" + name + "'");
    }
    return static_cast<std::size_t>(found - joints.begin());
}

Eigen::Vector3d parseVector3(YAML::Node const &node, std::string const &path) {
    if (not node.IsSequence() or node.size() != 3) {
        failAt(path, "expected [x, y, z]");
    }
    return {scalar<double>(node[0], path), scalar<double>(node[1], path), scalar<double>(node[2], path)};
}

// URDF convention: roll, pitch, yaw about the fixed x, y, z axes.
Pose3 parseTcp(YAML::Node const &node, std::string const &path) {
    requireKeys(node, path, {"xyz", "rpy"});
    auto tcp = Pose3{};
    if (node["xyz"]) {
        tcp.translation = parseVector3(node["xyz"], childPath(path, "xyz"));
    }
    if (node["rpy"]) {
        auto const rpy = parseVector3(node["rpy"], childPath(path, "rpy"));
        tcp.rotation = Eigen::AngleAxisd{rpy.z(), Eigen::Vector3d::UnitZ()} *
                       Eigen::AngleAxisd{rpy.y(), Eigen::Vector3d::UnitY()} *
                       Eigen::AngleAxisd{rpy.x(), Eigen::Vector3d::UnitX()};
    }
    return tcp;
}

CartesianLimits parseCartesianLimits(YAML::Node const &node, std::string const &path) {
    requireKeys(node, path,
                {"linear_velocity", "linear_acceleration", "angular_velocity", "angular_acceleration"});
    return CartesianLimits{
        .linearVelocity = positive(node, path, "linear_velocity"),
        .linearAcceleration = positive(node, path, "linear_acceleration"),
        .angularVelocity = positive(node, path, "angular_velocity"),
        .angularAcceleration = positive(node, path, "angular_acceleration"),
    };
}

std::vector<JointGroupSpec> parseGroups(YAML::Node const &node, std::string const &path,
                                        std::vector<JointSpec> const &joints) {
    if (not node.IsMap() or node.size() == 0) {
        failAt(path, "expected a mapping of joint groups");
    }
    auto groups = std::vector<JointGroupSpec>{};
    auto claimed = JointMask{};
    for (auto const &entry : node) {
        auto group = JointGroupSpec{.name = entry.first.as<std::string>()};
        auto const groupPath = childPath(path, group.name);
        auto const groupNode = YAML::Node{entry.second};
        requireKeys(groupNode, groupPath, {"joints", "base", "tool", "tcp", "cartesian_limits"});
        auto const members = required(groupNode, groupPath, "joints");
        auto const membersPath = childPath(groupPath, "joints");
        if (not members.IsSequence() or members.size() == 0) {
            failAt(membersPath, "expected a non-empty list of joint names");
        }
        for (auto const &member : members) {
            auto const index =
                jointIndexByName(joints, scalar<std::string>(member, membersPath), membersPath);
            if (claimed.test(index)) {
                failAt(membersPath, "joint '" + joints[index].name + "' already belongs to another group");
            }
            claimed.set(index);
            group.joints.push_back(index);
        }
        if (groupNode["base"]) {
            group.baseFrame = scalar<std::string>(groupNode["base"], childPath(groupPath, "base"));
        }
        if (groupNode["tool"]) {
            group.toolFrame = scalar<std::string>(groupNode["tool"], childPath(groupPath, "tool"));
        }
        for (auto const *const key : {"tcp", "cartesian_limits"}) {
            if (groupNode[key] and group.toolFrame.empty()) {
                failAt(childPath(groupPath, key), "needs the group's tool frame");
            }
        }
        if (groupNode["tcp"]) {
            group.tcp = parseTcp(groupNode["tcp"], childPath(groupPath, "tcp"));
        }
        if (groupNode["cartesian_limits"]) {
            group.cartesianLimits =
                parseCartesianLimits(groupNode["cartesian_limits"], childPath(groupPath, "cartesian_limits"));
        }
        groups.push_back(std::move(group));
    }
    return groups;
}

JointVector parseConfiguration(YAML::Node const &node, std::string const &path,
                               std::vector<JointSpec> const &joints) {
    if (not node.IsSequence() or node.size() != joints.size()) {
        failAt(path, "expected one value per joint");
    }
    auto configuration = zeroJointVector(joints.size());
    for (std::size_t i = 0; i < joints.size(); ++i) {
        auto const value = scalar<double>(node[i], path);
        if (not(value >= joints[i].limits.lower and value <= joints[i].limits.upper)) {
            failAt(path, "value for '" + joints[i].name + "' is outside its position limits");
        }
        configuration[idx(i)] = value;
    }
    return configuration;
}

std::map<std::string, JointVector, std::less<>> parsePoses(YAML::Node const &node, std::string const &path,
                                                           std::vector<JointSpec> const &joints) {
    auto poses = std::map<std::string, JointVector, std::less<>>{};
    if (not node) {
        return poses;
    }
    if (not node.IsMap()) {
        failAt(path, "expected a mapping of named poses");
    }
    for (auto const &entry : node) {
        auto const name = entry.first.as<std::string>();
        poses.emplace(name, parseConfiguration(entry.second, childPath(path, name), joints));
    }
    return poses;
}

SafetySpec parseSafety(YAML::Node const &node, std::string const &path,
                       std::vector<JointSpec> const &joints) {
    requireKeys(node, path,
                {"limit_tolerance", "feedback_timeout_cycles", "tracking_error_limit", "enable_ramp_ms",
                 "rest_pose"});
    auto safety = SafetySpec{};
    safety.limitTolerance = nonNegative(node, path, "limit_tolerance");
    auto const timeoutCycles = requiredScalar<std::int64_t>(node, path, "feedback_timeout_cycles");
    if (timeoutCycles < 1) {
        failAt(childPath(path, "feedback_timeout_cycles"), "must be at least 1");
    }
    safety.feedbackTimeoutCycles = static_cast<std::uint32_t>(timeoutCycles);

    auto const tracking = required(node, path, "tracking_error_limit");
    auto const trackingPath = childPath(path, "tracking_error_limit");
    requireKeys(tracking, trackingPath, {"radian", "meter"});
    safety.trackingErrorRadian = positive(tracking, trackingPath, "radian");
    safety.trackingErrorMeter = positive(tracking, trackingPath, "meter");

    safety.enableRamp =
        std::chrono::milliseconds{static_cast<std::int64_t>(nonNegative(node, path, "enable_ramp_ms"))};

    safety.restPose =
        parseConfiguration(required(node, path, "rest_pose"), childPath(path, "rest_pose"), joints);
    return safety;
}

ConfigNode section(YAML::Node const &root, char const *key, std::filesystem::path const &baseDirectory) {
    auto const node = root[key];
    return ConfigNode{.node = node ? YAML::Clone(node) : YAML::Node{}, .baseDirectory = baseDirectory};
}

RobotProfile parseProfileNode(YAML::Node const &root, std::filesystem::path const &baseDirectory) {
    requireKeys(root, "profile",
                {"robot", "description", "control", "joints", "groups", "safety", "poses", "sim", "driver"});
    auto profile = RobotProfile{};
    profile.name = requiredScalar<std::string>(root, "", "robot");

    auto const description = required(root, "", "description");
    requireKeys(description, "description", {"urdf", "mjcf", "srdf"});
    profile.urdf =
        resolvePath(baseDirectory, requiredScalar<std::string>(description, "description", "urdf"));
    profile.mjcf =
        resolvePath(baseDirectory, requiredScalar<std::string>(description, "description", "mjcf"));
    if (description["srdf"]) {
        profile.srdf =
            resolvePath(baseDirectory, requiredScalar<std::string>(description, "description", "srdf"));
    }

    auto const control = required(root, "", "control");
    requireKeys(control, "control", {"period_us"});
    profile.controlPeriod =
        std::chrono::microseconds{static_cast<std::int64_t>(positive(control, "control", "period_us"))};

    profile.joints = parseJoints(required(root, "", "joints"), "joints");
    profile.groups = parseGroups(required(root, "", "groups"), "groups", profile.joints);
    profile.safety = parseSafety(required(root, "", "safety"), "safety", profile.joints);
    profile.poses = parsePoses(root["poses"], "poses", profile.joints);
    profile.sim = section(root, "sim", baseDirectory);
    profile.driver = section(root, "driver", baseDirectory);
    return profile;
}

} // namespace

std::optional<std::size_t> RobotProfile::findJoint(std::string_view const jointName) const {
    auto const found = std::find_if(joints.begin(), joints.end(),
                                    [&](JointSpec const &joint) { return joint.name == jointName; });
    if (found == joints.end()) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(found - joints.begin());
}

std::optional<std::size_t> RobotProfile::findGroup(std::string_view const groupName) const {
    auto const found = std::find_if(groups.begin(), groups.end(),
                                    [&](JointGroupSpec const &group) { return group.name == groupName; });
    if (found == groups.end()) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(found - groups.begin());
}

Expected<RobotProfile> parseRobotProfile(std::string const &yaml,
                                         std::filesystem::path const &baseDirectory) {
    try {
        return parseProfileNode(YAML::Load(yaml), baseDirectory);
    } catch (ParseFailure const &failure) {
        return makeError(ErrorCode::InvalidConfig, failure.message);
    } catch (YAML::Exception const &exception) {
        return makeError(ErrorCode::InvalidConfig, exception.what());
    }
}

Expected<RobotProfile> loadRobotProfile(std::filesystem::path const &file) {
    auto stream = std::ifstream{file};
    if (not stream) {
        return makeError(ErrorCode::Io, "cannot read robot profile " + file.string());
    }
    auto content = std::stringstream{};
    content << stream.rdbuf();
    auto const baseDirectory = std::filesystem::absolute(file).parent_path();
    return parseRobotProfile(content.str(), baseDirectory).map_error([&](Error error) {
        error.message = file.string() + ": " + error.message;
        return error;
    });
}

} // namespace larm
