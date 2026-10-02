#include <larm/drivers/robstride/DriverConfig.h>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <set>

namespace larm::drivers::robstride {
namespace {

// An extended data frame with 8 bytes is 131 bits plus up to 24 stuff bits.
constexpr double kBitsPerFrame = 155.0;

struct ParseFailure {
    std::string message;
};

[[noreturn]] void fail(std::string const &path, std::string const &what) {
    throw ParseFailure{path + ": " + what};
}

void requireKeys(YAML::Node const &node, std::string const &path,
                 std::initializer_list<std::string_view> allowed) {
    if (not node.IsMap()) {
        fail(path, "expected a mapping");
    }
    for (auto const &entry : node) {
        auto const key = entry.first.as<std::string>();
        if (std::find(allowed.begin(), allowed.end(), key) == allowed.end()) {
            fail(path, "unknown key '" + key + "'");
        }
    }
}

template <class T> T scalar(YAML::Node const &node, std::string const &path) {
    if (not node or not node.IsScalar()) {
        fail(path, "expected a scalar");
    }
    try {
        return node.as<T>();
    } catch (YAML::Exception const &) {
        fail(path, "invalid value '" + node.Scalar() + "'");
    }
}

std::uint8_t canId(YAML::Node const &node, std::string const &path) {
    auto const value = scalar<int>(node, path);
    if (value < 1 or value > 0xFE) {
        fail(path, "must be a CAN ID between 0x01 and 0xFE");
    }
    return static_cast<std::uint8_t>(value);
}

Transmission parseTransmission(YAML::Node const &node, std::string const &path) {
    if (not node) {
        return {};
    }
    requireKeys(node, path, {"scale", "offset"});
    auto const transmission = Transmission{
        .scale = node["scale"] ? scalar<double>(node["scale"], path + ".scale") : 1.0,
        .offset = node["offset"] ? scalar<double>(node["offset"], path + ".offset") : 0.0,
    };
    if (not std::isfinite(transmission.scale) or transmission.scale == 0.0 or
        not std::isfinite(transmission.offset)) {
        fail(path, "scale must be finite and non-zero, offset finite");
    }
    return transmission;
}

// The joint's limits and gains, carried to the motor, must fit the frame encoding.
void checkRanges(JointSpec const &joint, ActuatorConfig const &actuator, std::string const &path) {
    auto const &ranges = rangesOf(actuator.model);
    auto const &[scale, offset] = actuator.transmission;
    auto const reach = std::max(std::abs((joint.limits.lower - offset) / scale),
                                std::abs((joint.limits.upper - offset) / scale));
    auto const checks = std::initializer_list<std::pair<char const *, std::pair<double, double>>>{
        {"position limits", {reach, ranges.position}},
        {"velocity limit", {joint.limits.velocity / std::abs(scale), ranges.velocity}},
        {"effort limit", {joint.limits.effort * std::abs(scale), ranges.torque}},
        {"stiffness", {joint.gains.stiffness * scale * scale, ranges.stiffness}},
        {"damping", {joint.gains.damping * scale * scale, ranges.damping}},
    };
    for (auto const &[what, values] : checks) {
        if (values.first > values.second) {
            fail(path, "joint '" + joint.name + "' " + what + " exceed the " +
                           std::string{toString(actuator.model)} + " encoding range at the motor (" +
                           std::to_string(values.first) + " > " + std::to_string(values.second) + ")");
        }
    }
}

DriverConfig parse(RobotProfile const &profile) {
    auto const &node = profile.driver.node;
    auto const path = std::string{"driver"};
    requireKeys(
        node, path,
        {"type", "interface", "host_id", "actuators", "disable_active_report", "can_timeout_ms", "bitrate"});
    if (scalar<std::string>(node["type"], path + ".type") != "robstride_socketcan") {
        fail(path + ".type", "expected 'robstride_socketcan'");
    }
    auto config = DriverConfig{
        .interface = scalar<std::string>(node["interface"], path + ".interface"),
        .hostId = canId(node["host_id"], path + ".host_id"),
    };
    if (node["disable_active_report"]) {
        config.disableActiveReport =
            scalar<bool>(node["disable_active_report"], path + ".disable_active_report");
    }
    if (node["can_timeout_ms"]) {
        auto const milliseconds = scalar<double>(node["can_timeout_ms"], path + ".can_timeout_ms");
        if (not(milliseconds >= 0.0)) {
            fail(path + ".can_timeout_ms", "must not be negative");
        }
        config.canTimeout = fromSeconds(milliseconds / 1000.0);
    }
    if (node["bitrate"]) {
        config.bitrate = scalar<std::uint32_t>(node["bitrate"], path + ".bitrate");
        if (config.bitrate == 0) {
            fail(path + ".bitrate", "must be positive");
        }
    }

    auto const actuators = node["actuators"];
    if (not actuators or not actuators.IsSequence()) {
        fail(path + ".actuators", "expected a list");
    }
    auto byJoint = std::vector<std::optional<ActuatorConfig>>(profile.dof());
    auto ids = std::set<std::uint8_t>{config.hostId};
    for (std::size_t i = 0; i < actuators.size(); ++i) {
        auto const entry = actuators[i];
        auto const entryPath = path + ".actuators[" + std::to_string(i) + "]";
        requireKeys(entry, entryPath, {"joint", "id", "model", "transmission"});
        auto const jointName = scalar<std::string>(entry["joint"], entryPath + ".joint");
        auto const joint = profile.findJoint(jointName);
        if (not joint) {
            fail(entryPath + ".joint", "unknown joint '" + jointName + "'");
        }
        if (byJoint[*joint]) {
            fail(entryPath + ".joint", "joint '" + jointName + "' has two actuators");
        }
        auto const modelName = scalar<std::string>(entry["model"], entryPath + ".model");
        auto const model = modelFromString(modelName);
        if (not model) {
            fail(entryPath + ".model", "unknown model '" + modelName + "' (rs-00, rs-06)");
        }
        auto const actuator = ActuatorConfig{
            .joint = *joint,
            .jointName = jointName,
            .id = canId(entry["id"], entryPath + ".id"),
            .model = *model,
            .transmission = parseTransmission(entry["transmission"], entryPath + ".transmission"),
        };
        if (not ids.insert(actuator.id).second) {
            fail(entryPath + ".id", "CAN ID is used twice or equals the host ID");
        }
        checkRanges(profile.joints[*joint], actuator, entryPath);
        byJoint[*joint] = actuator;
    }
    for (std::size_t joint = 0; joint < profile.dof(); ++joint) {
        if (not byJoint[joint]) {
            fail(path + ".actuators", "joint '" + profile.joints[joint].name + "' has no actuator");
        }
        config.actuators.push_back(*byJoint[joint]);
    }
    return config;
}

} // namespace

Expected<DriverConfig> parseDriverConfig(RobotProfile const &profile) {
    try {
        return parse(profile);
    } catch (ParseFailure const &failure) {
        return makeError(ErrorCode::InvalidConfig, failure.message);
    }
}

Duration cycleBusTime(DriverConfig const &config) noexcept {
    auto const frames = 2.0 * static_cast<double>(config.actuators.size());
    return fromSeconds(frames * kBitsPerFrame / static_cast<double>(config.bitrate));
}

} // namespace larm::drivers::robstride
