#pragma once

#include <larm/core/ConfigNode.h>
#include <larm/core/Error.h>
#include <larm/core/JointVector.h>
#include <larm/core/Pose.h>
#include <larm/core/Time.h>

#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace larm {

enum class JointUnit : std::uint8_t { Radian, Meter };

struct JointLimits {
    double lower{};
    double upper{};
    double velocity{};
    double acceleration{};
    double jerk{};
    double effort{};
};

struct JointGains {
    double stiffness{};
    double damping{};
};

struct JointSpec {
    std::string name;
    // Joint name in the URDF and MJCF; usually equal to `name`.
    std::string descriptionJoint;
    JointUnit unit{};
    JointLimits limits;
    JointGains gains;
};

// Limits of a tool center point moving along a straight line.
struct CartesianLimits {
    double linearVelocity{};
    double linearAcceleration{};
    double angularVelocity{};
    double angularAcceleration{};
};

struct JointGroupSpec {
    std::string name;
    std::vector<std::size_t> joints;
    std::string baseFrame;
    std::string toolFrame;
    // The tool center point in the tool frame; pose goals and Cartesian motions refer to it.
    Pose3 tcp;
    // Straight-line motions need them.
    std::optional<CartesianLimits> cartesianLimits;
};

struct SafetySpec {
    // Measured position beyond a limit by more than this is a system fault.
    double limitTolerance{};
    std::uint32_t feedbackTimeoutCycles{};
    double trackingErrorRadian{};
    double trackingErrorMeter{};
    Duration enableRamp{};
    JointVector restPose;
};

struct RobotProfile {
    std::string name;
    std::filesystem::path urdf;
    std::filesystem::path mjcf;
    // Link pairs whose collisions are not checked (disable_collisions); empty when the profile has none.
    std::filesystem::path srdf;
    Duration controlPeriod{};
    std::vector<JointSpec> joints;
    std::vector<JointGroupSpec> groups;
    SafetySpec safety;
    // Named configurations within the limits, one value per joint; "ready" is where work starts.
    std::map<std::string, JointVector, std::less<>> poses;
    ConfigNode sim;
    ConfigNode driver;

    std::size_t dof() const noexcept { return joints.size(); }
    std::optional<std::size_t> findJoint(std::string_view jointName) const;
    std::optional<std::size_t> findGroup(std::string_view groupName) const;
};

// Parses and validates a profile; unknown keys are rejected.
Expected<RobotProfile> loadRobotProfile(std::filesystem::path const &file);
Expected<RobotProfile> parseRobotProfile(std::string const &yaml, std::filesystem::path const &baseDirectory);

} // namespace larm
