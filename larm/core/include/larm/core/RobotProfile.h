#pragma once

#include <larm/core/ConfigNode.h>
#include <larm/core/Error.h>
#include <larm/core/JointVector.h>
#include <larm/core/Time.h>

#include <cstdint>
#include <filesystem>
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

struct JointGroupSpec {
    std::string name;
    std::vector<std::size_t> joints;
    std::string baseFrame;
    std::string toolFrame;
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
    Duration controlPeriod{};
    std::vector<JointSpec> joints;
    std::vector<JointGroupSpec> groups;
    SafetySpec safety;
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
