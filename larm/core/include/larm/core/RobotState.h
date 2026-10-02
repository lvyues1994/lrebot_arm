#pragma once

#include <larm/core/JointVector.h>
#include <larm/core/Time.h>

#include <array>
#include <cstdint>

namespace larm {

struct JointState {
    JointVector position;
    JointVector velocity;
    JointVector effort;

    static JointState zero(std::size_t dof);
};

// Joint impedance command, a superset of the MIT protocol:
// effort = stiffness * (position - q) + damping * (velocity - dq) + effort feed-forward.
struct JointCommand {
    JointVector position;
    JointVector velocity;
    JointVector effort;
    JointVector stiffness;
    JointVector damping;

    static JointCommand zero(std::size_t dof);
};

struct ActuatorStatus {
    bool enabled{};
    std::uint32_t faultBits{};
    float temperature{};
};

struct RobotState {
    std::uint64_t cycle{};
    TimePoint stamp{};
    JointState joints;
    std::array<ActuatorStatus, kMaxDof> actuators{};
    // Every joint reported this cycle and no reading is stale.
    bool isFresh{};

    static RobotState zero(std::size_t dof);
};

} // namespace larm
