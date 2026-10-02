#pragma once

#include <larm/runtime/RobotSession.h>

#include <lexec/execution.hpp>
#include <rclcpp/node.hpp>

#include <memory>

namespace larm::ros {

struct RuntimeNodeOptions {
    // Period of /joint_states and, when published, /clock.
    Duration statePeriod = std::chrono::milliseconds{10};
    // Period of ~/status.
    Duration statusPeriod = std::chrono::milliseconds{100};
    // Publish /clock from the robot timeline and stamp joint states with it; set for simulation.
    bool publishClock{};
};

// Serves a RobotSession on `node`:
//   /joint_states, ~/status, and /clock when requested;
//   per group with a tool frame: ~/<group>/follow_joint_trajectory, ~/<group>/move_to_joints,
//   ~/<group>/move_to_pose; per single-joint group: ~/<group>/gripper_command;
//   services ~/enable, ~/disable, ~/park, ~/reset_fault, ~/emergency_stop.
// All work runs in `scope`. Call close() when shutting down, then drain the scope with
// lrclexec::spin_with_scope before destroying this object, the node or the session.
struct RuntimeNode {
    virtual ~RuntimeNode() = default;
    // Stops accepting goals and stops the running ones. Safe to call from any thread.
    virtual void close() = 0;
};

std::unique_ptr<RuntimeNode> makeRuntimeNode(rclcpp::Node::SharedPtr node, runtime::RobotSession &session,
                                             lexec::counting_scope &scope, RuntimeNodeOptions const &options);

} // namespace larm::ros
