#pragma once

#include <larm/runtime/RobotSession.h>

#include <rclcpp/node.hpp>

#include <memory>
#include <string>

namespace larm::ros {

struct RemoteSessionOptions {
    // Fully qualified name of the runtime node.
    std::string server = "/larm_runtime";
    std::string jointStates = "/joint_states";
};

// A RobotSession served by a larm runtime node over ROS 2. `node` must spin on an executor, and
// operations complete on that executor's thread. latest() follows the joint states and the server's
// status; `command` and `effort` details the server does not publish stay zero. Destroy the session
// after the executor has stopped.
std::unique_ptr<runtime::RobotSession> makeRemoteSession(rclcpp::Node::SharedPtr node, RobotProfile profile,
                                                         RemoteSessionOptions const &options);

} // namespace larm::ros
