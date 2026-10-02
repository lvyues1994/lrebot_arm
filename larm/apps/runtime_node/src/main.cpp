// The larm runtime as a ROS 2 process: control loop, session and ROS interfaces.
//
// Parameters:
//   profile          robot profile file (required)
//   backend          "mujoco" (the only backend until the hardware drivers land)
//   real_time_factor simulated seconds per wall-clock second
//   start_position   initial joint positions for simulation; the profile's rest pose when empty
//   rt_priority      SCHED_FIFO priority of the control thread; 0 keeps normal scheduling
#include <larm/ros/RuntimeNode.h>
#include <larm/runtime/LocalRuntime.h>
#include <larm/sim/Simulation.h>

#include <lrclexec/SignalStop.h>
#include <lrclexec/SpinWithScope.h>
#include <rclcpp/rclcpp.hpp>

namespace {

larm::Expected<std::unique_ptr<larm::hal::Backend>> makeBackend(rclcpp::Node &node,
                                                                larm::RobotProfile const &profile) {
    auto const backend = node.declare_parameter<std::string>("backend", "mujoco");
    if (backend != "mujoco") {
        return larm::makeError(larm::ErrorCode::Unsupported, "unknown backend '" + backend + "'");
    }
    auto const factor = node.declare_parameter<double>("real_time_factor", 1.0);
    auto const start = node.declare_parameter<std::vector<double>>("start_position", std::vector<double>{});
    auto simulation =
        larm::sim::makeSimulation(profile, {.pacing = larm::sim::Pacing::RealTime, .realTimeFactor = factor});
    if (not simulation) {
        return tl::make_unexpected(simulation.error());
    }
    if (not start.empty()) {
        if (start.size() != profile.dof()) {
            return larm::makeError(larm::ErrorCode::InvalidArgument,
                                   "start_position needs one value per joint");
        }
        (*simulation)
            ->reset(Eigen::Map<Eigen::VectorXd const>(start.data(), static_cast<Eigen::Index>(start.size())));
    }
    return std::unique_ptr<larm::hal::Backend>{std::move(*simulation)};
}

int run(lexec::inplace_stop_source &stop) {
    auto node = std::make_shared<rclcpp::Node>("larm_runtime");
    auto const profilePath = node->declare_parameter<std::string>("profile", "");
    auto profile = larm::loadRobotProfile(profilePath);
    if (not profile) {
        RCLCPP_FATAL(node->get_logger(), "%s", profile.error().message.c_str());
        return 1;
    }
    auto backend = makeBackend(*node, *profile);
    if (not backend) {
        RCLCPP_FATAL(node->get_logger(), "%s", backend.error().message.c_str());
        return 1;
    }
    auto options = larm::runtime::RuntimeOptions{};
    if (auto const priority = node->declare_parameter<int>("rt_priority", 0); priority > 0) {
        options.realtime.priority = priority;
    }
    auto session = larm::runtime::startLocalRuntime(*profile, std::move(*backend), options);
    if (not session) {
        RCLCPP_FATAL(node->get_logger(), "%s", session.error().message.c_str());
        return 1;
    }

    auto scope = lexec::counting_scope{};
    auto runtimeNode = larm::ros::makeRuntimeNode(node, **session, scope, {.publishClock = true});
    auto const closeOnStop = lexec::inplace_stop_callback{stop.get_token(), [&] { runtimeNode->close(); }};
    auto executor = rclcpp::executors::SingleThreadedExecutor{};
    executor.add_node(node);
    RCLCPP_INFO(node->get_logger(), "serving %s", profile->name.c_str());
    lrclexec::spin_with_scope(executor, scope, stop.get_token());
    return 0;
}

} // namespace

int main(int argc, char **argv) {
    auto stop = lexec::inplace_stop_source{};
    auto const signalStop = lrclexec::SignalStop{stop};
    auto initOptions = rclcpp::InitOptions{};
    initOptions.shutdown_on_signal = false;
    rclcpp::init(argc, argv, initOptions, rclcpp::SignalHandlerOptions::None);
    auto const code = run(stop);
    rclcpp::shutdown();
    return code;
}
