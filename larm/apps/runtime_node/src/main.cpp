// The larm runtime as a ROS 2 process: control loop, session and ROS interfaces.
//
// Parameters:
//   profile          robot profile file (required)
//   backend          "mujoco"; "robstride" for the arm on the profile's CAN interface; "robstride_simulated"
//                    for the RobStride driver in front of simulated motors
//   real_time_factor simulated seconds per wall-clock second (mujoco)
//   start_position   initial joint positions (mujoco, robstride_simulated); the profile's rest pose when
//   empty rt_priority      SCHED_FIFO priority of the control thread; 0 keeps normal scheduling
#include <larm/ros/RuntimeNode.h>
#include <larm/runtime/LocalRuntime.h>
#include <larm/sim/Simulation.h>

#ifdef LARM_HAS_ROBSTRIDE
#include <larm/drivers/robstride/Backend.h>
#include <larm/model/RobotModel.h>
#endif

#include <lrclexec/SignalStop.h>
#include <lrclexec/SpinWithScope.h>
#include <rclcpp/rclcpp.hpp>

namespace {

struct Backend {
    std::unique_ptr<larm::hal::Backend> backend;
    // Simulation time is published as /clock; hardware runs on the wall clock.
    bool simulated{};
};

larm::Expected<std::vector<double>> startPosition(rclcpp::Node &node, larm::RobotProfile const &profile) {
    auto start = node.declare_parameter<std::vector<double>>("start_position", std::vector<double>{});
    if (start.empty()) {
        return std::vector<double>(profile.safety.restPose.begin(), profile.safety.restPose.end());
    }
    if (start.size() != profile.dof()) {
        return larm::makeError(larm::ErrorCode::InvalidArgument, "start_position needs one value per joint");
    }
    return start;
}

larm::Expected<Backend> makeBackend(rclcpp::Node &node, larm::RobotProfile const &profile) {
    auto const name = node.declare_parameter<std::string>("backend", "mujoco");
    if (name == "mujoco") {
        auto const factor = node.declare_parameter<double>("real_time_factor", 1.0);
        auto const start = startPosition(node, profile);
        if (not start) {
            return tl::make_unexpected(start.error());
        }
        auto simulation = larm::sim::makeSimulation(
            profile, {.pacing = larm::sim::Pacing::RealTime, .realTimeFactor = factor});
        if (not simulation) {
            return tl::make_unexpected(simulation.error());
        }
        (*simulation)
            ->reset(
                Eigen::Map<Eigen::VectorXd const>(start->data(), static_cast<Eigen::Index>(start->size())));
        return Backend{.backend = std::move(*simulation), .simulated = true};
    }
#ifdef LARM_HAS_ROBSTRIDE
    if (name == "robstride") {
        auto backend = larm::drivers::robstride::makeRobStrideBackend(profile);
        if (not backend) {
            return tl::make_unexpected(backend.error());
        }
        return Backend{.backend = std::move(*backend), .simulated = false};
    }
    if (name == "robstride_simulated") {
        auto const start = startPosition(node, profile);
        auto const config = larm::drivers::robstride::parseDriverConfig(profile);
        auto const model = larm::model::loadRobotModel(profile);
        if (not start or not config or not model) {
            return tl::make_unexpected(not start    ? start.error()
                                       : not config ? config.error()
                                                    : model.error());
        }
        auto motorPosition = std::vector<double>{};
        for (auto const &actuator : config->actuators) {
            auto const &[scale, offset] = actuator.transmission;
            motorPosition.push_back(((*start)[actuator.joint] - offset) / scale);
        }
        // The simulated motors carry the arm's weight, which the gravity feed-forward cancels.
        auto const dynamics = std::shared_ptr<larm::model::Dynamics>{(*model)->makeDynamics()};
        auto load = larm::drivers::robstride::jointSpaceLoad(
            *config, [dynamics](larm::JointVector const &position, larm::JointVector &torque) {
                dynamics->gravity(position, torque);
                torque *= -1.0;
            });
        auto backend = larm::drivers::robstride::makeSimulatedRobStrideBackend(
            profile, {.load = std::move(load), .position = std::move(motorPosition)});
        if (not backend) {
            return tl::make_unexpected(backend.error());
        }
        return Backend{.backend = std::move(*backend), .simulated = false};
    }
#endif
    return larm::makeError(larm::ErrorCode::Unsupported, "unknown backend '" + name + "'");
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
    auto session = larm::runtime::startLocalRuntime(*profile, std::move(backend->backend), options);
    if (not session) {
        RCLCPP_FATAL(node->get_logger(), "%s", session.error().message.c_str());
        return 1;
    }

    auto scope = lexec::counting_scope{};
    auto runtimeNode =
        larm::ros::makeRuntimeNode(node, **session, scope, {.publishClock = backend->simulated});
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
