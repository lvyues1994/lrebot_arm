#pragma once

#include <larm/ros/RuntimeNode.h>
#include <larm/runtime/LocalRuntime.h>
#include <larm/sim/Simulation.h>

#include <lrclexec/SpinWithScope.h>
#include <rclcpp/rclcpp.hpp>

#include <cstdlib>
#include <stdexcept>
#include <thread>
#include <vector>

namespace larm::ros::test_support {

// The simulated runtime served by a runtime node named /larm_runtime, spinning together with the
// test's client nodes on one executor thread.
struct TestServer {
    explicit TestServer(std::vector<rclcpp::Node::SharedPtr> const &clients) {
        auto const *const path = std::getenv("LARM_ROBOT_PROFILE");
        auto loaded = loadRobotProfile(path == nullptr ? "" : path);
        if (not loaded) {
            throw std::runtime_error{loaded.error().message};
        }
        profile = *loaded;
        auto simulation =
            sim::makeSimulation(profile, {.pacing = sim::Pacing::RealTime, .realTimeFactor = 4.0});
        if (not simulation) {
            throw std::runtime_error{simulation.error().message};
        }
        auto start = profile.safety.restPose;
        start << 0.0, 0.7, 1.1, 0.0, 0.0, 0.0, 0.02;
        (*simulation)->reset(start);
        auto started = runtime::startLocalRuntime(profile, std::move(*simulation), {});
        if (not started) {
            throw std::runtime_error{started.error().message};
        }
        session = std::move(*started);
        serverNode = std::make_shared<rclcpp::Node>("larm_runtime");
        runtimeNode = makeRuntimeNode(serverNode, *session, scope, {.publishClock = true});
        executor.add_node(serverNode);
        for (auto const &client : clients) {
            executor.add_node(client);
        }
        spinner = std::thread{[this] { lrclexec::spin_with_scope(executor, scope, stopSource.get_token()); }};
    }

    TestServer(TestServer const &) = delete;
    TestServer &operator=(TestServer const &) = delete;

    ~TestServer() { stop(); }

    // Closes the node and stops the executor; clients may be destroyed afterwards.
    void stop() {
        if (not spinner.joinable()) {
            return;
        }
        runtimeNode->close();
        stopSource.request_stop();
        spinner.join();
    }

    RobotProfile profile;
    std::unique_ptr<runtime::RobotSession> session;
    lexec::counting_scope scope;
    lexec::inplace_stop_source stopSource;
    rclcpp::Node::SharedPtr serverNode;
    std::unique_ptr<RuntimeNode> runtimeNode;
    rclcpp::executors::SingleThreadedExecutor executor;
    std::thread spinner;
};

} // namespace larm::ros::test_support
