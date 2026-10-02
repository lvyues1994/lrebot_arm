// The remote session against a runtime node over DDS: the same operations as the local session.
#include "TestServer.h"

#include <larm/model/RobotModel.h>
#include <larm/ros/RemoteSession.h>

#include <gtest/gtest.h>

#include <functional>
#include <thread>

namespace larm::ros {
namespace {

using runtime::MotionError;
using runtime::MotionFailure;

JointVector armTarget(double const base) {
    auto q = JointVector{6};
    q << base, 1.0, 1.4, -0.5, 0.4, 1.0;
    return q;
}

bool eventually(std::function<bool()> const &condition) {
    for (int i = 0; i < 100; ++i) {
        if (condition()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{20});
    }
    return condition();
}

template <class Sender> MotionError expectMotionError(Sender &&sender) {
    try {
        lexec::sync_wait(std::forward<Sender>(sender));
    } catch (MotionError const &error) {
        return error;
    }
    ADD_FAILURE() << "expected a MotionError";
    return MotionError{MotionFailure::Fault, control::FaultCode::None, "none"};
}

struct World {
    World() {
        clientNode = std::make_shared<rclcpp::Node>("larm_remote_client");
        server = std::make_unique<test_support::TestServer>(std::vector{clientNode});
        remote = makeRemoteSession(clientNode, server->profile, {});
        // Discovery: the status topic and every endpoint the tests use.
        if (not eventually([&] { return remote->latest().state.cycle > 0; })) {
            throw std::runtime_error{"no joint states from the runtime node"};
        }
        std::this_thread::sleep_for(std::chrono::seconds{1});
    }

    ~World() {
        server->stop();
        remote.reset();
        server.reset();
        clientNode.reset();
    }

    rclcpp::Node::SharedPtr clientNode;
    std::unique_ptr<test_support::TestServer> server;
    std::unique_ptr<runtime::RobotSession> remote;
};

struct RemoteSessionTest : testing::Test {
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
        world = new World{};
    }
    static void TearDownTestSuite() {
        delete world;
        world = nullptr;
        rclcpp::shutdown();
    }
    static runtime::RobotSession &remote() { return *world->remote; }
    static World *world;
};
World *RemoteSessionTest::world = nullptr;

TEST_F(RemoteSessionTest, EnablesAndFollowsTheRobotState) {
    EXPECT_EQ(remote().profile().name, "rebot_b601_rs");
    ASSERT_TRUE(lexec::sync_wait(remote().enable()));
    EXPECT_TRUE(eventually([] { return remote().latest().power == hal::DrivePower::Enabled; }));
    EXPECT_EQ(remote().motion("gripper"), nullptr);
    EXPECT_NE(remote().gripper("gripper"), nullptr);
}

TEST_F(RemoteSessionTest, MovesToJoints) {
    auto const result = lexec::sync_wait(remote().motion("arm")->moveToJoints({.position = armTarget(0.6)}));
    ASSERT_TRUE(result);
    EXPECT_LT((std::get<0>(*result).position - armTarget(0.6)).cwiseAbs().maxCoeff(), 0.02);
    EXPECT_TRUE(eventually([] { return std::abs(remote().latest().state.joints.position[0] - 0.6) < 0.02; }));
}

TEST_F(RemoteSessionTest, MovesToAPose) {
    auto model = model::loadRobotModel(remote().profile());
    ASSERT_TRUE(model);
    auto kinematics = (*model)->makeKinematics();
    auto const tool = *kinematics->findFrame("gripper_end");
    auto configuration = remote().profile().safety.restPose;
    configuration.head(6) = armTarget(-0.3);
    kinematics->update(configuration);
    auto const target = kinematics->framePose(tool);
    ASSERT_TRUE(lexec::sync_wait(remote().motion("arm")->moveToPose({.target = target})));
    kinematics->update(world->server->session->latest().state.joints.position);
    EXPECT_LT((kinematics->framePose(tool).translation - target.translation).norm(), 0.01);
}

TEST_F(RemoteSessionTest, FollowsAPath) {
    auto path = runtime::JointPath{};
    path.waypoints.push_back({.time = std::chrono::seconds{1}, .position = armTarget(0.0)});
    path.waypoints.push_back({.time = std::chrono::seconds{2}, .position = armTarget(0.3)});
    auto const result = lexec::sync_wait(remote().motion("arm")->followPath(std::move(path)));
    ASSERT_TRUE(result);
    EXPECT_LT(std::abs(std::get<0>(*result).position[0] - 0.3), 0.03);
}

TEST_F(RemoteSessionTest, Grips) {
    auto const result =
        lexec::sync_wait(remote().gripper("gripper")->grip({.position = 0.03, .maxEffort = 10.0}));
    ASSERT_TRUE(result);
    EXPECT_TRUE(std::get<0>(*result).reachedGoal);
}

TEST_F(RemoteSessionTest, StopRequestCancelsTheGoal) {
    auto source = lexec::inplace_stop_source{};
    auto canceller = std::jthread{[&] {
        std::this_thread::sleep_for(std::chrono::milliseconds{400});
        source.request_stop();
    }};
    auto const result = lexec::sync_wait(
        lexec::write_env(remote().motion("arm")->moveToJoints({.position = armTarget(2.0), .speed = 0.3}),
                         lexec::prop{lexec::get_stop_token, source.get_token()}));
    EXPECT_FALSE(result);
}

TEST_F(RemoteSessionTest, ReportsFailuresWithTheirReason) {
    EXPECT_EQ(expectMotionError(remote().disable({})).reason, MotionFailure::InvalidGoal);
    auto beyond = armTarget(0.0);
    beyond[1] = 5.0;
    auto const error = expectMotionError(remote().motion("arm")->moveToJoints({.position = beyond}));
    EXPECT_EQ(error.reason, MotionFailure::InvalidGoal) << error.what();
}

TEST_F(RemoteSessionTest, EmergencyStopsAndResets) {
    remote().emergencyStop();
    EXPECT_TRUE(
        eventually([] { return remote().latest().safety == control::SafetyState::EmergencyStopped; }));
    EXPECT_EQ(remote().latest().fault, control::FaultCode::EmergencyStop);
    ASSERT_TRUE(lexec::sync_wait(remote().resetFault()));
    EXPECT_TRUE(eventually([] { return remote().latest().safety == control::SafetyState::Normal; }));
}

TEST_F(RemoteSessionTest, ParksAndDisables) {
    auto const parked = lexec::sync_wait(remote().park());
    ASSERT_TRUE(parked);
    ASSERT_TRUE(lexec::sync_wait(remote().disable({})));
    EXPECT_TRUE(eventually([] { return remote().latest().power == hal::DrivePower::Disabled; }));
}

} // namespace
} // namespace larm::ros
