// Drives the runtime node over DDS with ROS clients, against the MuJoCo backend.
#include "TestServer.h"

#include <larm/model/RobotModel.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <larm_msgs/action/move_to_joints.hpp>
#include <larm_msgs/action/move_to_pose.hpp>
#include <larm_msgs/msg/arm_status.hpp>
#include <lrclexec/ExecuteAction.h>
#include <lrclexec/Service.h>
#include <lrclexec/SpinWithScope.h>
#include <lrclexec/Topic.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <gtest/gtest.h>

#include <cstdlib>
#include <future>
#include <thread>

namespace larm::ros {
namespace {

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GripperCommand = control_msgs::action::GripperCommand;
using MoveToJoints = larm_msgs::action::MoveToJoints;
using MoveToPose = larm_msgs::action::MoveToPose;
using Trigger = std_srvs::srv::Trigger;

constexpr auto kServer = "/larm_runtime";

std::vector<double> armTarget(double const base) { return {base, 1.0, 1.4, -0.5, 0.4, 1.0}; }

// One runtime, node and set of clients for the whole suite; tests run in order on the same robot.
struct World {
    World() {
        clientNode = std::make_shared<rclcpp::Node>("larm_test_client");
        server = std::make_unique<test_support::TestServer>(std::vector{clientNode});
        client = std::make_unique<lrclexec::TimerScheduler>(clientNode);
        follow = rclcpp_action::create_client<FollowJointTrajectory>(
            clientNode, std::string{kServer} + "/arm/follow_joint_trajectory");
        moveJoints = rclcpp_action::create_client<MoveToJoints>(clientNode,
                                                                std::string{kServer} + "/arm/move_to_joints");
        movePose =
            rclcpp_action::create_client<MoveToPose>(clientNode, std::string{kServer} + "/arm/move_to_pose");
        grip = rclcpp_action::create_client<GripperCommand>(clientNode, std::string{kServer} +
                                                                            "/gripper/gripper_command");
        for (auto const &action : std::vector<rclcpp_action::ClientBase *>{follow.get(), moveJoints.get(),
                                                                           movePose.get(), grip.get()}) {
            if (not action->wait_for_action_server(std::chrono::seconds{10})) {
                throw std::runtime_error{"action server not discovered"};
            }
        }
    }

    ~World() {
        server->stop();
        follow.reset();
        moveJoints.reset();
        movePose.reset();
        grip.reset();
        triggers.clear();
        client.reset();
        server.reset();
        clientNode.reset();
    }

    std::shared_ptr<Trigger::Response> trigger(std::string const &name) {
        auto &service = triggers[name];
        if (not service) {
            service = clientNode->create_client<Trigger>(std::string{kServer} + "/" + name);
        }
        auto const [response] =
            *lexec::sync_wait(lrclexec::call_service(*client, service, Trigger::Request{}));
        return response;
    }

    std::shared_ptr<larm_msgs::msg::ArmStatus const> status() {
        auto const [message] = *lexec::sync_wait(
            lrclexec::wait_message<larm_msgs::msg::ArmStatus>(*client, std::string{kServer} + "/status"));
        return message;
    }

    template <class Action>
    auto execute(std::shared_ptr<rclcpp_action::Client<Action>> const &action, typename Action::Goal goal) {
        return lexec::sync_wait(lrclexec::execute_action(*client, action, std::move(goal)));
    }

    MoveToJoints::Goal jointGoal(std::vector<double> const &positions, double const speed = 0.0) const {
        auto goal = MoveToJoints::Goal{};
        goal.positions = positions;
        goal.speed = speed;
        return goal;
    }

    RobotProfile const &profile() const { return server->profile; }
    runtime::RobotSession &session() const { return *server->session; }

    rclcpp::Node::SharedPtr clientNode;
    std::unique_ptr<test_support::TestServer> server;
    std::unique_ptr<lrclexec::TimerScheduler> client;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr follow;
    rclcpp_action::Client<MoveToJoints>::SharedPtr moveJoints;
    rclcpp_action::Client<MoveToPose>::SharedPtr movePose;
    rclcpp_action::Client<GripperCommand>::SharedPtr grip;
    std::map<std::string, rclcpp::Client<Trigger>::SharedPtr> triggers;
};

struct RuntimeNodeTest : testing::Test {
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
        world = new World{};
    }
    static void TearDownTestSuite() {
        delete world;
        world = nullptr;
        rclcpp::shutdown();
    }
    static World *world;
};
World *RuntimeNodeTest::world = nullptr;

TEST_F(RuntimeNodeTest, PublishesJointStatesWithDescriptionNames) {
    auto const [states] = *lexec::sync_wait(
        lrclexec::wait_message<sensor_msgs::msg::JointState>(*world->client, "/joint_states"));
    ASSERT_EQ(states->name.size(), 7u);
    EXPECT_EQ(states->name.front(), "joint1");
    EXPECT_EQ(states->name.back(), "joint_left");
    EXPECT_EQ(states->position.size(), 7u);
    EXPECT_EQ(states->effort.size(), 7u);
    // Stamped with simulated time, which has advanced since the runtime started.
    EXPECT_GT(rclcpp::Time{states->header.stamp}.nanoseconds(), 0);
}

TEST_F(RuntimeNodeTest, EnablesThroughTheService) {
    EXPECT_FALSE(world->status()->enabled);
    auto const response = world->trigger("enable");
    EXPECT_TRUE(response->success) << response->message;
    EXPECT_TRUE(world->status()->enabled);
}

TEST_F(RuntimeNodeTest, FollowsAJointTrajectory) {
    // Joint names in reverse order; the first point is after time zero, so the path starts from the arm.
    auto goal = FollowJointTrajectory::Goal{};
    goal.trajectory.joint_names = {"joint6", "joint5", "joint4", "joint3", "joint2", "joint1"};
    auto const start = std::vector<double>{0.0, 0.7, 1.1, 0.0, 0.0, 0.0};
    auto const target = armTarget(0.5);
    for (auto const &[seconds, fraction] : {std::pair{1.0, 0.5}, std::pair{2.0, 1.0}}) {
        auto point = trajectory_msgs::msg::JointTrajectoryPoint{};
        point.time_from_start = rclcpp::Duration::from_seconds(seconds);
        for (std::size_t j = start.size(); j-- > 0;) {
            point.positions.push_back(start[j] + fraction * (target[j] - start[j]));
        }
        goal.trajectory.points.push_back(point);
    }
    auto const result = world->execute(world->follow, goal);
    ASSERT_TRUE(result);
    EXPECT_EQ(std::get<0>(*result)->error_code, FollowJointTrajectory::Result::SUCCESSFUL);
    EXPECT_NEAR(world->session().latest().state.joints.position[0], 0.5, 0.02);
}

TEST_F(RuntimeNodeTest, MovesToJointsInTheGoalsOrder) {
    auto goal = world->jointGoal({1.0, 0.8, 1.4, -0.5, 0.4, 1.0});
    goal.joint_names = {"joint2", "joint1", "joint3", "joint4", "joint5", "joint6"};
    auto const result = world->execute(world->moveJoints, goal);
    ASSERT_TRUE(result);
    auto const &positions = std::get<0>(*result)->positions;
    ASSERT_EQ(positions.size(), 6u);
    EXPECT_NEAR(positions[0], 1.0, 0.02);
    EXPECT_NEAR(positions[1], 0.8, 0.02);
}

TEST_F(RuntimeNodeTest, MovesToAPose) {
    auto model = model::loadRobotModel(world->profile());
    ASSERT_TRUE(model);
    auto kinematics = (*model)->makeKinematics();
    auto const tool = *kinematics->findFrame("gripper_end");
    auto configuration = world->profile().safety.restPose;
    configuration << -0.4, 1.0, 1.4, -0.5, 0.4, 1.0, 0.0;
    kinematics->update(configuration);
    auto const pose = kinematics->framePose(tool);

    auto goal = MoveToPose::Goal{};
    goal.target.header.frame_id = "base_link";
    goal.target.pose.position.x = pose.translation.x();
    goal.target.pose.position.y = pose.translation.y();
    goal.target.pose.position.z = pose.translation.z();
    goal.target.pose.orientation.w = pose.rotation.w();
    goal.target.pose.orientation.x = pose.rotation.x();
    goal.target.pose.orientation.y = pose.rotation.y();
    goal.target.pose.orientation.z = pose.rotation.z();
    ASSERT_TRUE(world->execute(world->movePose, goal));
    kinematics->update(world->session().latest().state.joints.position);
    EXPECT_LT((kinematics->framePose(tool).translation - pose.translation).norm(), 0.01);
}

TEST_F(RuntimeNodeTest, GripsThroughGripperCommand) {
    auto goal = GripperCommand::Goal{};
    goal.command.position = 0.04;
    goal.command.max_effort = 10.0;
    auto const result = world->execute(world->grip, goal);
    ASSERT_TRUE(result);
    EXPECT_TRUE(std::get<0>(*result)->reached_goal);
    EXPECT_NEAR(std::get<0>(*result)->position, 0.04, 0.002);
}

TEST_F(RuntimeNodeTest, AbortsInvalidGoalsAndReportsWhy) {
    auto goal = world->jointGoal(armTarget(0.0));
    goal.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "elbow"};
    EXPECT_THROW(world->execute(world->moveJoints, goal), lrclexec::ActionError<MoveToJoints>);
    std::this_thread::sleep_for(std::chrono::milliseconds{200});
    EXPECT_NE(world->status()->last_error.find("elbow"), std::string::npos) << world->status()->last_error;
    auto const count = world->status()->error_count;
    EXPECT_GE(count, 1u);

    EXPECT_THROW(world->execute(world->moveJoints, goal), lrclexec::ActionError<MoveToJoints>);
    std::this_thread::sleep_for(std::chrono::milliseconds{200});
    EXPECT_EQ(world->status()->error_count, count + 1);
}

TEST_F(RuntimeNodeTest, NewGoalPreemptsTheRunningOne) {
    auto first = std::async(std::launch::async, [] {
        return world->execute(world->moveJoints, world->jointGoal(armTarget(2.0), 0.3));
    });
    std::this_thread::sleep_for(std::chrono::milliseconds{500});
    auto const second = world->execute(world->moveJoints, world->jointGoal(armTarget(0.2)));
    EXPECT_THROW(first.get(), lrclexec::ActionError<MoveToJoints>);
    ASSERT_TRUE(second);
    EXPECT_NEAR(std::get<0>(*second)->positions[0], 0.2, 0.02);
}

TEST_F(RuntimeNodeTest, CancelStopsTheMotion) {
    auto source = lexec::inplace_stop_source{};
    auto canceller = std::jthread{[&] {
        std::this_thread::sleep_for(std::chrono::milliseconds{500});
        source.request_stop();
    }};
    auto const result = lexec::sync_wait(lexec::write_env(
        lrclexec::execute_action(*world->client, world->moveJoints, world->jointGoal(armTarget(2.0), 0.3)),
        lexec::prop{lexec::get_stop_token, source.get_token()}));
    EXPECT_FALSE(result);
    EXPECT_LT(world->session().latest().state.joints.position[0], 1.9);
}

TEST_F(RuntimeNodeTest, EmergencyStopAbortsUntilReset) {
    auto moving = std::async(std::launch::async, [] {
        return world->execute(world->moveJoints, world->jointGoal(armTarget(-1.0), 0.3));
    });
    std::this_thread::sleep_for(std::chrono::milliseconds{500});
    EXPECT_TRUE(world->trigger("emergency_stop")->success);
    EXPECT_THROW(moving.get(), lrclexec::ActionError<MoveToJoints>);
    EXPECT_EQ(world->status()->safety, larm_msgs::msg::ArmStatus::SAFETY_EMERGENCY_STOPPED);
    auto const reset = world->trigger("reset_fault");
    EXPECT_TRUE(reset->success) << reset->message;
    EXPECT_EQ(world->status()->safety, larm_msgs::msg::ArmStatus::SAFETY_NORMAL);
}

TEST_F(RuntimeNodeTest, ParksAndDisables) {
    auto const early = world->trigger("disable");
    EXPECT_FALSE(early->success);
    auto const parked = world->trigger("park");
    EXPECT_TRUE(parked->success) << parked->message;
    auto const disabled = world->trigger("disable");
    EXPECT_TRUE(disabled->success) << disabled->message;
    EXPECT_FALSE(world->status()->enabled);
}

} // namespace
} // namespace larm::ros
