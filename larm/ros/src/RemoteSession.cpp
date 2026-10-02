#include "JointNames.h"

#include <larm/ros/RemoteSession.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <larm_msgs/action/move_to_joints.hpp>
#include <larm_msgs/action/move_to_pose.hpp>
#include <larm_msgs/msg/arm_status.hpp>
#include <lrclexec/ExecuteAction.h>
#include <lrclexec/Service.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <map>
#include <mutex>

namespace larm::ros {
namespace {

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GripperCommand = control_msgs::action::GripperCommand;
using MoveToJoints = larm_msgs::action::MoveToJoints;
using MoveToPose = larm_msgs::action::MoveToPose;
using ArmStatus = larm_msgs::msg::ArmStatus;
using Trigger = std_srvs::srv::Trigger;
using runtime::MotionError;
using runtime::MotionFailure;

template <class Action> using ActionClient = typename rclcpp_action::Client<Action>::SharedPtr;

// Longer than the server's status period, so the status after a failure has arrived.
constexpr auto kStatusGrace = std::chrono::milliseconds{200};

// The runtime node reports failures as "<reason>: <details>".
MotionError fromMessage(std::string const &message) {
    auto const colon = message.find(": ");
    if (colon != std::string::npos) {
        if (auto const reason = runtime::failureFromString(std::string_view{message}.substr(0, colon))) {
            return MotionError{*reason, control::FaultCode::None, message.substr(colon + 2)};
        }
    }
    return MotionError{MotionFailure::Fault, control::FaultCode::None, message};
}

MotionError unreachable() {
    return MotionError{MotionFailure::Fault, control::FaultCode::None, "the runtime node is not reachable"};
}

JointVector toJointVector(std::vector<double> const &values) {
    auto vector = zeroJointVector(values.size());
    std::copy(values.begin(), values.end(), vector.begin());
    return vector;
}

std::vector<double> toStd(JointVector const &vector) { return {vector.begin(), vector.end()}; }

struct RemoteSessionImpl;

struct RemoteMotion final : runtime::MotionApi {
    RemoteMotion(RemoteSessionImpl *session_, GroupJoints joints_, JointGroupSpec const &spec,
                 rclcpp::Node &node, std::string const &prefix);

    runtime::Async<runtime::MotionResult> moveToJoints(runtime::JointGoal goal) override;
    runtime::Async<runtime::MotionResult> moveToPose(runtime::PoseGoal goal) override;
    runtime::Async<runtime::MotionResult> followPath(runtime::JointPath path) override;

  private:
    RemoteSessionImpl *session;
    GroupJoints joints;
    std::string baseFrame;
    ActionClient<MoveToJoints> moveJoints;
    ActionClient<MoveToPose> movePose;
    ActionClient<FollowJointTrajectory> follow;
};

struct RemoteGripper final : runtime::GripperApi {
    RemoteGripper(RemoteSessionImpl *session_, rclcpp::Node &node, std::string const &name)
        : session{session_}, command{rclcpp_action::create_client<GripperCommand>(&node, name)} {}

    runtime::Async<runtime::GripResult> grip(runtime::GripGoal goal) override;

  private:
    RemoteSessionImpl *session;
    ActionClient<GripperCommand> command;
};

struct RemoteSessionImpl final : runtime::RobotSession {
    RemoteSessionImpl(rclcpp::Node::SharedPtr node_, RobotProfile profile_,
                      RemoteSessionOptions const &options)
        : node{std::move(node_)}, robot{std::move(profile_)}, scheduler{node} {
        snapshot.state = RobotState::zero(robot.dof());
        snapshot.command = JointCommand::zero(robot.dof());
        for (std::size_t i = 0; i < robot.dof(); ++i) {
            jointIndex[robot.joints[i].descriptionJoint] = i;
            jointIndex[robot.joints[i].name] = i;
        }
        jointStates = node->create_subscription<sensor_msgs::msg::JointState>(
            options.jointStates, rclcpp::SystemDefaultsQoS{},
            [this](sensor_msgs::msg::JointState::ConstSharedPtr const message) { onJointStates(*message); });
        status = node->create_subscription<ArmStatus>(
            options.server + "/status", rclcpp::SystemDefaultsQoS{},
            [this](ArmStatus::ConstSharedPtr const message) { onStatus(*message); });
        for (auto const *const name : {"enable", "disable", "park", "reset_fault", "emergency_stop"}) {
            triggers[name] = node->create_client<Trigger>(options.server + "/" + name);
        }
        for (auto const &group : robot.groups) {
            auto const prefix = options.server + "/" + group.name;
            motions.push_back(group.toolFrame.empty()
                                  ? nullptr
                                  : std::make_unique<RemoteMotion>(this, GroupJoints::of(robot, group), group,
                                                                   *node, prefix));
            grippers.push_back(group.joints.size() == 1
                                   ? std::make_unique<RemoteGripper>(this, *node, prefix + "/gripper_command")
                                   : nullptr);
        }
    }

    RobotProfile const &profile() const noexcept override { return robot; }
    runtime::Async<> enable() override { return trigger("enable"); }

    runtime::Async<> disable(runtime::DisableOptions const options) override {
        if (options.force) {
            return lexec::just_error(
                std::make_exception_ptr(MotionError{MotionFailure::InvalidGoal, control::FaultCode::None,
                                                    "a forced disable is not offered remotely"}));
        }
        return trigger("disable");
    }

    runtime::Async<runtime::MotionResult> park() override {
        return trigger("park") | lexec::then([this] {
                   return runtime::MotionResult{.position = latest().state.joints.position};
               });
    }

    runtime::Async<> resetFault() override { return trigger("reset_fault"); }

    void emergencyStop() noexcept override {
        try {
            triggers.at("emergency_stop")->async_send_request(std::make_shared<Trigger::Request>());
        } catch (std::exception const &exception) {
            RCLCPP_ERROR(node->get_logger(), "emergency stop request failed: %s", exception.what());
        }
    }

    control::RobotSnapshot latest() const override {
        auto const lock = std::lock_guard{mutex};
        return snapshot;
    }

    runtime::MotionApi *motion(std::string_view const group) override {
        auto const index = robot.findGroup(group);
        return index ? motions[*index].get() : nullptr;
    }

    runtime::GripperApi *gripper(std::string_view const group) override {
        auto const index = robot.findGroup(group);
        return index ? grippers[*index].get() : nullptr;
    }

    // Sends a goal once its server is discovered. An abort carries no reason, so after it the session
    // waits for the server's next status and reports the error it recorded, when it recorded a new one.
    // The trailing let_error also re-sends errors as rvalues: lrclexec completes with lvalue errors,
    // which a type-erased receiver rejects.
    template <class Result, class Action, class Map>
    runtime::Async<Result> act(std::shared_ptr<rclcpp_action::Client<Action>> const &client,
                               typename Action::Goal goal, Map map) {
        return lexec::just() | lexec::let_value([this, client, goal = std::move(goal)] {
                   if (not client->action_server_is_ready()) {
                       throw unreachable();
                   }
                   return lrclexec::execute_action(scheduler, client, goal);
               }) |
               lexec::then(std::move(map)) |
               lexec::let_error([this, before = lastError()](auto const &error) {
                   return lrclexec::schedule_after(scheduler, kStatusGrace) |
                          lexec::let_value([this, error, before] {
                              return lexec::just_error(exceptionFor(error, before));
                          });
               }) |
               lexec::let_error([](std::exception_ptr const &error) { return lexec::just_error(error); });
    }

  private:
    runtime::Async<> trigger(std::string const &name) {
        return lexec::just() | lexec::let_value([this, client = triggers.at(name)] {
                   if (not client->service_is_ready()) {
                       throw unreachable();
                   }
                   return lrclexec::call_service(scheduler, client, Trigger::Request{});
               }) |
               lexec::then([](std::shared_ptr<Trigger::Response> const &response) {
                   if (not response->success) {
                       throw fromMessage(response->message);
                   }
               }) |
               lexec::let_error([](std::exception_ptr const &error) { return lexec::just_error(error); });
    }

    std::string lastError() const {
        auto const lock = std::lock_guard{mutex};
        return lastReportedError;
    }

    static std::exception_ptr exceptionFor(std::exception_ptr const &error, std::string const &) {
        return error;
    }

    template <class Action>
    std::exception_ptr exceptionFor(lrclexec::ActionError<Action> const &error,
                                    std::string const &before) const {
        if (error.kind == lrclexec::ActionErrorKind::rejected) {
            return std::make_exception_ptr(
                MotionError{MotionFailure::Fault, control::FaultCode::None, "the runtime rejected the goal"});
        }
        auto const reported = lastError();
        if (reported != before and not reported.empty()) {
            auto const colon = reported.find(": ");
            return std::make_exception_ptr(
                fromMessage(colon == std::string::npos ? reported : reported.substr(colon + 2)));
        }
        return std::make_exception_ptr(MotionError{MotionFailure::Fault, control::FaultCode::None,
                                                   "the runtime aborted the goal (preempted or stopped)"});
    }

    void onJointStates(sensor_msgs::msg::JointState const &message) {
        auto const lock = std::lock_guard{mutex};
        auto &joints = snapshot.state.joints;
        for (std::size_t i = 0; i < message.name.size(); ++i) {
            auto const found = jointIndex.find(message.name[i]);
            if (found == jointIndex.end()) {
                continue;
            }
            auto const j = idx(found->second);
            joints.position[j] = message.position.at(i);
            joints.velocity[j] = i < message.velocity.size() ? message.velocity[i] : 0.0;
            joints.effort[j] = i < message.effort.size() ? message.effort[i] : 0.0;
        }
        snapshot.state.stamp = TimePoint{Duration{rclcpp::Time{message.header.stamp}.nanoseconds()}};
        ++snapshot.state.cycle;
    }

    void onStatus(ArmStatus const &message) {
        auto const lock = std::lock_guard{mutex};
        snapshot.power = message.enabled ? hal::DrivePower::Enabled : hal::DrivePower::Disabled;
        snapshot.state.isFresh = message.feedback_fresh;
        snapshot.safety = message.safety <= ArmStatus::SAFETY_EMERGENCY_STOPPED
                              ? static_cast<control::SafetyState>(message.safety)
                              : control::SafetyState::Faulted;
        snapshot.fault = control::faultFromString(message.fault).value_or(control::FaultCode::None);
        snapshot.activeCount = 0;
        for (auto const &name : message.active_groups) {
            auto const group = robot.findGroup(name);
            if (group and snapshot.activeCount < snapshot.active.size()) {
                auto mask = JointMask{};
                for (auto const joint : robot.groups[*group].joints) {
                    mask.set(joint);
                }
                snapshot.active[snapshot.activeCount++] = control::ActiveGoal{.joints = mask};
            }
        }
        for (std::size_t i = 0; i < robot.dof(); ++i) {
            snapshot.state.actuators[i].enabled = message.enabled;
        }
        lastReportedError = message.last_error;
    }

    rclcpp::Node::SharedPtr node;
    RobotProfile robot;
    lrclexec::TimerScheduler scheduler;
    std::map<std::string, std::size_t> jointIndex;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStates;
    rclcpp::Subscription<ArmStatus>::SharedPtr status;
    std::map<std::string, rclcpp::Client<Trigger>::SharedPtr> triggers;
    std::vector<std::unique_ptr<RemoteMotion>> motions;
    std::vector<std::unique_ptr<RemoteGripper>> grippers;
    mutable std::mutex mutex;
    control::RobotSnapshot snapshot;
    std::string lastReportedError;
};

RemoteMotion::RemoteMotion(RemoteSessionImpl *const session_, GroupJoints joints_, JointGroupSpec const &spec,
                           rclcpp::Node &node, std::string const &prefix)
    : session{session_}, joints{std::move(joints_)}, baseFrame{spec.baseFrame},
      moveJoints{rclcpp_action::create_client<MoveToJoints>(&node, prefix + "/move_to_joints")},
      movePose{rclcpp_action::create_client<MoveToPose>(&node, prefix + "/move_to_pose")},
      follow{
          rclcpp_action::create_client<FollowJointTrajectory>(&node, prefix + "/follow_joint_trajectory")} {}

runtime::Async<runtime::MotionResult> RemoteMotion::moveToJoints(runtime::JointGoal goal) {
    auto message = MoveToJoints::Goal{};
    message.joint_names = joints.names;
    message.positions = toStd(goal.position);
    message.speed = goal.speed;
    return session->act<runtime::MotionResult>(
        moveJoints, std::move(message), [](MoveToJoints::Result::SharedPtr const &result) {
            return runtime::MotionResult{.position = toJointVector(result->positions)};
        });
}

runtime::Async<runtime::MotionResult> RemoteMotion::moveToPose(runtime::PoseGoal goal) {
    auto message = MoveToPose::Goal{};
    message.target.header.frame_id = baseFrame;
    message.target.pose.position.x = goal.target.translation.x();
    message.target.pose.position.y = goal.target.translation.y();
    message.target.pose.position.z = goal.target.translation.z();
    message.target.pose.orientation.w = goal.target.rotation.w();
    message.target.pose.orientation.x = goal.target.rotation.x();
    message.target.pose.orientation.y = goal.target.rotation.y();
    message.target.pose.orientation.z = goal.target.rotation.z();
    message.speed = goal.speed;
    return session->act<runtime::MotionResult>(
        movePose, std::move(message), [](MoveToPose::Result::SharedPtr const &result) {
            return runtime::MotionResult{.position = toJointVector(result->positions)};
        });
}

runtime::Async<runtime::MotionResult> RemoteMotion::followPath(runtime::JointPath path) {
    auto message = FollowJointTrajectory::Goal{};
    message.trajectory.joint_names = joints.names;
    for (auto const &waypoint : path.waypoints) {
        auto point = trajectory_msgs::msg::JointTrajectoryPoint{};
        point.time_from_start = rclcpp::Duration{waypoint.time};
        point.positions = toStd(waypoint.position);
        if (waypoint.velocity) {
            point.velocities = toStd(*waypoint.velocity);
        }
        if (waypoint.acceleration) {
            point.accelerations = toStd(*waypoint.acceleration);
        }
        message.trajectory.points.push_back(std::move(point));
    }
    // FollowJointTrajectory reports no positions; the result reads the latest joint states.
    return session->act<runtime::MotionResult>(follow, std::move(message),
                                               [remote = session, indices = joints.profileIndices](
                                                   FollowJointTrajectory::Result::SharedPtr const &) {
                                                   auto const full = remote->latest().state.joints.position;
                                                   auto position = zeroJointVector(indices.size());
                                                   for (std::size_t i = 0; i < indices.size(); ++i) {
                                                       position[idx(i)] = full[idx(indices[i])];
                                                   }
                                                   return runtime::MotionResult{.position = position};
                                               });
}

runtime::Async<runtime::GripResult> RemoteGripper::grip(runtime::GripGoal goal) {
    auto message = GripperCommand::Goal{};
    message.command.position = goal.position;
    message.command.max_effort = goal.maxEffort;
    return session->act<runtime::GripResult>(
        command, std::move(message), [](GripperCommand::Result::SharedPtr const &result) {
            return runtime::GripResult{.position = result->position,
                                       .effort = result->effort,
                                       .reachedGoal = result->reached_goal,
                                       .stalled = result->stalled};
        });
}

} // namespace

std::unique_ptr<runtime::RobotSession> makeRemoteSession(rclcpp::Node::SharedPtr node, RobotProfile profile,
                                                         RemoteSessionOptions const &options) {
    return std::make_unique<RemoteSessionImpl>(std::move(node), std::move(profile), options);
}

} // namespace larm::ros
