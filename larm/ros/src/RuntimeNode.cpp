#include "JointNames.h"

#include <larm/ros/RuntimeNode.h>

#include <co2/co2.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <larm_msgs/action/move_to_joints.hpp>
#include <larm_msgs/action/move_to_pose.hpp>
#include <larm_msgs/msg/arm_status.hpp>
#include <lexec/coro/co2.hpp>
#include <lrclexec/ActionServer.h>
#include <lrclexec/TimerScheduler.h>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <functional>
#include <mutex>

namespace larm::ros {
namespace {

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GripperCommand = control_msgs::action::GripperCommand;
using MoveToJoints = larm_msgs::action::MoveToJoints;
using MoveToPose = larm_msgs::action::MoveToPose;
using ArmStatus = larm_msgs::msg::ArmStatus;
using Trigger = std_srvs::srv::Trigger;

template <class Action> using GoalHandle = std::shared_ptr<rclcpp_action::ServerGoalHandle<Action>>;

std::string describe(std::exception_ptr const &error) {
    try {
        std::rethrow_exception(error);
    } catch (runtime::MotionError const &motionError) {
        return std::string{runtime::toString(motionError.reason)} + ": " + motionError.what();
    } catch (std::exception const &exception) {
        return exception.what();
    } catch (...) {
        return "unknown error";
    }
}

builtin_interfaces::msg::Time toTime(Duration const sinceEpoch) { return rclcpp::Time{sinceEpoch.count()}; }

std::vector<double> toStd(JointVector const &vector) { return {vector.begin(), vector.end()}; }

Pose3 toPose(geometry_msgs::msg::Pose const &pose) {
    return Pose3{
        .translation = Eigen::Vector3d{pose.position.x, pose.position.y, pose.position.z},
        .rotation =
            Eigen::Quaterniond{pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z}
                .normalized(),
    };
}

double speedOf(double const requested) { return requested > 0.0 ? requested : 1.0; }

struct RuntimeNodeImpl final : RuntimeNode {
    RuntimeNodeImpl(rclcpp::Node::SharedPtr node_, runtime::RobotSession &session_,
                    lexec::counting_scope &scope_, RuntimeNodeOptions const &options_)
        : node{std::move(node_)}, session{session_}, scope{scope_}, options{options_}, scheduler{node},
          prefix{node->get_fully_qualified_name()} {
        auto const &profile = session.profile();
        for (auto const &joint : profile.joints) {
            jointNames.push_back(joint.descriptionJoint);
        }
        jointStates =
            node->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SystemDefaultsQoS{});
        status = node->create_publisher<ArmStatus>(prefix + "/status", rclcpp::SystemDefaultsQoS{});
        if (options.publishClock) {
            clock = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS{});
        }
        for (auto const &group : profile.groups) {
            auto const joints = GroupJoints::of(profile, group);
            if (auto *const motion = session.motion(group.name)) {
                serveMotion(group, joints, motion);
            }
            if (auto *const gripper = session.gripper(group.name)) {
                serveGripper(group, profile.joints[group.joints.front()].limits.effort, gripper);
            }
        }
        offer("enable", [this] { return session.enable(); });
        offer("disable", [this] { return session.disable({}); });
        offer("park", [this] { return session.park(); });
        offer("reset_fault", [this] { return session.resetFault(); });
        offer("emergency_stop", [this] {
            session.emergencyStop();
            return lexec::just();
        });
    }

    void start();

    void close() override {
        for (auto &closeServer : closers) {
            closeServer();
        }
    }

    void publishState() {
        auto const snapshot = session.latest();
        auto const stamp = options.publishClock ? toTime(snapshot.state.stamp.time_since_epoch())
                                                : builtin_interfaces::msg::Time{node->now()};
        auto joints = sensor_msgs::msg::JointState{};
        joints.header.stamp = stamp;
        joints.name = jointNames;
        joints.position = toStd(snapshot.state.joints.position);
        joints.velocity = toStd(snapshot.state.joints.velocity);
        joints.effort = toStd(snapshot.state.joints.effort);
        jointStates->publish(joints);
        if (clock and snapshot.state.stamp != lastClock) {
            lastClock = snapshot.state.stamp;
            auto message = rosgraph_msgs::msg::Clock{};
            message.clock = stamp;
            clock->publish(message);
        }
        if (++ticks % std::max<std::int64_t>(1, options.statusPeriod / options.statePeriod) == 0) {
            publishStatus(snapshot, stamp);
        }
    }

    lrclexec::TimerScheduler const &timers() const noexcept { return scheduler; }
    Duration statePeriod() const noexcept { return options.statePeriod; }

  private:
    void publishStatus(control::RobotSnapshot const &snapshot, builtin_interfaces::msg::Time const &stamp) {
        auto message = ArmStatus{};
        message.stamp = stamp;
        message.enabled = snapshot.power == hal::DrivePower::Enabled;
        message.feedback_fresh = snapshot.state.isFresh;
        message.safety = static_cast<std::uint8_t>(snapshot.safety);
        message.fault = control::toString(snapshot.fault);
        auto busy = JointMask{};
        for (std::size_t i = 0; i < snapshot.activeCount; ++i) {
            busy |= snapshot.active[i].joints;
        }
        for (auto const &group : session.profile().groups) {
            if (std::any_of(group.joints.begin(), group.joints.end(),
                            [&](std::size_t j) { return busy.test(j); })) {
                message.active_groups.push_back(group.name);
            }
        }
        {
            auto const lock = std::lock_guard{errorMutex};
            message.last_error = lastError;
            message.error_count = errorCount;
        }
        status->publish(message);
    }

    void recordError(std::string const &what, std::string const &message) {
        RCLCPP_WARN(node->get_logger(), "%s failed: %s", what.c_str(), message.c_str());
        auto const lock = std::lock_guard{errorMutex};
        lastError = what + ": " + message;
        ++errorCount;
    }

    // Records why a request was rejected before it reached the session, then rejects it.
    template <class Make> auto guarded(std::string const &what, Make make) {
        try {
            return make();
        } catch (std::exception const &exception) {
            recordError(what, exception.what());
            throw;
        }
    }

    template <class Sender> auto recorded(std::string what, Sender &&sender) {
        return std::forward<Sender>(sender) |
               lexec::let_error([this, what = std::move(what)](std::exception_ptr const &error) {
                   recordError(what, describe(error));
                   return lexec::just_error(error);
               });
    }

    template <class Action, class Factory> void serve(std::string const &name, Factory factory) {
        auto server = lrclexec::make_action_server_preempt<Action>(scheduler, scope, prefix + "/" + name,
                                                                   std::move(factory));
        closers.push_back([server]() mutable { server.close(); });
    }

    void serveMotion(JointGroupSpec const &group, GroupJoints const &joints, runtime::MotionApi *motion) {
        serve<FollowJointTrajectory>(
            group.name + "/follow_joint_trajectory",
            [this, joints, motion](GoalHandle<FollowJointTrajectory> const &handle) {
                auto const goal = handle->get_goal();
                auto path = guarded("follow_joint_trajectory", [&] {
                    auto const &points = goal->trajectory.points;
                    auto positions = std::vector<std::vector<double>>{};
                    auto velocities = std::vector<std::vector<double>>{};
                    auto accelerations = std::vector<std::vector<double>>{};
                    auto times = std::vector<Duration>{};
                    for (auto const &point : points) {
                        positions.push_back(point.positions);
                        velocities.push_back(point.velocities);
                        accelerations.push_back(point.accelerations);
                        times.push_back(Duration{rclcpp::Duration{point.time_from_start}.nanoseconds()});
                    }
                    return toJointPath(joints.order(goal->trajectory.joint_names), positions, velocities,
                                       accelerations, times);
                });
                return recorded("follow_joint_trajectory",
                                motion->followPath(std::move(path)) |
                                    lexec::then([](runtime::MotionResult const &) {
                                        auto result = std::make_shared<FollowJointTrajectory::Result>();
                                        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
                                        return result;
                                    }));
            });

        serve<MoveToJoints>(
            group.name + "/move_to_joints", [this, joints, motion](GoalHandle<MoveToJoints> const &handle) {
                auto const goal = handle->get_goal();
                auto const order = guarded("move_to_joints", [&] {
                    auto picked = joints.order(goal->joint_names);
                    if (goal->positions.size() != picked.size()) {
                        throw std::invalid_argument{"positions do not match the joint names"};
                    }
                    return picked;
                });
                auto target = zeroJointVector(order.size());
                for (std::size_t i = 0; i < order.size(); ++i) {
                    target[idx(i)] = goal->positions[order[i]];
                }
                return recorded("move_to_joints",
                                motion->moveToJoints({.position = target, .speed = speedOf(goal->speed)}) |
                                    lexec::then([order](runtime::MotionResult const &reached) {
                                        auto result = std::make_shared<MoveToJoints::Result>();
                                        result->positions.resize(order.size());
                                        for (std::size_t i = 0; i < order.size(); ++i) {
                                            result->positions[order[i]] = reached.position[idx(i)];
                                        }
                                        return result;
                                    }));
            });

        serve<MoveToPose>(group.name + "/move_to_pose", [this, base = group.baseFrame,
                                                         motion](GoalHandle<MoveToPose> const &handle) {
            auto const goal = handle->get_goal();
            guarded("move_to_pose", [&] {
                auto const &frame = goal->target.header.frame_id;
                if (not frame.empty() and frame != base) {
                    throw std::invalid_argument{"target must be expressed in '" + base + "'"};
                }
                return 0;
            });
            return recorded("move_to_pose", motion->moveToPose({.target = toPose(goal->target.pose),
                                                                .speed = speedOf(goal->speed)}) |
                                                lexec::then([](runtime::MotionResult const &reached) {
                                                    auto result = std::make_shared<MoveToPose::Result>();
                                                    result->positions = toStd(reached.position);
                                                    return result;
                                                }));
        });
    }

    void serveGripper(JointGroupSpec const &group, double const effortLimit, runtime::GripperApi *gripper) {
        serve<GripperCommand>(group.name + "/gripper_command", [this, effortLimit, gripper](
                                                                   GoalHandle<GripperCommand> const &handle) {
            auto const &command = handle->get_goal()->command;
            auto const effort = command.max_effort > 0.0 ? command.max_effort : effortLimit;
            return recorded("gripper_command",
                            gripper->grip({.position = command.position, .maxEffort = effort}) |
                                lexec::then([](runtime::GripResult const &grip) {
                                    auto result = std::make_shared<GripperCommand::Result>();
                                    result->position = grip.position;
                                    result->effort = grip.effort;
                                    result->stalled = grip.stalled;
                                    result->reached_goal = grip.reachedGoal;
                                    return result;
                                }));
        });
    }

    // A Trigger service answered once `work()` completes.
    template <class Work> void offer(std::string const &name, Work work) {
        services.push_back(node->create_service<Trigger>(
            prefix + "/" + name, [this, name, work](std::shared_ptr<rclcpp::Service<Trigger>> const service,
                                                    std::shared_ptr<rmw_request_id_t> const header,
                                                    std::shared_ptr<Trigger::Request> const) {
                auto const respond = [this, name, service, header](bool const success,
                                                                   std::string const &message) noexcept {
                    try {
                        if (not success) {
                            recordError(name, message);
                        }
                        auto response = Trigger::Response{};
                        response.success = success;
                        response.message = message;
                        service->send_response(*header, response);
                    } catch (std::exception const &exception) {
                        RCLCPP_ERROR(node->get_logger(), "%s response failed: %s", name.c_str(),
                                     exception.what());
                    }
                };
                lexec::spawn(work() | lexec::continues_on(scheduler) |
                                 lexec::then([respond](auto &&...) noexcept { respond(true, "done"); }) |
                                 lexec::upon_error([respond](std::exception_ptr const &error) noexcept {
                                     respond(false, describe(error));
                                 }) |
                                 lexec::upon_stopped([respond]() noexcept { respond(false, "stopped"); }),
                             scope.get_token());
            }));
    }

    rclcpp::Node::SharedPtr node;
    runtime::RobotSession &session;
    lexec::counting_scope &scope;
    RuntimeNodeOptions options;
    lrclexec::TimerScheduler scheduler;
    std::string prefix;
    std::vector<std::string> jointNames;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStates;
    rclcpp::Publisher<ArmStatus>::SharedPtr status;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock;
    std::vector<rclcpp::ServiceBase::SharedPtr> services;
    std::vector<std::function<void()>> closers;
    TimePoint lastClock{Duration{-1}};
    std::int64_t ticks{};
    std::mutex errorMutex;
    std::string lastError;
    std::uint32_t errorCount{};
};

// Publishes joint states, the clock and the status until the scope stops it.
auto publishLoop(RuntimeNodeImpl *node) CO2_BEG(co2::Task<>, (node), co2::stop_token token;) {
    CO2_AWAIT_SET(token, co2::getStopToken());
    while (not token.stop_requested()) {
        CO2_AWAIT(lexec::coro::as_awaitable(lrclexec::schedule_after(node->timers(), node->statePeriod())));
        node->publishState();
    }
}
CO2_END

void RuntimeNodeImpl::start() {
    lexec::spawn(lexec::coro::as_sender(publishLoop(this)) |
                     lexec::upon_error([](std::exception_ptr const &) noexcept {}),
                 scope.get_token());
}

} // namespace

std::unique_ptr<RuntimeNode> makeRuntimeNode(rclcpp::Node::SharedPtr node, runtime::RobotSession &session,
                                             lexec::counting_scope &scope,
                                             RuntimeNodeOptions const &options) {
    auto runtimeNode = std::make_unique<RuntimeNodeImpl>(std::move(node), session, scope, options);
    runtimeNode->start();
    return runtimeNode;
}

} // namespace larm::ros
