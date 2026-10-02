#include <larm/core/RobotProfile.h>

#include <gtest/gtest.h>

#include <cstdlib>

namespace larm {
namespace {

std::string const kValidProfile = R"(
robot: test_arm
description: {urdf: robot.urdf, mjcf: scene.xml}
control: {period_us: 2000}
joints:
  - name: a
    limits: {position: [-1.0, 1.0], velocity: 1.0, acceleration: 2.0, jerk: 3.0, effort: 4.0}
    gains: {stiffness: 10.0, damping: 1.0}
  - name: b
    description_joint: b_left
    unit: meter
    limits: {position: [0.0, 0.1], velocity: 1.0, acceleration: 2.0, jerk: 3.0, effort: 4.0}
    gains: {stiffness: 10.0, damping: 1.0}
groups:
  arm: {joints: [a], base: base_link, tool: tool}
  hand: {joints: [b]}
safety:
  limit_tolerance: 0.05
  feedback_timeout_cycles: 3
  tracking_error_limit: {radian: 0.3, meter: 0.02}
  enable_ramp_ms: 100
  rest_pose: [0.0, 0.0]
sim: {physics_step_us: 500}
)";

std::string replace(std::string text, std::string const &from, std::string const &to) {
    auto const position = text.find(from);
    EXPECT_NE(position, std::string::npos) << from;
    return text.replace(position, from.size(), to);
}

Error parseFailure(std::string const &yaml) {
    auto const result = parseRobotProfile(yaml, "/base");
    EXPECT_FALSE(result.has_value());
    return result ? Error{} : result.error();
}

TEST(RobotProfile, ParsesValidProfile) {
    auto const profile = parseRobotProfile(kValidProfile, "/base");
    ASSERT_TRUE(profile) << profile.error().message;
    EXPECT_EQ(profile->name, "test_arm");
    EXPECT_EQ(profile->urdf, std::filesystem::path{"/base/robot.urdf"});
    EXPECT_EQ(profile->controlPeriod, std::chrono::milliseconds{2});
    ASSERT_EQ(profile->dof(), 2u);
    EXPECT_EQ(profile->joints[0].descriptionJoint, "a");
    EXPECT_EQ(profile->joints[1].descriptionJoint, "b_left");
    EXPECT_EQ(profile->joints[1].unit, JointUnit::Meter);
    ASSERT_EQ(profile->groups.size(), 2u);
    EXPECT_EQ(profile->groups[0].name, "arm");
    EXPECT_EQ(profile->groups[0].toolFrame, "tool");
    EXPECT_EQ(profile->groups[1].joints, std::vector<std::size_t>{1});
    EXPECT_EQ(profile->safety.enableRamp, std::chrono::milliseconds{100});
    EXPECT_EQ(profile->findJoint("b"), 1u);
    EXPECT_EQ(profile->findGroup("hand"), 1u);
    EXPECT_FALSE(profile->findJoint("missing"));
    EXPECT_EQ(profile->sim.node["physics_step_us"].as<int>(), 500);
    EXPECT_TRUE(profile->driver.node.IsNull());
}

TEST(RobotProfile, RejectsUnknownKey) {
    auto const error = parseFailure(
        replace(kValidProfile, "control: {period_us: 2000}", "control: {period_us: 2000, rate: 5}"));
    EXPECT_EQ(error.code, ErrorCode::InvalidConfig);
    EXPECT_NE(error.message.find("unknown key 'rate'"), std::string::npos) << error.message;
}

TEST(RobotProfile, RejectsInvertedLimits) {
    auto const error = parseFailure(replace(kValidProfile, "[-1.0, 1.0]", "[1.0, -1.0]"));
    EXPECT_NE(error.message.find("joints[0].limits.position"), std::string::npos) << error.message;
}

TEST(RobotProfile, RejectsGroupWithUnknownJoint) {
    auto const error = parseFailure(replace(kValidProfile, "joints: [a]", "joints: [c]"));
    EXPECT_NE(error.message.find("unknown joint 'c'"), std::string::npos) << error.message;
}

TEST(RobotProfile, RejectsJointInTwoGroups) {
    auto const error = parseFailure(replace(kValidProfile, "joints: [b]", "joints: [a, b]"));
    EXPECT_NE(error.message.find("already belongs"), std::string::npos) << error.message;
}

TEST(RobotProfile, RejectsRestPoseOfWrongSize) {
    auto const error = parseFailure(replace(kValidProfile, "rest_pose: [0.0, 0.0]", "rest_pose: [0.0]"));
    EXPECT_NE(error.message.find("rest_pose"), std::string::npos) << error.message;
}

TEST(RobotProfile, RejectsRestPoseOutsideLimits) {
    auto const error = parseFailure(replace(kValidProfile, "rest_pose: [0.0, 0.0]", "rest_pose: [0.0, 0.5]"));
    EXPECT_NE(error.message.find("outside its position limits"), std::string::npos) << error.message;
}

TEST(RobotProfile, ReportsMissingFile) {
    auto const result = loadRobotProfile("/nonexistent/profile.yaml");
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().code, ErrorCode::Io);
}

TEST(RobotProfile, LoadsRebotProfile) {
    auto const *const path = std::getenv("LARM_ROBOT_PROFILE");
    ASSERT_NE(path, nullptr);
    auto const profile = loadRobotProfile(path);
    ASSERT_TRUE(profile) << profile.error().message;
    EXPECT_EQ(profile->name, "rebot_b601_rs");
    ASSERT_EQ(profile->dof(), 7u);
    EXPECT_EQ(profile->joints[6].descriptionJoint, "joint_left");
    auto const arm = profile->findGroup("arm");
    ASSERT_TRUE(arm);
    EXPECT_EQ(profile->groups[*arm].joints.size(), 6u);
    EXPECT_EQ(profile->groups[*arm].toolFrame, "gripper_end");
    EXPECT_EQ(profile->urdf.filename(), "rebot_b601_rs.urdf");
    EXPECT_TRUE(profile->urdf.is_absolute());
}

} // namespace
} // namespace larm
