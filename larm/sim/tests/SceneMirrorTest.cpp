#include <larm/model/RobotModel.h>
#include <larm/sim/SceneMirror.h>

#include <gtest/gtest.h>

#include <cstdlib>

namespace larm::sim {
namespace {

TEST(SceneMirror, PosesTheModelAndItsCoupledFinger) {
    auto const *const path = std::getenv("LARM_ROBOT_PROFILE");
    ASSERT_NE(path, nullptr);
    auto const profile = loadRobotProfile(path);
    ASSERT_TRUE(profile) << profile.error().message;
    auto mirror = loadSceneMirror(*profile);
    ASSERT_TRUE(mirror) << mirror.error().message;
    auto robot = model::loadRobotModel(*profile);
    ASSERT_TRUE(robot);
    auto kinematics = (*robot)->makeKinematics();

    auto q = JointVector{7};
    q << 0.4, 1.0, 1.3, -0.4, 0.5, 0.8, 0.03;
    (*mirror)->show(q);
    kinematics->update(q);

    auto const &model = (*mirror)->model();
    auto const &data = (*mirror)->data();
    auto const body = mj_name2id(&model, mjOBJ_BODY, "gripper_end");
    auto const expected = kinematics->framePose(*kinematics->findFrame("gripper_end"));
    auto const position =
        Eigen::Vector3d{data.xpos[3 * body], data.xpos[3 * body + 1], data.xpos[3 * body + 2]};
    EXPECT_LT((position - expected.translation).norm(), 1e-5);

    auto const left = model.jnt_qposadr[mj_name2id(&model, mjOBJ_JOINT, "joint_left")];
    auto const right = model.jnt_qposadr[mj_name2id(&model, mjOBJ_JOINT, "joint_right")];
    EXPECT_DOUBLE_EQ(data.qpos[left], 0.03);
    EXPECT_DOUBLE_EQ(data.qpos[right], 0.03);
}

} // namespace
} // namespace larm::sim
