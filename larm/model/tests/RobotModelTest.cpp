#include <larm/model/RobotModel.h>

#include <gtest/gtest.h>

#include <cstdlib>
#include <random>

namespace larm::model {
namespace {

struct RebotModel : testing::Test {
    void SetUp() override {
        auto const *const path = std::getenv("LARM_ROBOT_PROFILE");
        ASSERT_NE(path, nullptr);
        auto loaded = loadRobotProfile(path);
        ASSERT_TRUE(loaded) << loaded.error().message;
        profile = std::move(*loaded);
        auto robot = loadRobotModel(profile);
        ASSERT_TRUE(robot) << robot.error().message;
        model = std::move(*robot);
        kinematics = model->makeKinematics();
        auto found = kinematics->findFrame("gripper_end");
        ASSERT_TRUE(found);
        tool = *found;
    }

    JointVector randomConfiguration(std::mt19937_64 &random) const {
        auto q = zeroJointVector(profile.dof());
        for (std::size_t i = 0; i < profile.dof(); ++i) {
            auto const &limits = profile.joints[i].limits;
            auto const margin = 0.05 * (limits.upper - limits.lower);
            auto uniform =
                std::uniform_real_distribution<double>{limits.lower + margin, limits.upper - margin};
            q[idx(i)] = uniform(random);
        }
        return q;
    }

    Pose3 toolPose(JointVector const &q) {
        kinematics->update(q);
        return kinematics->framePose(tool);
    }

    RobotProfile profile;
    std::unique_ptr<RobotModel> model;
    std::unique_ptr<Kinematics> kinematics;
    FrameId tool;
};

TEST_F(RebotModel, MatchesProfileJoints) {
    EXPECT_EQ(model->dof(), 7u);
    EXPECT_FALSE(kinematics->findFrame("no_such_frame"));
    EXPECT_TRUE(kinematics->findFrame("base_link"));
}

TEST_F(RebotModel, JacobianMatchesFiniteDifferences) {
    auto random = std::mt19937_64{1};
    constexpr double kStep = 1e-6;
    for (int sample = 0; sample < 20; ++sample) {
        auto const q = randomConfiguration(random);
        kinematics->update(q);
        auto jacobian = Jacobian{};
        kinematics->frameJacobian(tool, jacobian);
        ASSERT_EQ(jacobian.cols(), 7);
        for (std::size_t i = 0; i < profile.dof(); ++i) {
            JointVector plus = q;
            JointVector minus = q;
            plus[idx(i)] += kStep;
            minus[idx(i)] -= kStep;
            Vector6 const numeric = poseError(toolPose(minus), toolPose(plus)) / (2.0 * kStep);
            EXPECT_TRUE(jacobian.col(idx(i)).isApprox(numeric, 1e-5) or
                        (jacobian.col(idx(i)) - numeric).norm() < 1e-6)
                << "joint " << i << "\nanalytic " << jacobian.col(idx(i)).transpose() << "\nnumeric  "
                << numeric.transpose();
        }
    }
}

TEST_F(RebotModel, GravityLoadsShoulderButNotVerticalBaseJoint) {
    auto dynamics = model->makeDynamics();
    auto random = std::mt19937_64{2};
    auto gravity = JointVector{};
    for (int sample = 0; sample < 10; ++sample) {
        dynamics->gravity(randomConfiguration(random), gravity);
        ASSERT_EQ(gravity.size(), 7);
        EXPECT_NEAR(gravity[0], 0.0, 1e-9);
    }
    dynamics->gravity(profile.safety.restPose, gravity);
    EXPECT_GT(std::abs(gravity[1]), 1.0);

    auto inverse = JointVector{};
    auto const zero = zeroJointVector(profile.dof());
    dynamics->inverseDynamics(profile.safety.restPose, zero, zero, inverse);
    EXPECT_TRUE(inverse.isApprox(gravity, 1e-12));
}

TEST_F(RebotModel, IkRecoversReachablePoses) {
    auto solver = model->makeIkSolver();
    auto random = std::mt19937_64{3};
    auto armJoints = JointMask{};
    for (auto const joint : profile.groups[*profile.findGroup("arm")].joints) {
        armJoints.set(joint);
    }
    for (int sample = 0; sample < 20; ++sample) {
        auto const goal = randomConfiguration(random);
        auto const target = toolPose(goal);
        JointVector seed = goal;
        seed.head(6) += JointVector::Constant(6, 0.3);
        auto const solution =
            solver->solve(IkRequest{.target = target, .frame = tool, .joints = armJoints}, seed);
        ASSERT_TRUE(solution.converged) << "sample " << sample << " position error " << solution.positionError
                                        << " orientation error " << solution.orientationError;
        EXPECT_LT(poseError(toolPose(solution.position), target).norm(), 2e-3);
        EXPECT_DOUBLE_EQ(solution.position[6], seed[6]);
    }
}

} // namespace
} // namespace larm::model
