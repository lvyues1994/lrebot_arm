#include <larm/model/RobotModel.h>

#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>

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

// A link's inertial as the description has it.
LinkInertial describedInertial(RobotProfile const &profile, std::string const &link) {
    auto const tree = ::urdf::parseURDFFile(profile.urdf.string());
    auto const &inertial = *tree->links_.at(link)->inertial;
    auto const &com = inertial.origin.position;
    return LinkInertial{.mass = inertial.mass, .centerOfMass = {com.x, com.y, com.z}};
}

TEST_F(RebotModel, LinkInertialsFromTheProfileReplaceTheDescriptions) {
    auto const described = describedInertial(profile, "link3");
    auto const q = profile.poses.at("ready");
    auto const gravityOf = [&](RobotProfile const &withInertials) {
        auto loaded = loadRobotModel(withInertials);
        EXPECT_TRUE(loaded) << loaded.error().message;
        auto gravity = JointVector{};
        (*loaded)->makeDynamics()->gravity(q, gravity);
        return gravity;
    };
    auto const nominal = gravityOf(profile);

    auto same = profile;
    same.linkInertials.emplace("link3", described);
    EXPECT_TRUE(gravityOf(same).isApprox(nominal, 1e-12));

    // Extra mass m at the link's center of mass adds m g dz/dq, z the height of that point.
    constexpr double kExtra = 0.5;
    auto heavier = profile;
    heavier.linkInertials.emplace(
        "link3", LinkInertial{.mass = described.mass + kExtra, .centerOfMass = described.centerOfMass});
    auto const added = (gravityOf(heavier) - nominal).eval();
    auto const link = *kinematics->findFrame("link3");
    auto const height = [&](JointVector const &at) {
        kinematics->update(at);
        return (kinematics->framePose(link) * described.centerOfMass).z();
    };
    constexpr double kStep = 1e-6;
    for (std::size_t i = 0; i < profile.dof(); ++i) {
        auto up = q;
        auto down = q;
        up[idx(i)] += kStep;
        down[idx(i)] -= kStep;
        EXPECT_NEAR(added[idx(i)], kExtra * 9.81 * (height(up) - height(down)) / (2.0 * kStep), 1e-6) << i;
    }
    EXPECT_GT(std::abs(added[2]), 0.1);

    auto unknown = profile;
    unknown.linkInertials.emplace("no_such_link", LinkInertial{.mass = 1.0});
    EXPECT_FALSE(loadRobotModel(unknown));
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

bool between(LinkContact const &contact, std::string const &a, std::string const &b) {
    return (contact.first == a and contact.second == b) or (contact.first == b and contact.second == a);
}

TEST_F(RebotModel, RestPoseIsFreeOfCollisionsWithTheGripperOpenOrClosed) {
    auto checker = model->makeCollisionChecker();
    auto q = profile.safety.restPose;
    EXPECT_FALSE(checker->collision(q));
    q[6] = profile.joints[6].limits.upper;
    EXPECT_FALSE(checker->collision(q));
}

TEST_F(RebotModel, WristLoweredIntoTheUpperArmCollides) {
    auto checker = model->makeCollisionChecker();
    auto q = profile.safety.restPose;
    q[3] = -0.85;
    auto const contact = checker->collision(q);
    ASSERT_TRUE(contact);
    EXPECT_TRUE(between(*contact, "link2", "link5") or between(*contact, "link1", "link5"))
        << contact->first << " / " << contact->second;
    EXPECT_GT(contact->depth, 0.005);
}

TEST_F(RebotModel, FoldedArmTippedOverTheBaseCollides) {
    auto checker = model->makeCollisionChecker();
    auto q = profile.safety.restPose;
    q[1] = 1.6;
    auto const contact = checker->collision(q);
    ASSERT_TRUE(contact);
    EXPECT_TRUE(contact->first == "base_link" or contact->second == "base_link")
        << contact->first << " / " << contact->second;
}

TEST_F(RebotModel, AllowedContactMayPersistButNotDeepen) {
    auto checker = model->makeCollisionChecker();
    auto pressed = profile.safety.restPose;
    pressed[3] = -0.03;
    ASSERT_TRUE(checker->collision(pressed));

    checker->allowContactsAt(pressed);
    EXPECT_FALSE(checker->collision(pressed));
    auto turned = pressed;
    turned[0] = 1.0;
    EXPECT_FALSE(checker->collision(turned));
    auto lifted = pressed;
    lifted[3] = 0.3;
    EXPECT_FALSE(checker->collision(lifted));
    auto deeper = pressed;
    deeper[3] = -0.1;
    auto const contact = checker->collision(deeper);
    ASSERT_TRUE(contact);
    EXPECT_TRUE(between(*contact, "link2", "link5")) << contact->first << " / " << contact->second;
}

} // namespace
} // namespace larm::model
