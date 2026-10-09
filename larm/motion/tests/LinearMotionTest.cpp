#include <larm/motion/LinearMotion.h>

#include <gtest/gtest.h>

#include <cstdlib>

namespace larm::motion {
namespace {

struct RebotLine : testing::Test {
    void SetUp() override {
        auto const *const path = std::getenv("LARM_ROBOT_PROFILE");
        ASSERT_NE(path, nullptr);
        auto loaded = loadRobotProfile(path);
        ASSERT_TRUE(loaded) << loaded.error().message;
        profile = std::move(*loaded);
        auto robot = model::loadRobotModel(profile);
        ASSERT_TRUE(robot) << robot.error().message;
        model = std::move(*robot);
        kinematics = model->makeKinematics();
        solver = model->makeIkSolver();
        auto const &arm = profile.groups[*profile.findGroup("arm")];
        tool = *kinematics->findFrame(arm.toolFrame);
        tcp = arm.tcp;
        cartesian = *arm.cartesianLimits;
        for (auto const joint : arm.joints) {
            joints.set(joint);
        }
        ready = profile.poses.find("ready")->second;
    }

    Pose3 tcpPose(JointVector const &q) {
        kinematics->update(q);
        return kinematics->framePose(tool) * tcp;
    }

    Expected<std::shared_ptr<JointTrajectory const>> plan(Pose3 const &target) {
        return planLinear({.start = ready,
                           .target = target,
                           .tool = tool,
                           .tcp = tcp,
                           .joints = joints,
                           .cartesian = cartesian,
                           .limits = motionLimits(profile)},
                          *kinematics, *solver);
    }

    RobotProfile profile;
    std::unique_ptr<model::RobotModel> model;
    std::unique_ptr<model::Kinematics> kinematics;
    std::unique_ptr<model::IkSolver> solver;
    model::FrameId tool;
    Pose3 tcp;
    CartesianLimits cartesian;
    JointMask joints;
    JointVector ready;
};

TEST_F(RebotLine, KeepsTheTcpOnTheLineAndTurnsItAlongTheWay) {
    auto const from = tcpPose(ready);
    auto target = from;
    target.translation += Eigen::Vector3d{0.05, -0.04, -0.06};
    target.rotation =
        from.rotation * Eigen::Quaterniond{Eigen::AngleAxisd{0.4, Eigen::Vector3d{1, 1, 0}.normalized()}};
    auto const trajectory = plan(target);
    ASSERT_TRUE(trajectory) << trajectory.error().message;

    Eigen::Vector3d const line = target.translation - from.translation;
    auto const length = line.norm();
    auto sample = JointSample::zero(profile.dof());
    for (auto t = Duration{0}; t <= (*trajectory)->duration(); t += std::chrono::milliseconds{5}) {
        (*trajectory)->sample(t, sample);
        auto const pose = tcpPose(sample.position);
        Eigen::Vector3d const offset = pose.translation - from.translation;
        auto const s = offset.dot(line) / (length * length);
        EXPECT_LT((offset - s * line).norm(), 5e-4) << "at " << toSeconds(t) << " s";
        EXPECT_GT(s, -1e-3);
        EXPECT_LT(s, 1.0 + 1e-3);
        auto const expected = from.rotation.slerp(std::clamp(s, 0.0, 1.0), target.rotation);
        EXPECT_LT(pose.rotation.angularDistance(expected), 0.01) << "at " << toSeconds(t) << " s";
        EXPECT_DOUBLE_EQ(sample.position[6], ready[6]);
    }
    (*trajectory)->sample((*trajectory)->duration(), sample);
    auto const reached = tcpPose(sample.position);
    EXPECT_LT((reached.translation - target.translation).norm(), 1e-3);
    EXPECT_LT(reached.rotation.angularDistance(target.rotation), 2e-3);
    EXPECT_LT(sample.velocity.norm(), 1e-6);
}

TEST_F(RebotLine, KeepsTheTcpSpeedAndTheJointLimits) {
    auto const from = tcpPose(ready);
    auto target = from;
    target.translation.z() -= 0.15;
    auto const trajectory = plan(target);
    ASSERT_TRUE(trajectory) << trajectory.error().message;
    // At most the linear speed limit, and not far below it either: 15 cm at 0.1 m/s.
    EXPECT_GT(toSeconds((*trajectory)->duration()), 1.5);
    EXPECT_LT(toSeconds((*trajectory)->duration()), 2.5);

    auto const limits = motionLimits(profile);
    auto sample = JointSample::zero(profile.dof());
    auto previous = from.translation;
    constexpr auto kStep = std::chrono::milliseconds{5};
    for (auto t = kStep; t <= (*trajectory)->duration(); t += kStep) {
        (*trajectory)->sample(t, sample);
        auto const position = tcpPose(sample.position).translation;
        EXPECT_LT((position - previous).norm() / toSeconds(kStep), cartesian.linearVelocity * 1.02);
        previous = position;
        for (std::size_t j = 0; j < 6; ++j) {
            EXPECT_LE(std::abs(sample.velocity[idx(j)]), limits.velocity[idx(j)] * 1.001) << "joint " << j;
            EXPECT_LE(std::abs(sample.acceleration[idx(j)]), limits.acceleration[idx(j)] * 1.05)
                << "joint " << j;
        }
    }
}

TEST_F(RebotLine, RejectsATargetOutOfReach) {
    auto target = tcpPose(ready);
    target.translation.x() += 0.6;
    auto const trajectory = plan(target);
    ASSERT_FALSE(trajectory);
    EXPECT_EQ(trajectory.error().code, ErrorCode::SolverFailed);
    EXPECT_NE(trajectory.error().message.find("reaches the target"), std::string::npos)
        << trajectory.error().message;
}

TEST_F(RebotLine, StaysPutWhenAlreadyThere) {
    auto const trajectory = plan(tcpPose(ready));
    ASSERT_TRUE(trajectory) << trajectory.error().message;
    EXPECT_EQ((*trajectory)->duration(), Duration{0});
}

// Places the frame at (q0, q1, q2).
struct PointKinematics final : model::Kinematics {
    std::optional<model::FrameId> findFrame(std::string_view) const override { return model::FrameId{}; }
    void update(JointVector const &q) noexcept override { position = q.head<3>(); }
    Pose3 framePose(model::FrameId) const noexcept override { return Pose3{.translation = position}; }
    void frameJacobian(model::FrameId, model::Jacobian &) const noexcept override {}

    Eigen::Vector3d position = Eigen::Vector3d::Zero();
};

// Solves exactly, but flips joint 3 once the target passes x = 0.05, as at a singularity.
struct FlippingSolver final : model::IkSolver {
    model::IkSolution solve(model::IkRequest const &request, JointVector const &seed) override {
        JointVector q = seed;
        q.head<3>() = request.target.translation;
        q[3] = request.target.translation.x() > 0.05 and request.target.translation.x() < 0.1 ? 1.0 : 0.0;
        return {.position = q, .positionError = 0.0, .orientationError = 0.0, .converged = true};
    }
};

TEST(Line, RejectsAJointJumpAsASingularity) {
    auto kinematics = PointKinematics{};
    auto solver = FlippingSolver{};
    auto joints = JointMask{};
    joints.set();
    auto const one = JointVector::Constant(4, 1.0);
    auto const trajectory = planLinear({.start = JointVector::Zero(4),
                                        .target = Pose3{.translation = Eigen::Vector3d{0.1, 0.0, 0.0}},
                                        .tool = {},
                                        .tcp = {},
                                        .joints = joints,
                                        .cartesian = {.linearVelocity = 0.1,
                                                      .linearAcceleration = 0.5,
                                                      .angularVelocity = 0.5,
                                                      .angularAcceleration = 2.0},
                                        .limits = {.velocity = one, .acceleration = one, .jerk = one}},
                                       kinematics, solver);
    ASSERT_FALSE(trajectory);
    EXPECT_EQ(trajectory.error().code, ErrorCode::InvalidArgument);
    EXPECT_NE(trajectory.error().message.find("singularity 51% of the way"), std::string::npos)
        << trajectory.error().message;
}

} // namespace
} // namespace larm::motion
