#include <larm/motion/Planning.h>

#include <gtest/gtest.h>

namespace larm::motion {
namespace {

constexpr std::size_t kDof = 3;

MotionLimits limits() {
    auto result = MotionLimits{
        .velocity = JointVector::Constant(idx(kDof), 1.0),
        .acceleration = JointVector::Constant(idx(kDof), 2.0),
        .jerk = JointVector::Constant(idx(kDof), 10.0),
    };
    result.velocity[2] = 0.5;
    return result;
}

PointToPointRequest request() {
    auto start = JointSample::zero(kDof);
    start.position << 0.1, -0.2, 0.3;
    auto target = JointVector{idx(kDof)};
    target << 1.1, -0.7, 0.3;
    auto joints = JointMask{};
    joints.set();
    return PointToPointRequest{.start = start, .target = target, .limits = limits(), .joints = joints};
}

TEST(PointToPoint, StartsAndEndsAtRestOnTheRequestedPositions) {
    auto const trajectory = planPointToPoint(request());
    ASSERT_TRUE(trajectory) << trajectory.error().message;
    auto const &plan = **trajectory;
    EXPECT_EQ(plan.dof(), kDof);
    EXPECT_GT(plan.duration(), std::chrono::milliseconds{500});

    auto sample = JointSample::zero(kDof);
    plan.sample(Duration{0}, sample);
    EXPECT_TRUE(sample.position.isApprox(request().start.position, 1e-12));
    plan.sample(plan.duration(), sample);
    EXPECT_TRUE(sample.position.isApprox(request().target, 1e-9));
    EXPECT_LT(sample.velocity.norm(), 1e-9);
    plan.sample(plan.duration() + std::chrono::seconds{5}, sample);
    EXPECT_TRUE(sample.position.isApprox(request().target, 1e-9));
}

TEST(PointToPoint, RespectsLimitsAndArrivesTogether) {
    auto const trajectory = planPointToPoint(request());
    ASSERT_TRUE(trajectory);
    auto const &plan = **trajectory;
    auto const bound = limits();
    auto sample = JointSample::zero(kDof);
    auto const steps = 1000;
    for (int step = 0; step <= steps; ++step) {
        plan.sample(plan.duration() * step / steps, sample);
        for (std::size_t i = 0; i < kDof; ++i) {
            EXPECT_LE(std::abs(sample.velocity[idx(i)]), bound.velocity[idx(i)] * (1.0 + 1e-9));
            EXPECT_LE(std::abs(sample.acceleration[idx(i)]), bound.acceleration[idx(i)] * (1.0 + 1e-9));
        }
    }
    // Synchronized: halfway through, every moving joint is still on its way.
    plan.sample(plan.duration() / 2, sample);
    EXPECT_GT(std::abs(sample.velocity[0]), 0.0);
    EXPECT_GT(std::abs(sample.velocity[1]), 0.0);
}

TEST(PointToPoint, MaskedJointsHoldTheirStart) {
    auto masked = request();
    masked.joints.reset(1);
    masked.target[1] = 5.0;
    auto const trajectory = planPointToPoint(masked);
    ASSERT_TRUE(trajectory);
    auto sample = JointSample::zero(kDof);
    (*trajectory)->sample((*trajectory)->duration() / 2, sample);
    EXPECT_DOUBLE_EQ(sample.position[1], masked.start.position[1]);
    EXPECT_DOUBLE_EQ(sample.velocity[1], 0.0);
}

TEST(PointToPoint, RejectsInconsistentRequests) {
    auto wrongSize = request();
    wrongSize.target = JointVector::Zero(2);
    EXPECT_EQ(planPointToPoint(wrongSize).error().code, ErrorCode::InvalidArgument);

    auto zeroLimit = request();
    zeroLimit.limits.jerk[0] = 0.0;
    EXPECT_EQ(planPointToPoint(zeroLimit).error().code, ErrorCode::InvalidArgument);
}

} // namespace
} // namespace larm::motion
