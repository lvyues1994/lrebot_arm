#include <larm/motion/Planning.h>

#include <gtest/gtest.h>

namespace larm::motion {
namespace {

TEST(StopPlanner, BringsAMovingStateToRestWithinLimits) {
    auto const limits = MotionLimits{
        .velocity = JointVector::Constant(2, 2.0),
        .acceleration = JointVector::Constant(2, 4.0),
        .jerk = JointVector::Constant(2, 40.0),
    };
    auto planner = makeStopPlanner(limits);
    auto from = JointSample::zero(2);
    from.position << 0.5, -0.5;
    from.velocity << 1.5, -0.4;
    from.acceleration << 1.0, 0.0;
    ASSERT_TRUE(planner->plan(from));

    auto const &stop = planner->trajectory();
    // Braking from 1.5 rad/s at 4 rad/s^2 takes at least 0.375 s.
    EXPECT_GE(stop.duration(), std::chrono::milliseconds{375});
    EXPECT_LT(stop.duration(), std::chrono::seconds{1});

    auto sample = JointSample::zero(2);
    stop.sample(Duration{0}, sample);
    EXPECT_TRUE(sample.velocity.isApprox(from.velocity, 1e-9));
    for (int step = 0; step <= 200; ++step) {
        stop.sample(stop.duration() * step / 200, sample);
        EXPECT_LE(sample.acceleration.cwiseAbs().maxCoeff(), 4.0 * (1.0 + 1e-9));
    }
    stop.sample(stop.duration(), sample);
    EXPECT_LT(sample.velocity.norm(), 1e-9);
    EXPECT_GT(sample.position[0], from.position[0]);
    EXPECT_LT(sample.position[1], from.position[1]);

    from.velocity.setZero();
    from.acceleration.setZero();
    ASSERT_TRUE(planner->plan(from));
    EXPECT_EQ(planner->trajectory().duration(), Duration{0});
}

} // namespace
} // namespace larm::motion
