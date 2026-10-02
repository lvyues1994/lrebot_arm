#include <larm/motion/Planning.h>

#include <gtest/gtest.h>

#include <vector>

namespace larm::motion {
namespace {

JointVector vector2(double const a, double const b) {
    auto v = JointVector{2};
    v << a, b;
    return v;
}

TEST(Waypoints, PassesThroughEveryWaypointWithGivenDerivatives) {
    auto const waypoints = std::vector<TimedWaypoint>{
        {.time = Duration{0},
         .position = vector2(0.0, 1.0),
         .velocity = vector2(0.0, 0.0),
         .acceleration = vector2(0.0, 0.0)},
        {.time = std::chrono::milliseconds{500},
         .position = vector2(0.5, 0.8),
         .velocity = vector2(1.0, -0.2),
         .acceleration = vector2(0.5, 0.1)},
        {.time = std::chrono::seconds{1},
         .position = vector2(1.0, 0.5),
         .velocity = vector2(0.0, 0.0),
         .acceleration = vector2(0.0, 0.0)},
    };
    auto const trajectory = interpolateWaypoints(waypoints);
    ASSERT_TRUE(trajectory) << trajectory.error().message;
    EXPECT_EQ((*trajectory)->duration(), std::chrono::seconds{1});
    auto sample = JointSample::zero(2);
    for (auto const &waypoint : waypoints) {
        (*trajectory)->sample(waypoint.time, sample);
        EXPECT_TRUE(sample.position.isApprox(waypoint.position, 1e-12));
        EXPECT_LT((sample.velocity - *waypoint.velocity).norm(), 1e-9);
        EXPECT_LT((sample.acceleration - *waypoint.acceleration).norm(), 1e-9);
    }
}

TEST(Waypoints, VelocityIsTheDerivativeOfPosition) {
    auto const waypoints = std::vector<TimedWaypoint>{
        {.time = Duration{0}, .position = vector2(0.0, 0.0)},
        {.time = std::chrono::milliseconds{300}, .position = vector2(0.2, -0.1)},
        {.time = std::chrono::milliseconds{900}, .position = vector2(0.6, 0.3)},
        {.time = std::chrono::milliseconds{1200}, .position = vector2(0.6, 0.4)},
    };
    auto const trajectory = interpolateWaypoints(waypoints);
    ASSERT_TRUE(trajectory);
    auto sample = JointSample::zero(2);
    auto before = JointSample::zero(2);
    auto after = JointSample::zero(2);
    constexpr auto kStep = std::chrono::microseconds{10};
    for (int ms = 10; ms < 1200; ms += 37) {
        auto const t = Duration{std::chrono::milliseconds{ms}};
        (*trajectory)->sample(t, sample);
        (*trajectory)->sample(t - kStep, before);
        (*trajectory)->sample(t + kStep, after);
        auto const numeric = JointVector{(after.position - before.position) / (2 * toSeconds(kStep))};
        EXPECT_LT((numeric - sample.velocity).norm(), 1e-4) << "at " << ms << " ms";
    }
    (*trajectory)->sample(std::chrono::seconds{5}, sample);
    EXPECT_TRUE(sample.position.isApprox(waypoints.back().position));
    EXPECT_EQ(sample.velocity.norm(), 0.0);
}

TEST(Waypoints, RejectsBadInput) {
    EXPECT_FALSE(interpolateWaypoints({}));
    auto const late =
        std::vector<TimedWaypoint>{{.time = std::chrono::seconds{1}, .position = vector2(0, 0)}};
    EXPECT_FALSE(interpolateWaypoints(late));
    auto const backwards = std::vector<TimedWaypoint>{
        {.time = Duration{0}, .position = vector2(0, 0)},
        {.time = Duration{0}, .position = vector2(1, 1)},
    };
    EXPECT_FALSE(interpolateWaypoints(backwards));
}

} // namespace
} // namespace larm::motion
