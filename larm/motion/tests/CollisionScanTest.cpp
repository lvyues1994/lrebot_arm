#include <larm/motion/CollisionScan.h>
#include <larm/motion/Planning.h>

#include <gtest/gtest.h>

#include <vector>

namespace larm::motion {
namespace {

constexpr std::size_t kDof = 2;

// Joint 0 past `wall` collides; records every query.
struct WallChecker final : model::CollisionChecker {
    explicit WallChecker(double const wall_) : wall{wall_} {}

    void allowContactsAt(JointVector const &position) override { allowedAt.push_back(position); }

    std::optional<model::LinkContact> collision(JointVector const &position) override {
        checked.push_back(position);
        if (position[0] > wall) {
            return model::LinkContact{.first = "arm", .second = "wall", .depth = position[0] - wall};
        }
        return std::nullopt;
    }

    double wall;
    std::vector<JointVector> allowedAt;
    std::vector<JointVector> checked;
};

std::shared_ptr<JointTrajectory const> motionTo(double const target) {
    auto joints = JointMask{};
    joints.set(0);
    joints.set(1);
    auto goal = JointVector{idx(kDof)};
    goal << target, -0.5 * target;
    auto const trajectory =
        planPointToPoint({.start = JointSample::zero(kDof),
                          .target = goal,
                          .limits = {.velocity = JointVector::Constant(idx(kDof), 1.0),
                                     .acceleration = JointVector::Constant(idx(kDof), 2.0),
                                     .jerk = JointVector::Constant(idx(kDof), 10.0)},
                          .joints = joints});
    EXPECT_TRUE(trajectory) << trajectory.error().message;
    return *trajectory;
}

JointVector steps(double const step) { return JointVector::Constant(idx(kDof), step); }

TEST(CollisionScan, AllowsContactsAtTheStartAndChecksWithinTheStep) {
    auto const trajectory = motionTo(1.0);
    auto checker = WallChecker{2.0};
    EXPECT_FALSE(findCollision(*trajectory, checker, steps(0.05)));

    ASSERT_EQ(checker.allowedAt.size(), 1u);
    EXPECT_LT(checker.allowedAt.front().norm(), 1e-12);
    ASSERT_GE(checker.checked.size(), 20u);
    auto previous = checker.allowedAt.front();
    for (auto const &position : checker.checked) {
        EXPECT_LE((position - previous).cwiseAbs().maxCoeff(), 0.05 + 1e-3);
        previous = position;
    }
    EXPECT_NEAR(checker.checked.back()[0], 1.0, 1e-9);
}

TEST(CollisionScan, ReportsTheFirstCollisionAndWhen) {
    auto const trajectory = motionTo(1.0);
    auto checker = WallChecker{0.5};
    auto const collision = findCollision(*trajectory, checker, steps(0.02));
    ASSERT_TRUE(collision);
    EXPECT_EQ(collision->contact.second, "wall");
    EXPECT_LT(collision->contact.depth, 0.02 + 1e-3);
    EXPECT_GT(collision->time, Duration{0});
    EXPECT_LT(collision->time, trajectory->duration());

    auto sample = JointSample::zero(kDof);
    trajectory->sample(collision->time, sample);
    EXPECT_GT(sample.position[0], 0.5);
}

TEST(CollisionScan, ChecksTheEndOfAShortMotion) {
    auto const trajectory = motionTo(0.01);
    auto checker = WallChecker{0.005};
    auto const collision = findCollision(*trajectory, checker, steps(0.05));
    ASSERT_TRUE(collision);
    EXPECT_EQ(collision->time, trajectory->duration());
}

} // namespace
} // namespace larm::motion
