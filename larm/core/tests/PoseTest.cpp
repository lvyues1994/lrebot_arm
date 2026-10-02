#include <larm/core/Pose.h>

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

namespace larm {
namespace {

Pose3 samplePose() {
    return Pose3{
        .translation = Eigen::Vector3d{0.1, -0.2, 0.3},
        .rotation = Eigen::Quaterniond{Eigen::AngleAxisd{0.7, Eigen::Vector3d{1.0, 2.0, -0.5}.normalized()}},
    };
}

TEST(Pose3, ComposesWithInverseToIdentity) {
    auto const pose = samplePose();
    auto const identity = pose * pose.inverse();
    EXPECT_LT(identity.translation.norm(), 1e-12);
    EXPECT_NEAR(std::abs(identity.rotation.w()), 1.0, 1e-12);
}

TEST(Pose3, TransformsPoints) {
    auto const pose = Pose3{
        .translation = Eigen::Vector3d{1.0, 0.0, 0.0},
        .rotation = Eigen::Quaterniond{Eigen::AngleAxisd{std::numbers::pi / 2, Eigen::Vector3d::UnitZ()}},
    };
    auto const point = pose * Eigen::Vector3d{1.0, 0.0, 0.0};
    EXPECT_TRUE(point.isApprox(Eigen::Vector3d{1.0, 1.0, 0.0}, 1e-12));
}

TEST(Pose3, ErrorIsZeroForEqualPosesAndMatchesSmallRotation) {
    auto const pose = samplePose();
    EXPECT_LT(poseError(pose, pose).norm(), 1e-12);

    auto rotated = pose;
    rotated.rotation = Eigen::Quaterniond{Eigen::AngleAxisd{0.1, Eigen::Vector3d::UnitZ()}} * pose.rotation;
    rotated.translation += Eigen::Vector3d{0.0, 0.0, 0.05};
    auto const error = poseError(pose, rotated);
    EXPECT_TRUE(error.head<3>().isApprox(Eigen::Vector3d{0.0, 0.0, 0.05}, 1e-12));
    EXPECT_TRUE(error.tail<3>().isApprox(Eigen::Vector3d{0.0, 0.0, 0.1}, 1e-9));
}

} // namespace
} // namespace larm
