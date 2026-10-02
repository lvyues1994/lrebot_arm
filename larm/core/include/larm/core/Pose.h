#pragma once

#include <Eigen/Geometry>

namespace larm {

using Vector6 = Eigen::Matrix<double, 6, 1>;

// Rigid transform. `rotation` is kept normalized by every operation here.
struct Pose3 {
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();

    Pose3 operator*(Pose3 const &other) const;
    Eigen::Vector3d operator*(Eigen::Vector3d const &point) const;
    Pose3 inverse() const;
};

// Twist [linear; angular] that moves `from` onto `to` in unit time, expressed in the world frame.
Vector6 poseError(Pose3 const &from, Pose3 const &to);

} // namespace larm
