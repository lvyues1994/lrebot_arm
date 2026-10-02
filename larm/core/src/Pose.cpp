#include <larm/core/Pose.h>

namespace larm {

Pose3 Pose3::operator*(Pose3 const &other) const {
    return Pose3{
        .translation = translation + rotation * other.translation,
        .rotation = (rotation * other.rotation).normalized(),
    };
}

Eigen::Vector3d Pose3::operator*(Eigen::Vector3d const &point) const {
    return translation + rotation * point;
}

Pose3 Pose3::inverse() const {
    auto const inverted = rotation.conjugate();
    return Pose3{.translation = -(inverted * translation), .rotation = inverted};
}

Vector6 poseError(Pose3 const &from, Pose3 const &to) {
    auto delta = Eigen::Quaterniond{to.rotation * from.rotation.conjugate()};
    if (delta.w() < 0.0) {
        delta.coeffs() = -delta.coeffs();
    }
    auto const angleAxis = Eigen::AngleAxisd{delta.normalized()};
    auto error = Vector6{};
    error.head<3>() = to.translation - from.translation;
    error.tail<3>() = angleAxis.angle() * angleAxis.axis();
    return error;
}

} // namespace larm
