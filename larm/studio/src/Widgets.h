#pragma once

#include <larm/core/Pose.h>

#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QString>

#include <algorithm>
#include <array>
#include <cmath>

namespace larm::studio {

inline QDoubleSpinBox *makeSpin(double const minimum, double const maximum, double const step,
                                int const decimals, QString const &suffix = {}) {
    auto *const spin = new QDoubleSpinBox;
    spin->setRange(minimum, maximum);
    spin->setSingleStep(step);
    spin->setDecimals(decimals);
    spin->setSuffix(suffix);
    spin->setKeyboardTracking(false);
    return spin;
}

inline QString formatJoint(double const value, int const decimals = 3) {
    return QString::number(value, 'f', decimals);
}

// Roll, pitch, yaw of R = Rz(yaw) Ry(pitch) Rx(roll).
inline std::array<double, 3> rollPitchYaw(Eigen::Quaterniond const &rotation) {
    auto const matrix = rotation.toRotationMatrix();
    return {std::atan2(matrix(2, 1), matrix(2, 2)), std::asin(std::clamp(-matrix(2, 0), -1.0, 1.0)),
            std::atan2(matrix(1, 0), matrix(0, 0))};
}

inline Eigen::Quaterniond fromRollPitchYaw(double const roll, double const pitch, double const yaw) {
    return Eigen::AngleAxisd{yaw, Eigen::Vector3d::UnitZ()} *
           Eigen::AngleAxisd{pitch, Eigen::Vector3d::UnitY()} *
           Eigen::AngleAxisd{roll, Eigen::Vector3d::UnitX()};
}

} // namespace larm::studio
