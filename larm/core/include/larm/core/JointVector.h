#pragma once

#include <Eigen/Core>

#include <bitset>
#include <cstddef>

namespace larm {

inline constexpr std::size_t kMaxDof = 16;

// Runtime-sized up to kMaxDof and stored inline, so copies and resizes never allocate.
using JointVector = Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor, static_cast<int>(kMaxDof), 1>;
using JointMask = std::bitset<kMaxDof>;

inline Eigen::Index idx(std::size_t const joint) noexcept { return static_cast<Eigen::Index>(joint); }

inline std::size_t dofOf(JointVector const &vector) noexcept {
    return static_cast<std::size_t>(vector.size());
}

inline JointVector zeroJointVector(std::size_t const dof) { return JointVector::Zero(idx(dof)); }

} // namespace larm
