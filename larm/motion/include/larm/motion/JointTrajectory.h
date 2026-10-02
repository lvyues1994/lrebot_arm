#pragma once

#include <larm/core/JointVector.h>
#include <larm/core/Time.h>

namespace larm::motion {

struct JointSample {
    JointVector position;
    JointVector velocity;
    JointVector acceleration;

    static JointSample zero(std::size_t dof);
};

// Immutable once built. Sampling never allocates, so the control loop may evaluate it.
struct JointTrajectory {
    virtual ~JointTrajectory() = default;
    virtual std::size_t dof() const noexcept = 0;
    virtual Duration duration() const noexcept = 0;
    // `time` is clamped to [0, duration()].
    virtual void sample(Duration time, JointSample &out) const noexcept = 0;
};

} // namespace larm::motion
