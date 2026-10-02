#include <larm/motion/Planning.h>

#include <ruckig/ruckig.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace larm::motion {
namespace {

using Otg = ruckig::Ruckig<kMaxDof>;
using OtgInput = ruckig::InputParameter<kMaxDof>;
using OtgTrajectory = ruckig::Trajectory<kMaxDof>;
using OtgVector = std::array<double, kMaxDof>;

bool sizesMatch(std::size_t const dof, std::initializer_list<JointVector const *> vectors) {
    return std::all_of(vectors.begin(), vectors.end(), [&](JointVector const *vector) {
        return dofOf(*vector) == dof and vector->allFinite();
    });
}

// Disabled degrees of freedom, including the padding beyond `dof`, stay where they start.
void fillInput(OtgInput &input, JointSample const &start, MotionLimits const &limits,
               JointMask const &joints) {
    auto const dof = dofOf(start.position);
    for (std::size_t i = 0; i < kMaxDof; ++i) {
        auto const active = i < dof and joints.test(i);
        input.enabled[i] = active;
        input.current_position[i] = i < dof ? start.position[idx(i)] : 0.0;
        input.current_velocity[i] = active ? start.velocity[idx(i)] : 0.0;
        input.current_acceleration[i] = active ? start.acceleration[idx(i)] : 0.0;
        input.target_position[i] = input.current_position[i];
        input.target_velocity[i] = 0.0;
        input.target_acceleration[i] = 0.0;
        input.max_velocity[i] = i < dof ? limits.velocity[idx(i)] : 1.0;
        input.max_acceleration[i] = i < dof ? limits.acceleration[idx(i)] : 1.0;
        input.max_jerk[i] = i < dof ? limits.jerk[idx(i)] : 1.0;
    }
}

struct RuckigJointTrajectory final : JointTrajectory {
    std::size_t dof() const noexcept override { return dofCount; }

    Duration duration() const noexcept override { return fromSeconds(trajectory.get_duration()); }

    void sample(Duration const time, JointSample &out) const noexcept override {
        auto const seconds = std::clamp(toSeconds(time), 0.0, trajectory.get_duration());
        auto position = OtgVector{};
        auto velocity = OtgVector{};
        auto acceleration = OtgVector{};
        trajectory.at_time(seconds, position, velocity, acceleration);
        out.position.resize(idx(dofCount));
        out.velocity.resize(idx(dofCount));
        out.acceleration.resize(idx(dofCount));
        for (std::size_t i = 0; i < dofCount; ++i) {
            auto const active = joints.test(i);
            out.position[idx(i)] = active ? position[i] : hold[idx(i)];
            out.velocity[idx(i)] = active ? velocity[i] : 0.0;
            out.acceleration[idx(i)] = active ? acceleration[i] : 0.0;
        }
    }

    std::size_t dofCount{};
    JointMask joints;
    JointVector hold;
    OtgTrajectory trajectory;
};

Error solverError(ruckig::Result const result) {
    return Error{.code = ErrorCode::SolverFailed,
                 .message = "trajectory generation failed (Ruckig result " + std::to_string(result) + ")"};
}

struct RuckigStopPlanner final : StopPlanner {
    explicit RuckigStopPlanner(MotionLimits limits_) : limits{std::move(limits_)} {
        stop.dofCount = dofOf(limits.velocity);
        stop.joints.set();
        stop.hold = zeroJointVector(stop.dofCount);
        input.control_interface = ruckig::ControlInterface::Velocity;
        // Each joint stops as fast as it can instead of waiting for the slowest one.
        input.synchronization = ruckig::Synchronization::None;
    }

    bool plan(JointSample const &from) noexcept override {
        auto all = JointMask{};
        all.set();
        fillInput(input, from, limits, all);
        auto const result = otg.calculate(input, stop.trajectory);
        return result == ruckig::Result::Working or result == ruckig::Result::Finished;
    }

    JointTrajectory const &trajectory() const noexcept override { return stop; }

  private:
    MotionLimits limits;
    Otg otg;
    OtgInput input;
    RuckigJointTrajectory stop;
};

} // namespace

MotionLimits motionLimits(RobotProfile const &profile, double const scale) {
    auto limits = MotionLimits{
        .velocity = zeroJointVector(profile.dof()),
        .acceleration = zeroJointVector(profile.dof()),
        .jerk = zeroJointVector(profile.dof()),
    };
    for (std::size_t i = 0; i < profile.dof(); ++i) {
        auto const &joint = profile.joints[i].limits;
        limits.velocity[idx(i)] = scale * joint.velocity;
        limits.acceleration[idx(i)] = scale * joint.acceleration;
        limits.jerk[idx(i)] = scale * joint.jerk;
    }
    return limits;
}

Expected<std::shared_ptr<JointTrajectory const>> planPointToPoint(PointToPointRequest const &request) {
    auto const dof = dofOf(request.start.position);
    if (dof == 0 or dof > kMaxDof or
        not sizesMatch(dof, {&request.start.velocity, &request.start.acceleration, &request.target,
                             &request.limits.velocity, &request.limits.acceleration, &request.limits.jerk})) {
        return makeError(ErrorCode::InvalidArgument,
                         "point-to-point request has inconsistent or non-finite vectors");
    }
    if ((request.limits.velocity.array() <= 0.0).any() or
        (request.limits.acceleration.array() <= 0.0).any() or (request.limits.jerk.array() <= 0.0).any()) {
        return makeError(ErrorCode::InvalidArgument, "motion limits must be positive");
    }

    auto input = OtgInput{};
    fillInput(input, request.start, request.limits, request.joints);
    for (std::size_t i = 0; i < dof; ++i) {
        if (request.joints.test(i)) {
            input.target_position[i] = request.target[idx(i)];
        }
    }

    auto trajectory = std::make_shared<RuckigJointTrajectory>();
    trajectory->dofCount = dof;
    trajectory->joints = request.joints;
    trajectory->hold = request.start.position;
    auto otg = Otg{};
    auto const result = otg.calculate(input, trajectory->trajectory);
    if (result != ruckig::Result::Working and result != ruckig::Result::Finished) {
        return tl::make_unexpected(solverError(result));
    }
    return std::shared_ptr<JointTrajectory const>{std::move(trajectory)};
}

std::unique_ptr<StopPlanner> makeStopPlanner(MotionLimits const &limits) {
    return std::make_unique<RuckigStopPlanner>(limits);
}

} // namespace larm::motion
