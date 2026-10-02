#include <larm/control/JointTrajectoryController.h>

namespace larm::control {
namespace {

bool exceeds(JointVector const &reference, JointVector const &measured, JointVector const &tolerance,
             JointMask const &joints) noexcept {
    for (std::size_t i = 0; i < dofOf(reference); ++i) {
        if (joints.test(i) and std::abs(reference[idx(i)] - measured[idx(i)]) > tolerance[idx(i)]) {
            return true;
        }
    }
    return false;
}

struct JointTrajectoryControllerImpl final : Controller {
    explicit JointTrajectoryControllerImpl(JointTrajectoryControllerConfig config_)
        : config{std::move(config_)}, stopPlanner{motion::makeStopPlanner(config.stopLimits)},
          reference{motion::JointSample::zero(config.trajectory->dof())} {}

    JointMask claims() const noexcept override { return config.joints; }

    ControlStep start(ControlContext const &context) noexcept override {
        startTime = context.now;
        config.trajectory->sample(Duration{0}, reference);
        if (exceeds(reference.position, context.state.joints.position, config.trackingTolerance,
                    config.joints)) {
            return ControlStep{.status = ControlStatus::Failed, .fault = FaultCode::StartRejected};
        }
        return ControlStep{};
    }

    ControlStep update(ControlContext const &context, JointCommand &out) noexcept override {
        if (stopRequested and not stopping) {
            beginStop(context);
        }
        if (stopping) {
            auto const &stop = stopPlanner->trajectory();
            auto const elapsed = context.now - stopStart;
            stop.sample(elapsed, reference);
            write(context, out);
            return ControlStep{.status = elapsed >= stop.duration() ? ControlStatus::Stopped
                                                                    : ControlStatus::Running};
        }

        auto const elapsed = context.now - startTime;
        config.trajectory->sample(elapsed, reference);
        write(context, out);
        auto const &measured = context.state.joints.position;
        if (exceeds(reference.position, measured, config.trackingTolerance, config.joints)) {
            return ControlStep{.status = ControlStatus::Failed, .fault = FaultCode::TrackingError};
        }
        if (elapsed >= config.trajectory->duration()) {
            if (not exceeds(reference.position, measured, config.goalTolerance, config.joints)) {
                return ControlStep{.status = ControlStatus::Succeeded};
            }
            if (elapsed >= config.trajectory->duration() + config.goalTimeout) {
                return ControlStep{.status = ControlStatus::Failed, .fault = FaultCode::GoalNotReached};
            }
        }
        return ControlStep{};
    }

    void requestStop() noexcept override { stopRequested = true; }

  private:
    void beginStop(ControlContext const &context) noexcept {
        stopping = true;
        stopStart = context.now;
        if (not stopPlanner->plan(reference)) {
            // No feasible deceleration: hold the last reference.
            reference.velocity.setZero();
            reference.acceleration.setZero();
            static_cast<void>(stopPlanner->plan(reference));
        }
    }

    void write(ControlContext const &context, JointCommand &out) const noexcept {
        for (std::size_t i = 0; i < dofOf(reference.position); ++i) {
            if (not config.joints.test(i)) {
                continue;
            }
            auto const j = idx(i);
            out.position[j] = reference.position[j];
            out.velocity[j] = reference.velocity[j];
            out.effort[j] = config.gravityCompensation ? context.model.gravity[j] : 0.0;
            out.stiffness[j] = config.impedance.stiffness[j];
            out.damping[j] = config.impedance.damping[j];
        }
    }

    JointTrajectoryControllerConfig config;
    std::unique_ptr<motion::StopPlanner> stopPlanner;
    motion::JointSample reference;
    TimePoint startTime{};
    TimePoint stopStart{};
    bool stopRequested{};
    bool stopping{};
};

} // namespace

JointImpedance profileImpedance(RobotProfile const &profile) {
    auto impedance = JointImpedance{
        .stiffness = zeroJointVector(profile.dof()),
        .damping = zeroJointVector(profile.dof()),
    };
    for (std::size_t i = 0; i < profile.dof(); ++i) {
        impedance.stiffness[idx(i)] = profile.joints[i].gains.stiffness;
        impedance.damping[idx(i)] = profile.joints[i].gains.damping;
    }
    return impedance;
}

JointVector profileTrackingTolerance(RobotProfile const &profile) {
    auto tolerance = zeroJointVector(profile.dof());
    for (std::size_t i = 0; i < profile.dof(); ++i) {
        tolerance[idx(i)] = profile.joints[i].unit == JointUnit::Meter ? profile.safety.trackingErrorMeter
                                                                       : profile.safety.trackingErrorRadian;
    }
    return tolerance;
}

std::unique_ptr<Controller> makeJointTrajectoryController(JointTrajectoryControllerConfig config) {
    return std::make_unique<JointTrajectoryControllerImpl>(std::move(config));
}

} // namespace larm::control
