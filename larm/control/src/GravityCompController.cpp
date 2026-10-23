#include <larm/control/GravityCompController.h>

#include <algorithm>
#include <cmath>

namespace larm::control {
namespace {

constexpr double kFreeDampingFraction = 0.2;
constexpr double kWallMarginRadian = 0.1;
constexpr double kWallMarginMeter = 0.005;

struct GravityCompControllerImpl final : Controller {
    explicit GravityCompControllerImpl(GravityCompControllerConfig config_) : config{std::move(config_)} {}

    JointMask claims() const noexcept override { return config.joints; }

    ControlStep start(ControlContext const &) noexcept override { return ControlStep{}; }

    ControlStep update(ControlContext const &context, JointCommand &out) noexcept override {
        auto const &joints = context.state.joints;
        for (std::size_t i = 0; i < kMaxDof; ++i) {
            if (not config.joints.test(i)) {
                continue;
            }
            auto const j = idx(i);
            auto const q = joints.position[j];
            auto target = q;
            auto stiffness = 0.0;
            auto damping = config.damping[j];
            if (not stopping and (q < config.wallLower[j] or q > config.wallUpper[j])) {
                target = std::clamp(q, config.wallLower[j], config.wallUpper[j]);
                stiffness = config.wallStiffness[j];
                damping = config.wallDamping[j];
            }
            out.position[j] = target;
            out.velocity[j] = 0.0;
            out.effort[j] = context.model.gravity[j];
            out.stiffness[j] = stiffness;
            out.damping[j] = damping;
        }
        if (stopping) {
            return ControlStep{.status = ControlStatus::Stopped};
        }
        for (std::size_t i = 0; i < kMaxDof; ++i) {
            if (config.joints.test(i) and std::abs(joints.velocity[idx(i)]) > config.velocityLimit[idx(i)]) {
                return ControlStep{.status = ControlStatus::Failed, .fault = FaultCode::TrackingError};
            }
        }
        return ControlStep{};
    }

    void requestStop() noexcept override { stopping = true; }

  private:
    GravityCompControllerConfig config;
    bool stopping{};
};

} // namespace

GravityCompControllerConfig profileGravityComp(RobotProfile const &profile, JointMask const joints) {
    auto const dof = profile.dof();
    auto config = GravityCompControllerConfig{
        .joints = joints,
        .damping = zeroJointVector(dof),
        .wallLower = zeroJointVector(dof),
        .wallUpper = zeroJointVector(dof),
        .wallStiffness = zeroJointVector(dof),
        .wallDamping = zeroJointVector(dof),
        .velocityLimit = zeroJointVector(dof),
    };
    for (std::size_t i = 0; i < dof; ++i) {
        auto const &joint = profile.joints[i];
        auto const j = idx(i);
        auto const margin = joint.unit == JointUnit::Radian ? kWallMarginRadian : kWallMarginMeter;
        auto const rest = std::clamp(profile.safety.restPose[j], joint.limits.lower, joint.limits.upper);
        config.damping[j] = kFreeDampingFraction * joint.gains.damping;
        config.wallLower[j] = std::min(joint.limits.lower + margin, rest);
        config.wallUpper[j] = std::max(joint.limits.upper - margin, rest);
        config.wallStiffness[j] = joint.gains.stiffness;
        config.wallDamping[j] = joint.gains.damping;
        config.velocityLimit[j] = joint.limits.velocity;
    }
    return config;
}

std::unique_ptr<Controller> makeGravityCompController(GravityCompControllerConfig config) {
    return std::make_unique<GravityCompControllerImpl>(std::move(config));
}

} // namespace larm::control
