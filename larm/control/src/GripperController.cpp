#include <larm/control/GripperController.h>

#include <algorithm>
#include <cmath>

namespace larm::control {
namespace {

struct GripperControllerImpl final : Controller {
    explicit GripperControllerImpl(GripperControllerConfig const &config_) : config{config_} {}

    JointMask claims() const noexcept override {
        auto mask = JointMask{};
        mask.set(config.joint);
        return mask;
    }

    ControlStep start(ControlContext const &context) noexcept override {
        reference = context.state.joints.position[idx(config.joint)];
        return ControlStep{};
    }

    ControlStep update(ControlContext const &context, JointCommand &out) noexcept override {
        auto const j = idx(config.joint);
        auto const position = context.state.joints.position[j];
        auto const velocity = context.state.joints.velocity[j];
        auto const goal = stopping ? reference : config.target;
        auto const step = config.speed * toSeconds(context.period);
        reference =
            std::abs(goal - reference) <= step ? goal : reference + std::copysign(step, goal - reference);

        auto const reach = config.stiffness > 0.0 ? config.maxEffort / config.stiffness : 0.0;
        out.position[j] = position + std::clamp(reference - position, -reach, reach);
        out.velocity[j] = 0.0;
        out.effort[j] = context.model.gravity[j];
        out.stiffness[j] = config.stiffness;
        out.damping[j] = config.damping;

        auto const still = std::abs(velocity) < config.stallVelocity;
        stillFor = still ? stillFor + context.period : Duration{0};
        if (stopping) {
            return ControlStep{.status = still ? ControlStatus::Stopped : ControlStatus::Running};
        }
        if (reference != config.target) {
            return ControlStep{};
        }
        if (std::abs(position - config.target) <= config.positionTolerance or stillFor >= config.stallTime) {
            return ControlStep{.status = ControlStatus::Succeeded};
        }
        return ControlStep{};
    }

    void requestStop() noexcept override { stopping = true; }

  private:
    GripperControllerConfig config;
    double reference{};
    Duration stillFor{};
    bool stopping{};
};

} // namespace

std::unique_ptr<Controller> makeGripperController(GripperControllerConfig const &config) {
    return std::make_unique<GripperControllerImpl>(config);
}

} // namespace larm::control
