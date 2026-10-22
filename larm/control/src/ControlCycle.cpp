#include <larm/control/ControlCycle.h>
#include <larm/control/JointTrajectoryController.h>

#include <algorithm>
#include <cmath>

namespace larm::control {
namespace {

constexpr std::size_t kMaxRequestsPerCycle = 16;
constexpr std::size_t kRetireBacklog = 2 * kMaxActiveControllers;

template <class... Handlers> struct Overloaded : Handlers... {
    using Handlers::operator()...;
};
template <class... Handlers> Overloaded(Handlers...) -> Overloaded<Handlers...>;

// Ramping: every actuator is on and some joints are still ramping their stiffness up.
// Degraded: some actuators dropped out while power was requested; the rest hold until disabled, or
// until a fault reset re-arms the dropped ones.
enum class PowerPhase : std::uint8_t { Off, Ramping, On, Degraded };

struct Slot {
    GoalId goal;
    JointMask claims;
    std::unique_ptr<Controller> controller;
};

struct JointLimitTable {
    JointVector lower;
    JointVector upper;
    JointVector velocity;
    JointVector effort;
    double tolerance{};
};

JointLimitTable limitTable(RobotProfile const &profile) {
    auto table = JointLimitTable{
        .lower = zeroJointVector(profile.dof()),
        .upper = zeroJointVector(profile.dof()),
        .velocity = zeroJointVector(profile.dof()),
        .effort = zeroJointVector(profile.dof()),
        .tolerance = profile.safety.limitTolerance,
    };
    for (std::size_t i = 0; i < profile.dof(); ++i) {
        auto const &limits = profile.joints[i].limits;
        table.lower[idx(i)] = limits.lower;
        table.upper[idx(i)] = limits.upper;
        table.velocity[idx(i)] = limits.velocity;
        table.effort[idx(i)] = limits.effort;
    }
    return table;
}

bool isFinite(JointCommand const &command, Eigen::Index const joint) noexcept {
    return std::isfinite(command.position[joint]) and std::isfinite(command.velocity[joint]) and
           std::isfinite(command.effort[joint]) and std::isfinite(command.stiffness[joint]) and
           std::isfinite(command.damping[joint]);
}

struct ControlCycleImpl final : ControlCycle {
    ControlCycleImpl(RobotProfile const &profile, ControlCycleDeps const &deps)
        : dof{profile.dof()}, period{profile.controlPeriod}, io{&deps.driver->io()}, channels{deps.channels},
          kinematics{deps.model->makeKinematics()}, dynamics{deps.model->makeDynamics()},
          limits{limitTable(profile)}, hold{profileImpedance(profile)},
          feedbackTimeoutCycles{profile.safety.feedbackTimeoutCycles}, enableRamp{profile.safety.enableRamp},
          state{RobotState::zero(profile.dof())}, command{JointCommand::zero(profile.dof())},
          holdTarget{zeroJointVector(profile.dof())},
          cache{.kinematics = kinematics.get(), .gravity = zeroJointVector(profile.dof())},
          snapshot{.state = RobotState::zero(profile.dof()), .command = JointCommand::zero(profile.dof())} {
        for (std::size_t i = 0; i < dof; ++i) {
            allJoints.set(i);
        }
    }

    void prepareThread() noexcept override { io->prepareThread(); }

    void tick() noexcept override {
        auto const started = std::chrono::steady_clock::now();
        io->read(state);
        trackFeedback();
        updateModel();
        updatePower();
        drainRequests();
        checkLimits();
        runControllers();
        fillUnclaimed();
        enforceLimits();
        io->write(command, hal::PowerRequest{requestedPower, rearm});
        publish(started);
        flushRetired();
    }

  private:
    ControlContext context() const noexcept {
        return ControlContext{.now = state.stamp, .period = period, .state = state, .model = cache};
    }

    // Stale feedback is a fault while power is requested. Once the motors are told to stop it changes
    // nothing: they keep being told until they answer, and disabled, stale feedback only keeps the arm
    // from being enabled. RobStride motors fall silent for 20-25 ms on a stop, sometimes before they
    // answer it.
    void trackFeedback() noexcept {
        if (state.isFresh or requestedPower == hal::DrivePower::Disabled) {
            staleCycles = 0;
            return;
        }
        if (++staleCycles >= feedbackTimeoutCycles) {
            raiseFault(FaultCode::FeedbackLost);
        }
    }

    void updateModel() noexcept {
        if (not state.isFresh) {
            return;
        }
        kinematics->update(state.joints.position);
        dynamics->gravity(state.joints.position, cache.gravity);
    }

    void updatePower() noexcept {
        auto on = JointMask{};
        for (std::size_t i = 0; i < dof; ++i) {
            on.set(i, state.actuators[i].enabled);
        }
        auto const actuatorsOn = on == allJoints;
        auto const restarted = on & ~powered;
        powered = on;
        switch (phase) {
        case PowerPhase::Off:
            if (actuatorsOn and state.isFresh and requestedPower == hal::DrivePower::Enabled) {
                phase = PowerPhase::Ramping;
                holdTarget = state.joints.position;
                startRamp(allJoints);
            }
            break;
        case PowerPhase::Ramping:
        case PowerPhase::On:
            if (not actuatorsOn) {
                loseActuators(on.any());
                break;
            }
            finishRamps();
            if (phase == PowerPhase::Ramping and ramping.none()) {
                phase = PowerPhase::On;
                emit(PowerChanged{.power = hal::DrivePower::Enabled});
            }
            break;
        case PowerPhase::Degraded:
            if (on.none()) {
                phase = PowerPhase::Off;
                break;
            }
            // Re-armed actuators resume where their joints are now, softly.
            for (std::size_t i = 0; i < dof; ++i) {
                if (restarted.test(i)) {
                    holdTarget[idx(i)] = state.joints.position[idx(i)];
                }
            }
            startRamp(restarted);
            if (actuatorsOn and state.isFresh) {
                phase = PowerPhase::Ramping;
                becomeNormal();
            }
            break;
        }
    }

    void startRamp(JointMask const &joints) noexcept {
        for (std::size_t i = 0; i < dof; ++i) {
            if (joints.test(i)) {
                rampStart[i] = state.stamp;
            }
        }
        ramping |= joints;
    }

    void finishRamps() noexcept {
        for (std::size_t i = 0; i < dof; ++i) {
            if (ramping.test(i) and state.stamp - rampStart[i] >= enableRamp) {
                ramping.reset(i);
            }
        }
    }

    // Fraction of the holding stiffness, damping and gravity feedforward of a powered joint. The
    // feedforward ramps too: a model heavier than the arm would otherwise lift it off its rest before
    // the stiffness can hold it.
    double rampScale(std::size_t const joint) const noexcept {
        if (not ramping.test(joint) or enableRamp <= Duration{0}) {
            return 1.0;
        }
        return std::clamp(toSeconds(state.stamp - rampStart[joint]) / toSeconds(enableRamp), 0.0, 1.0);
    }

    // An actuator that drops out unasked is a fault; the others keep holding where the arm is.
    void loseActuators(bool const anyOn) noexcept {
        auto const wasOn = phase == PowerPhase::On;
        auto const unasked = requestedPower == hal::DrivePower::Enabled;
        if (unasked) {
            raiseFault(FaultCode::ActuatorFault);
        }
        abortAll(ControlStatus::Stopped, FaultCode::NotEnabled);
        phase = unasked and anyOn ? PowerPhase::Degraded : PowerPhase::Off;
        if (wasOn) {
            emit(PowerChanged{.power = hal::DrivePower::Disabled});
        }
    }

    void drainRequests() noexcept {
        for (std::size_t handled = 0; handled < kMaxRequestsPerCycle; ++handled) {
            auto request = channels->requests.tryPop();
            if (not request) {
                return;
            }
            std::visit(Overloaded{
                           [&](ActivateController &activate) { activateController(activate); },
                           [&](CancelGoal const &cancel) {
                               if (auto *const slot = findSlot(cancel.goal)) {
                                   slot->controller->requestStop();
                               }
                           },
                           [&](SetDrivePower const &power) {
                               requestedPower = power.power;
                               if (power.power == hal::DrivePower::Disabled) {
                                   abortAll(ControlStatus::Stopped, FaultCode::NotEnabled);
                               }
                           },
                           [&](EmergencyStop) { emergencyStop(); },
                           [&](ResetFault) { resetFault(); },
                       },
                       *request);
        }
    }

    void activateController(ActivateController &activate) noexcept {
        if (not activate.controller) {
            return;
        }
        auto const reject = [&](FaultCode const reason) {
            emit(GoalFinished{.goal = activate.goal, .status = ControlStatus::Failed, .fault = reason});
            retire(std::move(activate.controller));
        };
        if (phase != PowerPhase::On) {
            return reject(FaultCode::NotEnabled);
        }
        if (safety != SafetyState::Normal) {
            return reject(fault);
        }
        auto const claims = activate.controller->claims();
        if (claims.none() or (claims & ~allJoints).any() or (claims & claimedJoints()).any()) {
            return reject(FaultCode::ClaimConflict);
        }
        auto *const slot = freeSlot();
        if (slot == nullptr) {
            return reject(FaultCode::ClaimConflict);
        }
        auto const step = activate.controller->start(context());
        if (step.status != ControlStatus::Running) {
            emit(GoalFinished{.goal = activate.goal, .status = step.status, .fault = step.fault});
            retire(std::move(activate.controller));
            return;
        }
        *slot = Slot{.goal = activate.goal, .claims = claims, .controller = std::move(activate.controller)};
    }

    void emergencyStop() noexcept {
        if (safety == SafetyState::EmergencyStopped) {
            return;
        }
        safety = SafetyState::EmergencyStopped;
        fault = FaultCode::EmergencyStop;
        abortAll(ControlStatus::Failed, FaultCode::EmergencyStop);
        emit(SafetyChanged{.state = safety, .cause = fault});
    }

    void raiseFault(FaultCode const cause) noexcept {
        if (safety != SafetyState::Normal) {
            return;
        }
        safety = SafetyState::Faulted;
        fault = cause;
        abortAll(ControlStatus::Failed, cause);
        emit(SafetyChanged{.state = safety, .cause = cause});
    }

    void resetFault() noexcept {
        if (safety == SafetyState::Normal or not state.isFresh or outsideLimits()) {
            return;
        }
        if (phase == PowerPhase::Degraded and requestedPower == hal::DrivePower::Enabled) {
            // The driver clears and re-enables the actuators that dropped out; safety returns to
            // normal once they are all back (updatePower).
            ++rearm;
            return;
        }
        becomeNormal();
    }

    void becomeNormal() noexcept {
        if (safety == SafetyState::Normal) {
            return;
        }
        safety = SafetyState::Normal;
        fault = FaultCode::None;
        holdTarget = state.joints.position;
        emit(SafetyChanged{.state = safety, .cause = fault});
    }

    bool outsideLimits() const noexcept {
        auto const &q = state.joints.position;
        return ((q.array() < limits.lower.array() - limits.tolerance) or
                (q.array() > limits.upper.array() + limits.tolerance))
            .any();
    }

    void checkLimits() noexcept {
        if (state.isFresh and outsideLimits()) {
            raiseFault(FaultCode::PositionLimit);
        }
    }

    void runControllers() noexcept {
        auto const current = context();
        for (auto &slot : slots) {
            if (not slot.controller) {
                continue;
            }
            auto const step = slot.controller->update(current, command);
            if (step.status == ControlStatus::Running) {
                continue;
            }
            // A failed controller may have left its reference far from the arm, so hold where the arm is.
            auto const &resume =
                step.status == ControlStatus::Failed ? state.joints.position : command.position;
            for (std::size_t i = 0; i < dof; ++i) {
                if (slot.claims.test(i)) {
                    holdTarget[idx(i)] = resume[idx(i)];
                }
            }
            emit(GoalFinished{.goal = slot.goal, .status = step.status, .fault = step.fault});
            release(slot);
        }
    }

    void fillUnclaimed() noexcept {
        if (phase == PowerPhase::Off) {
            holdTarget = state.joints.position;
            command.position = state.joints.position;
            command.velocity.setZero();
            command.effort.setZero();
            command.stiffness.setZero();
            command.damping.setZero();
            return;
        }
        auto const claimed = claimedJoints();
        for (std::size_t i = 0; i < dof; ++i) {
            if (claimed.test(i)) {
                continue;
            }
            auto const j = idx(i);
            // An unpowered joint is followed, so that it resumes where it is once powered again.
            if (not powered.test(i)) {
                holdTarget[j] = state.joints.position[j];
            }
            auto const scale = powered.test(i) ? rampScale(i) : 0.0;
            command.position[j] = holdTarget[j];
            command.velocity[j] = 0.0;
            command.effort[j] = scale * cache.gravity[j];
            command.stiffness[j] = scale * hold.stiffness[j];
            command.damping[j] = scale * hold.damping[j];
        }
    }

    void enforceLimits() noexcept {
        for (std::size_t i = 0; i < dof; ++i) {
            auto const j = idx(i);
            if (not isFinite(command, j)) {
                raiseFault(FaultCode::InvalidCommand);
                command.position[j] = state.joints.position[j];
                command.velocity[j] = 0.0;
                command.effort[j] = cache.gravity[j];
                command.stiffness[j] = hold.stiffness[j];
                command.damping[j] = hold.damping[j];
            }
            command.position[j] = std::clamp(command.position[j], limits.lower[j], limits.upper[j]);
            command.velocity[j] = std::clamp(command.velocity[j], -limits.velocity[j], limits.velocity[j]);
            command.effort[j] = std::clamp(command.effort[j], -limits.effort[j], limits.effort[j]);
            command.stiffness[j] = std::max(command.stiffness[j], 0.0);
            command.damping[j] = std::max(command.damping[j], 0.0);
        }
    }

    void publish(std::chrono::steady_clock::time_point const started) noexcept {
        snapshot.state = state;
        snapshot.command = command;
        snapshot.safety = safety;
        snapshot.fault = fault;
        snapshot.power = phase == PowerPhase::On ? hal::DrivePower::Enabled : hal::DrivePower::Disabled;
        snapshot.activeCount = 0;
        for (auto const &slot : slots) {
            if (slot.controller) {
                snapshot.active[snapshot.activeCount++] =
                    ActiveGoal{.goal = slot.goal, .joints = slot.claims};
            }
        }
        channels->snapshot.publish(snapshot);
        auto telemetry = CycleTelemetry{
            .cycle = state.cycle,
            .stamp = state.stamp,
            .computeTime = std::chrono::steady_clock::now() - started,
        };
        static_cast<void>(channels->telemetry.tryPush(std::move(telemetry)));
    }

    void abortAll(ControlStatus const status, FaultCode const cause) noexcept {
        for (auto &slot : slots) {
            if (slot.controller) {
                emit(GoalFinished{.goal = slot.goal, .status = status, .fault = cause});
                release(slot);
            }
        }
        holdTarget = state.joints.position;
    }

    void release(Slot &slot) noexcept {
        retire(std::move(slot.controller));
        slot = Slot{};
    }

    void retire(std::unique_ptr<Controller> controller) noexcept {
        if (not controller or channels->retired.tryPush(std::move(controller))) {
            return;
        }
        auto const free = std::find(retireBacklog.begin(), retireBacklog.end(), nullptr);
        if (free != retireBacklog.end()) {
            *free = std::move(controller);
        }
        // With the ring and the backlog full, the controller is freed here as a last resort.
    }

    void flushRetired() noexcept {
        for (auto &pending : retireBacklog) {
            if (pending and not channels->retired.tryPush(std::move(pending))) {
                return;
            }
        }
    }

    void emit(ControlEvent event) noexcept { static_cast<void>(channels->events.tryPush(std::move(event))); }

    JointMask claimedJoints() const noexcept {
        auto claimed = JointMask{};
        for (auto const &slot : slots) {
            claimed |= slot.claims;
        }
        return claimed;
    }

    Slot *findSlot(GoalId const goal) noexcept {
        auto const found = std::find_if(slots.begin(), slots.end(), [&](Slot const &slot) {
            return slot.controller and slot.goal == goal;
        });
        return found == slots.end() ? nullptr : &*found;
    }

    Slot *freeSlot() noexcept {
        auto const found =
            std::find_if(slots.begin(), slots.end(), [](Slot const &slot) { return not slot.controller; });
        return found == slots.end() ? nullptr : &*found;
    }

    std::size_t dof;
    Duration period;
    hal::RealtimeIo *io;
    RuntimeChannels *channels;
    std::unique_ptr<model::Kinematics> kinematics;
    std::unique_ptr<model::Dynamics> dynamics;
    JointLimitTable limits;
    JointImpedance hold;
    std::uint32_t feedbackTimeoutCycles;
    Duration enableRamp;
    JointMask allJoints;

    RobotState state;
    JointCommand command;
    JointVector holdTarget;
    ModelCache cache;
    RobotSnapshot snapshot;
    std::array<Slot, kMaxActiveControllers> slots{};
    std::array<std::unique_ptr<Controller>, kRetireBacklog> retireBacklog{};
    std::uint32_t staleCycles{};
    hal::DrivePower requestedPower = hal::DrivePower::Disabled;
    std::uint32_t rearm{};
    PowerPhase phase = PowerPhase::Off;
    // Actuators reporting enabled this cycle.
    JointMask powered;
    // Joints whose stiffness is still ramping up, and since when.
    JointMask ramping;
    std::array<TimePoint, kMaxDof> rampStart{};
    SafetyState safety = SafetyState::Normal;
    FaultCode fault = FaultCode::None;
};

} // namespace

Expected<std::unique_ptr<ControlCycle>> makeControlCycle(RobotProfile const &profile,
                                                         ControlCycleDeps const &deps) {
    if (deps.driver == nullptr or deps.model == nullptr or deps.channels == nullptr) {
        return makeError(ErrorCode::InvalidArgument, "control cycle needs a driver, a model and channels");
    }
    auto const capabilities = deps.driver->capabilities();
    if (not capabilities.supports(hal::CommandMode::Impedance)) {
        return makeError(ErrorCode::Unsupported, "the driver does not accept impedance commands");
    }
    if (profile.controlPeriod < capabilities.minPeriod) {
        return makeError(ErrorCode::InvalidConfig,
                         "the control period of " + std::to_string(toSeconds(profile.controlPeriod) * 1e3) +
                             " ms is shorter than the driver's minimum of " +
                             std::to_string(toSeconds(capabilities.minPeriod) * 1e3) + " ms");
    }
    if (deps.model->dof() != profile.dof()) {
        return makeError(ErrorCode::InvalidArgument, "the model and the profile disagree on the joint count");
    }
    return std::unique_ptr<ControlCycle>{std::make_unique<ControlCycleImpl>(profile, deps)};
}

} // namespace larm::control
