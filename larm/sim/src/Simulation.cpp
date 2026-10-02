#include "MujocoWorld.h"

#include <larm/sim/Simulation.h>

#include <algorithm>
#include <thread>

namespace larm::sim {
namespace {

using WallClock = std::chrono::steady_clock;

struct SimulationConfig {
    Duration physicsStep;
    std::uint64_t substeps{};
};

Expected<SimulationConfig> parseConfig(RobotProfile const &profile) {
    auto const &node = profile.sim.node;
    if (not node.IsMap() or node.size() != 1 or not node["physics_step_us"]) {
        return makeError(ErrorCode::InvalidConfig, "sim: expected exactly the key 'physics_step_us'");
    }
    auto const microseconds = node["physics_step_us"].as<std::int64_t>(0);
    if (microseconds <= 0) {
        return makeError(ErrorCode::InvalidConfig, "sim.physics_step_us must be a positive integer");
    }
    auto const step = Duration{std::chrono::microseconds{microseconds}};
    if (profile.controlPeriod % step != Duration{0}) {
        return makeError(ErrorCode::InvalidConfig,
                         "the control period must be a whole number of physics steps");
    }
    return SimulationConfig{.physicsStep = step,
                            .substeps = static_cast<std::uint64_t>(profile.controlPeriod / step)};
}

// State shared by the driver, I/O and timeline views of one simulation.
struct SimulationState {
    MujocoWorld world;
    SimulationConfig config;
    SimulationOptions options;
    std::array<double, kMaxDof> effortLimit{};
    JointCommand command;
    hal::DrivePower power = hal::DrivePower::Disabled;
    std::uint64_t cycle{};
    std::uint64_t steps{};
    std::optional<WallClock::time_point> wallEpoch;

    Duration simulatedTime() const noexcept { return config.physicsStep * static_cast<std::int64_t>(steps); }

    // MIT impedance law with torque saturation, evaluated at every physics step.
    void applyActuators() noexcept {
        auto const enabled = power == hal::DrivePower::Enabled;
        auto const &data = *world.data;
        for (std::size_t i = 0; i < world.dof; ++i) {
            auto const &joint = world.joints[i];
            auto effort = 0.0;
            if (enabled) {
                auto const j = idx(i);
                effort = command.stiffness[j] * (command.position[j] - data.qpos[joint.qposAddress]) +
                         command.damping[j] * (command.velocity[j] - data.qvel[joint.dofAddress]) +
                         command.effort[j];
            }
            world.data->ctrl[joint.actuator] = std::clamp(effort, -effortLimit[i], effortLimit[i]);
        }
    }

    void step() noexcept {
        for (std::uint64_t substep = 0; substep < config.substeps; ++substep) {
            applyActuators();
            mj_step(world.model.get(), world.data.get());
            ++steps;
        }
        ++cycle;
    }

    void pace() {
        if (options.pacing != Pacing::RealTime) {
            return;
        }
        auto const simulated = std::chrono::duration_cast<WallClock::duration>(
            std::chrono::duration<double>{toSeconds(simulatedTime()) / options.realTimeFactor});
        if (not wallEpoch) {
            wallEpoch = WallClock::now() - simulated;
        }
        std::this_thread::sleep_until(*wallEpoch + simulated);
    }
};

struct SimulationIo final : hal::RealtimeIo {
    explicit SimulationIo(SimulationState *state_) : state{state_} {}

    void read(RobotState &out) noexcept override {
        auto const &data = *state->world.data;
        auto const dof = state->world.dof;
        out.cycle = state->cycle;
        out.stamp = TimePoint{state->simulatedTime()};
        out.isFresh = true;
        out.joints.position.resize(idx(dof));
        out.joints.velocity.resize(idx(dof));
        out.joints.effort.resize(idx(dof));
        for (std::size_t i = 0; i < dof; ++i) {
            auto const &joint = state->world.joints[i];
            out.joints.position[idx(i)] = data.qpos[joint.qposAddress];
            out.joints.velocity[idx(i)] = data.qvel[joint.dofAddress];
            out.joints.effort[idx(i)] = data.actuator_force[joint.actuator];
            out.actuators[i] = ActuatorStatus{.enabled = state->power == hal::DrivePower::Enabled};
        }
    }

    void write(JointCommand const &command, hal::DrivePower const requested) noexcept override {
        state->command = command;
        state->power = requested;
    }

  private:
    SimulationState *state;
};

struct SimulationDriver final : hal::RobotDriver {
    explicit SimulationDriver(SimulationState *state) : realtimeIo{state} {}

    hal::DriverCapabilities capabilities() const override {
        auto modes = std::bitset<4>{};
        modes.set(static_cast<std::size_t>(hal::CommandMode::Impedance));
        return hal::DriverCapabilities{.modes = modes};
    }

    Expected<void> connect() override { return {}; }
    void disconnect() noexcept override {}
    hal::RealtimeIo &io() override { return realtimeIo; }

  private:
    SimulationIo realtimeIo;
};

struct SimulationTimeline final : hal::Timeline {
    explicit SimulationTimeline(SimulationState *state_) : state{state_} {}

    TimePoint now() const noexcept override { return TimePoint{state->simulatedTime()}; }

    void advance() noexcept override {
        state->step();
        state->pace();
    }

  private:
    SimulationState *state;
};

struct SimulationImpl final : Simulation {
    explicit SimulationImpl(SimulationState state_) : state{std::move(state_)} {}
    SimulationImpl(SimulationImpl const &) = delete;
    SimulationImpl &operator=(SimulationImpl const &) = delete;

    hal::RobotDriver &driver() override { return simulationDriver; }
    hal::Timeline &timeline() override { return simulationTimeline; }

    void reset(JointVector const &position) override {
        auto &world = state.world;
        mj_resetData(world.model.get(), world.data.get());
        for (std::size_t i = 0; i < world.dof; ++i) {
            world.data->qpos[world.joints[i].qposAddress] = position[idx(i)];
        }
        mj_forward(world.model.get(), world.data.get());
        state.command = JointCommand::zero(world.dof);
        state.command.position = position;
        state.steps = 0;
        state.cycle = 0;
        state.wallEpoch.reset();
    }

    mjModel const &model() const noexcept override { return *state.world.model; }
    mjData const &data() const noexcept override { return *state.world.data; }
    Duration physicsStep() const noexcept override { return state.config.physicsStep; }

  private:
    SimulationState state;
    SimulationDriver simulationDriver{&state};
    SimulationTimeline simulationTimeline{&state};
};

} // namespace

Expected<std::unique_ptr<Simulation>> makeSimulation(RobotProfile const &profile,
                                                     SimulationOptions const &options) {
    if (options.pacing == Pacing::RealTime and not(options.realTimeFactor > 0.0)) {
        return makeError(ErrorCode::InvalidArgument, "the real-time factor must be positive");
    }
    auto config = parseConfig(profile);
    if (not config) {
        return tl::make_unexpected(config.error());
    }
    auto world = loadMujocoWorld(profile);
    if (not world) {
        return tl::make_unexpected(world.error());
    }
    world->model->opt.timestep = toSeconds(config->physicsStep);

    auto state = SimulationState{
        .world = std::move(*world),
        .config = *config,
        .options = options,
        .command = JointCommand::zero(profile.dof()),
    };
    for (std::size_t i = 0; i < profile.dof(); ++i) {
        state.effortLimit[i] = profile.joints[i].limits.effort;
    }
    auto simulation = std::make_unique<SimulationImpl>(std::move(state));
    simulation->reset(profile.safety.restPose);
    return std::unique_ptr<Simulation>{std::move(simulation)};
}

} // namespace larm::sim
