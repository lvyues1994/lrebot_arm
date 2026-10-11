#include <larm/hal/IdealBackend.h>

namespace larm::hal {
namespace {

struct IdealPlant {
    Duration period;
    TimePoint time{};
    std::uint64_t cycle{};
    RobotState state;
    JointCommand command;
    PowerRequest power;
    JointMask dropped;
    bool feedbackLost{};

    void request(PowerRequest const &requested) noexcept {
        if (requested.power == DrivePower::Disabled or requested.rearm != power.rearm) {
            dropped.reset();
        }
        power = requested;
    }

    void step() noexcept {
        for (std::size_t i = 0; i < dofOf(state.joints.position); ++i) {
            auto const enabled = power.power == DrivePower::Enabled and not dropped.test(i);
            state.actuators[i].enabled = enabled;
            auto const j = idx(i);
            state.joints.position[j] = enabled ? command.position[j] : state.joints.position[j];
            state.joints.velocity[j] = enabled ? command.velocity[j] : 0.0;
            state.joints.effort[j] = enabled ? command.effort[j] : 0.0;
        }
        time += period;
        ++cycle;
    }
};

struct IdealIo final : RealtimeIo {
    explicit IdealIo(IdealPlant *plant_) : plant{plant_} {}

    void read(RobotState &out) noexcept override {
        if (not plant->feedbackLost) {
            out = plant->state;
        }
        out.cycle = plant->cycle;
        out.stamp = plant->time;
        out.isFresh = not plant->feedbackLost;
    }

    void write(JointCommand const &command, PowerRequest const &requested) noexcept override {
        plant->command = command;
        plant->request(requested);
    }

  private:
    IdealPlant *plant;
};

struct IdealDriver final : RobotDriver {
    explicit IdealDriver(IdealPlant *plant) : realtimeIo{plant} {}

    DriverCapabilities capabilities() const override {
        auto modes = std::bitset<4>{};
        modes.set(static_cast<std::size_t>(CommandMode::Impedance));
        return DriverCapabilities{.modes = modes};
    }

    Expected<void> connect() override { return {}; }
    void disconnect() noexcept override {}
    RealtimeIo &io() override { return realtimeIo; }

  private:
    IdealIo realtimeIo;
};

struct IdealTimeline final : Timeline {
    explicit IdealTimeline(IdealPlant *plant_) : plant{plant_} {}

    TimePoint now() const noexcept override { return plant->time; }
    void advance() noexcept override { plant->step(); }

  private:
    IdealPlant *plant;
};

struct IdealBackendImpl final : IdealBackend {
    explicit IdealBackendImpl(IdealBackendConfig const &config)
        : plant{.period = config.period,
                .state = RobotState::zero(dofOf(config.initialPosition)),
                .command = JointCommand::zero(dofOf(config.initialPosition))} {
        plant.state.joints.position = config.initialPosition;
        plant.command.position = config.initialPosition;
    }
    IdealBackendImpl(IdealBackendImpl const &) = delete;
    IdealBackendImpl &operator=(IdealBackendImpl const &) = delete;

    RobotDriver &driver() override { return idealDriver; }
    Timeline &timeline() override { return idealTimeline; }
    void setFeedbackLost(bool const lost) noexcept override { plant.feedbackLost = lost; }
    void dropActuator(std::size_t const actuator) noexcept override { plant.dropped.set(actuator); }
    JointCommand const &lastCommand() const noexcept override { return plant.command; }
    DrivePower lastRequestedPower() const noexcept override { return plant.power.power; }

  private:
    IdealPlant plant;
    IdealDriver idealDriver{&plant};
    IdealTimeline idealTimeline{&plant};
};

} // namespace

std::unique_ptr<IdealBackend> makeIdealBackend(IdealBackendConfig const &config) {
    return std::make_unique<IdealBackendImpl>(config);
}

} // namespace larm::hal
