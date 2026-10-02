#include <larm/drivers/robstride/RobStrideDriver.h>
#include <larm/drivers/robstride/SimulatedMotors.h>

#include <gtest/gtest.h>

#include <cstdlib>

namespace larm::drivers::robstride {
namespace {

struct SteppedClock final : hal::Timeline {
    TimePoint now() const noexcept override { return time; }
    void advance() noexcept override { time += std::chrono::milliseconds{4}; }

    TimePoint time{};
};

// A recording bus between the driver and the simulated motors.
struct Tap final : can::CanTransport {
    explicit Tap(std::unique_ptr<can::CanTransport> inner_) : inner{std::move(inner_)} {}

    std::size_t send(std::span<CanFrame const> const frames) noexcept override {
        sent.insert(sent.end(), frames.begin(), frames.end());
        return inner->send(frames);
    }
    std::size_t receive(std::span<CanFrame> const frames) noexcept override { return inner->receive(frames); }
    bool waitReadable(Duration const timeout) noexcept override { return inner->waitReadable(timeout); }
    can::TransportStatistics statistics() const noexcept override { return inner->statistics(); }

    std::unique_ptr<can::CanTransport> inner;
    std::vector<CanFrame> sent;
};

struct RobStrideDriverTest : testing::Test {
    void SetUp() override {
        auto loaded = loadRobotProfile(std::getenv("LARM_ROBOT_PROFILE"));
        ASSERT_TRUE(loaded) << loaded.error().message;
        profile = std::move(*loaded);
        auto parsed = parseDriverConfig(profile);
        ASSERT_TRUE(parsed) << parsed.error().message;
        config = *parsed;
        motors = makeSimulatedMotors(config, {.position = {0.1, 0.2, 0.3, -0.1, 0.05, 0.0, 2.0}});
        auto tap = std::make_unique<Tap>(motors->transport());
        bus = tap.get();
        driver = makeRobStrideDriver(config, std::move(tap), clock);
        state = RobotState::zero(profile.dof());
        command = JointCommand::zero(profile.dof());
    }

    // One control cycle as the control loop runs it.
    void cycle(hal::DrivePower const power, int const count = 1) {
        for (int i = 0; i < count; ++i) {
            driver->io().read(state);
            driver->io().write(command, power);
            clock.advance();
        }
    }

    // Typed frames sent since `from`.
    std::size_t sentOfType(FrameType const type, std::size_t const from = 0) const {
        return static_cast<std::size_t>(
            std::count_if(bus->sent.begin() + static_cast<long>(from), bus->sent.end(),
                          [&](CanFrame const &frame) { return headerOf(frame)->type == type; }));
    }

    bool allEnabled() const {
        return std::all_of(state.actuators.begin(),
                           state.actuators.begin() + static_cast<long>(profile.dof()),
                           [](ActuatorStatus const &actuator) { return actuator.enabled; });
    }

    void enable() {
        ASSERT_TRUE(driver->connect());
        cycle(hal::DrivePower::Disabled, 2);
        command.position = state.joints.position;
        cycle(hal::DrivePower::Enabled, 4);
        ASSERT_TRUE(allEnabled());
    }

    RobotProfile profile;
    DriverConfig config;
    SteppedClock clock;
    std::unique_ptr<SimulatedMotors> motors;
    Tap *bus{};
    std::unique_ptr<RobStrideDriver> driver;
    RobotState state;
    JointCommand command;
};

TEST_F(RobStrideDriverTest, ConnectChecksEveryActuator) {
    ASSERT_TRUE(driver->connect());
    EXPECT_EQ(sentOfType(FrameType::Ping), profile.dof());
    EXPECT_EQ(sentOfType(FrameType::ReadParameter), 2 * profile.dof());
    EXPECT_EQ(sentOfType(FrameType::ActiveReport), profile.dof());
    EXPECT_EQ(sentOfType(FrameType::Enable) + sentOfType(FrameType::Disable), 0u);

    motors->setSilent(3, true);
    auto const silent = driver->connect();
    ASSERT_FALSE(silent);
    EXPECT_NE(silent.error().message.find("actuator 0x4 (joint4) does not answer"), std::string::npos)
        << silent.error().message;

    motors->setSilent(3, false);
    motors->setRunMode(1, 2);
    auto const wrongMode = driver->connect();
    ASSERT_FALSE(wrongMode);
    EXPECT_NE(wrongMode.error().message.find("run_mode 2"), std::string::npos) << wrongMode.error().message;
}

TEST_F(RobStrideDriverTest, PollsPositionsWithoutTouchingPowerWhileDisabled) {
    ASSERT_TRUE(driver->connect());
    auto const before = bus->sent.size();
    cycle(hal::DrivePower::Disabled);
    EXPECT_FALSE(state.isFresh);
    cycle(hal::DrivePower::Disabled, 3);
    EXPECT_TRUE(state.isFresh);
    EXPECT_NEAR(state.joints.position[0], 0.1, 1e-6);
    EXPECT_NEAR(state.joints.position[3], -0.1, 1e-6);
    // The gripper reads 2 rad at the motor through the 7.353 mm/rad pinion.
    EXPECT_NEAR(state.joints.position[6], 2.0 * 0.007353, 1e-8);
    EXPECT_FALSE(state.actuators[0].enabled);
    EXPECT_EQ(sentOfType(FrameType::ReadParameter, before), 4 * profile.dof());
    EXPECT_EQ(sentOfType(FrameType::Enable, before) + sentOfType(FrameType::Disable, before), 0u);
    EXPECT_EQ(state.stamp, TimePoint{std::chrono::milliseconds{12}});
}

TEST_F(RobStrideDriverTest, EnablesAndTracksImpedanceCommands) {
    enable();
    for (std::size_t i = 0; i < profile.dof(); ++i) {
        EXPECT_TRUE(motors->enabled(i));
    }
    auto const before = bus->sent.size();
    command.position[0] = 0.3;
    command.stiffness[0] = 50.0;
    command.damping[0] = 3.0;
    cycle(hal::DrivePower::Enabled, 250);
    EXPECT_EQ(sentOfType(FrameType::Motion, before), 250 * profile.dof());
    EXPECT_NEAR(state.joints.position[0], 0.3, 2e-3);
    EXPECT_NEAR(motors->position(0), 0.3, 2e-3);
}

TEST_F(RobStrideDriverTest, CarriesGripperCommandsThroughTheTransmission) {
    enable();
    command.position[6] = 0.02;
    command.stiffness[6] = 2.0e4;
    command.damping[6] = 200.0;
    cycle(hal::DrivePower::Enabled, 500);
    EXPECT_NEAR(motors->position(6), 0.02 / 0.007353, 1e-2);
    EXPECT_NEAR(state.joints.position[6], 0.02, 1e-4);
}

TEST_F(RobStrideDriverTest, AFaultedMotorStaysOffUntilPowerIsRequestedAgain) {
    enable();
    motors->injectFault(2, status_flag::kOvercurrent, 1U << 16U);
    cycle(hal::DrivePower::Enabled, 20);
    EXPECT_FALSE(state.actuators[2].enabled);
    EXPECT_NE(state.actuators[2].faultBits, 0u);
    EXPECT_TRUE(state.actuators[1].enabled);
    EXPECT_FALSE(motors->enabled(2));
    EXPECT_TRUE(state.isFresh);

    cycle(hal::DrivePower::Disabled, 5);
    for (std::size_t i = 0; i < profile.dof(); ++i) {
        EXPECT_FALSE(motors->enabled(i)) << i;
    }
    command.position = state.joints.position;
    cycle(hal::DrivePower::Enabled, 5);
    EXPECT_TRUE(allEnabled());
    EXPECT_EQ(state.actuators[2].faultBits, 0u);
}

TEST_F(RobStrideDriverTest, DisablingReturnsToPositionPolling) {
    enable();
    cycle(hal::DrivePower::Disabled, 3);
    for (std::size_t i = 0; i < profile.dof(); ++i) {
        EXPECT_FALSE(motors->enabled(i)) << i;
        EXPECT_FALSE(state.actuators[i].enabled) << i;
    }
    auto const before = bus->sent.size();
    cycle(hal::DrivePower::Disabled, 2);
    EXPECT_EQ(sentOfType(FrameType::ReadParameter, before), 2 * profile.dof());
    EXPECT_TRUE(state.isFresh);
}

TEST_F(RobStrideDriverTest, MissingRepliesMakeTheFeedbackStale) {
    enable();
    motors->setSilent(4, true);
    cycle(hal::DrivePower::Enabled, 2);
    EXPECT_FALSE(state.isFresh);
    EXPECT_GE(driver->statistics().missedReplies, 1u);
    motors->setSilent(4, false);
    cycle(hal::DrivePower::Enabled, 2);
    EXPECT_TRUE(state.isFresh);
    EXPECT_EQ(driver->statistics().foreignFrames, 0u);
}

TEST_F(RobStrideDriverTest, ReportsTheBusBudgetAsMinimumPeriod) {
    auto const capabilities = driver->capabilities();
    EXPECT_TRUE(capabilities.supports(hal::CommandMode::Impedance));
    EXPECT_FALSE(capabilities.hasBrakes);
    EXPECT_EQ(capabilities.minPeriod, cycleBusTime(config));
}

} // namespace
} // namespace larm::drivers::robstride
