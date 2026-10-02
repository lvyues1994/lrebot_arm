#include <larm/hal/IdealBackend.h>

#include <gtest/gtest.h>

namespace larm::hal {
namespace {

TEST(IdealBackend, FollowsCommandsOnlyWhenEnabled) {
    auto initial = JointVector{2};
    initial << 0.1, 0.2;
    auto backend = makeIdealBackend({.initialPosition = initial, .period = std::chrono::milliseconds{4}});
    auto &io = backend->driver().io();
    ASSERT_TRUE(backend->driver().connect());
    EXPECT_TRUE(backend->driver().capabilities().supports(CommandMode::Impedance));

    auto command = JointCommand::zero(2);
    command.position << 1.0, 2.0;
    io.write(command, DrivePower::Disabled);
    backend->timeline().advance();
    auto state = RobotState::zero(2);
    io.read(state);
    EXPECT_TRUE(state.isFresh);
    EXPECT_FALSE(state.actuators[0].enabled);
    EXPECT_TRUE(state.joints.position.isApprox(initial));

    io.write(command, DrivePower::Enabled);
    backend->timeline().advance();
    io.read(state);
    EXPECT_TRUE(state.actuators[1].enabled);
    EXPECT_TRUE(state.joints.position.isApprox(command.position));
    EXPECT_EQ(state.cycle, 2u);
    EXPECT_EQ(backend->timeline().now().time_since_epoch(), std::chrono::milliseconds{8});
}

TEST(IdealBackend, ReportsStaleFeedbackWhileLost) {
    auto backend =
        makeIdealBackend({.initialPosition = JointVector::Zero(1), .period = std::chrono::milliseconds{1}});
    backend->setFeedbackLost(true);
    backend->timeline().advance();
    auto state = RobotState::zero(1);
    backend->driver().io().read(state);
    EXPECT_FALSE(state.isFresh);
    EXPECT_EQ(state.cycle, 1u);
}

} // namespace
} // namespace larm::hal
