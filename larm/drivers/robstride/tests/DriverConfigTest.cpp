#include <larm/drivers/robstride/DriverConfig.h>

#include <gtest/gtest.h>

#include <cstdlib>

namespace larm::drivers::robstride {
namespace {

struct RebotDriverConfig : testing::Test {
    void SetUp() override {
        auto loaded = loadRobotProfile(std::getenv("LARM_ROBOT_PROFILE"));
        ASSERT_TRUE(loaded) << loaded.error().message;
        profile = std::move(*loaded);
    }

    // The actuator entry for `joint` in the profile's driver section.
    YAML::Node actuator(std::string const &joint) {
        for (auto entry : profile.driver.node["actuators"]) {
            if (entry["joint"].as<std::string>() == joint) {
                return entry;
            }
        }
        return {};
    }

    void expectRejected(std::string const &fragment) {
        auto const config = parseDriverConfig(profile);
        ASSERT_FALSE(config);
        EXPECT_NE(config.error().message.find(fragment), std::string::npos) << config.error().message;
    }

    RobotProfile profile;
};

TEST_F(RebotDriverConfig, ParsesTheProfile) {
    auto const config = parseDriverConfig(profile);
    ASSERT_TRUE(config) << config.error().message;
    EXPECT_EQ(config->interface, "can0");
    EXPECT_EQ(config->hostId, 0xFD);
    ASSERT_EQ(config->actuators.size(), profile.dof());
    for (std::size_t i = 0; i < profile.dof(); ++i) {
        EXPECT_EQ(config->actuators[i].joint, i);
        EXPECT_EQ(config->actuators[i].jointName, profile.joints[i].name);
        EXPECT_EQ(config->actuators[i].id, i + 1);
    }
    EXPECT_EQ(config->actuators[1].model, MotorModel::Rs06);
    EXPECT_EQ(config->actuators[4].model, MotorModel::Rs00);
    EXPECT_DOUBLE_EQ(config->actuators[0].transmission.scale, 1.0);
    EXPECT_NEAR(config->actuators[6].transmission.scale, 0.007353, 1e-9);
    EXPECT_TRUE(config->disableActiveReport);
    EXPECT_FALSE(config->canTimeout);
}

TEST_F(RebotDriverConfig, BusTimeFitsTheControlPeriod) {
    auto const config = parseDriverConfig(profile);
    ASSERT_TRUE(config);
    // 14 frames of about 155 bits at 1 Mbit/s.
    EXPECT_NEAR(toSeconds(cycleBusTime(*config)), 14 * 155e-6, 1e-9);
    EXPECT_LT(cycleBusTime(*config), profile.controlPeriod);
}

TEST_F(RebotDriverConfig, ReadsOptionalSettings) {
    profile.driver.node["can_timeout_ms"] = 100;
    profile.driver.node["disable_active_report"] = false;
    auto const config = parseDriverConfig(profile);
    ASSERT_TRUE(config) << config.error().message;
    EXPECT_EQ(config->canTimeout, std::chrono::milliseconds{100});
    EXPECT_FALSE(config->disableActiveReport);
}

TEST_F(RebotDriverConfig, RejectsAMissingActuator) {
    auto actuators = YAML::Node{YAML::NodeType::Sequence};
    for (auto const &entry : profile.driver.node["actuators"]) {
        if (entry["joint"].as<std::string>() != "joint4") {
            actuators.push_back(entry);
        }
    }
    profile.driver.node["actuators"] = actuators;
    expectRejected("joint 'joint4' has no actuator");
}

TEST_F(RebotDriverConfig, RejectsDuplicateIds) {
    actuator("joint5")["id"] = 0x04;
    expectRejected("used twice");
}

TEST_F(RebotDriverConfig, RejectsTheHostIdAsAMotorId) {
    actuator("joint5")["id"] = 0xFD;
    expectRejected("used twice or equals the host ID");
}

TEST_F(RebotDriverConfig, RejectsUnknownModelsAndKeys) {
    actuator("joint5")["model"] = "rs-99";
    expectRejected("unknown model 'rs-99'");
    actuator("joint5")["model"] = "rs-00";
    profile.driver.node["bus_speed"] = 1;
    expectRejected("unknown key 'bus_speed'");
}

TEST_F(RebotDriverConfig, RejectsGainsBeyondTheEncodingRange) {
    profile.joints[3].gains.damping = 6.0;
    expectRejected("joint 'joint4' damping exceed the rs-00 encoding range");
}

TEST_F(RebotDriverConfig, RejectsTransmissionsThatPushLimitsOutOfRange) {
    actuator("gripper")["transmission"]["scale"] = 0.001;
    expectRejected("joint 'gripper' position limits exceed");
    actuator("gripper")["transmission"]["scale"] = 0.0;
    expectRejected("non-zero");
}

TEST_F(RebotDriverConfig, RejectsOtherDriverTypes) {
    profile.driver.node["type"] = "damiao";
    expectRejected("expected 'robstride_socketcan'");
}

} // namespace
} // namespace larm::drivers::robstride
