#include <larm/model/RobotModel.h>
#include <larm/sim/Simulation.h>

#include <gtest/gtest.h>

#include <cstdlib>
#include <random>

namespace larm::sim {
namespace {

using hal::DrivePower;

// MuJoCo writes the compiled MJCF with 6 significant digits, so it agrees with the URDF to about 1e-6.
constexpr double kModelTolerance = 1e-5;

// Elbow-up pose clear of the floor and of the arm itself.
JointVector clearancePose() {
    auto q = JointVector{7};
    q << 0.0, 0.7, 1.1, 0.0, 0.0, 0.0, 0.02;
    return q;
}

struct RebotSimulation : testing::Test {
    void SetUp() override {
        auto const *const path = std::getenv("LARM_ROBOT_PROFILE");
        ASSERT_NE(path, nullptr);
        auto loaded = loadRobotProfile(path);
        ASSERT_TRUE(loaded) << loaded.error().message;
        profile = std::move(*loaded);
        auto robot = model::loadRobotModel(profile);
        ASSERT_TRUE(robot) << robot.error().message;
        robotModel = std::move(*robot);
    }

    std::unique_ptr<Simulation> simulation(SimulationOptions const &options = {}) const {
        auto made = makeSimulation(profile, options);
        EXPECT_TRUE(made) << made.error().message;
        return made ? std::move(*made) : nullptr;
    }

    // Holds `target` with the profile gains plus gravity feed-forward for `cycles` control periods.
    RobotState hold(Simulation &sim, JointVector const &target, int const cycles) const {
        auto dynamics = robotModel->makeDynamics();
        auto state = RobotState::zero(profile.dof());
        auto command = JointCommand::zero(profile.dof());
        command.position = target;
        for (std::size_t i = 0; i < profile.dof(); ++i) {
            command.stiffness[idx(i)] = profile.joints[i].gains.stiffness;
            command.damping[idx(i)] = profile.joints[i].gains.damping;
        }
        for (int cycle = 0; cycle < cycles; ++cycle) {
            sim.driver().io().read(state);
            dynamics->gravity(state.joints.position, command.effort);
            sim.driver().io().write(command, DrivePower::Enabled);
            sim.timeline().advance();
        }
        sim.driver().io().read(state);
        return state;
    }

    RobotProfile profile;
    std::unique_ptr<model::RobotModel> robotModel;
};

TEST_F(RebotSimulation, ToolPoseMatchesPinocchio) {
    auto sim = simulation();
    ASSERT_NE(sim, nullptr);
    auto kinematics = robotModel->makeKinematics();
    auto const tool = kinematics->findFrame("gripper_end");
    ASSERT_TRUE(tool);
    auto const body = mj_name2id(&sim->model(), mjOBJ_BODY, "gripper_end");
    ASSERT_GE(body, 0);

    auto random = std::mt19937_64{7};
    for (int sample = 0; sample < 50; ++sample) {
        auto q = zeroJointVector(profile.dof());
        for (std::size_t i = 0; i < profile.dof(); ++i) {
            auto const &limits = profile.joints[i].limits;
            q[idx(i)] = std::uniform_real_distribution<double>{limits.lower, limits.upper}(random);
        }
        sim->reset(q);
        kinematics->update(q);
        auto const expected = kinematics->framePose(*tool);
        auto const &data = sim->data();
        auto const mujoco = Pose3{
            .translation =
                Eigen::Vector3d{data.xpos[3 * body], data.xpos[3 * body + 1], data.xpos[3 * body + 2]},
            .rotation = Eigen::Quaterniond{data.xquat[4 * body], data.xquat[4 * body + 1],
                                           data.xquat[4 * body + 2], data.xquat[4 * body + 3]},
        };
        EXPECT_LT(poseError(expected, mujoco).norm(), kModelTolerance) << "sample " << sample;
    }
}

TEST_F(RebotSimulation, GravityMatchesPinocchio) {
    auto sim = simulation();
    ASSERT_NE(sim, nullptr);
    auto dynamics = robotModel->makeDynamics();
    auto const q = clearancePose();
    sim->reset(q);
    auto gravity = JointVector{};
    dynamics->gravity(q, gravity);
    auto const &model = sim->model();
    for (std::size_t i = 0; i < profile.dof(); ++i) {
        auto const joint = mj_name2id(&model, mjOBJ_JOINT, profile.joints[i].descriptionJoint.c_str());
        auto const bias = sim->data().qfrc_bias[model.jnt_dofadr[joint]];
        EXPECT_NEAR(bias, gravity[idx(i)], kModelTolerance * std::max(1.0, std::abs(gravity[idx(i)])))
            << profile.joints[i].name;
    }
}

TEST_F(RebotSimulation, HoldsAPoseWithImpedanceAndGravityFeedForward) {
    auto sim = simulation();
    ASSERT_NE(sim, nullptr);
    auto const target = clearancePose();
    sim->reset(target);
    auto const state = hold(*sim, target, 250);
    EXPECT_TRUE(state.isFresh);
    EXPECT_TRUE(state.actuators[0].enabled);
    EXPECT_EQ(state.cycle, 250u);
    EXPECT_EQ(state.stamp.time_since_epoch(), std::chrono::seconds{1});
    EXPECT_LT((state.joints.position - target).head(6).cwiseAbs().maxCoeff(), 0.01);
    EXPECT_LT(state.joints.velocity.head(6).cwiseAbs().maxCoeff(), 0.01);
}

TEST_F(RebotSimulation, DisabledArmFallsUnderGravity) {
    auto sim = simulation();
    ASSERT_NE(sim, nullptr);
    auto const start = clearancePose();
    sim->reset(start);
    auto command = JointCommand::zero(profile.dof());
    command.position = start;
    for (int cycle = 0; cycle < 50; ++cycle) {
        sim->driver().io().write(command, DrivePower::Disabled);
        sim->timeline().advance();
    }
    auto state = RobotState::zero(profile.dof());
    sim->driver().io().read(state);
    EXPECT_FALSE(state.actuators[1].enabled);
    EXPECT_GT((state.joints.position - start).head(6).cwiseAbs().maxCoeff(), 0.05);
}

TEST_F(RebotSimulation, IsDeterministic) {
    auto first = simulation();
    auto second = simulation();
    ASSERT_NE(first, nullptr);
    ASSERT_NE(second, nullptr);
    auto target = clearancePose();
    target[0] = 0.5;
    first->reset(clearancePose());
    second->reset(clearancePose());
    auto const a = hold(*first, target, 100);
    auto const b = hold(*second, target, 100);
    for (int i = 0; i < first->model().nq; ++i) {
        EXPECT_EQ(first->data().qpos[i], second->data().qpos[i]);
    }
    EXPECT_EQ(a.joints.position, b.joints.position);
}

TEST_F(RebotSimulation, PacesToWallClock) {
    auto sim = simulation({.pacing = Pacing::RealTime, .realTimeFactor = 1.0});
    ASSERT_NE(sim, nullptr);
    sim->reset(clearancePose());
    auto const start = std::chrono::steady_clock::now();
    hold(*sim, clearancePose(), 26);
    EXPECT_GE(std::chrono::steady_clock::now() - start, std::chrono::milliseconds{95});
}

TEST_F(RebotSimulation, RejectsMisalignedPhysicsStep) {
    auto misaligned = profile;
    misaligned.sim.node["physics_step_us"] = 300;
    auto const made = makeSimulation(misaligned, {});
    ASSERT_FALSE(made);
    EXPECT_EQ(made.error().code, ErrorCode::InvalidConfig);
}

} // namespace
} // namespace larm::sim
