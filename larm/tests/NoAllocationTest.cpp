// Verifies the control loop contract: once running, tick() and advance() never touch the heap.
// The test binary replaces malloc and friends to count calls made while `counting` is set.
#include <larm/control/ControlCycle.h>
#include <larm/control/JointTrajectoryController.h>
#include <larm/hal/IdealBackend.h>
#include <larm/sim/Simulation.h>

#ifdef LARM_HAS_ROBSTRIDE
#include <larm/drivers/robstride/Backend.h>
#endif

#include <gtest/gtest.h>

#include <cerrno>
#include <cstdlib>

extern "C" {
void *__libc_malloc(std::size_t size);
void *__libc_calloc(std::size_t count, std::size_t size);
void *__libc_realloc(void *pointer, std::size_t size);
void *__libc_memalign(std::size_t alignment, std::size_t size);
void __libc_free(void *pointer);
}

namespace {
thread_local bool counting = false;
thread_local std::size_t allocations = 0;

void noteAllocation() noexcept {
    if (counting) {
        ++allocations;
    }
}
} // namespace

extern "C" {
void *malloc(std::size_t size) noexcept {
    noteAllocation();
    return __libc_malloc(size);
}
void *calloc(std::size_t count, std::size_t size) noexcept {
    noteAllocation();
    return __libc_calloc(count, size);
}
void *realloc(void *pointer, std::size_t size) noexcept {
    noteAllocation();
    return __libc_realloc(pointer, size);
}
void *memalign(std::size_t alignment, std::size_t size) noexcept {
    noteAllocation();
    return __libc_memalign(alignment, size);
}
void *aligned_alloc(std::size_t alignment, std::size_t size) noexcept {
    noteAllocation();
    return __libc_memalign(alignment, size);
}
int posix_memalign(void **out, std::size_t alignment, std::size_t size) noexcept {
    noteAllocation();
    *out = __libc_memalign(alignment, size);
    return *out == nullptr ? ENOMEM : 0;
}
void free(void *pointer) noexcept { __libc_free(pointer); }
}

namespace larm {
namespace {

JointVector clearancePose() {
    auto q = JointVector{7};
    q << 0.0, 0.7, 1.1, 0.0, 0.0, 0.0, 0.02;
    return q;
}

enum class BackendKind : std::uint8_t { Ideal, MuJoCo, RobStride };

struct NoAllocation : testing::TestWithParam<BackendKind> {
    void SetUp() override {
        auto const *const path = std::getenv("LARM_ROBOT_PROFILE");
        ASSERT_NE(path, nullptr);
        auto loaded = loadRobotProfile(path);
        ASSERT_TRUE(loaded) << loaded.error().message;
        profile = std::move(*loaded);
        auto robot = model::loadRobotModel(profile);
        ASSERT_TRUE(robot) << robot.error().message;
        robotModel = std::move(*robot);
        switch (GetParam()) {
        case BackendKind::Ideal:
            backend =
                hal::makeIdealBackend({.initialPosition = clearancePose(), .period = profile.controlPeriod});
            break;
        case BackendKind::MuJoCo: {
            auto made = sim::makeSimulation(profile, {});
            ASSERT_TRUE(made) << made.error().message;
            (*made)->reset(clearancePose());
            backend = std::move(*made);
            break;
        }
        case BackendKind::RobStride:
            makeRobStrideBackend();
            break;
        }
        if (IsSkipped()) {
            return;
        }
        ASSERT_TRUE(backend);
        ASSERT_TRUE(backend->driver().connect());
        channels = std::make_unique<control::RuntimeChannels>(profile.dof());
        auto made = control::makeControlCycle(
            profile, {.driver = &backend->driver(), .model = robotModel.get(), .channels = channels.get()});
        ASSERT_TRUE(made) << made.error().message;
        cycle = std::move(*made);
        for (std::size_t i = 0; i < 6; ++i) {
            arm.set(i);
        }
    }

    // Runs `cycles` periods with allocation counting on; returns the number of allocations.
    std::size_t runCounted(int const cycles) {
        allocations = 0;
        counting = true;
        for (int i = 0; i < cycles; ++i) {
            cycle->tick();
            backend->timeline().advance();
        }
        counting = false;
        return allocations;
    }

    void drain() {
        while (channels->events.tryPop()) {
        }
        while (channels->retired.tryPop()) {
        }
        channels->snapshot.refresh();
    }

    std::unique_ptr<control::Controller> trajectoryTo(JointVector const &target) {
        auto start = motion::JointSample::zero(profile.dof());
        start.position = channels->snapshot.current().state.joints.position;
        auto planned = motion::planPointToPoint(
            {.start = start, .target = target, .limits = motion::motionLimits(profile), .joints = arm});
        EXPECT_TRUE(planned);
        return control::makeJointTrajectoryController({
            .trajectory = *planned,
            .joints = arm,
            .impedance = control::profileImpedance(profile),
            .trackingTolerance = control::profileTrackingTolerance(profile),
            .goalTolerance = control::profileTrackingTolerance(profile),
            .goalTimeout = std::chrono::seconds{1},
            .stopLimits = motion::motionLimits(profile),
        });
    }

    // Simulated RobStride motors at the clearance pose, carrying the arm's gravity.
    void makeRobStrideBackend() {
#ifdef LARM_HAS_ROBSTRIDE
        auto const config = drivers::robstride::parseDriverConfig(profile);
        ASSERT_TRUE(config) << config.error().message;
        auto start = std::vector<double>{};
        for (auto const &actuator : config->actuators) {
            start.push_back((clearancePose()[idx(actuator.joint)] - actuator.transmission.offset) /
                            actuator.transmission.scale);
        }
        auto const dynamics = std::shared_ptr<model::Dynamics>{robotModel->makeDynamics()};
        auto made = drivers::robstride::makeSimulatedRobStrideBackend(
            profile, {.load = drivers::robstride::jointSpaceLoad(
                          *config,
                          [dynamics](JointVector const &position, JointVector &torque) {
                              dynamics->gravity(position, torque);
                              torque *= -1.0;
                          }),
                      .position = start});
        ASSERT_TRUE(made) << made.error().message;
        backend = std::move(*made);
#else
        GTEST_SKIP() << "built without the RobStride driver";
#endif
    }

    RobotProfile profile;
    std::unique_ptr<model::RobotModel> robotModel;
    std::unique_ptr<hal::Backend> backend;
    std::unique_ptr<control::RuntimeChannels> channels;
    std::unique_ptr<control::ControlCycle> cycle;
    JointMask arm;
};

TEST_P(NoAllocation, ControlLoopNeverAllocates) {
    ASSERT_TRUE(channels->requests.tryPush(control::SetDrivePower{.power = hal::DrivePower::Enabled}));
    EXPECT_EQ(runCounted(200), 0u);
    drain();

    auto target = clearancePose();
    target[0] = 0.5;
    target[4] = 0.3;
    ASSERT_TRUE(channels->requests.tryPush(
        control::ActivateController{.goal = control::GoalId{1}, .controller = trajectoryTo(target)}));
    EXPECT_EQ(runCounted(500), 0u);
    drain();

    target[0] = -1.0;
    ASSERT_TRUE(channels->requests.tryPush(
        control::ActivateController{.goal = control::GoalId{2}, .controller = trajectoryTo(target)}));
    EXPECT_EQ(runCounted(100), 0u);
    ASSERT_TRUE(channels->requests.tryPush(control::CancelGoal{.goal = control::GoalId{2}}));
    EXPECT_EQ(runCounted(300), 0u);
    drain();
    EXPECT_EQ(channels->snapshot.current().activeCount, 0u);
}

std::unique_ptr<int[]> escaped;

TEST(AllocationCounter, SeesHeapAllocations) {
    allocations = 0;
    counting = true;
    escaped = std::make_unique<int[]>(16);
    counting = false;
    EXPECT_GT(allocations, 0u);
}

INSTANTIATE_TEST_SUITE_P(Backends, NoAllocation,
                         testing::Values(BackendKind::Ideal, BackendKind::MuJoCo, BackendKind::RobStride),
                         [](testing::TestParamInfo<BackendKind> const &backendParam) {
                             switch (backendParam.param) {
                             case BackendKind::Ideal:
                                 return "Ideal";
                             case BackendKind::MuJoCo:
                                 return "MuJoCo";
                             case BackendKind::RobStride:
                                 return "RobStride";
                             }
                             return "Unknown";
                         });

} // namespace
} // namespace larm
