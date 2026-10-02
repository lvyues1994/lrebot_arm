// Verifies the control loop contract: once running, tick() and advance() never touch the heap.
// The test binary replaces malloc and friends to count calls made while `counting` is set.
#include <larm/control/ControlCycle.h>
#include <larm/control/JointTrajectoryController.h>
#include <larm/hal/IdealBackend.h>
#include <larm/sim/Simulation.h>

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

struct NoAllocation : testing::TestWithParam<bool> {
    void SetUp() override {
        auto const *const path = std::getenv("LARM_ROBOT_PROFILE");
        ASSERT_NE(path, nullptr);
        auto loaded = loadRobotProfile(path);
        ASSERT_TRUE(loaded) << loaded.error().message;
        profile = std::move(*loaded);
        auto robot = model::loadRobotModel(profile);
        ASSERT_TRUE(robot) << robot.error().message;
        robotModel = std::move(*robot);
        if (GetParam()) {
            auto made = sim::makeSimulation(profile, {});
            ASSERT_TRUE(made) << made.error().message;
            (*made)->reset(clearancePose());
            backend = std::move(*made);
        } else {
            backend =
                hal::makeIdealBackend({.initialPosition = clearancePose(), .period = profile.controlPeriod});
        }
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

INSTANTIATE_TEST_SUITE_P(Backends, NoAllocation, testing::Values(false, true),
                         [](testing::TestParamInfo<bool> const &backendParam) {
                             return backendParam.param ? "MuJoCo" : "Ideal";
                         });

} // namespace
} // namespace larm
