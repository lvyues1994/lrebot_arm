// Runs one joint-space motion through the production control stack against MuJoCo, without ROS.
// Prints a JSON summary; --trace adds one JSON line per control cycle.
#include <larm/control/ControlCycle.h>
#include <larm/control/JointTrajectoryController.h>
#include <larm/sim/Simulation.h>

#include <algorithm>
#include <charconv>
#include <cstdint>
#include <format>
#include <fstream>
#include <iostream>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace larm::app {
namespace {

constexpr std::string_view kUsage = R"(usage: larm_sim_cli --profile FILE --target Q1,Q2,... [options]
  --start Q1,Q2,...     initial joint positions (default: the profile's rest pose)
  --speed S             fraction of the profile's motion limits in (0, 1] (default: 0.5)
  --realtime [RTF]      pace the simulation to the wall clock (default: as fast as possible)
  --cancel-after SEC    cancel the motion after SEC seconds of simulated motion
  --trace FILE          write one JSON line per control cycle
)";

struct Options {
    std::filesystem::path profile;
    JointVector target;
    std::optional<JointVector> start;
    double speed = 0.5;
    sim::SimulationOptions simulation;
    std::optional<double> cancelAfter;
    std::optional<std::filesystem::path> trace;
};

Expected<double> parseNumber(std::string_view const text) {
    auto value = 0.0;
    auto const [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc{} or end != text.data() + text.size()) {
        return makeError(ErrorCode::InvalidArgument, "not a number: '" + std::string{text} + "'");
    }
    return value;
}

Expected<JointVector> parseJointList(std::string_view text) {
    auto values = std::vector<double>{};
    while (true) {
        auto const comma = text.find(',');
        auto const value = parseNumber(text.substr(0, comma));
        if (not value) {
            return tl::make_unexpected(value.error());
        }
        values.push_back(*value);
        if (comma == std::string_view::npos) {
            break;
        }
        text.remove_prefix(comma + 1);
    }
    if (values.size() > kMaxDof) {
        return makeError(ErrorCode::InvalidArgument, "too many joint values");
    }
    auto result = zeroJointVector(values.size());
    std::copy(values.begin(), values.end(), result.begin());
    return result;
}

Expected<Options> parseOptions(std::span<char *const> const arguments) {
    auto options = Options{};
    auto haveTarget = false;
    for (std::size_t i = 1; i < arguments.size(); ++i) {
        auto const flag = std::string_view{arguments[i]};
        auto const hasValue =
            i + 1 < arguments.size() and std::string_view{arguments[i + 1]}.substr(0, 2) != "--";
        auto const value = [&]() -> Expected<std::string_view> {
            if (not hasValue) {
                return makeError(ErrorCode::InvalidArgument, std::string{flag} + " needs a value");
            }
            return std::string_view{arguments[++i]};
        };
        auto const failed = [](Error const &error) -> Expected<Options> {
            return tl::make_unexpected(error);
        };

        if (flag == "--realtime") {
            options.simulation.pacing = sim::Pacing::RealTime;
            if (hasValue) {
                auto const factor = parseNumber(*value());
                if (not factor) {
                    return failed(factor.error());
                }
                options.simulation.realTimeFactor = *factor;
            }
            continue;
        }
        auto const text = value();
        if (not text) {
            return failed(text.error());
        }
        if (flag == "--profile") {
            options.profile = std::string{*text};
        } else if (flag == "--target" or flag == "--start") {
            auto const joints = parseJointList(*text);
            if (not joints) {
                return failed(joints.error());
            }
            (flag == "--target" ? options.target : options.start.emplace()) = *joints;
            haveTarget = haveTarget or flag == "--target";
        } else if (flag == "--speed" or flag == "--cancel-after") {
            auto const number = parseNumber(*text);
            if (not number) {
                return failed(number.error());
            }
            if (flag == "--speed") {
                options.speed = *number;
            } else {
                options.cancelAfter = *number;
            }
        } else if (flag == "--trace") {
            options.trace = std::string{*text};
        } else {
            return makeError(ErrorCode::InvalidArgument, "unknown option " + std::string{flag});
        }
    }
    if (options.profile.empty() or not haveTarget) {
        return makeError(ErrorCode::InvalidArgument, "--profile and --target are required");
    }
    if (not(options.speed > 0.0 and options.speed <= 1.0)) {
        return makeError(ErrorCode::InvalidArgument, "--speed must be in (0, 1]");
    }
    return options;
}

// Everything the production control loop needs, wired as the runtime will wire it.
struct Stack {
    RobotProfile profile;
    std::unique_ptr<model::RobotModel> model;
    std::unique_ptr<sim::Simulation> simulation;
    std::unique_ptr<control::RuntimeChannels> channels;
    std::unique_ptr<control::ControlCycle> cycle;
};

Expected<Stack> buildStack(Options const &options) {
    auto stack = Stack{};
    auto profile = loadRobotProfile(options.profile);
    if (not profile) {
        return tl::make_unexpected(profile.error());
    }
    stack.profile = std::move(*profile);
    auto const dof = stack.profile.dof();
    if (dofOf(options.target) != dof or (options.start and dofOf(*options.start) != dof)) {
        return makeError(ErrorCode::InvalidArgument, std::format("joint lists need {} values", dof));
    }
    auto robot = model::loadRobotModel(stack.profile);
    if (not robot) {
        return tl::make_unexpected(robot.error());
    }
    stack.model = std::move(*robot);
    auto simulation = sim::makeSimulation(stack.profile, options.simulation);
    if (not simulation) {
        return tl::make_unexpected(simulation.error());
    }
    stack.simulation = std::move(*simulation);
    stack.simulation->reset(options.start.value_or(stack.profile.safety.restPose));
    stack.channels = std::make_unique<control::RuntimeChannels>(dof);
    auto cycle = control::makeControlCycle(stack.profile, {.driver = &stack.simulation->driver(),
                                                           .model = stack.model.get(),
                                                           .channels = stack.channels.get()});
    if (not cycle) {
        return tl::make_unexpected(cycle.error());
    }
    stack.cycle = std::move(*cycle);
    return stack;
}

std::string formatVector(JointVector const &vector) {
    auto text = std::string{"["};
    for (Eigen::Index i = 0; i < vector.size(); ++i) {
        text += std::format("{}{:.6g}", i == 0 ? "" : ", ", vector[i]);
    }
    return text + "]";
}

// FNV-1a over the full MuJoCo position and velocity state.
std::string stateDigest(mjModel const &model, mjData const &data) {
    auto hash = std::uint64_t{0xcbf29ce484222325ULL};
    auto const mix = [&](mjtNum const *values, mjtSize const count) {
        auto const bytes = std::as_bytes(std::span{values, static_cast<std::size_t>(count)});
        for (auto const byte : bytes) {
            hash = (hash ^ static_cast<std::uint64_t>(byte)) * 0x100000001b3ULL;
        }
    };
    mix(data.qpos, model.nq);
    mix(data.qvel, model.nv);
    return std::format("{:016x}", hash);
}

// Steps the stack and records what the summary reports.
struct Session {
    explicit Session(Stack &stack_, std::ofstream *trace_) : stack{stack_}, trace{trace_} {}

    void step() {
        stack.cycle->tick();
        stack.simulation->timeline().advance();
        ++cycles;
        while (auto event = stack.channels->events.tryPop()) {
            if (auto const *const goal = std::get_if<control::GoalFinished>(&*event)) {
                finished = *goal;
            } else if (auto const *const power = std::get_if<control::PowerChanged>(&*event)) {
                powered = power->power == hal::DrivePower::Enabled;
            }
        }
        while (stack.channels->retired.tryPop()) {
        }
        while (auto const telemetry = stack.channels->telemetry.tryPop()) {
            computeTimes.push_back(telemetry->computeTime);
        }
        stack.channels->snapshot.refresh();
        auto const &current = stack.channels->snapshot.current();
        if (current.activeCount > 0) {
            auto const error =
                (current.command.position - current.state.joints.position).cwiseAbs().maxCoeff();
            worstTracking = std::max(worstTracking, error);
        }
        if (trace != nullptr) {
            *trace << std::format("{{\"t\": {:.4f}, \"q\": {}, \"command\": {}, \"effort\": {}}}\n",
                                  toSeconds(current.state.stamp.time_since_epoch()),
                                  formatVector(current.state.joints.position),
                                  formatVector(current.command.position),
                                  formatVector(current.state.joints.effort));
        }
    }

    control::RobotSnapshot const &snapshot() const { return stack.channels->snapshot.current(); }

    Stack &stack;
    std::ofstream *trace;
    std::uint64_t cycles{};
    bool powered{};
    std::optional<control::GoalFinished> finished;
    double worstTracking{};
    std::vector<Duration> computeTimes;
};

std::unique_ptr<control::Controller> plan(Stack const &stack, Options const &options, Duration &duration) {
    auto joints = JointMask{};
    for (std::size_t i = 0; i < stack.profile.dof(); ++i) {
        joints.set(i);
    }
    auto start = motion::JointSample::zero(stack.profile.dof());
    start.position = stack.channels->snapshot.current().state.joints.position;
    auto const trajectory =
        motion::planPointToPoint({.start = start,
                                  .target = options.target,
                                  .limits = motion::motionLimits(stack.profile, options.speed),
                                  .joints = joints});
    if (not trajectory) {
        std::cerr << "planning failed: " << trajectory.error().message << "\n";
        return nullptr;
    }
    duration = (*trajectory)->duration();
    auto const tolerance = control::profileTrackingTolerance(stack.profile);
    return control::makeJointTrajectoryController({
        .trajectory = *trajectory,
        .joints = joints,
        .impedance = control::profileImpedance(stack.profile),
        .trackingTolerance = tolerance,
        .goalTolerance = tolerance * 0.05,
        .goalTimeout = std::chrono::seconds{1},
        .stopLimits = motion::motionLimits(stack.profile),
    });
}

void printSummary(Session const &session, Duration const motionDuration, double const wallSeconds,
                  JointVector const &target) {
    auto const &snapshot = session.snapshot();
    auto const simulated = toSeconds(snapshot.state.stamp.time_since_epoch());
    auto times = session.computeTimes;
    std::sort(times.begin(), times.end());
    auto const microseconds = [&](double const quantile) {
        if (times.empty()) {
            return 0.0;
        }
        auto const index = static_cast<std::size_t>(quantile * static_cast<double>(times.size() - 1));
        return toSeconds(times[index]) * 1e6;
    };
    auto const status = session.finished ? control::toString(session.finished->status) : "timeout";
    auto const fault = session.finished ? control::toString(session.finished->fault) : "none";
    auto const &simulation = *session.stack.simulation;
    std::cout << std::format(
        "{{\n  \"status\": \"{}\",\n  \"fault\": \"{}\",\n  \"motion_duration_s\": {:.4f},\n"
        "  \"simulated_s\": {:.4f},\n  \"wall_s\": {:.4f},\n  \"real_time_factor\": {:.2f},\n"
        "  \"cycles\": {},\n  \"max_tracking_error\": {:.6f},\n  \"final_error\": {:.6f},\n"
        "  \"final_position\": {},\n  \"compute_us\": {{\"p50\": {:.1f}, \"p99\": {:.1f}, \"max\": "
        "{:.1f}}},\n"
        "  \"state_digest\": \"{}\"\n}}\n",
        status, fault, toSeconds(motionDuration), simulated, wallSeconds,
        wallSeconds > 0.0 ? simulated / wallSeconds : 0.0, session.cycles, session.worstTracking,
        (snapshot.state.joints.position - target).cwiseAbs().maxCoeff(),
        formatVector(snapshot.state.joints.position), microseconds(0.5), microseconds(0.99),
        microseconds(1.0), stateDigest(simulation.model(), simulation.data()));
}

int run(Options const &options) {
    auto stack = buildStack(options);
    if (not stack) {
        std::cerr << "error: " << stack.error().message << "\n";
        return 2;
    }
    auto traceFile = std::ofstream{};
    if (options.trace) {
        traceFile.open(*options.trace);
    }
    auto session = Session{*stack, options.trace ? &traceFile : nullptr};
    auto const wallStart = std::chrono::steady_clock::now();

    static_cast<void>(
        stack->channels->requests.tryPush(control::SetDrivePower{.power = hal::DrivePower::Enabled}));
    auto const enableBudget = 2 * stack->profile.safety.enableRamp / stack->profile.controlPeriod + 10;
    for (std::int64_t i = 0; i < enableBudget and not session.powered; ++i) {
        session.step();
    }
    if (not session.powered) {
        std::cerr << "error: the arm did not enable\n";
        return 1;
    }

    auto motionDuration = Duration{};
    auto controller = plan(*stack, options, motionDuration);
    if (not controller) {
        return 1;
    }
    static_cast<void>(stack->channels->requests.tryPush(
        control::ActivateController{.goal = control::GoalId{1}, .controller = std::move(controller)}));
    auto const motionStart = session.snapshot().state.stamp;
    auto const deadline = motionStart + motionDuration + std::chrono::seconds{5};
    auto cancelled = false;
    while (not session.finished and session.snapshot().state.stamp < deadline) {
        if (options.cancelAfter and not cancelled and
            session.snapshot().state.stamp - motionStart >= fromSeconds(*options.cancelAfter)) {
            cancelled = stack->channels->requests.tryPush(control::CancelGoal{.goal = control::GoalId{1}});
        }
        session.step();
    }
    auto const settleCycles = std::chrono::milliseconds{500} / stack->profile.controlPeriod;
    for (std::int64_t i = 0; i < settleCycles; ++i) {
        session.step();
    }

    auto const wallSeconds = toSeconds(std::chrono::steady_clock::now() - wallStart);
    printSummary(session, motionDuration, wallSeconds, options.target);
    auto const expected = cancelled ? control::ControlStatus::Stopped : control::ControlStatus::Succeeded;
    return session.finished and session.finished->status == expected ? 0 : 1;
}

} // namespace
} // namespace larm::app

int main(int const argc, char **argv) {
    auto const options =
        larm::app::parseOptions(std::span<char *const>{argv, static_cast<std::size_t>(argc)});
    if (not options) {
        std::cerr << "error: " << options.error().message << "\n\n" << larm::app::kUsage;
        return 2;
    }
    return larm::app::run(*options);
}
