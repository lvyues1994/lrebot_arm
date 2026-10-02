// larm_driver_probe: brings RobStride hardware up one step at a time.
//
//   larm_driver_probe --profile FILE [--simulated] [--output FILE] [--rt-priority N] COMMAND
//
//   scan                          ping every actuator, read its settings and position (read-only)
//   monitor SECONDS               sample positions at 50 Hz through parameter reads (read-only)
//   hold SECONDS --confirm-power  enable at the rest pose, hold, disable
//   jog JOINT RADIANS --confirm-motion
//                                 enable, move one arm joint by at most 0.2 rad and back slowly, disable
//
// Writes one JSON object per line. Commands that power the motors need their confirmation flag and
// refuse to start unless the arm rests at the profile's rest pose without faults. Ctrl+C stops the
// motion; the arm is disabled only where it rests, otherwise it is left holding and reported.
// --simulated runs the same code against simulated motors carrying the arm's gravity.
#include "JsonLine.h"
#include "ReadOnlyBus.h"

#include <larm/drivers/robstride/Backend.h>
#include <larm/model/RobotModel.h>
#include <larm/runtime/LocalRuntime.h>

#include <csignal>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>

namespace {

using larm::JointVector;
using larm::probe::JsonLine;
using larm::probe::ReadOnlyBus;
namespace robstride = larm::drivers::robstride;

constexpr double kMaxJogRadians = 0.2;
constexpr double kJogSpeed = 0.2;
constexpr double kRestToleranceRadian = 0.1;
constexpr auto kSamplePeriod = std::chrono::milliseconds{20};

enum ExitCode : int { kOk = 0, kFailed = 1, kUsage = 2, kRefused = 3 };

struct Options {
    std::string profile;
    bool simulated{};
    std::string output;
    int rtPriority{};
    std::string command;
    std::vector<std::string> arguments;
    bool confirmPower{};
    bool confirmMotion{};
};

std::optional<Options> parse(int const argc, char **argv) {
    auto options = Options{};
    for (int i = 1; i < argc; ++i) {
        auto const argument = std::string_view{argv[i]};
        auto const next = [&]() -> std::optional<std::string> {
            return i + 1 < argc ? std::optional<std::string>{argv[++i]} : std::nullopt;
        };
        if (argument == "--profile" or argument == "--output" or argument == "--rt-priority") {
            auto const value = next();
            if (not value) {
                return std::nullopt;
            }
            if (argument == "--profile") {
                options.profile = *value;
            } else if (argument == "--output") {
                options.output = *value;
            } else {
                auto stream = std::istringstream{*value};
                if (not(stream >> options.rtPriority)) {
                    return std::nullopt;
                }
            }
        } else if (argument == "--simulated") {
            options.simulated = true;
        } else if (argument == "--confirm-power") {
            options.confirmPower = true;
        } else if (argument == "--confirm-motion") {
            options.confirmMotion = true;
        } else if (options.command.empty()) {
            options.command = argument;
        } else {
            options.arguments.emplace_back(argument);
        }
    }
    if (options.profile.empty() or options.command.empty()) {
        return std::nullopt;
    }
    return options;
}

// Blocks SIGINT and SIGTERM in every thread and turns them into a stop request.
struct SignalStop {
    explicit SignalStop(lexec::inplace_stop_source &stop) {
        sigemptyset(&signals);
        sigaddset(&signals, SIGINT);
        sigaddset(&signals, SIGTERM);
        pthread_sigmask(SIG_BLOCK, &signals, nullptr);
        waiter = std::jthread{[this, &stop](std::stop_token const token) {
            auto const timeout = timespec{.tv_sec = 0, .tv_nsec = 100'000'000};
            while (not token.stop_requested()) {
                if (sigtimedwait(&signals, nullptr, &timeout) > 0) {
                    stop.request_stop();
                }
            }
        }};
    }

    sigset_t signals{};
    std::jthread waiter;
};

struct Probe {
    larm::RobotProfile profile;
    robstride::DriverConfig config;
    std::ostream *out{};
    lexec::inplace_stop_source *stop{};
    Options options;
    // The sampler writes lines from its own thread.
    mutable std::mutex outputLock;

    void emit(JsonLine const &line) const {
        auto const lock = std::lock_guard{outputLock};
        line.writeTo(*out);
    }

    int fail(std::string_view const what, int const code = kFailed) const {
        emit(JsonLine{"error"}.field("message", what));
        return code;
    }

    std::vector<double> motorToJoint(std::span<double const> const motor) const {
        auto joint = std::vector<double>(motor.size());
        for (std::size_t i = 0; i < motor.size(); ++i) {
            auto const &[scale, offset] = config.actuators[i].transmission;
            joint[i] = scale * motor[i] + offset;
        }
        return joint;
    }

    // Read-only access: the CAN interface, or simulated motors resting at the rest pose.
    ReadOnlyBus readOnlyBus(std::unique_ptr<robstride::SimulatedMotors> &simulated) const {
        if (options.simulated) {
            simulated = robstride::makeSimulatedMotors(config, {.position = restMotorPosition()});
            return ReadOnlyBus{config, simulated->transport()};
        }
        auto transport = larm::drivers::can::openSocketCan(config.interface);
        if (not transport) {
            throw std::runtime_error{transport.error().message};
        }
        return ReadOnlyBus{config, std::move(*transport)};
    }

    std::vector<double> restMotorPosition() const {
        auto motor = std::vector<double>{};
        for (auto const &actuator : config.actuators) {
            auto const &[scale, offset] = actuator.transmission;
            motor.push_back((profile.safety.restPose[larm::idx(actuator.joint)] - offset) / scale);
        }
        return motor;
    }

    int scan() const {
        auto simulated = std::unique_ptr<robstride::SimulatedMotors>{};
        auto bus = readOnlyBus(simulated);
        auto answered = 0;
        auto ready = true;
        for (auto const &actuator : config.actuators) {
            auto line = JsonLine{"actuator"};
            line.field("joint", actuator.jointName)
                .field("id", static_cast<int>(actuator.id))
                .field("model", robstride::toString(actuator.model));
            auto const present = bus.ping(actuator);
            line.field("answered", present);
            if (present) {
                ++answered;
                auto const runMode = bus.read(actuator, robstride::parameter::kRunMode);
                auto const zeroState = bus.read(actuator, 0x7029);
                auto const voltage = bus.read(actuator, robstride::parameter::kBusVoltage);
                auto const position = bus.read(actuator, robstride::parameter::kMechanicalPosition);
                if (runMode) {
                    line.field("run_mode", static_cast<int>(runMode->asInt8()));
                }
                if (zeroState) {
                    line.field("zero_sta", static_cast<int>(zeroState->raw[0]));
                }
                if (voltage) {
                    line.field("bus_voltage", static_cast<double>(voltage->asFloat()));
                }
                if (position) {
                    auto const motor = static_cast<double>(position->asFloat());
                    auto const &[scale, offset] = actuator.transmission;
                    line.field("motor_position", motor).field("joint_position", scale * motor + offset);
                }
                ready = ready and runMode and runMode->asInt8() == robstride::kRunModeMotion and zeroState and
                        zeroState->raw[0] == 1;
            }
            emit(line);
        }
        emit(JsonLine{"scan"}
                 .field("interface", options.simulated ? std::string{"simulated"} : config.interface)
                 .field("answered", answered)
                 .field("actuators", static_cast<int>(config.actuators.size()))
                 .field("active_report_frames", bus.unsolicitedStatus())
                 .field("ready", ready and answered == static_cast<int>(config.actuators.size())));
        return answered == static_cast<int>(config.actuators.size()) and ready ? kOk : kFailed;
    }

    int monitor(double const seconds) const {
        auto simulated = std::unique_ptr<robstride::SimulatedMotors>{};
        auto bus = readOnlyBus(simulated);
        auto const start = std::chrono::steady_clock::now();
        auto previous = std::vector<double>{};
        auto previousTime = 0.0;
        for (auto tick = start; not stop->stop_requested(); tick += kSamplePeriod) {
            std::this_thread::sleep_until(tick);
            auto const time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
            if (time > seconds) {
                break;
            }
            auto motor = std::vector<double>{};
            auto velocity = std::vector<double>{};
            auto missing = 0;
            for (auto const &actuator : config.actuators) {
                auto const position = bus.read(actuator, robstride::parameter::kMechanicalPosition);
                auto const speed = bus.read(actuator, robstride::parameter::kMechanicalVelocity);
                missing += position ? 0 : 1;
                motor.push_back(position ? static_cast<double>(position->asFloat()) : std::nan(""));
                velocity.push_back(speed ? actuator.transmission.scale * static_cast<double>(speed->asFloat())
                                         : std::nan(""));
            }
            auto const joint = motorToJoint(motor);
            auto line = JsonLine{"sample"};
            line.field("t", time)
                .field("position", joint)
                .field("mech_velocity", velocity)
                .field("missing", missing);
            if (not previous.empty()) {
                auto difference = std::vector<double>(joint.size());
                for (std::size_t i = 0; i < joint.size(); ++i) {
                    difference[i] = (joint[i] - previous[i]) / (time - previousTime);
                }
                line.field("difference_velocity", difference);
            }
            emit(line);
            previous = joint;
            previousTime = time;
        }
        emit(JsonLine{"monitor"}.field("active_report_frames", bus.unsolicitedStatus()));
        return kOk;
    }

    // The runtime on the real or simulated backend; the arm is not yet enabled.
    struct Running {
        std::unique_ptr<larm::runtime::RobotSession> session;
        robstride::RobStrideBackend *backend{};
    };

    larm::Expected<Running> startRuntime() const {
        auto running = Running{};
        auto options_ = larm::runtime::RuntimeOptions{};
        if (options.rtPriority > 0) {
            options_.realtime.priority = options.rtPriority;
        }
        auto backend = larm::Expected<std::unique_ptr<robstride::RobStrideBackend>>{};
        if (options.simulated) {
            auto model = larm::model::loadRobotModel(profile);
            if (not model) {
                return tl::make_unexpected(model.error());
            }
            auto const dynamics = std::shared_ptr<larm::model::Dynamics>{(*model)->makeDynamics()};
            auto load = robstride::jointSpaceLoad(
                config, [dynamics](JointVector const &position, JointVector &torque) {
                    dynamics->gravity(position, torque);
                    torque *= -1.0;
                });
            backend = robstride::makeSimulatedRobStrideBackend(
                profile, {.load = std::move(load), .position = restMotorPosition()});
        } else {
            backend = robstride::makeRobStrideBackend(profile);
        }
        if (not backend) {
            return tl::make_unexpected(backend.error());
        }
        running.backend = backend->get();
        auto session = larm::runtime::startLocalRuntime(profile, std::move(*backend), options_);
        if (not session) {
            return tl::make_unexpected(session.error());
        }
        running.session = std::move(*session);
        return running;
    }

    // Joints away from the rest pose, by name; the gripper is not checked.
    std::vector<std::string> awayFromRest(larm::control::RobotSnapshot const &snapshot) const {
        auto away = std::vector<std::string>{};
        for (std::size_t i = 0; i < profile.dof(); ++i) {
            auto const j = larm::idx(i);
            if (profile.joints[i].unit == larm::JointUnit::Radian and
                std::abs(snapshot.state.joints.position[j] - profile.safety.restPose[j]) >
                    kRestToleranceRadian) {
                away.push_back(profile.joints[i].name);
            }
        }
        return away;
    }

    // Waits for fresh feedback, then checks the arm rests without faults.
    int checkStart(larm::runtime::RobotSession &session) const {
        auto snapshot = session.latest();
        for (int i = 0; i < 100 and not snapshot.state.isFresh; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds{10});
            snapshot = session.latest();
        }
        if (not snapshot.state.isFresh) {
            return fail("no fresh feedback from every actuator", kRefused);
        }
        if (snapshot.safety != larm::control::SafetyState::Normal) {
            return fail("the runtime reports a fault before enabling: " +
                            std::string{larm::control::toString(snapshot.fault)},
                        kRefused);
        }
        if (auto const away = awayFromRest(snapshot); not away.empty()) {
            auto names = std::string{};
            for (auto const &name : away) {
                names += (names.empty() ? "" : ", ") + name;
            }
            return fail("not at the rest pose (" + names + "); move the arm there by hand first", kRefused);
        }
        return kOk;
    }

    enum class Interruptible : std::uint8_t { No, ByCtrlC };

    template <class Sender>
    bool succeeded(Sender &&sender, std::string_view const step,
                   Interruptible const interruptible = Interruptible::ByCtrlC) const {
        try {
            auto const token =
                interruptible == Interruptible::ByCtrlC ? stop->get_token() : lexec::inplace_stop_token{};
            auto const result = lexec::sync_wait(
                lexec::write_env(std::forward<Sender>(sender), lexec::prop{lexec::get_stop_token, token}));
            emit(JsonLine{"step"}.field("name", step).field("outcome", result ? "succeeded" : "stopped"));
            return result.has_value();
        } catch (std::exception const &error) {
            emit(JsonLine{"step"}
                     .field("name", step)
                     .field("outcome", "failed")
                     .field("message", error.what()));
        } catch (...) {
            emit(JsonLine{"step"}.field("name", step).field("outcome", "failed"));
        }
        return false;
    }

    // Samples the session until `stopSampling`, as JSON lines.
    std::jthread sampler(larm::runtime::RobotSession &session) const {
        return std::jthread{[this, &session](std::stop_token const token) {
            auto const start = std::chrono::steady_clock::now();
            for (auto tick = start; not token.stop_requested(); tick += kSamplePeriod) {
                std::this_thread::sleep_until(tick);
                auto const snapshot = session.latest();
                auto const &state = snapshot.state;
                auto const dof = static_cast<std::size_t>(state.joints.position.size());
                emit(JsonLine{"sample"}
                         .field(
                             "t",
                             std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count())
                         .field("position", std::span{state.joints.position.data(), dof})
                         .field("velocity", std::span{state.joints.velocity.data(), dof})
                         .field("effort", std::span{state.joints.effort.data(), dof})
                         .field("command", std::span{snapshot.command.position.data(), dof})
                         .field("fresh", state.isFresh)
                         .field("power", snapshot.power == larm::hal::DrivePower::Enabled)
                         .field("fault", larm::control::toString(snapshot.fault)));
            }
        }};
    }

    // Leaves the arm disabled at rest, or holding where it is if it cannot get there. Runs to the end
    // even after Ctrl+C; the hardware emergency stop is the way to interrupt it.
    int finish(Running &running, int code) const {
        auto &session = *running.session;
        auto const latest = session.latest();
        if (latest.power == larm::hal::DrivePower::Enabled and not awayFromRest(latest).empty() and
            not succeeded(session.park(), "park", Interruptible::No)) {
            emit(JsonLine{"warning"}.field("message", "the arm is not at rest and stays enabled, holding"));
            report(running);
            return kFailed;
        }
        if (not succeeded(session.disable({}), "disable", Interruptible::No)) {
            code = kFailed;
        }
        report(running);
        return code;
    }

    void report(Running const &running) const {
        auto const statistics = running.backend->driver().statistics();
        emit(JsonLine{"statistics"}
                 .field("cycles", statistics.cycles)
                 .field("missed_replies", statistics.missedReplies)
                 .field("foreign_frames", statistics.foreignFrames)
                 .field("sent", statistics.transport.sent)
                 .field("received", statistics.transport.received)
                 .field("send_failures", statistics.transport.sendFailures)
                 .field("error_frames", statistics.transport.errorFrames)
                 .field("bus_off", statistics.transport.busOff)
                 .field("missed_periods", running.backend->timeline().missedPeriods())
                 .field("bus_time_ms", larm::toSeconds(robstride::cycleBusTime(config)) * 1e3)
                 .field("period_ms", larm::toSeconds(profile.controlPeriod) * 1e3));
    }

    int hold(double const seconds) {
        if (not options.confirmPower) {
            return fail("hold powers the motors; add --confirm-power", kUsage);
        }
        auto running = startRuntime();
        if (not running) {
            return fail(running.error().message);
        }
        if (auto const refused = checkStart(*running->session); refused != kOk) {
            return refused;
        }
        auto const sampling = sampler(*running->session);
        if (not succeeded(running->session->enable(), "enable")) {
            return finish(*running, kFailed);
        }
        auto const until = std::chrono::steady_clock::now() + larm::fromSeconds(seconds);
        while (std::chrono::steady_clock::now() < until and not stop->stop_requested()) {
            std::this_thread::sleep_for(std::chrono::milliseconds{50});
        }
        return finish(*running, kOk);
    }

    int jog(std::string const &jointName, double const amplitude) {
        if (not options.confirmMotion) {
            return fail("jog moves the arm; add --confirm-motion", kUsage);
        }
        if (not(std::abs(amplitude) <= kMaxJogRadians)) {
            return fail("the jog amplitude is limited to 0.2 rad", kUsage);
        }
        auto const group = profile.findGroup("arm");
        auto const joint = profile.findJoint(jointName);
        if (not group or not joint) {
            return fail("'" + jointName + "' is not a joint of the arm group", kUsage);
        }
        auto const &members = profile.groups[*group].joints;
        auto const member = std::find(members.begin(), members.end(), *joint);
        if (member == members.end()) {
            return fail("'" + jointName + "' is not a joint of the arm group", kUsage);
        }
        auto rest = JointVector{static_cast<Eigen::Index>(members.size())};
        for (std::size_t i = 0; i < members.size(); ++i) {
            rest[larm::idx(i)] = profile.safety.restPose[larm::idx(members[i])];
        }
        auto target = rest;
        target[std::distance(members.begin(), member)] += amplitude;
        auto const &limits = profile.joints[*joint].limits;
        if (target[std::distance(members.begin(), member)] < limits.lower or
            target[std::distance(members.begin(), member)] > limits.upper) {
            return fail("the jog target leaves the joint limits; jog in the other direction", kUsage);
        }

        auto running = startRuntime();
        if (not running) {
            return fail(running.error().message);
        }
        if (auto const refused = checkStart(*running->session); refused != kOk) {
            return refused;
        }
        auto const sampling = sampler(*running->session);
        auto *const arm = running->session->motion("arm");
        auto const ok = succeeded(running->session->enable(), "enable") and
                        succeeded(arm->moveToJoints({.position = target, .speed = kJogSpeed}), "jog out") and
                        succeeded(arm->moveToJoints({.position = rest, .speed = kJogSpeed}), "jog back");
        return finish(*running, ok ? kOk : kFailed);
    }
};

void usage() {
    std::cerr
        << "usage: larm_driver_probe --profile FILE [--simulated] [--output FILE] [--rt-priority N] COMMAND\n"
           "  scan\n  monitor SECONDS\n  hold SECONDS --confirm-power\n"
           "  jog JOINT RADIANS --confirm-motion\n";
}

} // namespace

int main(int argc, char **argv) {
    auto stop = lexec::inplace_stop_source{};
    auto const signals = SignalStop{stop};
    auto const options = parse(argc, argv);
    if (not options) {
        usage();
        return kUsage;
    }
    auto file = std::ofstream{};
    if (not options->output.empty()) {
        file.open(options->output);
        if (not file) {
            std::cerr << "larm_driver_probe: cannot write " << options->output << '\n';
            return kUsage;
        }
    }
    auto profile = larm::loadRobotProfile(options->profile);
    if (not profile) {
        std::cerr << "larm_driver_probe: " << profile.error().message << '\n';
        return kUsage;
    }
    auto config = robstride::parseDriverConfig(*profile);
    if (not config) {
        std::cerr << "larm_driver_probe: " << config.error().message << '\n';
        return kUsage;
    }
    auto probe = Probe{.profile = std::move(*profile),
                       .config = std::move(*config),
                       .out = options->output.empty() ? &std::cout : &file,
                       .stop = &stop,
                       .options = *options};
    auto const &arguments = options->arguments;
    try {
        if (options->command == "scan" and arguments.empty()) {
            return probe.scan();
        }
        if (options->command == "monitor" and arguments.size() == 1) {
            return probe.monitor(std::stod(arguments[0]));
        }
        if (options->command == "hold" and arguments.size() == 1) {
            return probe.hold(std::stod(arguments[0]));
        }
        if (options->command == "jog" and arguments.size() == 2) {
            return probe.jog(arguments[0], std::stod(arguments[1]));
        }
    } catch (std::exception const &error) {
        return probe.fail(error.what());
    }
    usage();
    return kUsage;
}
