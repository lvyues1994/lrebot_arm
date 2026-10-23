// larm_driver_probe: brings RobStride hardware up one step at a time.
//
//   larm_driver_probe --profile FILE [--joints J1,J2,...] [--simulated] [--output FILE] [--rt-priority N]
//                     COMMAND
//
//   scan                          ping every actuator, read its settings and position (read-only)
//   monitor SECONDS               sample positions at 50 Hz through parameter reads (read-only)
//   hold SECONDS --confirm-power  enable at the rest pose, hold, disable
//   jog JOINT RADIANS [--speed FRACTION] --confirm-motion
//                                 enable, move one arm joint by at most 0.2 rad and back slowly, disable;
//                                 FRACTION of the joint limits is at most 0.2, the default
//   pose NAME SECONDS [--speed FRACTION] --confirm-motion
//                                 enable, move the arm from rest to the profile pose NAME, hold it, move
//                                 back, disable
//   calibrate FILE [SET] [--speed FRACTION] --confirm-motion
//                                 enable, visit the poses SET (default "poses") of a gravity calibration
//                                 file, approaching each from both sides, and record the holding torque;
//                                 move back, disable
//   grip METERS [--effort NEWTONS] --confirm-motion
//                                 enable, open (positive) or close the gripper by at most 0.01 m and
//                                 back, disable; the effort limit is at most the profile's, the default
//
// --joints keeps only those joints of the profile, with their actuators: the other motors are never
// addressed, neither read nor powered. Writes one JSON object per line. Commands that power the motors
// need their confirmation flag and refuse to start unless the arm rests at the profile's rest pose
// without faults. Ctrl+C stops the motion; the arm is disabled only where it rests, otherwise it is
// left holding and reported. --simulated runs the same code against simulated motors carrying the
// arm's gravity.
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
constexpr double kMaxGripMeters = 0.01;
constexpr double kJogSpeed = 0.2;
constexpr auto kSamplePeriod = std::chrono::milliseconds{20};
// How long the disabled arm is still sampled before the statistics are taken.
constexpr auto kObserveDisabled = std::chrono::milliseconds{500};
// A calibration pose is recorded for kRecord after the arm has been still there for kSettle.
constexpr auto kSettle = std::chrono::milliseconds{1000};
constexpr auto kRecord = std::chrono::milliseconds{1000};

enum ExitCode : int { kOk = 0, kFailed = 1, kUsage = 2, kRefused = 3 };

struct Options {
    std::string profile;
    bool simulated{};
    std::string output;
    int rtPriority{};
    // Empty: every joint of the profile.
    std::vector<std::string> joints;
    double speed = kJogSpeed;
    // Gripper effort limit for grip; the profile's when empty.
    std::optional<double> effort;
    std::string command;
    std::vector<std::string> arguments;
    bool confirmPower{};
    bool confirmMotion{};
};

std::vector<std::string> splitNames(std::string const &list) {
    auto names = std::vector<std::string>{};
    auto stream = std::istringstream{list};
    for (auto name = std::string{}; std::getline(stream, name, ',');) {
        if (not name.empty()) {
            names.push_back(name);
        }
    }
    return names;
}

std::optional<Options> parse(int const argc, char **argv) {
    auto options = Options{};
    for (int i = 1; i < argc; ++i) {
        auto const argument = std::string_view{argv[i]};
        auto const next = [&]() -> std::optional<std::string> {
            return i + 1 < argc ? std::optional<std::string>{argv[++i]} : std::nullopt;
        };
        if (argument == "--profile" or argument == "--output" or argument == "--rt-priority" or
            argument == "--joints" or argument == "--speed" or argument == "--effort") {
            auto const value = next();
            if (not value) {
                return std::nullopt;
            }
            if (argument == "--profile") {
                options.profile = *value;
            } else if (argument == "--output") {
                options.output = *value;
            } else if (argument == "--joints") {
                options.joints = splitNames(*value);
            } else if (argument == "--speed") {
                auto stream = std::istringstream{*value};
                if (not(stream >> options.speed) or not(options.speed > 0.0 and options.speed <= kJogSpeed)) {
                    return std::nullopt;
                }
            } else if (argument == "--effort") {
                auto stream = std::istringstream{*value};
                auto effort = 0.0;
                if (not(stream >> effort) or not(effort > 0.0)) {
                    return std::nullopt;
                }
                options.effort = effort;
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

// The profile cut down to the named joints, in profile order. Groups lose the other members (and go
// when none is left), and the driver section keeps only the actuators of the named joints.
larm::Expected<larm::RobotProfile> restrictTo(larm::RobotProfile const &profile,
                                              std::vector<std::string> const &names) {
    for (auto const &name : names) {
        if (not profile.findJoint(name)) {
            return larm::makeError(larm::ErrorCode::InvalidArgument,
                                   "the profile has no joint '" + name + "'");
        }
    }
    auto const kept = [&](std::string const &name) {
        return std::find(names.begin(), names.end(), name) != names.end();
    };
    auto reduced = profile;
    reduced.joints.clear();
    auto remap = std::vector<std::optional<std::size_t>>(profile.dof());
    for (std::size_t i = 0; i < profile.dof(); ++i) {
        if (kept(profile.joints[i].name)) {
            remap[i] = reduced.joints.size();
            reduced.joints.push_back(profile.joints[i]);
        }
    }
    auto const subset = [&](JointVector const &values) {
        auto out = larm::zeroJointVector(reduced.dof());
        for (std::size_t i = 0; i < profile.dof(); ++i) {
            if (remap[i]) {
                out[larm::idx(*remap[i])] = values[larm::idx(i)];
            }
        }
        return out;
    };
    reduced.groups.clear();
    for (auto group : profile.groups) {
        auto members = std::vector<std::size_t>{};
        for (auto const joint : group.joints) {
            if (remap[joint]) {
                members.push_back(*remap[joint]);
            }
        }
        if (not members.empty()) {
            group.joints = std::move(members);
            reduced.groups.push_back(std::move(group));
        }
    }
    reduced.safety.restPose = subset(profile.safety.restPose);
    for (auto &[name, pose] : reduced.poses) {
        pose = subset(pose);
    }
    auto actuators = YAML::Node{YAML::NodeType::Sequence};
    for (auto const &actuator : profile.driver.node["actuators"]) {
        if (actuator["joint"] and kept(actuator["joint"].as<std::string>())) {
            actuators.push_back(YAML::Clone(actuator));
        }
    }
    // Nodes share their data when copied.
    reduced.driver.node = YAML::Clone(profile.driver.node);
    reduced.driver.node["actuators"] = actuators;
    return reduced;
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

    // A motor angle as read after power-on, as the driver maps it to its joint.
    static double jointOf(robstride::ActuatorConfig const &actuator, double const motor) {
        auto const &[scale, offset] = actuator.transmission;
        return scale * (motor + robstride::turnCorrection(actuator, motor)) + offset;
    }

    std::vector<double> motorToJoint(std::span<double const> const motor) const {
        auto joint = std::vector<double>(motor.size());
        for (std::size_t i = 0; i < motor.size(); ++i) {
            joint[i] = jointOf(config.actuators[i], motor[i]);
        }
        return joint;
    }

    // Read-only access: the CAN interface, or simulated motors resting at the rest pose.
    ReadOnlyBus readOnlyBus(std::unique_ptr<robstride::SimulatedMotors> &simulated) const {
        if (options.simulated) {
            simulated = robstride::makeSimulatedMotors(config, {.position = restMotorPosition()});
            return ReadOnlyBus{config, simulated->transport()};
        }
        auto transport =
            larm::drivers::can::openSocketCan(config.interface, {.burst = config.actuators.size()});
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
                    line.field("motor_position", motor).field("joint_position", jointOf(actuator, motor));
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

    // Joints too far from the rest pose for the runtime to disable, by name; the gripper is not checked.
    std::vector<std::string> awayFromRest(larm::control::RobotSnapshot const &snapshot) const {
        auto away = std::vector<std::string>{};
        for (std::size_t i = 0; i < profile.dof(); ++i) {
            auto const j = larm::idx(i);
            if (profile.joints[i].unit == larm::JointUnit::Radian and
                std::abs(snapshot.state.joints.position[j] - profile.safety.restPose[j]) >
                    larm::runtime::kRestToleranceRadian) {
                away.push_back(profile.joints[i].name);
            }
        }
        return away;
    }

    // The joints of the arm group, in group order.
    std::vector<std::size_t> armJoints() const {
        auto const group = profile.findGroup("arm");
        return group ? profile.groups[*group].joints : std::vector<std::size_t>{};
    }

    JointVector armRest() const {
        auto const joints = armJoints();
        auto rest = JointVector{static_cast<Eigen::Index>(joints.size())};
        for (std::size_t i = 0; i < joints.size(); ++i) {
            rest[larm::idx(i)] = profile.safety.restPose[larm::idx(joints[i])];
        }
        return rest;
    }

    bool anyPowered(larm::control::RobotSnapshot const &snapshot) const {
        for (std::size_t i = 0; i < profile.dof(); ++i) {
            if (snapshot.state.actuators[i].enabled) {
                return true;
            }
        }
        return snapshot.power == larm::hal::DrivePower::Enabled;
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
        if (auto const latest = session.latest();
            latest.power == larm::hal::DrivePower::Enabled and not awayFromRest(latest).empty()) {
            // At the probe's speed: the arm may be far from rest, and the runtime parks at half speed.
            auto *const arm = session.motion("arm");
            auto const parked =
                arm != nullptr ? succeeded(arm->moveToJoints({.position = armRest(), .speed = options.speed}),
                                           "park", Interruptible::No)
                               : succeeded(session.park(), "park", Interruptible::No);
            code = parked ? code : kFailed;
        }
        // A park that ends short of its goal may still leave the arm close enough to disable.
        if (awayFromRest(session.latest()).empty() and
            not succeeded(session.disable({}), "disable", Interruptible::No)) {
            code = kFailed;
        }
        std::this_thread::sleep_for(kObserveDisabled);
        if (auto const latest = session.latest(); anyPowered(latest)) {
            auto names = std::string{};
            for (auto const &name : awayFromRest(latest)) {
                names += (names.empty() ? "" : ", ") + name;
            }
            emit(JsonLine{"warning"}.field(
                "message", "the motors stay enabled and hold the arm after this program ends" +
                               (names.empty() ? std::string{} : " (away from rest: " + names + ")") +
                               "; support the arm, then press the emergency stop"));
            code = kFailed;
        }
        if (auto const latest = session.latest(); latest.safety != larm::control::SafetyState::Normal) {
            emit(
                JsonLine{"warning"}.field("message", "the runtime ends with a fault: " +
                                                         std::string{larm::control::toString(latest.fault)}));
            code = kFailed;
        }
        report(running);
        return code;
    }

    void report(Running const &running) const {
        auto const statistics = running.backend->driver().statistics();
        auto const latest = running.session->latest();
        emit(JsonLine{"statistics"}
                 .field("fault", larm::control::toString(latest.fault))
                 .field("fresh", latest.state.isFresh)
                 .field("cycles", statistics.cycles)
                 .field("missed_replies", statistics.missedReplies)
                 .field("foreign_frames", statistics.foreignFrames)
                 .field("sent", statistics.transport.sent)
                 .field("received", statistics.transport.received)
                 .field("send_failures", statistics.transport.sendFailures)
                 .field("queue_full", statistics.transport.queueFull)
                 .field("error_frames", statistics.transport.errorFrames)
                 .field("unacknowledged", statistics.transport.unacknowledged)
                 .field("bus_state", larm::drivers::can::toString(statistics.transport.busState))
                 .field("interface_down", statistics.transport.interfaceDown)
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
        auto const ok =
            succeeded(running->session->enable(), "enable") and
            succeeded(arm->moveToJoints({.position = target, .speed = options.speed}), "jog out") and
            succeeded(arm->moveToJoints({.position = rest, .speed = options.speed}), "jog back");
        return finish(*running, ok ? kOk : kFailed);
    }

    // Moves the arm from rest to the profile pose `name`, holds it for `seconds`, and moves back.
    int pose(std::string const &name, double const seconds) {
        if (not options.confirmMotion) {
            return fail("pose moves the arm; add --confirm-motion", kUsage);
        }
        auto const group = profile.findGroup("arm");
        auto const found = profile.poses.find(name);
        if (not group or found == profile.poses.end()) {
            return fail("the profile has no arm group or no pose '" + name + "'", kUsage);
        }
        auto const &members = profile.groups[*group].joints;
        auto target = JointVector{static_cast<Eigen::Index>(members.size())};
        auto rest = target;
        for (std::size_t i = 0; i < members.size(); ++i) {
            target[larm::idx(i)] = found->second[larm::idx(members[i])];
            rest[larm::idx(i)] = profile.safety.restPose[larm::idx(members[i])];
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
        auto ok =
            succeeded(running->session->enable(), "enable") and
            succeeded(arm->moveToJoints({.position = target, .speed = options.speed}), "move to " + name);
        if (ok) {
            auto const until = std::chrono::steady_clock::now() + larm::fromSeconds(seconds);
            while (std::chrono::steady_clock::now() < until and not stop->stop_requested()) {
                std::this_thread::sleep_for(std::chrono::milliseconds{50});
            }
            ok = succeeded(arm->moveToJoints({.position = rest, .speed = options.speed}), "move back");
        }
        return finish(*running, ok ? kOk : kFailed);
    }

    struct CalibrationPlan {
        JointVector approach;
        std::vector<JointVector> poses;
    };

    // The approach offsets and the poses named `set` of a calibration file, for the arm group.
    larm::Expected<CalibrationPlan> loadCalibrationPlan(std::string const &file,
                                                        std::string const &set) const {
        auto const joints = armJoints();
        auto const vector = [&](YAML::Node const &node,
                                std::string const &what) -> larm::Expected<JointVector> {
            if (not node.IsSequence() or node.size() != joints.size()) {
                return larm::makeError(larm::ErrorCode::InvalidConfig,
                                       file + ": " + what + " needs one value per arm joint");
            }
            auto values = JointVector{static_cast<Eigen::Index>(joints.size())};
            for (std::size_t i = 0; i < joints.size(); ++i) {
                values[larm::idx(i)] = node[i].as<double>();
            }
            return values;
        };
        try {
            auto const root = YAML::LoadFile(file);
            auto approach = vector(root["approach"], "approach");
            if (not approach) {
                return tl::make_unexpected(approach.error());
            }
            auto plan = CalibrationPlan{.approach = *approach, .poses = {}};
            if (not root[set] or not root[set].IsSequence()) {
                return larm::makeError(larm::ErrorCode::InvalidConfig,
                                       file + ": no list of poses '" + set + "'");
            }
            for (std::size_t n = 0; n < root[set].size(); ++n) {
                auto pose = vector(root[set][n], set + "[" + std::to_string(n) + "]");
                if (not pose) {
                    return tl::make_unexpected(pose.error());
                }
                for (std::size_t i = 0; i < joints.size(); ++i) {
                    auto const &limits = profile.joints[joints[i]].limits;
                    for (auto const side : {-1.0, 0.0, 1.0}) {
                        auto const value = (*pose)[larm::idx(i)] + side * plan.approach[larm::idx(i)];
                        if (not(value >= limits.lower and value <= limits.upper)) {
                            return larm::makeError(larm::ErrorCode::InvalidConfig,
                                                   file + ": " + set + "[" + std::to_string(n) +
                                                       "] or its approach leaves the limits of " +
                                                       profile.joints[joints[i]].name);
                        }
                    }
                }
                plan.poses.push_back(*pose);
            }
            return plan;
        } catch (YAML::Exception const &error) {
            return larm::makeError(larm::ErrorCode::InvalidConfig, file + ": " + error.what());
        }
    }

    // Means and spreads over a still interval, per arm joint.
    struct Measurement {
        std::vector<double> position;
        // The impedance torque commanded: effort + stiffness (q_cmd - q) + damping (v_cmd - v).
        std::vector<double> torque;
        std::vector<double> torqueSpread;
        // What the motors report.
        std::vector<double> effort;
        std::vector<double> effortSpread;
        double maxSpeed{};
        int samples{};
    };

    Measurement measure(larm::runtime::RobotSession &session, larm::Duration const duration) const {
        auto const joints = armJoints();
        auto const n = joints.size();
        auto sums = std::vector<double>(4 * n);
        auto squares = std::vector<double>(4 * n);
        auto result = Measurement{};
        auto const until = std::chrono::steady_clock::now() + duration;
        for (auto tick = std::chrono::steady_clock::now(); tick < until and not stop->stop_requested();
             tick += kSamplePeriod) {
            std::this_thread::sleep_until(tick);
            auto const snapshot = session.latest();
            auto const &state = snapshot.state.joints;
            auto const &command = snapshot.command;
            for (std::size_t i = 0; i < n; ++i) {
                auto const j = larm::idx(joints[i]);
                auto const torque = command.effort[j] +
                                    command.stiffness[j] * (command.position[j] - state.position[j]) +
                                    command.damping[j] * (command.velocity[j] - state.velocity[j]);
                for (auto const &[k, value] : {std::pair{0U, state.position[j]}, std::pair{1U, torque},
                                               std::pair{2U, state.effort[j]}}) {
                    sums[k * n + i] += value;
                    squares[k * n + i] += value * value;
                }
                result.maxSpeed = std::max(result.maxSpeed, std::abs(state.velocity[j]));
            }
            ++result.samples;
        }
        auto const count = static_cast<double>(std::max(result.samples, 1));
        auto const mean = [&](unsigned const k, std::size_t const i) { return sums[k * n + i] / count; };
        auto const spread = [&](unsigned const k, std::size_t const i) {
            return std::sqrt(std::max(squares[k * n + i] / count - mean(k, i) * mean(k, i), 0.0));
        };
        for (std::size_t i = 0; i < n; ++i) {
            result.position.push_back(mean(0, i));
            result.torque.push_back(mean(1, i));
            result.torqueSpread.push_back(spread(1, i));
            result.effort.push_back(mean(2, i));
            result.effortSpread.push_back(spread(2, i));
        }
        return result;
    }

    // Visits each pose of the set, approaching it from below and from above every joint, and records the
    // torque that holds the arm there after it has come to rest.
    int calibrate(std::string const &file, std::string const &set) {
        if (not options.confirmMotion) {
            return fail("calibrate moves the arm; add --confirm-motion", kUsage);
        }
        if (armJoints().empty()) {
            return fail("the profile has no arm group", kUsage);
        }
        auto const plan = loadCalibrationPlan(file, set);
        if (not plan) {
            return fail(plan.error().message, kUsage);
        }

        auto running = startRuntime();
        if (not running) {
            return fail(running.error().message);
        }
        if (auto const refused = checkStart(*running->session); refused != kOk) {
            return refused;
        }
        auto &session = *running->session;
        auto const sampling = sampler(session);
        auto *const arm = session.motion("arm");
        auto ok = succeeded(session.enable(), "enable");
        for (std::size_t n = 0; ok and n < plan->poses.size(); ++n) {
            auto const &pose = plan->poses[n];
            for (auto const side : {-1, 1}) {
                ok = succeeded(arm->moveToJoints(
                                   {.position = pose + side * plan->approach, .speed = options.speed}),
                               "approach") and
                     succeeded(arm->moveToJoints({.position = pose, .speed = options.speed}), "arrive");
                if (not ok) {
                    break;
                }
                std::this_thread::sleep_for(kSettle);
                auto const measured = measure(session, kRecord);
                emit(JsonLine{"calibration"}
                         .field("set", set)
                         .field("index", static_cast<int>(n))
                         .field("side", side)
                         .field("target", std::span{pose.data(), static_cast<std::size_t>(pose.size())})
                         .field("position", measured.position)
                         .field("torque", measured.torque)
                         .field("torque_spread", measured.torqueSpread)
                         .field("effort", measured.effort)
                         .field("effort_spread", measured.effortSpread)
                         .field("max_speed", measured.maxSpeed)
                         .field("samples", measured.samples));
            }
        }
        if (ok) {
            ok = succeeded(arm->moveToJoints({.position = armRest(), .speed = options.speed}), "move back");
        }
        return finish(*running, ok ? kOk : kFailed);
    }

    // Opens (positive) or closes the gripper by `step` from where it is, and back.
    int grip(double const step) {
        if (not options.confirmMotion) {
            return fail("grip moves the gripper; add --confirm-motion", kUsage);
        }
        if (not(std::abs(step) <= kMaxGripMeters)) {
            return fail("the grip step is limited to 0.01 m", kUsage);
        }
        auto const group = profile.findGroup("gripper");
        if (not group) {
            return fail("the profile has no gripper group; add the gripper to --joints", kUsage);
        }
        auto const joint = profile.groups[*group].joints.front();
        auto const &limits = profile.joints[joint].limits;
        auto const effort = options.effort.value_or(limits.effort);
        if (effort > limits.effort) {
            return fail("the profile limits the gripper effort to " + std::to_string(limits.effort) + " N",
                        kUsage);
        }

        auto running = startRuntime();
        if (not running) {
            return fail(running.error().message);
        }
        if (auto const refused = checkStart(*running->session); refused != kOk) {
            return refused;
        }
        auto const start = running->session->latest().state.joints.position[larm::idx(joint)];
        if (start + step < limits.lower or start + step > limits.upper) {
            return fail("the gripper at " + std::to_string(start) + " m cannot move " + std::to_string(step) +
                            " m within its limits; step the other way",
                        kUsage);
        }
        auto const sampling = sampler(*running->session);
        auto *const hand = running->session->gripper("gripper");
        auto const ok = succeeded(running->session->enable(), "enable") and
                        gripped(hand->grip({.position = start + step, .maxEffort = effort}), "grip out") and
                        gripped(hand->grip({.position = start, .maxEffort = effort}), "grip back");
        return finish(*running, ok ? kOk : kFailed);
    }

    template <class Sender> bool gripped(Sender &&sender, std::string_view const step) const {
        try {
            auto const result = lexec::sync_wait(lexec::write_env(
                std::forward<Sender>(sender), lexec::prop{lexec::get_stop_token, stop->get_token()}));
            if (not result) {
                emit(JsonLine{"step"}.field("name", step).field("outcome", "stopped"));
                return false;
            }
            auto const &[grip] = *result;
            emit(JsonLine{"step"}
                     .field("name", step)
                     .field("outcome", grip.reachedGoal ? "succeeded" : "stalled")
                     .field("position", grip.position)
                     .field("effort", grip.effort));
            return grip.reachedGoal;
        } catch (std::exception const &error) {
            emit(JsonLine{"step"}
                     .field("name", step)
                     .field("outcome", "failed")
                     .field("message", error.what()));
        }
        return false;
    }
};

void usage() {
    std::cerr
        << "usage: larm_driver_probe --profile FILE [--joints J1,J2,...] [--simulated] [--output FILE]\n"
           "                         [--rt-priority N] COMMAND\n"
           "  scan\n  monitor SECONDS\n  hold SECONDS --confirm-power\n"
           "  jog JOINT RADIANS [--speed FRACTION<=0.2] --confirm-motion\n"
           "  pose NAME SECONDS [--speed FRACTION<=0.2] --confirm-motion\n"
           "  calibrate FILE [SET] [--speed FRACTION<=0.2] --confirm-motion\n"
           "  grip METERS [--effort NEWTONS] --confirm-motion\n";
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
    if (profile and not options->joints.empty()) {
        profile = restrictTo(*profile, options->joints);
    }
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
    auto joints = std::string{};
    for (auto const &actuator : probe.config.actuators) {
        joints += (joints.empty() ? "" : ",") + actuator.jointName;
    }
    probe.emit(
        JsonLine{"profile"}
            .field("joints", joints)
            .field("interface", options->simulated ? std::string{"simulated"} : probe.config.interface));
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
        if (options->command == "calibrate" and (arguments.size() == 1 or arguments.size() == 2)) {
            return probe.calibrate(arguments[0], arguments.size() == 2 ? arguments[1] : "poses");
        }
        if (options->command == "pose" and arguments.size() == 2) {
            return probe.pose(arguments[0], std::stod(arguments[1]));
        }
        if (options->command == "grip" and arguments.size() == 1) {
            return probe.grip(std::stod(arguments[0]));
        }
    } catch (std::exception const &error) {
        return probe.fail(error.what());
    }
    usage();
    return kUsage;
}
