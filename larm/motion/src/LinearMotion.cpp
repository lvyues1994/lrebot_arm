#include <larm/motion/LinearMotion.h>

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <numbers>
#include <vector>

namespace larm::motion {
namespace {

// Spacing of the IK samples along the line.
constexpr double kLinearStep = 0.001;
constexpr double kAngularStep = 0.5 * std::numbers::pi / 180.0;
// A larger change of a joint between neighbouring samples means a singularity or another IK branch.
constexpr double kMaxJointStep = 0.05;
// Below this the TCP does not move.
constexpr double kStill = 1e-9;
// Jerk of the path timing per unit of its acceleration, in 1/s.
constexpr double kJerkPerAcceleration = 10.0;
constexpr auto kWaypointPeriod = std::chrono::milliseconds{10};
constexpr int kMaxRetiming = 8;

// The joint path q(s) for s in [0, 1] through IK samples at equal steps of s, as cubic Hermite segments.
struct SampledPath {
    std::vector<JointVector> points;
    // dq/ds at the points.
    std::vector<JointVector> tangents;

    struct Sample {
        JointVector position;
        JointVector first;
        JointVector second;
    };

    Sample at(double const s) const {
        auto const segments = points.size() - 1;
        auto const h = 1.0 / static_cast<double>(segments);
        auto const scaled = std::clamp(s, 0.0, 1.0) * static_cast<double>(segments);
        auto const i = std::min(static_cast<std::size_t>(scaled), segments - 1);
        auto const u = scaled - static_cast<double>(i);
        auto const &p0 = points[i];
        auto const &p1 = points[i + 1];
        JointVector const m0 = tangents[i] * h;
        JointVector const m1 = tangents[i + 1] * h;
        auto const u2 = u * u;
        auto const u3 = u2 * u;
        return Sample{
            .position = (2 * u3 - 3 * u2 + 1) * p0 + (u3 - 2 * u2 + u) * m0 + (-2 * u3 + 3 * u2) * p1 +
                        (u3 - u2) * m1,
            .first = ((6 * u2 - 6 * u) * p0 + (3 * u2 - 4 * u + 1) * m0 + (-6 * u2 + 6 * u) * p1 +
                      (3 * u2 - 2 * u) * m1) /
                     h,
            .second =
                ((12 * u - 6) * p0 + (6 * u - 4) * m0 + (-12 * u + 6) * p1 + (6 * u - 2) * m1) / (h * h),
        };
    }
};

Pose3 interpolate(Pose3 const &from, Pose3 const &to, double const s) {
    return Pose3{.translation = from.translation + s * (to.translation - from.translation),
                 .rotation = from.rotation.slerp(s, to.rotation).normalized()};
}

int percent(double const s) { return static_cast<int>(std::lround(100.0 * s)); }

Expected<SampledPath> followLine(LinearRequest const &request, Pose3 const &from, model::IkSolver &solver,
                                 std::size_t const steps) {
    auto path = SampledPath{.points = {request.start}, .tangents = {}};
    auto const toolFromTcp = request.tcp.inverse();
    for (std::size_t step = 1; step <= steps; ++step) {
        auto const s = static_cast<double>(step) / static_cast<double>(steps);
        auto const solution =
            solver.solve(model::IkRequest{.target = interpolate(from, request.target, s) * toolFromTcp,
                                          .frame = request.tool,
                                          .joints = request.joints,
                                          .restarts = 0},
                         path.points.back());
        if (not solution.converged) {
            return makeError(
                ErrorCode::SolverFailed,
                std::format("no joint configuration follows the line {}% of the way", percent(s)));
        }
        if ((solution.position - path.points.back()).cwiseAbs().maxCoeff() > kMaxJointStep) {
            return makeError(
                ErrorCode::InvalidArgument,
                std::format("the line passes too close to a singularity {}% of the way", percent(s)));
        }
        path.points.push_back(solution.position);
    }
    auto const h = 1.0 / static_cast<double>(steps);
    for (std::size_t i = 0; i <= steps; ++i) {
        auto const before = i == 0 ? i : i - 1;
        auto const after = i == steps ? i : i + 1;
        path.tangents.push_back((path.points[after] - path.points[before]) /
                                (static_cast<double>(after - before) * h));
    }
    return path;
}

// Limits on the path parameter s and its derivatives.
struct Timing {
    double velocity{};
    double acceleration{};
};

std::vector<TimedWaypoint> timeWaypoints(SampledPath const &path, Timing const &timing) {
    auto const one = JointVector::Constant(1, 1.0);
    auto mask = JointMask{};
    mask.set(0);
    auto const progress =
        planPointToPoint({.start = JointSample::zero(1),
                          .target = one,
                          .limits = {.velocity = one * timing.velocity,
                                     .acceleration = one * timing.acceleration,
                                     .jerk = one * timing.acceleration * kJerkPerAcceleration},
                          .joints = mask});
    auto waypoints = std::vector<TimedWaypoint>{};
    if (not progress) {
        return waypoints;
    }
    auto const duration = (*progress)->duration();
    auto sample = JointSample::zero(1);
    for (auto time = Duration{0};; time += kWaypointPeriod) {
        auto const last = time + kWaypointPeriod / 2 >= duration;
        if (last) {
            time = duration;
        }
        (*progress)->sample(time, sample);
        auto const rate = sample.velocity[0];
        auto const joints = path.at(sample.position[0]);
        waypoints.push_back({.time = time,
                             .position = time == Duration{0} ? path.points.front() : joints.position,
                             .velocity = JointVector{joints.first * rate},
                             .acceleration = JointVector{joints.second * rate * rate +
                                                         joints.first * sample.acceleration[0]}});
        if (last) {
            return waypoints;
        }
    }
}

// How far the waypoints exceed the joint limits: the largest velocity and acceleration ratios.
std::pair<double, double> excess(std::vector<TimedWaypoint> const &waypoints, LinearRequest const &request) {
    auto velocity = 0.0;
    auto acceleration = 0.0;
    for (auto const &waypoint : waypoints) {
        for (std::size_t j = 0; j < dofOf(request.start); ++j) {
            if (request.joints.test(j)) {
                velocity = std::max(velocity,
                                    std::abs((*waypoint.velocity)[idx(j)]) / request.limits.velocity[idx(j)]);
                acceleration = std::max(acceleration, std::abs((*waypoint.acceleration)[idx(j)]) /
                                                          request.limits.acceleration[idx(j)]);
            }
        }
    }
    return {velocity, acceleration};
}

// The fastest timing the TCP limits allow, and that keeps the joint velocities along the path.
Timing initialTiming(SampledPath const &path, LinearRequest const &request, double const distance,
                     double const angle) {
    auto timing = Timing{.velocity = std::numeric_limits<double>::infinity(),
                         .acceleration = std::numeric_limits<double>::infinity()};
    if (distance > kStill) {
        timing.velocity = std::min(timing.velocity, request.cartesian.linearVelocity / distance);
        timing.acceleration = std::min(timing.acceleration, request.cartesian.linearAcceleration / distance);
    }
    if (angle > kStill) {
        timing.velocity = std::min(timing.velocity, request.cartesian.angularVelocity / angle);
        timing.acceleration = std::min(timing.acceleration, request.cartesian.angularAcceleration / angle);
    }
    for (auto const &tangent : path.tangents) {
        for (std::size_t j = 0; j < dofOf(request.start); ++j) {
            if (request.joints.test(j) and std::abs(tangent[idx(j)]) > 0.0) {
                timing.velocity =
                    std::min(timing.velocity, request.limits.velocity[idx(j)] / std::abs(tangent[idx(j)]));
            }
        }
    }
    return timing;
}

} // namespace

Expected<std::shared_ptr<JointTrajectory const>>
planLinear(LinearRequest const &request, model::Kinematics &kinematics, model::IkSolver &solver) {
    kinematics.update(request.start);
    auto const from = kinematics.framePose(request.tool) * request.tcp;
    auto const distance = (request.target.translation - from.translation).norm();
    auto const angle = from.rotation.angularDistance(request.target.rotation);
    if (distance <= kStill and angle <= kStill) {
        auto const still = std::vector<TimedWaypoint>{{.time = Duration{0}, .position = request.start}};
        return interpolateWaypoints(still);
    }

    auto const reached = solver.solve(model::IkRequest{.target = request.target * request.tcp.inverse(),
                                                       .frame = request.tool,
                                                       .joints = request.joints},
                                      request.start);
    if (not reached.converged) {
        return makeError(ErrorCode::SolverFailed, "no joint configuration reaches the target");
    }
    auto const steps = static_cast<std::size_t>(
        std::max(1.0, std::ceil(std::max(distance / kLinearStep, angle / kAngularStep))));
    auto const path = followLine(request, from, solver, steps);
    if (not path) {
        return tl::make_unexpected(path.error());
    }
    auto timing = initialTiming(*path, request, distance, angle);
    for (int attempt = 0; attempt < kMaxRetiming; ++attempt) {
        auto const waypoints = timeWaypoints(*path, timing);
        if (waypoints.empty()) {
            return makeError(ErrorCode::Internal, "cannot time the line");
        }
        auto const [velocity, acceleration] = excess(waypoints, request);
        if (velocity <= 1.0 and acceleration <= 1.0) {
            return interpolateWaypoints(waypoints);
        }
        // Velocities scale with the path rate, accelerations with its square.
        auto const slower =
            0.95 * std::min(1.0 / std::max(velocity, 1.0), 1.0 / std::sqrt(std::max(acceleration, 1.0)));
        timing.velocity *= slower;
        timing.acceleration *= slower * slower;
    }
    return makeError(ErrorCode::InvalidArgument, "cannot keep the joint limits along the line");
}

} // namespace larm::motion
