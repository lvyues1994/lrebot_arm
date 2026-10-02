#include <larm/motion/Planning.h>

#include <algorithm>
#include <vector>

namespace larm::motion {
namespace {

struct Knot {
    double time{};
    JointVector position;
    JointVector velocity;
    JointVector acceleration;
    bool hasAcceleration{};
};

// Position, velocity and acceleration of a Hermite segment at normalized time s over length h.
struct HermiteSample {
    double position{};
    double velocity{};
    double acceleration{};
};

HermiteSample cubic(double const s, double const h, double const p0, double const v0, double const p1,
                    double const v1) noexcept {
    auto const s2 = s * s;
    auto const s3 = s2 * s;
    auto const m0 = v0 * h;
    auto const m1 = v1 * h;
    auto const position =
        (2 * s3 - 3 * s2 + 1) * p0 + (s3 - 2 * s2 + s) * m0 + (-2 * s3 + 3 * s2) * p1 + (s3 - s2) * m1;
    auto const velocity =
        (6 * s2 - 6 * s) * p0 + (3 * s2 - 4 * s + 1) * m0 + (-6 * s2 + 6 * s) * p1 + (3 * s2 - 2 * s) * m1;
    auto const acceleration = (12 * s - 6) * p0 + (6 * s - 4) * m0 + (-12 * s + 6) * p1 + (6 * s - 2) * m1;
    return HermiteSample{
        .position = position, .velocity = velocity / h, .acceleration = acceleration / (h * h)};
}

HermiteSample quintic(double const s, double const h, double const p0, double const v0, double const a0,
                      double const p1, double const v1, double const a1) noexcept {
    // p(s) = c0 + c1 s + ... + c5 s^5 matching position, velocity and acceleration at both ends.
    auto const c0 = p0;
    auto const c1 = v0 * h;
    auto const c2 = 0.5 * a0 * h * h;
    auto const d = p1 - p0 - c1 - c2;
    auto const e = v1 * h - c1 - 2 * c2;
    auto const f = a1 * h * h - 2 * c2;
    auto const c3 = 10 * d - 4 * e + 0.5 * f;
    auto const c4 = -15 * d + 7 * e - f;
    auto const c5 = 6 * d - 3 * e + 0.5 * f;
    auto const position = c0 + s * (c1 + s * (c2 + s * (c3 + s * (c4 + s * c5))));
    auto const velocity = c1 + s * (2 * c2 + s * (3 * c3 + s * (4 * c4 + s * 5 * c5)));
    auto const acceleration = 2 * c2 + s * (6 * c3 + s * (12 * c4 + s * 20 * c5));
    return HermiteSample{
        .position = position, .velocity = velocity / h, .acceleration = acceleration / (h * h)};
}

struct WaypointTrajectory final : JointTrajectory {
    std::size_t dof() const noexcept override { return dofCount; }

    Duration duration() const noexcept override { return fromSeconds(knots.back().time); }

    void sample(Duration const time, JointSample &out) const noexcept override {
        out.position.resize(idx(dofCount));
        out.velocity.resize(idx(dofCount));
        out.acceleration.resize(idx(dofCount));
        auto const t = std::clamp(toSeconds(time), 0.0, knots.back().time);
        if (knots.size() == 1 or t >= knots.back().time) {
            out.position = knots.back().position;
            out.velocity = knots.back().velocity;
            out.acceleration =
                knots.back().hasAcceleration ? knots.back().acceleration : zeroJointVector(dofCount);
            return;
        }
        auto const next =
            std::upper_bound(knots.begin(), knots.end(), t,
                             [](double const value, Knot const &knot) { return value < knot.time; });
        auto const &end = *next;
        auto const &begin = *(next - 1);
        auto const h = end.time - begin.time;
        auto const s = (t - begin.time) / h;
        auto const useQuintic = begin.hasAcceleration and end.hasAcceleration;
        for (std::size_t i = 0; i < dofCount; ++i) {
            auto const j = idx(i);
            auto const value =
                useQuintic
                    ? quintic(s, h, begin.position[j], begin.velocity[j], begin.acceleration[j],
                              end.position[j], end.velocity[j], end.acceleration[j])
                    : cubic(s, h, begin.position[j], begin.velocity[j], end.position[j], end.velocity[j]);
            out.position[j] = value.position;
            out.velocity[j] = value.velocity;
            out.acceleration[j] = value.acceleration;
        }
    }

    std::size_t dofCount{};
    std::vector<Knot> knots;
};

bool hasSize(std::optional<JointVector> const &vector, std::size_t const dof) {
    return not vector or (dofOf(*vector) == dof and vector->allFinite());
}

} // namespace

Expected<std::shared_ptr<JointTrajectory const>>
interpolateWaypoints(std::span<TimedWaypoint const> const waypoints) {
    if (waypoints.empty() or waypoints.front().time != Duration{0}) {
        return makeError(ErrorCode::InvalidArgument, "waypoints must start at time zero");
    }
    auto const dof = dofOf(waypoints.front().position);
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
        auto const &waypoint = waypoints[i];
        if (dofOf(waypoint.position) != dof or not waypoint.position.allFinite() or
            not hasSize(waypoint.velocity, dof) or not hasSize(waypoint.acceleration, dof)) {
            return makeError(ErrorCode::InvalidArgument,
                             "waypoint " + std::to_string(i) + " has inconsistent vectors");
        }
        if (i > 0 and waypoint.time <= waypoints[i - 1].time) {
            return makeError(ErrorCode::InvalidArgument, "waypoint times must increase");
        }
    }

    auto trajectory = std::make_shared<WaypointTrajectory>();
    trajectory->dofCount = dof;
    for (std::size_t i = 0; i < waypoints.size(); ++i) {
        auto const &waypoint = waypoints[i];
        auto velocity = zeroJointVector(dof);
        if (waypoint.velocity) {
            velocity = *waypoint.velocity;
        } else if (i > 0 and i + 1 < waypoints.size()) {
            velocity = (waypoints[i + 1].position - waypoints[i - 1].position) /
                       toSeconds(waypoints[i + 1].time - waypoints[i - 1].time);
        }
        trajectory->knots.push_back(Knot{
            .time = toSeconds(waypoint.time),
            .position = waypoint.position,
            .velocity = velocity,
            .acceleration = waypoint.acceleration.value_or(zeroJointVector(dof)),
            .hasAcceleration = waypoint.acceleration.has_value(),
        });
    }
    return std::shared_ptr<JointTrajectory const>{std::move(trajectory)};
}

} // namespace larm::motion
