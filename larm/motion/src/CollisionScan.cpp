#include <larm/motion/CollisionScan.h>

namespace larm::motion {
namespace {

// Fine enough that no joint moves a whole step between two samples.
constexpr auto kSamplePeriod = std::chrono::milliseconds{1};

} // namespace

std::optional<TrajectoryCollision> findCollision(JointTrajectory const &trajectory,
                                                 model::CollisionChecker &checker, JointVector const &step) {
    auto sample = JointSample::zero(trajectory.dof());
    trajectory.sample(Duration{0}, sample);
    checker.allowContactsAt(sample.position);
    JointVector checked = sample.position;
    auto const end = trajectory.duration();
    for (auto time = Duration{0}; time < end;) {
        time = std::min<Duration>(time + kSamplePeriod, end);
        trajectory.sample(time, sample);
        if (time < end and ((sample.position - checked).cwiseAbs().array() < step.array()).all()) {
            continue;
        }
        if (auto contact = checker.collision(sample.position)) {
            return TrajectoryCollision{.time = time, .contact = std::move(*contact)};
        }
        checked = sample.position;
    }
    return std::nullopt;
}

} // namespace larm::motion
