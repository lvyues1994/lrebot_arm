#pragma once

#include <larm/core/Error.h>
#include <larm/core/JointVector.h>
#include <larm/core/Pose.h>
#include <larm/core/RobotProfile.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace larm::model {

// Rows [linear; angular] in world axes at the frame origin; columns in profile joint order.
using Jacobian = Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::ColMajor, 6, static_cast<int>(kMaxDof)>;

struct FrameId {
    std::uint32_t value{};
    friend bool operator==(FrameId, FrameId) = default;
};

// Forward kinematics owned by one thread. After construction no call allocates.
struct Kinematics {
    virtual ~Kinematics() = default;
    // Cold path: resolve a frame name once and keep the id.
    virtual std::optional<FrameId> findFrame(std::string_view name) const = 0;
    virtual void update(JointVector const &q) noexcept = 0;
    // Valid for the configuration passed to the last update().
    virtual Pose3 framePose(FrameId frame) const noexcept = 0;
    virtual void frameJacobian(FrameId frame, Jacobian &out) const noexcept = 0;
};

// Rigid-body dynamics owned by one thread. After construction no call allocates.
struct Dynamics {
    virtual ~Dynamics() = default;
    virtual void gravity(JointVector const &q, JointVector &out) noexcept = 0;
    virtual void inverseDynamics(JointVector const &q, JointVector const &velocity,
                                 JointVector const &acceleration, JointVector &out) noexcept = 0;
};

struct IkRequest {
    Pose3 target;
    FrameId frame;
    // Joints the solver may move; the others keep their seed value.
    JointMask joints;
    double positionTolerance = 1e-4;
    double orientationTolerance = 1e-3;
    int maxIterations = 200;
    // Extra attempts from deterministic pseudo-random seeds when the first one fails.
    int restarts = 8;
};

struct IkSolution {
    JointVector position;
    double positionError{};
    double orientationError{};
    bool converged{};
};

// Not real-time: may allocate.
struct IkSolver {
    virtual ~IkSolver() = default;
    virtual IkSolution solve(IkRequest const &request, JointVector const &seed) = 0;
};

// Two links whose collision geometry overlaps.
struct LinkContact {
    std::string first;
    std::string second;
    // Penetration depth in meters.
    double depth{};
};

// How much deeper than allowed two links may overlap before they collide, in meters.
inline constexpr double kContactTolerance = 1e-3;

// Self-collision queries owned by one thread; not real-time. Every pair of links is checked except
// links adjacent in the kinematic tree and pairs the profile's SRDF disables.
struct CollisionChecker {
    virtual ~CollisionChecker() = default;
    // Contacts present at `position` may stay as deep as they are there, so a motion may leave a
    // resting contact (a folded arm lying on itself) but not press further into it. Until the first
    // call no overlap is allowed.
    virtual void allowContactsAt(JointVector const &position) = 0;
    // A contact more than kContactTolerance deeper than allowed at `position`, if any.
    virtual std::optional<LinkContact> collision(JointVector const &position) = 0;
};

// A parsed robot model that hands out evaluators; each evaluator belongs to one thread.
struct RobotModel {
    virtual ~RobotModel() = default;
    virtual std::size_t dof() const noexcept = 0;
    virtual std::unique_ptr<Kinematics> makeKinematics() const = 0;
    virtual std::unique_ptr<Dynamics> makeDynamics() const = 0;
    virtual std::unique_ptr<IkSolver> makeIkSolver() const = 0;
    virtual std::unique_ptr<CollisionChecker> makeCollisionChecker() const = 0;
};

// Builds the model from the profile's URDF. Every profile joint must exist in the URDF under its
// description name; URDF joints absent from the profile are locked at their neutral position, except
// mimic joints, which follow their leader. Collision meshes must be convex.
Expected<std::unique_ptr<RobotModel>> loadRobotModel(RobotProfile const &profile);

} // namespace larm::model
