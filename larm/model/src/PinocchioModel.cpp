#include "DampedLeastSquaresIk.h"
#include "PinocchioCollision.h"

#include <larm/model/RobotModel.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <array>
#include <exception>

namespace larm::model {
namespace {

// Immutable after loading and shared by every evaluator.
struct ModelData {
    pinocchio::Model model;
    std::size_t dof{};
    // Index into Pinocchio's q and v for each profile joint (all are single-dof joints).
    std::array<Eigen::Index, kMaxDof> modelIndex{};
    IkLimits limits;
};

struct PinocchioKinematics final : Kinematics {
    explicit PinocchioKinematics(std::shared_ptr<ModelData const> shared_)
        : shared{std::move(shared_)}, data{shared->model}, q{pinocchio::neutral(shared->model)},
          jacobian{Eigen::MatrixXd::Zero(6, shared->model.nv)} {}

    std::optional<FrameId> findFrame(std::string_view const name) const override {
        auto const frameName = std::string{name};
        if (not shared->model.existFrame(frameName)) {
            return std::nullopt;
        }
        return FrameId{static_cast<std::uint32_t>(shared->model.getFrameId(frameName))};
    }

    void update(JointVector const &position) noexcept override {
        for (std::size_t i = 0; i < shared->dof; ++i) {
            q[shared->modelIndex[i]] = position[idx(i)];
        }
        pinocchio::computeJointJacobians(shared->model, data, q);
        pinocchio::updateFramePlacements(shared->model, data);
    }

    Pose3 framePose(FrameId const frame) const noexcept override {
        auto const &placement = data.oMf[frame.value];
        return Pose3{
            .translation = placement.translation(),
            .rotation = Eigen::Quaterniond{placement.rotation()}.normalized(),
        };
    }

    void frameJacobian(FrameId const frame, Jacobian &out) const noexcept override {
        jacobian.setZero();
        pinocchio::getFrameJacobian(shared->model, data, frame.value, pinocchio::LOCAL_WORLD_ALIGNED,
                                    jacobian);
        out.resize(6, idx(shared->dof));
        for (std::size_t i = 0; i < shared->dof; ++i) {
            out.col(idx(i)) = jacobian.col(shared->modelIndex[i]);
        }
    }

  private:
    std::shared_ptr<ModelData const> shared;
    // Pinocchio caches intermediate results in Data; queries refresh frame data in place.
    mutable pinocchio::Data data;
    Eigen::VectorXd q;
    mutable Eigen::MatrixXd jacobian;
};

struct PinocchioDynamics final : Dynamics {
    explicit PinocchioDynamics(std::shared_ptr<ModelData const> shared_)
        : shared{std::move(shared_)}, data{shared->model}, q{pinocchio::neutral(shared->model)},
          v{Eigen::VectorXd::Zero(shared->model.nv)}, a{Eigen::VectorXd::Zero(shared->model.nv)} {}

    void gravity(JointVector const &position, JointVector &out) noexcept override {
        gather(position, q);
        pinocchio::computeGeneralizedGravity(shared->model, data, q);
        scatter(data.g, out);
    }

    void inverseDynamics(JointVector const &position, JointVector const &velocity,
                         JointVector const &acceleration, JointVector &out) noexcept override {
        gather(position, q);
        gather(velocity, v);
        gather(acceleration, a);
        pinocchio::rnea(shared->model, data, q, v, a);
        scatter(data.tau, out);
    }

  private:
    void gather(JointVector const &from, Eigen::VectorXd &to) const noexcept {
        for (std::size_t i = 0; i < shared->dof; ++i) {
            to[shared->modelIndex[i]] = from[idx(i)];
        }
    }

    void scatter(Eigen::VectorXd const &from, JointVector &to) const noexcept {
        to.resize(idx(shared->dof));
        for (std::size_t i = 0; i < shared->dof; ++i) {
            to[idx(i)] = from[shared->modelIndex[i]];
        }
    }

    std::shared_ptr<ModelData const> shared;
    pinocchio::Data data;
    Eigen::VectorXd q;
    Eigen::VectorXd v;
    Eigen::VectorXd a;
};

struct PinocchioRobotModel final : RobotModel {
    PinocchioRobotModel(std::shared_ptr<ModelData const> shared_,
                        std::shared_ptr<CollisionGeometry const> collisionGeometry_)
        : shared{std::move(shared_)}, collisionGeometry{std::move(collisionGeometry_)} {}

    std::size_t dof() const noexcept override { return shared->dof; }

    std::unique_ptr<Kinematics> makeKinematics() const override {
        return std::make_unique<PinocchioKinematics>(shared);
    }

    std::unique_ptr<Dynamics> makeDynamics() const override {
        return std::make_unique<PinocchioDynamics>(shared);
    }

    std::unique_ptr<IkSolver> makeIkSolver() const override {
        return makeDampedLeastSquaresIk(makeKinematics(), shared->limits);
    }

    std::unique_ptr<CollisionChecker> makeCollisionChecker() const override {
        return makePinocchioCollisionChecker(collisionGeometry);
    }

  private:
    std::shared_ptr<ModelData const> shared;
    std::shared_ptr<CollisionGeometry const> collisionGeometry;
};

Expected<pinocchio::Model> parseUrdf(std::filesystem::path const &urdf) {
    try {
        auto model = pinocchio::Model{};
        pinocchio::urdf::buildModel(urdf.string(), model);
        return model;
    } catch (std::exception const &exception) {
        return makeError(ErrorCode::InvalidConfig,
                         "cannot load URDF " + urdf.string() + ": " + exception.what());
    }
}

Expected<std::shared_ptr<ModelData const>> reduceToProfile(pinocchio::Model const &full,
                                                           RobotProfile const &profile) {
    auto used = std::vector<bool>(static_cast<std::size_t>(full.njoints), false);
    for (auto const &joint : profile.joints) {
        if (not full.existJointName(joint.descriptionJoint)) {
            return makeError(ErrorCode::InvalidConfig,
                             "joint '" + joint.descriptionJoint + "' is not in " + profile.urdf.string());
        }
        auto const id = full.getJointId(joint.descriptionJoint);
        if (full.nqs[id] != 1 or full.nvs[id] != 1) {
            return makeError(ErrorCode::Unsupported,
                             "joint '" + joint.descriptionJoint + "' is not a 1-dof joint");
        }
        used[id] = true;
    }
    auto locked = std::vector<pinocchio::JointIndex>{};
    for (pinocchio::JointIndex id = 1; id < static_cast<pinocchio::JointIndex>(full.njoints); ++id) {
        if (not used[id]) {
            locked.push_back(id);
        }
    }

    auto data = std::make_shared<ModelData>();
    data->model = pinocchio::buildReducedModel(full, locked, pinocchio::neutral(full));
    data->dof = profile.dof();
    data->limits = IkLimits{.lower = zeroJointVector(profile.dof()), .upper = zeroJointVector(profile.dof())};
    for (std::size_t i = 0; i < profile.dof(); ++i) {
        auto const &joint = profile.joints[i];
        data->modelIndex[i] = data->model.idx_qs[data->model.getJointId(joint.descriptionJoint)];
        data->limits.lower[idx(i)] = joint.limits.lower;
        data->limits.upper[idx(i)] = joint.limits.upper;
    }
    if (static_cast<std::size_t>(data->model.nq) != profile.dof()) {
        return makeError(ErrorCode::Internal, "reduced model size does not match the profile");
    }
    return std::shared_ptr<ModelData const>{std::move(data)};
}

} // namespace

Expected<std::unique_ptr<RobotModel>> loadRobotModel(RobotProfile const &profile) {
    auto shared = parseUrdf(profile.urdf).and_then([&](pinocchio::Model const &full) {
        return reduceToProfile(full, profile);
    });
    if (not shared) {
        return tl::make_unexpected(shared.error());
    }
    auto collisionGeometry = loadCollisionGeometry(profile);
    if (not collisionGeometry) {
        return tl::make_unexpected(collisionGeometry.error());
    }
    return std::make_unique<PinocchioRobotModel>(std::move(*shared), std::move(*collisionGeometry));
}

} // namespace larm::model
