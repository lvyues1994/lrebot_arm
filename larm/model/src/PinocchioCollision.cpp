#include "PinocchioCollision.h"

#include <coal/BVH/BVH_model.h>
#include <coal/distance.h>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/collision/coal-pinocchio-conversions.hpp>
#include <pinocchio/geometry.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <array>
#include <exception>

namespace larm::model {

struct CollisionGeometry {
    // The whole URDF; mimic joints follow their leader and have no configuration of their own.
    pinocchio::Model model;
    // Convex shapes; collisionPairs lists the pairs that are checked.
    pinocchio::GeometryModel geometry;
    // Link names of each checked pair.
    std::vector<std::pair<std::string, std::string>> pairLinks;
    std::size_t dof{};
    // Index into the model's configuration for each profile joint.
    std::array<Eigen::Index, kMaxDof> modelIndex{};
};

namespace {

// Hull meshes come from single-precision STL files, so their faces are planar only to about this.
// generate_description.py checks its hulls with the same values.
constexpr double kConvexityTolerance = 1e-5;
// Slivers have no reliable plane.
constexpr double kMinNormalLength = 1e-12;

bool isConvex(coal::BVHModelBase const &mesh) {
    auto const &vertices = *mesh.vertices;
    for (auto const &triangle : *mesh.tri_indices) {
        auto const &corner = vertices[triangle[0]];
        coal::Vec3s const normal = (vertices[triangle[1]] - corner).cross(vertices[triangle[2]] - corner);
        auto const area = normal.norm();
        if (area <= kMinNormalLength) {
            continue;
        }
        auto above = false;
        auto below = false;
        for (auto const &vertex : vertices) {
            auto const height = normal.dot(vertex - corner) / area;
            above = above or height > kConvexityTolerance;
            below = below or height < -kConvexityTolerance;
        }
        if (above and below) {
            return false;
        }
    }
    return true;
}

// The body frame of the link a body frame hangs from; 0 for the root link.
pinocchio::FrameIndex parentLink(pinocchio::Model const &model, pinocchio::FrameIndex frame) {
    do {
        frame = model.frames[frame].parentFrame;
    } while (frame != 0 and model.frames[frame].type != pinocchio::BODY);
    return frame;
}

std::string const &linkOf(CollisionGeometry const &shared, pinocchio::GeomIndex const object) {
    return shared.model.frames[shared.geometry.geometryObjects[object].parentFrame].name;
}

Expected<void> makeConvex(pinocchio::Model const &model, pinocchio::GeometryObject &object) {
    if (auto const mesh = std::dynamic_pointer_cast<coal::BVHModelBase>(object.geometry)) {
        if (not isConvex(*mesh)) {
            return makeError(ErrorCode::InvalidConfig, "the collision mesh of link '" +
                                                           model.frames[object.parentFrame].name +
                                                           "' is not convex");
        }
        mesh->buildConvexRepresentation(false);
        object.geometry = mesh->convex;
    }
    object.geometry->computeLocalAABB();
    return {};
}

void removeAdjacentPairs(pinocchio::Model const &model, pinocchio::GeometryModel &geometry) {
    for (auto index = geometry.collisionPairs.size(); index-- > 0;) {
        auto const pair = geometry.collisionPairs[index];
        auto const first = geometry.geometryObjects[pair.first].parentFrame;
        auto const second = geometry.geometryObjects[pair.second].parentFrame;
        if (parentLink(model, first) == second or parentLink(model, second) == first) {
            geometry.removeCollisionPair(pair);
        }
    }
}

// An axis-aligned box in the world frame.
struct WorldBox {
    Eigen::Vector3d center;
    Eigen::Vector3d half;
};

struct PinocchioCollisionChecker final : CollisionChecker {
    explicit PinocchioCollisionChecker(std::shared_ptr<CollisionGeometry const> shared_)
        : shared{std::move(shared_)}, data{shared->model}, placements{shared->geometry},
          q{pinocchio::neutral(shared->model)}, boxes(shared->geometry.geometryObjects.size()),
          allowed(shared->geometry.collisionPairs.size(), 0.0) {}

    void allowContactsAt(JointVector const &position) override {
        place(position);
        for (std::size_t pair = 0; pair < allowed.size(); ++pair) {
            allowed[pair] = std::max(0.0, overlap(pair));
        }
    }

    std::optional<LinkContact> collision(JointVector const &position) override {
        place(position);
        for (std::size_t pair = 0; pair < allowed.size(); ++pair) {
            auto const depth = overlap(pair);
            if (depth > allowed[pair] + kContactTolerance) {
                auto const &[first, second] = shared->pairLinks[pair];
                return LinkContact{.first = first, .second = second, .depth = depth};
            }
        }
        return std::nullopt;
    }

  private:
    void place(JointVector const &position) {
        for (std::size_t i = 0; i < shared->dof; ++i) {
            q[shared->modelIndex[i]] = position[idx(i)];
        }
        pinocchio::updateGeometryPlacements(shared->model, data, shared->geometry, placements, q);
        for (std::size_t object = 0; object < boxes.size(); ++object) {
            auto const &local = shared->geometry.geometryObjects[object].geometry->aabb_local;
            auto const &placement = placements.oMg[object];
            boxes[object] =
                WorldBox{.center = placement.act(local.center()),
                         .half = placement.rotation().cwiseAbs() * (0.5 * (local.max_ - local.min_))};
        }
    }

    // Penetration depth of a pair; negative while the shapes are apart.
    double overlap(std::size_t const pair) const {
        auto const &objects = shared->geometry.collisionPairs[pair];
        auto const &firstBox = boxes[objects.first];
        auto const &secondBox = boxes[objects.second];
        // Boxes apart along an axis: the shapes are at least that far apart.
        auto const gap =
            ((firstBox.center - secondBox.center).cwiseAbs() - firstBox.half - secondBox.half).maxCoeff();
        if (gap > 0.0) {
            return -gap;
        }
        auto result = coal::DistanceResult{};
        return -coal::distance(shared->geometry.geometryObjects[objects.first].geometry.get(),
                               pinocchio::toCoalTransform3s(placements.oMg[objects.first]),
                               shared->geometry.geometryObjects[objects.second].geometry.get(),
                               pinocchio::toCoalTransform3s(placements.oMg[objects.second]),
                               coal::DistanceRequest{}, result);
    }

    std::shared_ptr<CollisionGeometry const> shared;
    pinocchio::Data data;
    pinocchio::GeometryData placements;
    Eigen::VectorXd q;
    // World boxes of the geometry objects at the last placed configuration.
    std::vector<WorldBox> boxes;
    std::vector<double> allowed;
};

} // namespace

Expected<std::shared_ptr<CollisionGeometry const>> loadCollisionGeometry(RobotProfile const &profile) {
    auto shared = std::make_shared<CollisionGeometry>();
    auto &model = shared->model;
    auto &geometry = shared->geometry;
    try {
        pinocchio::urdf::buildModel(profile.urdf.string(), model, false, true);
        pinocchio::urdf::buildGeom(model, profile.urdf.string(), pinocchio::COLLISION, geometry,
                                   std::vector<std::string>{profile.urdf.parent_path().string()});
    } catch (std::exception const &exception) {
        return makeError(ErrorCode::InvalidConfig, "cannot load the collision geometry of " +
                                                       profile.urdf.string() + ": " + exception.what());
    }
    for (auto &object : geometry.geometryObjects) {
        if (auto made = makeConvex(model, object); not made) {
            return tl::make_unexpected(made.error());
        }
    }

    geometry.addAllCollisionPairs();
    removeAdjacentPairs(model, geometry);
    if (not profile.srdf.empty()) {
        try {
            pinocchio::srdf::removeCollisionPairs(model, geometry, profile.srdf.string());
        } catch (std::exception const &exception) {
            return makeError(ErrorCode::InvalidConfig,
                             "cannot read " + profile.srdf.string() + ": " + exception.what());
        }
    }
    for (auto const &pair : geometry.collisionPairs) {
        shared->pairLinks.emplace_back(linkOf(*shared, pair.first), linkOf(*shared, pair.second));
    }

    shared->dof = profile.dof();
    for (std::size_t i = 0; i < profile.dof(); ++i) {
        auto const &name = profile.joints[i].descriptionJoint;
        if (not model.existJointName(name) or model.nqs[model.getJointId(name)] != 1) {
            return makeError(ErrorCode::InvalidConfig,
                             "joint '" + name + "' is not a 1-dof joint of " + profile.urdf.string());
        }
        shared->modelIndex[i] = model.idx_qs[model.getJointId(name)];
    }
    return std::shared_ptr<CollisionGeometry const>{std::move(shared)};
}

std::unique_ptr<CollisionChecker>
makePinocchioCollisionChecker(std::shared_ptr<CollisionGeometry const> geometry) {
    return std::make_unique<PinocchioCollisionChecker>(std::move(geometry));
}

} // namespace larm::model
