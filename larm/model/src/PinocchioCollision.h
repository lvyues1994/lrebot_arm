#pragma once

#include <larm/model/RobotModel.h>

namespace larm::model {

// The robot's convex collision geometry and the link pairs to check; immutable once loaded.
struct CollisionGeometry;

Expected<std::shared_ptr<CollisionGeometry const>> loadCollisionGeometry(RobotProfile const &profile);

std::unique_ptr<CollisionChecker>
makePinocchioCollisionChecker(std::shared_ptr<CollisionGeometry const> geometry);

} // namespace larm::model
