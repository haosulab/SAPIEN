#pragma once
#include "sapien/math/math.h"
namespace sapien {
namespace component {

class PhysxCollisionShape;
class PhysxRigidBaseComponent;

struct PhysxHitInfo {
  Vec3 position;
  Vec3 normal;
  float distance;
  PhysxCollisionShape *shape;
  PhysxRigidBaseComponent *component;
};

} // namespace component
} // namespace sapien
