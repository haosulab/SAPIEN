#pragma once
#include "sapien/math/math.h"
namespace sapien {
namespace physx {

class PhysxCollisionShape;
class PhysxRigidBaseComponent;

struct PhysxHitInfo {
  Vec3 position;
  Vec3 normal;
  float distance;
  PhysxCollisionShape *shape;
  PhysxRigidBaseComponent *component;
};

} // namespace physx
} // namespace sapien
