#include "sapien/physx/physx_default.h"
#include "sapien/physx/material.h"

namespace sapien {
namespace physx {

PhysxDefault &PhysxDefault::Get() {
  static PhysxDefault gPhysxConfig;
  return gPhysxConfig;
}

std::shared_ptr<PhysxMaterial> PhysxDefault::getDefaultMaterial() {
  return Get()._getDefaultMaterial();
}

void PhysxDefault::setDefaultMaterial(float staticFriction, float dynamicFriction,
                                      float restitution) {
  Get()._setDefaultMaterial(staticFriction, dynamicFriction, restitution);
}

std::shared_ptr<PhysxMaterial> PhysxDefault::_getDefaultMaterial() {
  auto m = mDefaultMaterial.lock();
  if (m) {
    return m;
  }
  mDefaultMaterial = m =
      std::make_shared<PhysxMaterial>(mStaticFriction, mDynamicFriction, mRestitution);
  return m;
}

void PhysxDefault::_setDefaultMaterial(float staticFriction, float dynamicFriction,
                                       float restitution) {
  mDefaultMaterial.reset();
  mStaticFriction = staticFriction;
  mDynamicFriction = dynamicFriction;
  mRestitution = restitution;
}

} // namespace physx
} // namespace sapien
