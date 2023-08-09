#include "sapien/component/physx/material.h"
#include "sapien/component/physx/physx_system.h"

namespace sapien::component {

PhysxMaterial::PhysxMaterial(float staticFriction, float dynamicFriction, float restitution)
    : mEngine(PhysxEngine::Get()) {
  mMaterial =
      mEngine->getPxPhysics()->createMaterial(staticFriction, dynamicFriction, restitution);
  mMaterial->setFlag(PxMaterialFlag::eIMPROVED_PATCH_FRICTION, true);
}

PhysxMaterial::~PhysxMaterial() { mMaterial->release(); }

} // namespace sapien::component
