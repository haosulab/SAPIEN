#pragma once
#include <PxPhysicsAPI.h>

namespace sapien {

class SPhysicalMaterial {
  physx::PxMaterial *mMaterial;

public:
  SPhysicalMaterial(physx::PxMaterial *material);

  SPhysicalMaterial(SPhysicalMaterial const &other) = delete;
  SPhysicalMaterial &operator=(SPhysicalMaterial const &other) = delete;
  SPhysicalMaterial(SPhysicalMaterial &&other) = default;
  SPhysicalMaterial &operator=(SPhysicalMaterial &&other) = default;

  ~SPhysicalMaterial();
  inline physx::PxMaterial *getPxMaterial() const { return mMaterial; };

  inline physx::PxReal getStaticFriction() const { return mMaterial->getStaticFriction(); }
  inline physx::PxReal getDynamicFriction() const { return mMaterial->getDynamicFriction(); }
  inline physx::PxReal getRestitution() const { return mMaterial->getRestitution(); }

  inline void setStaticFriction(physx::PxReal coef) const { mMaterial->setStaticFriction(coef); }
  inline void setDynamicFriction(physx::PxReal coef) const { mMaterial->setDynamicFriction(coef); }
  inline void setRestitution(physx::PxReal coef) const { mMaterial->setRestitution(coef); }
};

} // namespace sapien
