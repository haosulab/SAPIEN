#pragma once
#include <PxPhysicsAPI.h>
#include <memory>

namespace sapien {

class SPhysicalMaterial : public std::enable_shared_from_this<SPhysicalMaterial> {
  physx::PxMaterial *mMaterial;
  std::shared_ptr<class Simulation const> mSimulation;

public:
  SPhysicalMaterial(std::shared_ptr<class Simulation const> simulation, physx::PxMaterial *material);

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
