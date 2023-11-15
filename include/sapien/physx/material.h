#pragma once
#include <PxPhysicsAPI.h>
#include <memory>

namespace sapien {
namespace physx {
class PhysxEngine;

class PhysxMaterial : public std::enable_shared_from_this<PhysxMaterial> {
public:
  PhysxMaterial() : PhysxMaterial(0.f, 0.f, 0.f) {}
  PhysxMaterial(float dynamicFriction, float staticFriction, float restitution);

  inline ::physx::PxMaterial *getPxMaterial() const { return mMaterial; };

  inline float getStaticFriction() const { return mMaterial->getStaticFriction(); }
  inline float getDynamicFriction() const { return mMaterial->getDynamicFriction(); }
  inline float getRestitution() const { return mMaterial->getRestitution(); }

  inline void setStaticFriction(float coef) const { mMaterial->setStaticFriction(coef); }
  inline void setDynamicFriction(float coef) const { mMaterial->setDynamicFriction(coef); }
  inline void setRestitution(float coef) const { mMaterial->setRestitution(coef); }

  PhysxMaterial(PhysxMaterial const &other) = delete;
  PhysxMaterial &operator=(PhysxMaterial const &other) = delete;
  PhysxMaterial(PhysxMaterial &&other) = default;
  PhysxMaterial &operator=(PhysxMaterial &&other) = default;

  ~PhysxMaterial();

  template <class Archive> void save(Archive &ar) const {
    ar(getStaticFriction(), getDynamicFriction(), getRestitution());
  }

  template <class Archive> void load(Archive &ar) {
    float staticFriction, dynamicFriction, restitution;
    ar(staticFriction, dynamicFriction, restitution);
    setDynamicFriction(dynamicFriction);
    setStaticFriction(staticFriction);
    setRestitution(restitution);
  }

private:
  std::shared_ptr<PhysxEngine> mEngine;
  ::physx::PxMaterial *mMaterial;
};

} // namespace physx
} // namespace sapien
