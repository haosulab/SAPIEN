#pragma once
#include <cstdint>
#include <memory>

namespace sapien {
namespace physx {
class PhysxMaterial;

class PhysxDefault {
public:
  static PhysxDefault &Get();
  static std::shared_ptr<PhysxMaterial> getDefaultMaterial();
  static void setDefaultMaterial(float staticFriction, float dynamicFriction, float restitution);

private:
  std::shared_ptr<PhysxMaterial> _getDefaultMaterial();
  void _setDefaultMaterial(float staticFriction, float dynamicFriction, float restitution);

  float mStaticFriction{0.3};
  float mDynamicFriction{0.3};
  float mRestitution{0.1};
  std::weak_ptr<PhysxMaterial> mDefaultMaterial;
};

} // namespace physx
} // namespace sapien
