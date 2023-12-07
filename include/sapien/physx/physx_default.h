#pragma once
#include <cstdint>
#include <memory>

namespace sapien {
namespace physx {
class PhysxMaterial;

class PhysxDefault {
public:
  static std::shared_ptr<PhysxMaterial> GetDefaultMaterial();
  static void SetDefaultMaterial(float staticFriction, float dynamicFriction, float restitution);

  // enable GPU simulation, may not be disabled
  static void EnableGPU();
  static bool GetGPUEnabled();
};

} // namespace physx
} // namespace sapien
