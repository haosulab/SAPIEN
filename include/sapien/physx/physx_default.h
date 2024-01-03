#pragma once
#include <cstdint>
#include <memory>

namespace physx {
struct PxgDynamicsMemoryConfig;
};

namespace sapien {
namespace physx {
class PhysxMaterial;

class PhysxDefault {
public:
  static std::shared_ptr<PhysxMaterial> GetDefaultMaterial();
  static void SetDefaultMaterial(float staticFriction, float dynamicFriction, float restitution);
  static void setGpuMemoryConfig(uint32_t tempBufferCapacity, uint32_t maxRigidContactCount,
                                 uint32_t maxRigidPatchCount, uint32_t heapCapacity,
                                 uint32_t foundLostPairsCapacity,
                                 uint32_t foundLostAggregatePairsCapacity,
                                 uint32_t totalAggregatePairsCapacity);
  static ::physx::PxgDynamicsMemoryConfig const &getGpuMemoryConfig();

  // enable GPU simulation, may not be disabled
  static void EnableGPU();
  static bool GetGPUEnabled();
};

} // namespace physx
} // namespace sapien
