#include "sapien/physx/physx_default.h"
#include "sapien/physx/material.h"
#include "sapien/physx/physx_system.h"

namespace sapien {
namespace physx {

static float gStaticFriction{0.3};
static float gDynamicFriction{0.3};
static float gRestitution{0.1};
static std::weak_ptr<PhysxMaterial> gDefaultMaterial;
static bool gGPUEnabled{false};

void PhysxDefault::SetDefaultMaterial(float staticFriction, float dynamicFriction,
                                      float restitution) {
  gDefaultMaterial.reset();
  gStaticFriction = staticFriction;
  gDynamicFriction = dynamicFriction;
  gRestitution = restitution;
}

std::shared_ptr<PhysxMaterial> PhysxDefault::GetDefaultMaterial() {
  auto m = gDefaultMaterial.lock();
  if (m) {
    return m;
  }
  gDefaultMaterial = m =
      std::make_shared<PhysxMaterial>(gStaticFriction, gDynamicFriction, gRestitution);
  return m;
}

void PhysxDefault::EnableGPU() {
  if (PhysxEngine::GetIfExists()) {
    throw std::runtime_error(
        "GPU PhysX can only be enabled once before any other code involving PhysX");
  }
  gGPUEnabled = true;
}

bool PhysxDefault::GetGPUEnabled() { return gGPUEnabled; }

} // namespace physx
} // namespace sapien
