#include "sapien/physx/base_component.h"
#include "sapien/physx/physx_system.h"

namespace sapien::physx {

PhysxBaseComponent::PhysxBaseComponent() : mEngine(PhysxEngine::Get()) {}

} // namespace sapien::physx
