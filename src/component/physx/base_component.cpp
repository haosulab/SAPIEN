#include "sapien/component/physx/base_component.h"
#include "sapien/component/physx/physx_system.h"

namespace sapien::component {

PhysxBaseComponent::PhysxBaseComponent() : mEngine(PhysxEngine::Get()) {}

} // namespace sapien::component
