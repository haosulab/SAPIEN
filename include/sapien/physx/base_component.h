#pragma once

#include "../component.h"
#include <memory>

namespace sapien {
namespace physx {
class PhysxEngine;

class PhysxBaseComponent : public Component {
public:
  PhysxBaseComponent();

protected:
  std::shared_ptr<PhysxEngine> mEngine;
};

} // namespace physx
} // namespace sapien
