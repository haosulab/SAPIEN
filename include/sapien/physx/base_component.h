#pragma once

#include "../component.h"
#include "sapien/serialize.h"
#include <memory>

namespace sapien {
namespace physx {
class PhysxEngine;

class PhysxBaseComponent : public Component {
public:
  PhysxBaseComponent();

  template <class Archive> void save(Archive &ar) const {
    ar(cereal::base_class<Component>(this));
  }
  template <class Archive> void load(Archive &ar) { ar(cereal::base_class<Component>(this)); }

protected:
  std::shared_ptr<PhysxEngine> mEngine;
};

} // namespace physx
} // namespace sapien

CEREAL_REGISTER_TYPE(sapien::physx::PhysxBaseComponent);

CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::Component, sapien::physx::PhysxBaseComponent);
