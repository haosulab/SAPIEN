#pragma once

#include "../component.h"
#include "sapien/serialize.h"
#include <memory>

namespace sapien {
namespace component {
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

} // namespace component
} // namespace sapien

CEREAL_REGISTER_TYPE(sapien::component::PhysxBaseComponent);

CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::component::Component,
                                     sapien::component::PhysxBaseComponent);
