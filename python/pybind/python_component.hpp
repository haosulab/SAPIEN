#pragma once
#include "sapien/component/component.h"
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/smart_holder.h>
#include <pybind11/stl.h>

#include <cereal/archives/binary.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>

namespace py = pybind11;
using namespace pybind11::literals;
using namespace sapien;

class PythonComponent;
extern cereal::construct<PythonComponent> *gCurrentConstruct;

class PythonComponent : public component::Component, public py::trampoline_self_life_support {

public:
  PythonComponent() : mObjectId(gCurrentObjectId++) {}

  void onAddToScene(Scene &scene) override {
    PYBIND11_OVERRIDE_NAME(void, PythonComponent, "on_add_to_scene", onAddToSceneImpl, scene);
  }
  void onAddToSceneImpl(Scene &scene) {}

  void onRemoveFromScene(Scene &scene) override {
    PYBIND11_OVERRIDE_NAME(void, PythonComponent, "on_remove_from_scene", onRemoveFromSceneImpl,
                           scene);
  }
  void onRemoveFromSceneImpl(Scene &scene) {}

  void onSetPose(Pose const &pose) override {
    PYBIND11_OVERRIDE_NAME(void, component::Component, "on_set_pose", onSetPose, pose);
  }

  void onAttach() override {
    PYBIND11_OVERRIDE_NAME(void, component::Component, "on_attach", onAttach, );
  }

  void onDetach() override {
    PYBIND11_OVERRIDE_NAME(void, component::Component, "on_detach", onDetach, );
  }

  template <class Archive> void save(Archive &ar) const {
    ar(mObjectId);
    ar(cereal::base_class<component::Component>(this));
  }

  template <class Archive> void load(Archive &ar) {
    ar(mObjectId);
    ar(cereal::base_class<component::Component>(this));
    mPlaceholder = true;
  }

  uint64_t getSerializationId() const { return mObjectId; }
  void setSerializationId(uint64_t id) { mObjectId = id; }

private:
  // TODO: make thread safe?
  static uint64_t gCurrentObjectId;
  uint64_t mObjectId{0};

  // indicate this component is a placeholder and should be replaced with an actual python
  // component loaded by python serialization
  bool mPlaceholder{false};
};

uint64_t PythonComponent::gCurrentObjectId = 0;

CEREAL_REGISTER_TYPE(PythonComponent);
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::component::Component, PythonComponent);
