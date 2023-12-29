#pragma once
#include "sapien/component.h"
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/smart_holder.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals;
using namespace sapien;

class PythonComponent;

class PythonComponent : public Component, public py::trampoline_self_life_support {

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
    PYBIND11_OVERRIDE_NAME(void, Component, "on_set_pose", onSetPose, pose);
  }

private:
  // TODO: make thread safe?
  static uint64_t gCurrentObjectId;
  uint64_t mObjectId{0};
};

uint64_t PythonComponent::gCurrentObjectId = 0;
