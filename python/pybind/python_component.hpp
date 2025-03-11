/*
 * Copyright 2025 Hillbot Inc.
 * Copyright 2020-2024 UCSD SU Lab
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once
#include "sapien/component.h"
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
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
