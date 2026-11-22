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
#include "svulkan2/common/glm.h"
#include "widget.h"
#include <functional>

namespace svulkan2 {
namespace ui {

UI_CLASS(Gizmo) {
  UI_DECLARE_LABEL(Gizmo);
  UI_ATTRIBUTE(Gizmo, glm::mat4, Matrix);

  UI_BINDING(Gizmo, glm::mat4, Matrix);

  int mCurrentGizmoOperation{7};
  int mCurrentGizmoMode{1};

  bool mUseSnap{};
  glm::vec3 mSnapTranslation{0.1, 0.1, 0.1};
  float mSnapRotation{5};

  glm::mat4 mView{1};
  glm::mat4 mProjection{1};

  void editTransform();

public:
  void build() override;
  inline void setCameraParameters(glm::mat4 const &view, glm::mat4 const &projection) {
    mView = view;
    mProjection = projection;
  }
};

} // namespace ui
} // namespace svulkan2