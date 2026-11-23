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
#include "svulkan2/ui/gizmo.h"

// clang-format off
#include <imgui.h>
#include <ImGuizmo.h>
// clang-format on

namespace svulkan2 {
namespace ui {

void Gizmo::editTransform() {
  if (mMatrixGetter) {
    mMatrix = mMatrixGetter();
  }
  if (ImGui::RadioButton("Translate##gizmoxx", mCurrentGizmoOperation == ImGuizmo::TRANSLATE))
    mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
  ImGui::SameLine();
  if (ImGui::RadioButton("Rotate##gizmoxx", mCurrentGizmoOperation == ImGuizmo::ROTATE))
    mCurrentGizmoOperation = ImGuizmo::ROTATE;

  float matrixTranslation[3], matrixRotation[3], matrixScale[3];
  ImGuizmo::DecomposeMatrixToComponents(&mMatrix[0][0], matrixTranslation, matrixRotation,
                                        matrixScale);

  bool edited = false;
  if (ImGui::InputFloat3("Position##gizmo", matrixTranslation, "%3f",
                         ImGuiInputTextFlags_EnterReturnsTrue)) {
    edited = true;
  }
  if (ImGui::InputFloat3("Rotation##gizmo", matrixRotation, "%3f",
                         ImGuiInputTextFlags_EnterReturnsTrue)) {
    edited = true;
  }
  // ImGui::InputFloat3("Sc##gizmo", matrixScale);
  ImGuizmo::RecomposeMatrixFromComponents(matrixTranslation, matrixRotation, matrixScale,
                                          &mMatrix[0][0]);
  if (edited && mMatrixSetter) {
    mMatrixSetter(mMatrix);
  }

  if (mCurrentGizmoOperation != ImGuizmo::SCALE) {
    if (ImGui::RadioButton("Local##gizmo", mCurrentGizmoMode == ImGuizmo::LOCAL))
      mCurrentGizmoMode = ImGuizmo::LOCAL;
    ImGui::SameLine();
    if (ImGui::RadioButton("World##gizmo", mCurrentGizmoMode == ImGuizmo::WORLD))
      mCurrentGizmoMode = ImGuizmo::WORLD;
  }

  ImGui::Checkbox("Snap##gizmocheckbox", &mUseSnap);

  float *snap = nullptr;
  if (mUseSnap) {
    switch (mCurrentGizmoOperation) {
    case ImGuizmo::TRANSLATE:
      snap = &mSnapTranslation.x;
      ImGui::InputFloat3("##snap", &mSnapTranslation.x);
      break;
    case ImGuizmo::ROTATE:
      snap = &mSnapRotation;
      ImGui::InputFloat("Degree##angle_snap", &mSnapRotation);
      break;
    }
  }

  ImGuiIO &io = ImGui::GetIO();
  ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

  if (ImGuizmo::Manipulate(&mView[0][0], &mProjection[0][0],
                           static_cast<ImGuizmo::OPERATION>(mCurrentGizmoOperation),
                           static_cast<ImGuizmo::MODE>(mCurrentGizmoMode), &mMatrix[0][0], nullptr,
                           mUseSnap ? snap : nullptr)) {
    if (mMatrixSetter) {
      mMatrixSetter(mMatrix);
    }
  }
}

void Gizmo::build() { editTransform(); }

} // namespace ui
} // namespace svulkan2