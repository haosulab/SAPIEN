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
#include "svulkan2/ui/popup.h"
#include <imgui.h>
#include <string>

namespace svulkan2 {
namespace ui {

void Popup::build() {
  if (!ImGui::IsPopupOpen(getLabelId().c_str())) {
    ImGui::OpenPopup(getLabelId().c_str());
  }

  if (ImGui::BeginPopupModal(getLabelId().c_str(), nullptr)) {
    for (auto c : mChildren) {
      c->build();
    }

    if (ImGui::IsKeyDown(ImGuiKey_Escape)) {
      if (mEscCallback) {
        mEscCallback();
      }
    }

    ImGui::EndPopup();
  }
}

} // namespace ui
} // namespace svulkan2