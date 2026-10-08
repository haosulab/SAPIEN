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
#include "svulkan2/ui/checkbox.h"
#include <imgui.h>

namespace svulkan2 {
namespace ui {

void Checkbox::build() {
  if (mCheckedGetter) {
    mChecked = mCheckedGetter();
  }
  if (ImGui::Checkbox(getLabelId().c_str(), &mChecked)) {
    if (mCheckedSetter) {
      mCheckedSetter(mChecked);
    }
    if (mCallback) {
      mCallback(std::static_pointer_cast<Checkbox>(shared_from_this()));
    }
  }
}

} // namespace ui
} // namespace svulkan2