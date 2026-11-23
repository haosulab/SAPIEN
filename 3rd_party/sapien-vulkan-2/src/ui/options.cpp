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
#include "svulkan2/ui/options.h"
#include <imgui.h>
#include <stdexcept>

namespace svulkan2 {
namespace ui {

std::string Options::get() {
  if (mItems.empty()) {
    return "";
  }
  if (mIndex < 0 || static_cast<uint32_t>(mIndex) >= mItems.size()) {
    mIndex = 0;
  }
  return mItems[mIndex];
}

void Options::build() {
  if (mItemsGetter) {
    mItems = mItemsGetter();
  }
  if (mIndexGetter) {
    mIndex = mIndexGetter();
  }

  if (mIndex < 0 || static_cast<uint32_t>(mIndex) >= mItems.size()) {
    mIndex = 0;
  }
  if (mStyle == "radio") {
    for (uint32_t i = 0; i < mItems.size(); ++i) {
      if (ImGui::RadioButton(mItems[i].c_str(), &mIndex, i)) {
        if (mCallback) {
          mCallback(std::static_pointer_cast<Options>(shared_from_this()));
        }
        if (mIndexSetter) {
          mIndexSetter(mIndex);
        }
      }
    }
  } else if (mStyle == "select") {
    if (ImGui::BeginCombo(getLabelId().c_str(), get().c_str())) {
      for (uint32_t i = 0; i < mItems.size(); ++i) {
        ImGui::PushID(i);
        if (ImGui::Selectable(mItems[i].c_str(), mIndex == static_cast<int>(i))) {
          mIndex = i;
          if (mCallback) {
            mCallback(std::static_pointer_cast<Options>(shared_from_this()));
          }
          if (mIndexSetter) {
            mIndexSetter(mIndex);
          }
        }
        ImGui::PopID();
      }
      ImGui::EndCombo();
    }
  } else {
    throw std::runtime_error("\"radio\" or \"select\" style must be specified for UI options.");
  }
}

} // namespace ui
} // namespace svulkan2