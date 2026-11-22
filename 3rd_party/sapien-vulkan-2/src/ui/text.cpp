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
#include "svulkan2/ui/text.h"
#include <imgui.h>

namespace svulkan2 {
namespace ui {

void DisplayText::build() {
  if (mTextGetter) {
    mText = mTextGetter();
  }
  ImGui::Text("%s", mText.c_str());
}

void InputText::build() {
  if (mWidthRatio > 0) {
    ImGui::SetNextItemWidth(mWidthRatio * ImGui::GetWindowContentRegionWidth());
  }
  ImGuiInputTextFlags flags = mReadOnly ? ImGuiInputTextFlags_ReadOnly : 0;
  flags |= ImGuiInputTextFlags_EnterReturnsTrue;
  if (ImGui::InputText(getLabelId().c_str(), mBuffer.data(), mBuffer.size(), flags)) {
    if (mCallback) {
      mCallback(std::static_pointer_cast<InputText>(shared_from_this()));
    }
  }
}

void InputTextMultiline::build() {
  ImGuiInputTextFlags flags = mReadOnly ? ImGuiInputTextFlags_ReadOnly : 0;
  flags |= ImGuiInputTextFlags_EnterReturnsTrue;

  if (ImGui::InputTextMultiline(getLabelId().c_str(), mBuffer.data(), mBuffer.size(),
                                ImVec2(-FLT_MIN, -FLT_MIN), flags)) {
    if (mCallback) {
      mCallback(std::static_pointer_cast<InputTextMultiline>(shared_from_this()));
    }
  }
}

void InputFloat::build() {
  if (mWidthRatio > 0) {
    ImGui::SetNextItemWidth(mWidthRatio * ImGui::GetWindowContentRegionWidth());
  }
  ImGuiInputTextFlags flags = mReadOnly ? ImGuiInputTextFlags_ReadOnly : 0;
  flags |= ImGuiInputTextFlags_EnterReturnsTrue;
  if (mValueGetter) {
    mValue = mValueGetter();
  }
  if (ImGui::InputFloat(getLabelId().c_str(), &mValue, 0.f, 0.f, "%.3f", flags)) {
    if (mValueSetter) {
      mValueSetter(mValue);
    }
    if (mCallback) {
      mCallback(std::static_pointer_cast<InputFloat>(shared_from_this()));
    }
  }
}
void InputFloat2::build() {
  if (mWidthRatio > 0) {
    ImGui::SetNextItemWidth(mWidthRatio * ImGui::GetWindowContentRegionWidth());
  }
  ImGuiInputTextFlags flags = mReadOnly ? ImGuiInputTextFlags_ReadOnly : 0;
  flags |= ImGuiInputTextFlags_EnterReturnsTrue;
  if (mValueGetter) {
    mValue = mValueGetter();
  }
  if (ImGui::InputFloat2(getLabelId().c_str(), &mValue[0], "%.3f", flags)) {
    if (mValueSetter) {
      mValueSetter(mValue);
    }
    if (mCallback) {
      mCallback(std::static_pointer_cast<InputFloat2>(shared_from_this()));
    }
  }
}
void InputFloat3::build() {
  if (mWidthRatio > 0) {
    ImGui::SetNextItemWidth(mWidthRatio * ImGui::GetWindowContentRegionWidth());
  }
  ImGuiInputTextFlags flags = mReadOnly ? ImGuiInputTextFlags_ReadOnly : 0;
  flags |= ImGuiInputTextFlags_EnterReturnsTrue;
  if (mValueGetter) {
    mValue = mValueGetter();
  }
  if (ImGui::InputFloat3(getLabelId().c_str(), &mValue[0], "%.3f", flags)) {
    if (mValueSetter) {
      mValueSetter(mValue);
    }
    if (mCallback) {
      mCallback(std::static_pointer_cast<InputFloat3>(shared_from_this()));
    }
  }
}
void InputFloat4::build() {
  if (mWidthRatio > 0) {
    ImGui::SetNextItemWidth(mWidthRatio * ImGui::GetWindowContentRegionWidth());
  }
  ImGuiInputTextFlags flags = mReadOnly ? ImGuiInputTextFlags_ReadOnly : 0;
  flags |= ImGuiInputTextFlags_EnterReturnsTrue;
  if (mValueGetter) {
    mValue = mValueGetter();
  }
  if (ImGui::InputFloat4(getLabelId().c_str(), &mValue[0], "%.3f", flags)) {
    if (mValueSetter) {
      mValueSetter(mValue);
    }
    if (mCallback) {
      mCallback(std::static_pointer_cast<InputFloat4>(shared_from_this()));
    }
  }
}

void InputInt::build() {
  if (mWidthRatio > 0) {
    ImGui::SetNextItemWidth(mWidthRatio * ImGui::GetWindowContentRegionWidth());
  }
  ImGuiInputTextFlags flags = mReadOnly ? ImGuiInputTextFlags_ReadOnly : 0;
  flags |= ImGuiInputTextFlags_EnterReturnsTrue;
  if (mValueGetter) {
    mValue = mValueGetter();
  }
  if (ImGui::InputInt(getLabelId().c_str(), &mValue, 1, 100, flags)) {
    if (mValueSetter) {
      mValueSetter(mValue);
    }
    if (mCallback) {
      mCallback(std::static_pointer_cast<InputInt>(shared_from_this()));
    }
  }
}
void InputInt2::build() {
  if (mWidthRatio > 0) {
    ImGui::SetNextItemWidth(mWidthRatio * ImGui::GetWindowContentRegionWidth());
  }
  ImGuiInputTextFlags flags = mReadOnly ? ImGuiInputTextFlags_ReadOnly : 0;
  flags |= ImGuiInputTextFlags_EnterReturnsTrue;
  if (mValueGetter) {
    mValue = mValueGetter();
  }
  if (ImGui::InputInt2(getLabelId().c_str(), &mValue[0], flags)) {
    if (mValueSetter) {
      mValueSetter(mValue);
    }
    if (mCallback) {
      mCallback(std::static_pointer_cast<InputInt2>(shared_from_this()));
    }
  }
}
void InputInt3::build() {
  if (mWidthRatio > 0) {
    ImGui::SetNextItemWidth(mWidthRatio * ImGui::GetWindowContentRegionWidth());
  }
  ImGuiInputTextFlags flags = mReadOnly ? ImGuiInputTextFlags_ReadOnly : 0;
  flags |= ImGuiInputTextFlags_EnterReturnsTrue;
  if (mValueGetter) {
    mValue = mValueGetter();
  }
  if (ImGui::InputInt3(getLabelId().c_str(), &mValue[0], flags)) {
    if (mValueSetter) {
      mValueSetter(mValue);
    }
    if (mCallback) {
      mCallback(std::static_pointer_cast<InputInt3>(shared_from_this()));
    }
  }
}
void InputInt4::build() {
  if (mWidthRatio > 0) {
    ImGui::SetNextItemWidth(mWidthRatio * ImGui::GetWindowContentRegionWidth());
  }
  ImGuiInputTextFlags flags = mReadOnly ? ImGuiInputTextFlags_ReadOnly : 0;
  flags |= ImGuiInputTextFlags_EnterReturnsTrue;
  if (mValueGetter) {
    mValue = mValueGetter();
  }
  if (ImGui::InputInt4(getLabelId().c_str(), &mValue[0], flags)) {
    if (mValueSetter) {
      mValueSetter(mValue);
    }
    if (mCallback) {
      mCallback(std::static_pointer_cast<InputInt4>(shared_from_this()));
    }
  }
}

} // namespace ui
} // namespace svulkan2