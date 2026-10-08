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
#include <array>

namespace svulkan2 {
namespace ui {

UI_CLASS(DisplayText) {
  UI_ATTRIBUTE(DisplayText, std::string, Text);

  UI_BINDING_READONLY(DisplayText, std::string, Text);

  void build() override;
};

UI_CLASS(InputText) {
  UI_DECLARE_LABEL(InputText);
  UI_ATTRIBUTE(InputText, float, WidthRatio);
  UI_ATTRIBUTE(InputText, bool, ReadOnly);
  UI_ATTRIBUTE(InputText, std::function<void(std::shared_ptr<InputText>)>, Callback);

protected:
  std::vector<char> mBuffer{std::vector<char>(128, 0)};

public:
  inline std::shared_ptr<InputText> Size(uint32_t size) {
    mBuffer.resize(size);
    return std::static_pointer_cast<InputText>(shared_from_this());
  };

  inline std::shared_ptr<InputText> Value(std::string const &value) {
    value.copy(mBuffer.data(), mBuffer.size() - 1);
    return std::static_pointer_cast<InputText>(shared_from_this());
  };

  inline std::string get() const { return std::string(mBuffer.data()); }
  void build() override;
};

UI_CLASS(InputTextMultiline) {
  UI_DECLARE_LABEL(InputTextMultiline);
  UI_ATTRIBUTE(InputTextMultiline, bool, ReadOnly);
  UI_ATTRIBUTE(InputTextMultiline, std::function<void(std::shared_ptr<InputTextMultiline>)>,
               Callback);

protected:
  std::vector<char> mBuffer{std::vector<char>(128 * 16, 0)};

public:
  inline std::shared_ptr<InputTextMultiline> Size(uint32_t size) {
    mBuffer.resize(size);
    return std::static_pointer_cast<InputTextMultiline>(shared_from_this());
  };

  inline std::shared_ptr<InputTextMultiline> Value(std::string const &value) {
    value.copy(mBuffer.data(), mBuffer.size() - 1);
    return std::static_pointer_cast<InputTextMultiline>(shared_from_this());
  };

  inline std::string get() const { return std::string(mBuffer.data()); }
  void build() override;
};

UI_CLASS(InputFloat) {
  UI_DECLARE_LABEL(InputFloat);
  UI_ATTRIBUTE(InputFloat, float, WidthRatio);
  UI_ATTRIBUTE(InputFloat, float, Value);
  UI_ATTRIBUTE(InputFloat, bool, ReadOnly);
  UI_ATTRIBUTE(InputFloat, std::function<void(std::shared_ptr<InputFloat>)>, Callback);

  UI_BINDING(InputFloat, float, Value);

public:
  inline float get() const { return mValue; }
  void build() override;
};

using ArrayVec2 = std::array<float, 2>;
using ArrayVec3 = std::array<float, 3>;
using ArrayVec4 = std::array<float, 4>;
using ArrayIVec2 = std::array<int, 2>;
using ArrayIVec3 = std::array<int, 3>;
using ArrayIVec4 = std::array<int, 4>;

UI_CLASS(InputFloat2) {
  UI_DECLARE_LABEL(InputFloat2);
  UI_ATTRIBUTE(InputFloat2, float, WidthRatio);
  UI_ATTRIBUTE(InputFloat2, ArrayVec2, Value);
  UI_ATTRIBUTE(InputFloat2, bool, ReadOnly);
  UI_ATTRIBUTE(InputFloat2, std::function<void(std::shared_ptr<InputFloat2>)>, Callback);

  UI_BINDING(InputFloat2, ArrayVec2, Value);

public:
  inline ArrayVec2 get() const { return mValue; }
  void build() override;
};

UI_CLASS(InputFloat3) {
  UI_DECLARE_LABEL(InputFloat3);
  UI_ATTRIBUTE(InputFloat3, float, WidthRatio);
  UI_ATTRIBUTE(InputFloat3, ArrayVec3, Value);
  UI_ATTRIBUTE(InputFloat3, bool, ReadOnly);
  UI_ATTRIBUTE(InputFloat3, std::function<void(std::shared_ptr<InputFloat3>)>, Callback);

  UI_BINDING(InputFloat3, ArrayVec3, Value);

public:
  inline ArrayVec3 get() const { return mValue; }
  void build() override;
};

UI_CLASS(InputFloat4) {
  UI_DECLARE_LABEL(InputFloat4);
  UI_ATTRIBUTE(InputFloat4, float, WidthRatio);
  UI_ATTRIBUTE(InputFloat4, ArrayVec4, Value);
  UI_ATTRIBUTE(InputFloat4, bool, ReadOnly);
  UI_ATTRIBUTE(InputFloat4, std::function<void(std::shared_ptr<InputFloat4>)>, Callback);

  UI_BINDING(InputFloat4, ArrayVec4, Value);

public:
  inline ArrayVec4 get() const { return mValue; }
  void build() override;
};

UI_CLASS(InputInt) {
  UI_DECLARE_LABEL(InputInt);
  UI_ATTRIBUTE(InputInt, float, WidthRatio);
  UI_ATTRIBUTE(InputInt, int, Value);
  UI_ATTRIBUTE(InputInt, bool, ReadOnly);
  UI_ATTRIBUTE(InputInt, std::function<void(std::shared_ptr<InputInt>)>, Callback);

  UI_BINDING(InputInt, int, Value);

public:
  inline int get() const { return mValue; }
  void build() override;
};

UI_CLASS(InputInt2) {
  UI_DECLARE_LABEL(InputInt2);
  UI_ATTRIBUTE(InputInt2, float, WidthRatio);
  UI_ATTRIBUTE(InputInt2, ArrayIVec2, Value);
  UI_ATTRIBUTE(InputInt2, bool, ReadOnly);
  UI_ATTRIBUTE(InputInt2, std::function<void(std::shared_ptr<InputInt2>)>, Callback);

  UI_BINDING(InputInt2, ArrayIVec2, Value);

public:
  inline ArrayIVec2 get() const { return mValue; }
  void build() override;
};

UI_CLASS(InputInt3) {
  UI_DECLARE_LABEL(InputInt3);
  UI_ATTRIBUTE(InputInt3, float, WidthRatio);
  UI_ATTRIBUTE(InputInt3, ArrayIVec3, Value);
  UI_ATTRIBUTE(InputInt3, bool, ReadOnly);
  UI_ATTRIBUTE(InputInt3, std::function<void(std::shared_ptr<InputInt3>)>, Callback);

  UI_BINDING(InputInt3, ArrayIVec3, Value);

public:
  inline ArrayIVec3 get() const { return mValue; }
  void build() override;
};

UI_CLASS(InputInt4) {
  UI_DECLARE_LABEL(InputInt4);
  UI_ATTRIBUTE(InputInt4, float, WidthRatio);
  UI_ATTRIBUTE(InputInt4, ArrayIVec4, Value);
  UI_ATTRIBUTE(InputInt4, bool, ReadOnly);
  UI_ATTRIBUTE(InputInt4, std::function<void(std::shared_ptr<InputInt4>)>, Callback);

  UI_BINDING(InputInt4, ArrayIVec4, Value);

public:
  inline ArrayIVec4 get() const { return mValue; }
  void build() override;
};

} // namespace ui
} // namespace svulkan2