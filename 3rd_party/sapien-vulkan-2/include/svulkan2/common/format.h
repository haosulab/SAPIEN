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

#include <glm/glm.hpp>
#include <vulkan/vulkan.hpp>

namespace svulkan2 {

enum class ComponentFormat { eSnorm, eUnorm, eSint, eUint, eSfloat, eOther };

struct VulkanFormatInfo {
  uint32_t size;
  uint32_t channels;
  uint32_t elemSize;
  ComponentFormat format;
  bool srgb;
  vk::ImageAspectFlags aspect;
};

uint32_t getFormatSize(vk::Format format);
uint32_t getFormatChannels(vk::Format format);
uint32_t getFormatElementSize(vk::Format format);
ComponentFormat getFormatComponentFormat(vk::Format format);
bool getFormatSupportSrgb(vk::Format format);
vk::ImageAspectFlags getFormatAspectFlags(vk::Format format);
template <typename T> bool isFormatCompatible(vk::Format format);

struct TypeKind {
  static constexpr char eInt = 'i';
  static constexpr char eUint = 'u';
  static constexpr char eFloat = 'f';
  static constexpr char eRaw = 'V';
  static constexpr char eStruct = 'O';
};

struct DataType {
  uint32_t shape;
  char kind;
  uint32_t bytes;

  bool operator==(DataType const &other) const = default;

  static constexpr const DataType UINT() { return {1, TypeKind::eUint, 4}; }
  static constexpr const DataType UINT2() { return {2, TypeKind::eUint, 4}; }
  static constexpr const DataType UINT3() { return {3, TypeKind::eUint, 4}; }
  static constexpr const DataType UINT4() { return {4, TypeKind::eUint, 4}; }

  static constexpr const DataType INT() { return {1, TypeKind::eInt, 4}; }
  static constexpr const DataType INT2() { return {2, TypeKind::eInt, 4}; }
  static constexpr const DataType INT3() { return {3, TypeKind::eInt, 4}; }
  static constexpr const DataType INT4() { return {4, TypeKind::eInt, 4}; }

  static constexpr const DataType FLOAT() { return {1, TypeKind::eFloat, 4}; }
  static constexpr const DataType FLOAT2() { return {2, TypeKind::eFloat, 4}; }
  static constexpr const DataType FLOAT3() { return {3, TypeKind::eFloat, 4}; }
  static constexpr const DataType FLOAT4() { return {4, TypeKind::eFloat, 4}; }

  static constexpr const DataType INT44() { return {16, TypeKind::eInt, 4}; }
  static constexpr const DataType FLOAT44() { return {16, TypeKind::eFloat, 4}; }

  static constexpr const DataType STRUCT() { return {0, TypeKind::eStruct, 0}; }

  uint32_t size() const { return shape * bytes; }
  std::string typestr() const {
    return std::to_string(shape) + std::to_string(kind) + std::to_string(bytes);
  }
};

template <typename T> struct DataTypeFor {};

template <> struct DataTypeFor<int> {
  static constexpr DataType value = DataType::INT();
};
template <> struct DataTypeFor<glm::ivec4> {
  static constexpr DataType value = DataType::INT4();
};

template <> struct DataTypeFor<uint32_t> {
  static constexpr DataType value = DataType::UINT();
};

template <> struct DataTypeFor<glm::uvec4> {
  static constexpr DataType value = DataType::UINT4();
};

template <> struct DataTypeFor<float> {
  static constexpr DataType value = DataType::FLOAT();
};

template <> struct DataTypeFor<glm::vec4> {
  static constexpr DataType value = DataType::FLOAT4();
};

template <> struct DataTypeFor<glm::vec3> {
  static constexpr DataType value = DataType::FLOAT3();
};

template <> struct DataTypeFor<glm::mat4> {
  static constexpr DataType value = DataType::FLOAT44();
};

} // namespace svulkan2