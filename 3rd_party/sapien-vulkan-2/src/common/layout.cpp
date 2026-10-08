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
#include "svulkan2/common/layout.h"

namespace svulkan2 {

std::vector<SpecializationConstantLayout::Element>
SpecializationConstantLayout::getElementsSorted() const {
  std::vector<SpecializationConstantLayout::Element> result;
  std::transform(elements.begin(), elements.end(), std::back_inserter(result),
                 [](auto const &kv) { return kv.second; });
  std::sort(
      result.begin(), result.end(),
      [](auto const &elem1, auto const &elem2) { return elem1.id < elem2.id; });
  return result;
}

std::vector<CombinedSamplerLayout::Element>
CombinedSamplerLayout::getElementsSorted() const {
  std::vector<CombinedSamplerLayout::Element> result;
  std::transform(elements.begin(), elements.end(), std::back_inserter(result),
                 [](auto const &kv) { return kv.second; });
  std::sort(result.begin(), result.end(),
            [](auto const &elem1, auto const &elem2) {
              return elem1.binding < elem2.binding;
            });
  return result;
}

std::vector<InputDataLayout::Element>
InputDataLayout::getElementsSorted() const {
  std::vector<InputDataLayout::Element> result;
  std::transform(elements.begin(), elements.end(), std::back_inserter(result),
                 [](auto const &kv) { return kv.second; });
  std::sort(result.begin(), result.end(),
            [](auto const &elem1, auto const &elem2) {
              return elem1.location < elem2.location;
            });
  return result;
}

uint32_t InputDataLayout::getSize() const {
  uint32_t sum = 0;
  for (auto &e : elements) {
    sum += e.second.getSize();
  }
  return sum;
}

std::vector<vk::VertexInputBindingDescription>
InputDataLayout::computeVertexInputBindingDescriptions() const {
  return {vk::VertexInputBindingDescription(0, getSize())};
}

std::vector<vk::VertexInputAttributeDescription>
InputDataLayout::computeVertexInputAttributesDescriptions() const {
  std::vector<vk::VertexInputAttributeDescription>
      vertexInputAttributeDescriptions;
  uint32_t offset = 0;
  uint32_t count = 0;
  auto elements = getElementsSorted();
  for (auto &elem : elements) {
    if (elem.location != count) {
      throw std::runtime_error("vertex layout locations must be "
                               "consecutive integers starting from 0");
    }
    if (elem.dtype == DataType::FLOAT()) {
      vertexInputAttributeDescriptions.push_back(
          {elem.location, 0, vk::Format::eR32Sfloat, offset});
    } else if (elem.dtype == DataType::FLOAT2()) {
      vertexInputAttributeDescriptions.push_back(
          {elem.location, 0, vk::Format::eR32G32Sfloat, offset});
    } else if (elem.dtype == DataType::FLOAT3()) {
      vertexInputAttributeDescriptions.push_back(
          {elem.location, 0, vk::Format::eR32G32B32Sfloat, offset});
    } else if (elem.dtype == DataType::FLOAT4()) {
      vertexInputAttributeDescriptions.push_back(
          {elem.location, 0, vk::Format::eR32G32B32A32Sfloat, offset});
    } else {
      throw std::runtime_error("vertex attributes only supports float, "
                               "float2, float3, float4 formats");
    }

    offset += elem.getSize();
    count += 1;
  }
  return vertexInputAttributeDescriptions;
}

bool InputDataLayout::operator==(InputDataLayout const &other) const {
  return elements == other.elements;
}

bool InputDataLayout::operator!=(InputDataLayout const &other) const {
  return !operator==(other);
}

std::vector<OutputDataLayout::Element>
OutputDataLayout::getElementsSorted() const {
  std::vector<OutputDataLayout::Element> result;
  std::transform(elements.begin(), elements.end(), std::back_inserter(result),
                 [](auto const &kv) { return kv.second; });
  std::sort(result.begin(), result.end(),
            [](auto const &elem1, auto const &elem2) {
              return elem1.location < elem2.location;
            });
  return result;
}

std::vector<StructDataLayout::Element const *>
StructDataLayout::getElementsSorted() const {
  std::vector<StructDataLayout::Element const *> result;
  std::transform(elements.begin(), elements.end(), std::back_inserter(result),
                 [](auto const &kv) { return &kv.second; });
  std::sort(result.begin(), result.end(),
            [](auto const &elem1, auto const &elem2) {
              return elem1->offset < elem2->offset;
            });
  return result;
}

bool StructDataLayout::Element::operator==(
    StructDataLayout::Element const &other) const {
  if (!(name == other.name && size == other.size && offset == other.offset &&
        array == other.array && dtype == other.dtype)) {
    return false;
  }
  if (member == nullptr && other.member == nullptr) {
    return true;
  }
  if (member != nullptr && other.member != nullptr) {
    return *member == *other.member;
  }
  return false;
}

bool StructDataLayout::operator==(StructDataLayout const &other) const {
  return size == other.size && elements == other.elements;
}

bool StructDataLayout::operator!=(StructDataLayout const &other) const {
  return !(*this == other);
}

uint32_t StructDataLayout::getAlignedSize(uint32_t alignment) const {
  uint32_t newSize = size / alignment * alignment;
  if (newSize == size) {
    return size;
  }
  return newSize + alignment;
}

}; // namespace svulkan2