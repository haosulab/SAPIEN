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

#include "./vk.h"

#include <algorithm>
#include <glm/glm.hpp>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace svulkan2 {

struct SpecializationConstantLayout {
  struct Element {
    std::string name{};
    uint32_t id{0};
    DataType dtype{};
    std::byte buffer[128]{};
  };
  std::unordered_map<std::string, SpecializationConstantLayout::Element> elements;
  std::vector<SpecializationConstantLayout::Element> getElementsSorted() const;

  uint32_t size() const {
    uint32_t s = 0;
    for (auto &[n, e] : elements) {
      s += e.dtype.size();
    }
    return s;
  }
};

struct CombinedSamplerLayout {
  struct Element {
    std::string name{};
    uint32_t binding{0};
    uint32_t set{0};
  };
  std::unordered_map<std::string, CombinedSamplerLayout::Element> elements;
  std::vector<CombinedSamplerLayout::Element> getElementsSorted() const;
};

struct InputDataLayout {
  struct Element {
    std::string name{};
    uint32_t location{0};
    DataType dtype{DataType::FLOAT()};
    inline uint32_t getSize() const { return dtype.size(); }
    inline bool operator==(Element const &other) const {
      return name == other.name && location == other.location && dtype == other.dtype;
    }
  };

  std::unordered_map<std::string, InputDataLayout::Element> elements;

  std::vector<InputDataLayout::Element> getElementsSorted() const;
  uint32_t getSize() const;

  std::vector<vk::VertexInputBindingDescription> computeVertexInputBindingDescriptions() const;
  std::vector<vk::VertexInputAttributeDescription>
  computeVertexInputAttributesDescriptions() const;

  bool operator==(InputDataLayout const &other) const;
  bool operator!=(InputDataLayout const &other) const;
};

struct OutputDataLayout {
  struct Element {
    std::string name{};
    uint32_t location{0};
    DataType dtype{DataType::FLOAT()};
  };

  std::unordered_map<std::string, OutputDataLayout::Element> elements;

  std::vector<OutputDataLayout::Element> getElementsSorted() const;
};

struct StructDataLayout {
  struct Element {
    std::string name{};
    uint32_t size{0};
    uint32_t offset{0};
    std::vector<uint32_t> array{};
    DataType dtype{DataType::FLOAT()};
    std::shared_ptr<StructDataLayout> member;
    bool operator==(Element const &other) const;
  };

  uint32_t size;
  std::unordered_map<std::string, StructDataLayout::Element> elements;
  std::vector<StructDataLayout::Element const *> getElementsSorted() const;

  bool operator==(StructDataLayout const &other) const;
  bool operator!=(StructDataLayout const &other) const;
  uint32_t getAlignedSize(uint32_t alignment) const;
};

inline void strided_memcpy(void *target, void *source, size_t chunk_size, size_t chunks,
                           size_t stride) {
  char *target_ = reinterpret_cast<char *>(target);
  char *source_ = reinterpret_cast<char *>(source);

  for (size_t i = 0; i < chunks; ++i) {
    std::memcpy(target_, source_, chunk_size);
    target_ += stride;
    source_ += chunk_size;
  }
}

struct SpecializationConstantValue {
  DataType dtype;
  std::byte buffer[128]{};

  bool operator==(SpecializationConstantValue const &other) const {
    if (dtype != other.dtype) {
      return false;
    }
    return std::memcmp(other.buffer, buffer, dtype.size()) == 0;
  }

  template <typename T> SpecializationConstantValue &operator=(T const &value) {
    dtype = DataTypeFor<T>::value;
    std::memcpy(buffer, &value, sizeof(T));
    return *this;
  }
};

}; // namespace svulkan2