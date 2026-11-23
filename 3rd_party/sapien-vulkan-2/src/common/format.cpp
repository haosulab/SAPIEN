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
#include "svulkan2/common/format.h"
#include <map>

namespace svulkan2 {

// clang-format off
static const std::map<vk::Format, VulkanFormatInfo> gFormatTable = {
    // Uint
    {vk::Format::eR8Uint, {1, 1, 1, ComponentFormat::eUint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR8G8Uint, {2, 2, 1, ComponentFormat::eUint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR8G8B8Uint, {3, 3, 1, ComponentFormat::eUint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR8G8B8A8Uint, {4, 4, 1, ComponentFormat::eUint, false, vk::ImageAspectFlagBits::eColor}},

    {vk::Format::eR16Uint, {2, 1, 2, ComponentFormat::eUint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR16G16Uint, {4, 2, 2, ComponentFormat::eUint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR16G16B16Uint, {6, 3, 2, ComponentFormat::eUint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR16G16B16A16Uint, {8, 4, 2, ComponentFormat::eUint, false, vk::ImageAspectFlagBits::eColor}},

    {vk::Format::eR32Uint, {4, 1, 4, ComponentFormat::eUint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR32G32Uint, {8, 2, 4, ComponentFormat::eUint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR32G32B32Uint, {12, 3, 4, ComponentFormat::eUint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR32G32B32A32Uint, {16, 4, 4, ComponentFormat::eUint, false, vk::ImageAspectFlagBits::eColor}},

    {vk::Format::eR64Uint, {8, 1, 8, ComponentFormat::eUint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR64G64Uint, {16, 2, 8, ComponentFormat::eUint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR64G64B64Uint, {24, 3, 8, ComponentFormat::eUint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR64G64B64A64Uint, {32, 4, 8, ComponentFormat::eUint, false, vk::ImageAspectFlagBits::eColor}},

    // Sint
    {vk::Format::eR8Sint, {1, 1, 1, ComponentFormat::eSint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR8G8Sint, {2, 2, 1, ComponentFormat::eSint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR8G8B8Sint, {3, 3, 1, ComponentFormat::eSint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR8G8B8A8Sint, {4, 4, 1, ComponentFormat::eSint, false, vk::ImageAspectFlagBits::eColor}},

    {vk::Format::eR16Sint, {2, 1, 2, ComponentFormat::eSint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR16G16Sint, {4, 2, 2, ComponentFormat::eSint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR16G16B16Sint, {6, 3, 2, ComponentFormat::eSint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR16G16B16A16Sint, {8, 4, 2, ComponentFormat::eSint, false, vk::ImageAspectFlagBits::eColor}},

    {vk::Format::eR32Sint, {4, 1, 4, ComponentFormat::eSint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR32G32Sint, {8, 2, 4, ComponentFormat::eSint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR32G32B32Sint, {12, 3, 4, ComponentFormat::eSint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR32G32B32A32Sint, {16, 4, 4, ComponentFormat::eSint, false, vk::ImageAspectFlagBits::eColor}},

    {vk::Format::eR64Sint, {8, 1, 8, ComponentFormat::eSint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR64G64Sint, {16, 2, 8, ComponentFormat::eSint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR64G64B64Sint, {24, 3, 8, ComponentFormat::eSint, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR64G64B64A64Sint, {32, 4, 8, ComponentFormat::eSint, false, vk::ImageAspectFlagBits::eColor}},

    // Unorm
    {vk::Format::eR8Unorm, {1, 1, 1, ComponentFormat::eUnorm, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR8G8Unorm, {2, 2, 1, ComponentFormat::eUnorm, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR8G8B8Unorm, {3, 3, 1, ComponentFormat::eUnorm, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR8G8B8A8Unorm, {4, 4, 1, ComponentFormat::eUnorm, false, vk::ImageAspectFlagBits::eColor}},

    {vk::Format::eR16Unorm, {2, 1, 2, ComponentFormat::eUnorm, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR16G16Unorm, {4, 2, 2, ComponentFormat::eUnorm, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR16G16B16Unorm, {6, 3, 2, ComponentFormat::eUnorm, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR16G16B16A16Unorm, {8, 4, 2, ComponentFormat::eUnorm, false, vk::ImageAspectFlagBits::eColor}},

    // Srgb
    {vk::Format::eR8Srgb, {1, 1, 1, ComponentFormat::eUnorm, true, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR8G8Srgb, {2, 2, 1, ComponentFormat::eUnorm, true, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR8G8B8Srgb, {3, 3, 1, ComponentFormat::eUnorm, true, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR8G8B8A8Srgb, {4, 4, 1, ComponentFormat::eUnorm, true, vk::ImageAspectFlagBits::eColor}},

    // Snorm
    {vk::Format::eR8Snorm, {1, 1, 1, ComponentFormat::eSnorm, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR8G8Snorm, {2, 2, 1, ComponentFormat::eSnorm, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR8G8B8Snorm, {3, 3, 1, ComponentFormat::eSnorm, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR8G8B8A8Snorm, {4, 4, 1, ComponentFormat::eSnorm, false, vk::ImageAspectFlagBits::eColor}},

    {vk::Format::eR16Snorm, {2, 1, 2, ComponentFormat::eSnorm, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR16G16Snorm, {4, 2, 2, ComponentFormat::eSnorm, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR16G16B16Snorm, {6, 3, 2, ComponentFormat::eSnorm, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR16G16B16A16Snorm, {8, 4, 2, ComponentFormat::eSnorm, false, vk::ImageAspectFlagBits::eColor}},

    // Sfloat
    {vk::Format::eR16Sfloat, {2, 1, 2, ComponentFormat::eSfloat, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR16G16Sfloat, {4, 2, 2, ComponentFormat::eSfloat, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR16G16B16Sfloat, {6, 3, 2, ComponentFormat::eSfloat, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR16G16B16A16Sfloat, {8, 4, 2, ComponentFormat::eSfloat, false, vk::ImageAspectFlagBits::eColor}},

    {vk::Format::eR32Sfloat, {4, 1, 4, ComponentFormat::eSfloat, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR32G32Sfloat, {8, 2, 4, ComponentFormat::eSfloat, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR32G32B32Sfloat, {12, 3, 4, ComponentFormat::eSfloat, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR32G32B32A32Sfloat, {16, 4, 4, ComponentFormat::eSfloat, false, vk::ImageAspectFlagBits::eColor}},

    {vk::Format::eR64Sfloat, {8, 1, 8, ComponentFormat::eSfloat, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR64G64Sfloat, {16, 2, 8, ComponentFormat::eSfloat, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR64G64B64Sfloat, {24, 3, 8, ComponentFormat::eSfloat, false, vk::ImageAspectFlagBits::eColor}},
    {vk::Format::eR64G64B64A64Sfloat, {32, 4, 8, ComponentFormat::eSfloat, false, vk::ImageAspectFlagBits::eColor}},

    // // depth formats
    {vk::Format::eD16Unorm, {2, 1, 2, ComponentFormat::eUnorm, false, vk::ImageAspectFlagBits::eDepth}},
    {vk::Format::eD32Sfloat, {4, 1, 4, ComponentFormat::eSfloat, false, vk::ImageAspectFlagBits::eDepth}},

    {vk::Format::eD16UnormS8Uint, {3, 2, 0, ComponentFormat::eOther, false, vk::ImageAspectFlagBits::eDepth | vk::ImageAspectFlagBits::eStencil}},
    {vk::Format::eD24UnormS8Uint, {4, 2, 0, ComponentFormat::eOther, false, vk::ImageAspectFlagBits::eDepth | vk::ImageAspectFlagBits::eStencil}},
    {vk::Format::eD32SfloatS8Uint, {8, 2, 0, ComponentFormat::eOther, false, vk::ImageAspectFlagBits::eDepth | vk::ImageAspectFlagBits::eStencil}},
};
// clang-format on

uint32_t getFormatSize(vk::Format format) {
  if (!gFormatTable.contains(format)) {
    throw std::runtime_error("invalid image format");
  }
  return gFormatTable.at(format).size;
}
uint32_t getFormatElementSize(vk::Format format) {
  if (!gFormatTable.contains(format)) {
    throw std::runtime_error("invalid image format");
  }
  return gFormatTable.at(format).elemSize;
}
uint32_t getFormatChannels(vk::Format format) {
  if (!gFormatTable.contains(format)) {
    throw std::runtime_error("invalid image format");
  }
  return gFormatTable.at(format).channels;
}

bool getFormatSupportSrgb(vk::Format format) {
  return format == vk::Format::eR8Unorm || format == vk::Format::eR8G8Unorm ||
         format == vk::Format::eR8G8B8Unorm || format == vk::Format::eR8G8B8A8Unorm;
}

ComponentFormat getFormatComponentFormat(vk::Format format) {
  if (!gFormatTable.contains(format)) {
    throw std::runtime_error("invalid image format");
  }
  return gFormatTable.at(format).format;
}

vk::ImageAspectFlags getFormatAspectFlags(vk::Format format) {
  if (!gFormatTable.contains(format)) {
    throw std::runtime_error("invalid image format");
  }
  return gFormatTable.at(format).aspect;
}

template <> bool isFormatCompatible<int8_t>(vk::Format format) {
  auto componentFormat = getFormatComponentFormat(format);
  return (componentFormat == ComponentFormat::eSint ||
          componentFormat == ComponentFormat::eSnorm) &&
         getFormatElementSize(format) == 1;
}

template <> bool isFormatCompatible<int16_t>(vk::Format format) {
  auto componentFormat = getFormatComponentFormat(format);
  return (componentFormat == ComponentFormat::eSint ||
          componentFormat == ComponentFormat::eSnorm) &&
         getFormatElementSize(format) == 2;
}

template <> bool isFormatCompatible<int32_t>(vk::Format format) {
  auto componentFormat = getFormatComponentFormat(format);
  return (componentFormat == ComponentFormat::eSint ||
          componentFormat == ComponentFormat::eSnorm) &&
         getFormatElementSize(format) == 4;
}

template <> bool isFormatCompatible<int64_t>(vk::Format format) {
  auto componentFormat = getFormatComponentFormat(format);
  return (componentFormat == ComponentFormat::eSint ||
          componentFormat == ComponentFormat::eSnorm) &&
         getFormatElementSize(format) == 8;
}

template <> bool isFormatCompatible<uint8_t>(vk::Format format) {
  auto componentFormat = getFormatComponentFormat(format);
  return (componentFormat == ComponentFormat::eUint ||
          componentFormat == ComponentFormat::eUnorm) &&
         getFormatElementSize(format) == 1;
}

template <> bool isFormatCompatible<uint16_t>(vk::Format format) {
  auto componentFormat = getFormatComponentFormat(format);
  return (componentFormat == ComponentFormat::eUint ||
          componentFormat == ComponentFormat::eUnorm) &&
         getFormatElementSize(format) == 2;
}

template <> bool isFormatCompatible<uint32_t>(vk::Format format) {
  auto componentFormat = getFormatComponentFormat(format);
  return (componentFormat == ComponentFormat::eUint ||
          componentFormat == ComponentFormat::eUnorm) &&
         getFormatElementSize(format) == 4;
}

template <> bool isFormatCompatible<uint64_t>(vk::Format format) {
  auto componentFormat = getFormatComponentFormat(format);
  return (componentFormat == ComponentFormat::eUint ||
          componentFormat == ComponentFormat::eUnorm) &&
         getFormatElementSize(format) == 8;
}

template <> bool isFormatCompatible<float>(vk::Format format) {
  auto componentFormat = getFormatComponentFormat(format);
  return componentFormat == ComponentFormat::eSfloat && getFormatElementSize(format) == 4;
}

template <> bool isFormatCompatible<double>(vk::Format format) {
  auto componentFormat = getFormatComponentFormat(format);
  return componentFormat == ComponentFormat::eSfloat && getFormatElementSize(format) == 8;
}

} // namespace svulkan2