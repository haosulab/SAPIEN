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
#include "sapien/sapien_renderer/image.h"
#include <svulkan2/common/format.h>

namespace sapien {
namespace sapien_renderer {

std::string getFormatDescriptor(vk::Format format) {
  switch (svulkan2::getFormatComponentFormat(format)) {
  case svulkan2::ComponentFormat::eSfloat:
    return "f";
  case svulkan2::ComponentFormat::eSnorm:
  case svulkan2::ComponentFormat::eSint:
    return "i";
  case svulkan2::ComponentFormat::eUint:
  case svulkan2::ComponentFormat::eUnorm:
    return "u";
  case svulkan2::ComponentFormat::eOther:
  default:
    return "V";
  }
}

int getFormatChannelSize(vk::Format format) {
  int size = svulkan2::getFormatElementSize(format);

  // mixed channels, treat as 1 channel
  if (size == 0) {
    size = svulkan2::getFormatSize(format);
  }

  return size;
}

int getFormatChannels(vk::Format format) {
  int size = svulkan2::getFormatChannels(format);

  // channels of mixed types, treat as 1 channel
  if (svulkan2::getFormatElementSize(format) == 0) {
    size = 1;
  }
  return size;
}

std::string getFormatTypestr(vk::Format format) {
  return getFormatDescriptor(format) + std::to_string(getFormatChannelSize(format));
}

SapienRenderImageCpu::SapienRenderImageCpu(int width, int height, vk::Format format, void *ptr) {
  this->format = format;

  int channels = getFormatChannels(format);
  int itemsize = getFormatChannelSize(format);
  this->shape = {height, width};
  this->strides = {itemsize * channels * width, itemsize * channels};
  if (channels != 1) {
    this->shape.push_back(channels);
    this->strides.push_back(itemsize);
  }

  this->type = getFormatTypestr(format);
  this->ptr = ptr;
}

SapienRenderImageCuda::SapienRenderImageCuda(int width, int height, vk::Format format, void *ptr,
                                             int cudaId) {
  this->format = format;

  int channels = getFormatChannels(format);
  int itemsize = getFormatChannelSize(format);
  this->shape = {height, width};
  this->strides = {itemsize * channels * width, itemsize * channels};
  if (channels != 1) {
    this->shape.push_back(channels);
    this->strides.push_back(itemsize);
  }

  this->type = getFormatTypestr(format);
  this->ptr = ptr;
  this->cudaId = cudaId;
}

} // namespace sapien_renderer
} // namespace sapien
