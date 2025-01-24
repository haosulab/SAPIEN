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
#include "sapien/array.h"

namespace sapien {
namespace sapien_renderer {

std::string getFormatDescriptor(vk::Format format);
int getFormatChannelSize(vk::Format format);
int getFormatChannels(vk::Format format);
std::string getFormatTypestr(vk::Format format);

struct SapienRenderImageCpu : public CpuArrayHandle {
  vk::Format format;

  SapienRenderImageCpu(int width, int height, vk::Format format, void *ptr);
};

struct SapienRenderImageCuda : public CudaArrayHandle {
  vk::Format format;

  SapienRenderImageCuda(int width, int height, vk::Format format, void *ptr, int cudaId);
};

} // namespace sapien_renderer
} // namespace sapien
