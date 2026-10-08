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
#include "layout.h"
#include <memory>
#include <unordered_map>

namespace svulkan2 {

/** Renderer options configured by API */
struct RendererConfig {
  std::string shaderDir;

  // default 1-channel texture format
  vk::Format colorFormat1{vk::Format::eR32Sfloat}; // R8Unorm, R32Sfloat

  // default 4-channel texture format
  vk::Format colorFormat4{vk::Format::eR32G32B32A32Sfloat}; // R8G8B8A8Unorm, R32G32B32A32Sfloat

  vk::SampleCountFlagBits msaa{vk::SampleCountFlagBits::e1}; // msaa

  // texture foramt for specific textures
  std::unordered_map<std::string, vk::Format> textureFormat;

  vk::Format depthFormat{vk::Format::eD32Sfloat}; // D32Sfloat
  // vk::CullModeFlags culling{vk::CullModeFlagBits::eBack};

  // when true, a single depth texture is used for all gbuffer passes (incremental rendering)
  bool shareGbufferDepths{true};

  bool operator==(RendererConfig const &other) const = default;
};

} // namespace svulkan2