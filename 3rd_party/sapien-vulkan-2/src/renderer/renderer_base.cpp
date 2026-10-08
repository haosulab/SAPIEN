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
#include "svulkan2/renderer/renderer_base.h"
#include "svulkan2/renderer/renderer.h"
#include "svulkan2/renderer/rt_renderer.h"
#include "svulkan2/shader/rt.h"
#include <filesystem>

namespace fs = std::filesystem;

namespace svulkan2 {
namespace renderer {
std::unique_ptr<RendererBase>
RendererBase::Create(std::shared_ptr<RendererConfig> config) {
  if (fs::exists((fs::path(config->shaderDir) / "gbuffer.vert"))) {
    return std::make_unique<Renderer>(config);
  }
  if (fs::exists((fs::path(config->shaderDir) / "camera.rgen"))) {
    return std::make_unique<RTRenderer>(config->shaderDir);
  }
  throw std::runtime_error("Shader directory must contain gbuffer.vert for "
                           "rasterization or camera.rgen for ray tracing");
}

} // namespace renderer
} // namespace svulkan2