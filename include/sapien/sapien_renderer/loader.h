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
#include "light_component.h"
#include "render_body_component.h"
#include <variant>

namespace sapien {
namespace sapien_renderer {

struct RenderSceneLoaderNode {
  std::string name;
  Pose pose{};
  Vec3 scale{1};

  std::vector<std::shared_ptr<RenderSceneLoaderNode>> children;
  std::shared_ptr<RenderShapeTriangleMesh> mesh;
  std::shared_ptr<SapienRenderLightComponent> light;

  std::tuple<std::vector<std::shared_ptr<RenderShapeTriangleMesh>>,
             std::vector<std::shared_ptr<SapienRenderLightComponent>>>
  flatten();
};

std::unique_ptr<RenderSceneLoaderNode> LoadScene(std::string const &filename,
                                                 bool applyScale = false);

} // namespace sapien_renderer
} // namespace sapien
