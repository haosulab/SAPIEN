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
#include "svulkan2/resource/shape.h"

namespace svulkan2 {
namespace resource {

std::shared_ptr<SVShape> SVShape::Create(std::shared_ptr<SVMesh> mesh,
                                         std::shared_ptr<SVMaterial> material) {
  auto shape = std::make_shared<SVShape>();
  shape->mesh = mesh;
  shape->material = material;
  return shape;
}

} // namespace resource
} // namespace svulkan2