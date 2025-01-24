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

#include "mesh.h"
#include <PxPhysicsAPI.h>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace sapien {
namespace physx {

class PhysxEngine;

class MeshManager {
public:
  static std::shared_ptr<MeshManager> Get();
  static void Clear();

  MeshManager();

  std::shared_ptr<PhysxTriangleMesh> loadTriangleMesh(const std::string &filename);
  std::shared_ptr<PhysxTriangleMesh> loadTriangleMeshWithSDF(const std::string &filename);
  std::shared_ptr<PhysxConvexMesh> loadConvexMesh(const std::string &filename);
  std::vector<std::shared_ptr<PhysxConvexMesh>> loadConvexMeshGroup(const std::string &filename);

private:
  std::map<std::string, std::shared_ptr<PhysxTriangleMesh>> mTriangleMeshRegistry;
  std::map<std::string, std::shared_ptr<PhysxConvexMesh>> mConvexMeshRegistry;
  std::map<std::string, std::vector<std::shared_ptr<PhysxConvexMesh>>> mConvexMeshGroupRegistry;

  std::map<std::string, std::shared_ptr<PhysxTriangleMesh>> mTriangleMeshWithSDFRegistry;
};

} // namespace physx
} // namespace sapien
