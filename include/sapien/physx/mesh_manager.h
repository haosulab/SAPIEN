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
  std::shared_ptr<PhysxConvexMesh> loadConvexMesh(const std::string &filename);
  std::vector<std::shared_ptr<PhysxConvexMesh>> loadConvexMeshGroup(const std::string &filename);

private:
  std::map<std::string, std::shared_ptr<PhysxTriangleMesh>> mTriangleMeshRegistry;
  std::map<std::string, std::shared_ptr<PhysxConvexMesh>> mConvexMeshRegistry;
  std::map<std::string, std::vector<std::shared_ptr<PhysxConvexMesh>>> mConvexMeshGroupRegistry;
};

} // namespace physx
} // namespace sapien
