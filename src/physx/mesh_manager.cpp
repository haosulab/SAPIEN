#include "sapien/physx/mesh_manager.h"
#include "../logger.h"
#include "sapien/physx/physx_default.h"
#include "sapien/physx/physx_system.h"
#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <filesystem>
#include <set>

namespace fs = std::filesystem;
using namespace physx;

namespace sapien {
namespace physx {

static std::shared_ptr<MeshManager> gManager;
std::shared_ptr<MeshManager> MeshManager::Get() {
  if (!gManager) {
    gManager = std::make_shared<MeshManager>();
  }
  return gManager;
}

void MeshManager::Clear() {
  if (gManager) {
    gManager->mTriangleMeshWithSDFRegistry.clear();
    gManager->mTriangleMeshRegistry.clear();
    gManager->mConvexMeshRegistry.clear();
    gManager->mConvexMeshGroupRegistry.clear();
  }
}

static std::string getFullPath(std::string const &filename) {
  if (!fs::is_regular_file(fs::path(filename))) {
    logger::error("File not found: {}", filename);
    return nullptr;
  }
  return fs::canonical(filename).string();
}

MeshManager::MeshManager() {}

std::shared_ptr<PhysxTriangleMesh> MeshManager::loadTriangleMesh(const std::string &filename) {
  std::string fullPath = getFullPath(filename);

  auto it = mTriangleMeshRegistry.find(fullPath);
  if (it != mTriangleMeshRegistry.end()) {
    logger::info("Using loaded mesh: {}", filename);
    return it->second;
  }

  auto mesh = std::make_shared<PhysxTriangleMesh>(fullPath, false);
  mTriangleMeshRegistry[fullPath] = mesh;

  return mesh;
}

std::shared_ptr<PhysxTriangleMesh>
MeshManager::loadTriangleMeshWithSDF(const std::string &filename) {
  std::string fullPath = getFullPath(filename);

  auto it = mTriangleMeshWithSDFRegistry.find(fullPath);
  if (it != mTriangleMeshWithSDFRegistry.end()) {
    if (it->second->getSDFSpacing() == PhysxDefault::getSDFShapeConfig().spacing &&
        it->second->getSDFSubgridSize() == PhysxDefault::getSDFShapeConfig().subgridSize) {
      logger::info("Using loaded mesh with SDF: {}", filename);
      return it->second;
    } else {
      logger::warn(
          "Loading same mesh with different SDF parameters: {}. This may be due to an error.",
          filename);
    }
  }

  auto mesh = std::make_shared<PhysxTriangleMesh>(fullPath, true);
  mTriangleMeshWithSDFRegistry[fullPath] = mesh;

  return mesh;
}

std::vector<std::shared_ptr<PhysxConvexMesh>>
MeshManager::loadConvexMeshGroup(const std::string &filename) {
  std::string fullPath = getFullPath(filename);
  auto it = mConvexMeshGroupRegistry.find(fullPath);
  if (it != mConvexMeshGroupRegistry.end()) {
    logger::info("Using loaded mesh group: {}", filename);
    return it->second;
  }

  auto meshes = PhysxConvexMesh::LoadByConnectedParts(fullPath);
  mConvexMeshGroupRegistry[fullPath] = meshes;

  return meshes;
}

std::shared_ptr<PhysxConvexMesh> MeshManager::loadConvexMesh(const std::string &filename) {

  std::string fullPath = getFullPath(filename);
  auto it = mConvexMeshRegistry.find(fullPath);
  if (it != mConvexMeshRegistry.end()) {
    logger::info("Using loaded mesh: {}", filename);
    return it->second;
  }

  auto mesh = std::make_shared<PhysxConvexMesh>(fullPath);
  mConvexMeshRegistry[fullPath] = mesh;
  return mesh;
}

} // namespace physx
} // namespace sapien
