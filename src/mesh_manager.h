#pragma once
#include <PxPhysicsAPI.h>
#include <map>
#include <memory>
#include <vector>

namespace sapien {
class Simulation;

struct MeshRecord {
  bool cached;
  std::string filename;
  physx::PxConvexMesh *mesh;
};

struct MeshGroupRecord {
  std::string filename;
  std::vector<physx::PxConvexMesh *> meshes;
};

class MeshManager {
private:
  std::string mCacheSuffix = ".convex.stl";

  Simulation *mSimulation;
  std::map<std::string, MeshRecord> mMeshRegistry;
  std::map<std::string, MeshGroupRecord> mMeshGroupRegistry;

public:
  explicit MeshManager(Simulation *simulation);

  physx::PxConvexMesh *loadMesh(const std::string &filename, bool useCache = true,
                                bool saveCache = true);

  std::vector<physx::PxConvexMesh *> loadMeshGroup(const std::string &filename);

  /* 1. VHACD
   * 2. QuickHull
   * Undefined behavior on non-watertight meshes
   */
  std::vector<physx::PxConvexMesh *> loadMeshGroupVHACD(const std::string &filename);

  /* 1. Robust Watertight Manifold Surface Generation Method for ShapeNet Models
   * 2. VHACD
   * 3. QuickHull
   */
  std::vector<physx::PxConvexMesh *> loadMeshGroupManifoldVHACD(const std::string &filename);

public:
  // cache config

  void setCacheSuffix(const std::string &filename);
  std::string getCachedFilename(const std::string &filename);
};
} // namespace sapien
