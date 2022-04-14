#pragma once
#include <PxPhysicsAPI.h>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace sapien {
class Simulation;

struct NonConvexMeshRecord {
  bool cached;
  std::string filename;
  physx::PxTriangleMesh *mesh;
};

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
  std::string mCacheSuffixNonConvex = ".nonconvex.stl";

  Simulation *mSimulation;
  std::map<std::string, NonConvexMeshRecord> mNonConvexMeshRegistry;
  std::map<std::string, MeshRecord> mMeshRegistry;
  std::map<std::string, MeshGroupRecord> mMeshGroupRegistry;

public:
  explicit MeshManager(Simulation *simulation);

  physx::PxTriangleMesh *loadNonConvexMesh(const std::string &filename, bool useCache = true,
                                           bool saveCache = true);

  physx::PxConvexMesh *loadMesh(const std::string &filename, bool useCache = true,
                                bool saveCache = true);

  std::vector<physx::PxConvexMesh *> loadMeshGroup(const std::string &filename);

public:
  // cache config

  void setCacheSuffix(const std::string &filename);
  std::string getCachedFilename(const std::string &filename);
  std::string getCachedFilenameNonConvex(const std::string &filename);
};
} // namespace sapien
