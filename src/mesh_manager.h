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

class MeshManager {
private:
  std::string mCacheSuffix = ".convex.stl";

  Simulation *mSimulation;
  std::map<std::string, MeshRecord> mMeshRegistry;

public:
  MeshManager(Simulation *simulation);

  physx::PxConvexMesh *loadMesh(const std::string &filename, bool useCache = true,
                                bool saveCache = true);

public:
  // cache config

  void setCacheSuffix(const std::string &filename);
  std::string getCachedFilename(const std::string &filename);
};
} // namespace sapien
