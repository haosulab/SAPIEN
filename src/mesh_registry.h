#include <PxPhysicsAPI.h>
#include <map>
#include <memory>
#include <vector>

namespace sapien {
namespace MeshUtil {
using namespace physx;

/* TODO: Implement this class */
class MeshLoader {
  PxConvexMesh *loadMesh(const std::string &filename, PxPhysics *physics, PxCooking *cooking,
                         bool useCache = true, bool createCache = true);
};

/* Load and cache obj mesh by filename */
PxConvexMesh *loadObjMesh(const std::string &filename, PxPhysics *physics, PxCooking *cooking,
                          bool useCache = true, bool createCache = true);

std::vector<PxConvexMesh *> loadMultipleObjMesh(const std::string &filename, PxPhysics *physics,
                                                PxCooking *cooking);

} // namespace MeshUtil

} // namespace sapien
