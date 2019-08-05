#include <PxPhysicsAPI.h>
#include <map>
#include <memory>
#include <vector>

namespace MeshUtil {
using namespace physx;

/* TODO: Implement this class */
class MeshLoader {
  enum PoseFlag {
    /* The mesh coordinates are loaded as is, ignoring conventions (.obj and .stl)
     * or explicit specifications (.dae)
     */
    eAS_IS,

    /* The mesh are loaded matching convention or explicit specifications
     * .obj, y axis mapped to z axis, -z axis mapped to y axis
     * .stl, same as eAS_IS
     * .dae, same as .stl if up axis is z, same as .obj if up axis is y
     */
    eMATCH_DEFAULT,

    /* Allows custom control (as is with custom transformation matrix) */
    eCUSTOM
  } poseFlag = eAS_IS;
  PxVec3 up = {0, 0, 1};
  PxVec3 forward = {0, 1, 0};

  PxConvexMesh *loadMesh(const std::string &filename, PxPhysics *physics, PxCooking *cooking,
                         bool useCache = true, bool createCache = true);
};

/* Load and cache obj mesh by filename */
PxConvexMesh *loadObjMesh(const std::string &filename, PxPhysics *physics, PxCooking *cooking,
                          bool useCache = true, bool createCache = true);

} // namespace MeshUtil
