#include <PxPhysicsAPI.h>
#include <memory>
#include <vector>
#include <map>

namespace MeshUtil {
using namespace physx;

/* Load and cache obj mesh by filename */
PxConvexMesh *loadObjMesh(const std::string &filename,
                          PxPhysics *physics, PxCooking *cooking);

}
