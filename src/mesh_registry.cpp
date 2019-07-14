#include "mesh_registry.h"
#include <fstream>
#include <iostream>
#include <sstream>

namespace MeshUtil {

static std::map<const std::string, PxConvexMesh *> meshCache;

PxConvexMesh *loadObjMesh(const std::string &filename, PxPhysics *physics, PxCooking *cooking) {

  if (meshCache.find(filename) != meshCache.end()) {
#ifdef _VERBOSE
    std::cout << "Using cached mesh for \"" << filename << "\"" << std::endl;
#endif
    return meshCache[filename];
  }
#ifdef _VERBOSE
  std::cout << "Creating convex mesh for \"" << filename << "\"" << std::endl;
#endif

  std::vector<PxVec3> vertices;

  std::ifstream f(filename);
  std::string line;

  std::string t;
  float a, b, c;
  while (std::getline(f, line)) {
    if (line[0] == '#') {
      continue;
    }
    std::istringstream iss(line);
    iss >> t;
    if (t == "v") {
      iss >> a >> b >> c;
      vertices.push_back({a, b, c});
    }
  }

  PxConvexMeshDesc convexDesc;
  convexDesc.points.count = vertices.size();
  convexDesc.points.stride = sizeof(PxVec3);
  convexDesc.points.data = vertices.data();
  convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX | PxConvexFlag::eSHIFT_VERTICES;

  PxDefaultMemoryOutputStream buf;
  PxConvexMeshCookingResult::Enum result;
  if (!cooking->cookConvexMesh(convexDesc, buf, &result)) {
    std::cerr << "Unable to cook convex mesh." << std::endl;
    std::cerr << "Exiting..." << std::endl;
    exit(1);
  }
  PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
  PxConvexMesh *convexMesh = physics->createConvexMesh(input);
  meshCache[filename] = convexMesh;
  return convexMesh;
}

} // namespace MeshUtil
