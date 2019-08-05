#include "mesh_registry.h"
#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

namespace MeshUtil {
namespace fs = std::experimental::filesystem;

static std::map<const std::string, PxConvexMesh *> meshCache;

void exportMeshToFile(PxConvexMesh *pxMesh, const std::string &filename) {
  // check stl supported
  int formatCount = aiGetExportFormatCount();
  const char *stlId = nullptr;
  for (int i = 0; i < formatCount; ++i) {
    const aiExportFormatDesc *formatDesc = aiGetExportFormatDescription(i);
    if (std::string(formatDesc->fileExtension) == "stl") {
      stlId = formatDesc->id;
      break;
    }
  }
  if (!stlId) {
    std::cerr << "Export failed: you need to build Assimp with .stl export support." << std::endl;
    return;
  }

  Assimp::Exporter exporter;
  aiScene scene;
  scene.mRootNode = new aiNode();

  // create empty material
  scene.mMaterials = new aiMaterial *[1];
  scene.mMaterials[0] = new aiMaterial;
  scene.mNumMaterials = 1;

  // create mesh
  scene.mMeshes = new aiMesh *[1];
  scene.mMeshes[0] = new aiMesh;
  scene.mNumMeshes = 1;
  scene.mMeshes[0]->mMaterialIndex = 0;

  // add mesh to root
  scene.mRootNode->mMeshes = new uint32_t[1];
  scene.mRootNode->mMeshes[0] = 0;
  scene.mRootNode->mNumMeshes = 1;

  uint32_t nbvertices = pxMesh->getNbVertices();
  scene.mMeshes[0]->mNumVertices = nbvertices;
  scene.mMeshes[0]->mNormals = new aiVector3D[nbvertices];
  scene.mMeshes[0]->mVertices = new aiVector3D[nbvertices];
  const PxVec3 *vertices = pxMesh->getVertices();
  for (uint32_t i = 0; i < nbvertices; ++i) {
    // scene.mMeshes[0]->mVertices[i] = aiVector3D(vertices[i].x, vertices[i].z, -vertices[i].y);
    scene.mMeshes[0]->mVertices[i] = aiVector3D(vertices[i].x, vertices[i].y, -vertices[i].z);
  }

  uint32_t nbfaces = pxMesh->getNbPolygons();
  scene.mMeshes[0]->mNumFaces = nbfaces;
  scene.mMeshes[0]->mFaces = new aiFace[nbfaces];

  const uint8_t *indexBuffer = pxMesh->getIndexBuffer();

  for (uint32_t i = 0; i < nbfaces; ++i) {
    PxHullPolygon polygon;
    pxMesh->getPolygonData(i, polygon);
    scene.mMeshes[0]->mFaces[i].mNumIndices = polygon.mNbVerts;
    scene.mMeshes[0]->mFaces[i].mIndices = new uint32_t[polygon.mNbVerts];
    uint16_t offset = polygon.mIndexBase;
    for (uint32_t j = 0; j < polygon.mNbVerts; ++j) {
      scene.mMeshes[0]->mFaces[i].mIndices[j] = indexBuffer[offset + j];
    }
  }
  exporter.Export(&scene, stlId, filename);

  // memory freed by aiScene destructor
}

std::vector<PxVec3> verticesFromMeshFile(const std::string &filename) {
  std::vector<PxVec3> vertices;
  Assimp::Importer importer;
  const aiScene *scene = importer.ReadFile(filename, aiProcess_PreTransformVertices);
  if (!scene) {
    fprintf(stderr, "%s\n", importer.GetErrorString());
    exit(1);
  }
#ifdef _VERBOSE
  printf("Loaded collision mesh %s\n", filename.c_str());
#endif

  for (uint32_t i = 0; i < scene->mNumMeshes; ++i) {
    auto mesh = scene->mMeshes[i];
    for (uint32_t v = 0; v < mesh->mNumVertices; ++v) {
      auto vertex = mesh->mVertices[v];
      vertices.push_back({vertex.x, vertex.y, vertex.z});
    }
  }
  return vertices;
}

// TODO: Finish implementation
PxConvexMesh *MeshLoader::loadMesh(const std::string &filename, PxPhysics *physics,
                                   PxCooking *cooking, bool useCache, bool createCache) {
  // TODO: implement import flags
  throw "Not implemented";

  if (meshCache.find(filename) != meshCache.end()) {
#ifdef _VERBOSE
    std::cout << "Using cached mesh for \"" << filename << "\"" << std::endl;
#endif
    return meshCache[filename];
  }
#ifdef _VERBOSE
  std::cout << "Creating new convex mesh for \"" << filename << "\"" << std::endl;
#endif
  std::string cacheFilename = filename + ".convex.stl";

  std::string fullPath;
  bool shouldWriteCache = useCache;
  if (useCache) {
    if (fs::exists(cacheFilename)) {
      fullPath = fs::absolute(cacheFilename);
      shouldWriteCache = false;
    } else if (fs::exists(filename)) {
      fullPath = fs::absolute(filename);
    } else {
      std::cerr << "File: " << filename << " not found" << std::endl;
      return nullptr;
    }
  } else if (fs::exists(filename)) {
    fullPath = fs::absolute(filename);
  } else {
    std::cerr << "File: " << filename << " not found" << std::endl;
    return nullptr;
  }
  std::vector<PxVec3> vertices = verticesFromMeshFile(fullPath);
#ifdef _VERBOSE
  std::cout << "Starting convex mesh cooking: " << vertices.size() << " vertices." << std::endl;
#endif
  PxConvexMeshDesc convexDesc;
  convexDesc.points.count = vertices.size();
  convexDesc.points.stride = sizeof(PxVec3);
  convexDesc.points.data = vertices.data();
  convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX; // | PxConvexFlag::eSHIFT_VERTICES;
  convexDesc.vertexLimit = 256;

  PxDefaultMemoryOutputStream buf;
  PxConvexMeshCookingResult::Enum result;
  if (!cooking->cookConvexMesh(convexDesc, buf, &result)) {
    std::cerr << "Unable to cook convex mesh." << std::endl;
    std::cerr << "Exiting..." << std::endl;
    exit(1);
  }
  PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
  PxConvexMesh *convexMesh = physics->createConvexMesh(input);

#ifdef _VERBOSE
  std::cout << "Convex mesh generated: " << convexMesh->getNbVertices() << " vertices."
            << std::endl;
#endif
  meshCache[filename] = convexMesh;

  if (shouldWriteCache) {
#ifdef _VERBOSE
    std::cout << "Exporting convex mesh to " << cacheFilename << std::endl;
#endif
    exportMeshToFile(convexMesh, cacheFilename);
  }

  return convexMesh;
}

PxConvexMesh *loadObjMesh(const std::string &filename, PxPhysics *physics, PxCooking *cooking,
                          bool useCache, bool createCacahe) {

  if (meshCache.find(filename) != meshCache.end()) {
#ifdef _VERBOSE
    std::cout << "Using cached mesh for \"" << filename << "\"" << std::endl;
#endif
    return meshCache[filename];
  }
#ifdef _VERBOSE
  std::cout << "Creating new convex mesh for \"" << filename << "\"" << std::endl;
#endif
  std::string cacheFilename = filename + ".convex.stl";

  std::string fullPath;
  bool shouldWriteCache = useCache;
  if (useCache) {
    if (fs::exists(cacheFilename)) {
      fullPath = fs::absolute(cacheFilename);
      shouldWriteCache = false;
    } else if (fs::exists(filename)) {
      fullPath = fs::absolute(filename);
    } else {
      std::cerr << "File: " << filename << " not found" << std::endl;
      return nullptr;
    }
  } else if (fs::exists(filename)) {
    fullPath = fs::absolute(filename);
  } else {
    std::cerr << "File: " << filename << " not found" << std::endl;
    return nullptr;
  }
  std::vector<PxVec3> vertices = verticesFromMeshFile(fullPath);
#ifdef _VERBOSE
  std::cout << "Starting convex mesh cooking: " << vertices.size() << " vertices." << std::endl;
#endif
  PxConvexMeshDesc convexDesc;
  convexDesc.points.count = vertices.size();
  convexDesc.points.stride = sizeof(PxVec3);
  convexDesc.points.data = vertices.data();
  convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX; // | PxConvexFlag::eSHIFT_VERTICES;
  convexDesc.vertexLimit = 256;

  PxDefaultMemoryOutputStream buf;
  PxConvexMeshCookingResult::Enum result;
  if (!cooking->cookConvexMesh(convexDesc, buf, &result)) {
    std::cerr << "Unable to cook convex mesh." << std::endl;
    std::cerr << "Exiting..." << std::endl;
    exit(1);
  }
  PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
  PxConvexMesh *convexMesh = physics->createConvexMesh(input);

#ifdef _VERBOSE
  std::cout << "Convex mesh generated: " << convexMesh->getNbVertices() << " vertices."
            << std::endl;
#endif
  meshCache[filename] = convexMesh;

  if (shouldWriteCache) {
#ifdef _VERBOSE
    std::cout << "Exporting convex mesh to " << cacheFilename << std::endl;
#endif
    exportMeshToFile(convexMesh, cacheFilename);
  }

  return convexMesh;
}

} // namespace MeshUtil
