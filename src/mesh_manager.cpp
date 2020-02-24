#include "mesh_manager.h"
#include "VHACD.h"
#include "simulation.h"
#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <spdlog/spdlog.h>
#include <sstream>

namespace sapien {
namespace fs = std::experimental::filesystem;

static void exportMeshToFile(PxConvexMesh *pxMesh, const std::string &filename) {
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
    spdlog::critical("Export failed: you need to build Assimp with .stl export support.");
    throw std::runtime_error("Assimp is not built with .stl support.");
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
    scene.mMeshes[0]->mVertices[i] = aiVector3D(vertices[i].x, vertices[i].y, vertices[i].z);
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

static std::vector<PxVec3> getVerticesFromMeshFile(const std::string &filename) {
  std::vector<PxVec3> vertices;
  Assimp::Importer importer;
  uint32_t flags = aiProcess_Triangulate | aiProcess_PreTransformVertices;
  importer.SetPropertyInteger(AI_CONFIG_PP_PTV_ADD_ROOT_TRANSFORMATION, 1);

  const aiScene *scene = importer.ReadFile(filename, flags);

  if (!scene) {
    spdlog::error(importer.GetErrorString());
    return {};
  }

  for (uint32_t i = 0; i < scene->mNumMeshes; ++i) {
    auto mesh = scene->mMeshes[i];
    for (uint32_t v = 0; v < mesh->mNumVertices; ++v) {
      auto vertex = mesh->mVertices[v];
      vertices.push_back({vertex.x, vertex.y, vertex.z});
    }
  }
  return vertices;
}

MeshManager::MeshManager(Simulation *simulation) : mSimulation(simulation) {}

void MeshManager::setCacheSuffix(const std::string &filename) {
  if (filename.empty()) {
    throw std::runtime_error("Invalid suffix: empty string.");
  }
  mCacheSuffix = filename;
}

std::string MeshManager::getCachedFilename(const std::string &filename) {
  return filename + mCacheSuffix;
}

physx::PxConvexMesh *MeshManager::loadMesh(const std::string &filename, bool useCache,
                                           bool saveCache) {

  if (!fs::is_regular_file(filename)) {
    spdlog::error("File not found: {}", filename);
    return nullptr;
  }

  std::string fullPath = fs::canonical(filename);
  auto it = mMeshRegistry.find(fullPath);
  if (it != mMeshRegistry.end()) {
    spdlog::info("Using loaded mesh: {}", filename);
    return it->second.mesh;
  }

  bool cacheDidLoad = false;
  std::string fileToLoad = filename;
  if (useCache) {
    std::string cachedFilename = getCachedFilename(filename);
    if (fs::is_regular_file(cachedFilename)) {
      fileToLoad = cachedFilename;
      saveCache = false; // no need to save cache if it is loaded
      cacheDidLoad = true;
    }
  }

  std::vector<PxVec3> vertices = getVerticesFromMeshFile(fileToLoad);
  PxConvexMeshDesc convexDesc;
  convexDesc.points.count = vertices.size();
  convexDesc.points.stride = sizeof(PxVec3);
  convexDesc.points.data = vertices.data();
  convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX; // FIXME: shift vertices may improve statbility
  convexDesc.vertexLimit = 256;

  PxDefaultMemoryOutputStream buf;
  PxConvexMeshCookingResult::Enum result;
  if (!mSimulation->mCooking->cookConvexMesh(convexDesc, buf, &result)) {
    spdlog::error("Failed to cook mesh: {}", filename);
    return nullptr;
  }
  PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
  PxConvexMesh *convexMesh = mSimulation->mPhysicsSDK->createConvexMesh(input);

  spdlog::info("Created {} vertices from: {}", std::to_string(convexMesh->getNbVertices()),
               filename);

  if (saveCache) {
    std::string cachedFilename = getCachedFilename(filename);
    exportMeshToFile(convexMesh, cachedFilename);
    spdlog::info("Saved cache file: {}", cachedFilename);
  }

  mMeshRegistry[fullPath] = {/* cached */ cacheDidLoad || saveCache, /* filename */ fullPath,
                             /* mesh */ convexMesh};

  return convexMesh;
}

std::vector<PxConvexMesh *> MeshManager::loadMeshGroup(const std::string &filename) {
  std::vector<PxConvexMesh *> meshes;

  if (!fs::is_regular_file(filename)) {
    spdlog::error("File not found: {}", filename);
    return meshes;
  }

  std::string fullPath = fs::canonical(filename);
  auto it = mMeshGroupRegistry.find(fullPath);
  if (it != mMeshGroupRegistry.end()) {
    spdlog::info("Using loaded mesh group: {}", filename);
    for (PxConvexMesh *mesh : it->second.meshes) {
      meshes.push_back(mesh);
    }
    return meshes;
  }

  // import obj using assimp
  Assimp::Importer importer;
  uint32_t flags = aiProcess_Triangulate;
  const aiScene *scene = importer.ReadFile(filename, flags);
  if (!scene) {
    spdlog::error(importer.GetErrorString());
    return meshes;
  }

  for (uint32_t i = 0; i < scene->mNumMeshes; ++i) {
    std::vector<PxVec3> vertices;
    auto mesh = scene->mMeshes[i];
    for (uint32_t v = 0; v < mesh->mNumVertices; ++v) {
      auto vertex = mesh->mVertices[v];
      vertices.push_back({vertex.x, vertex.y, vertex.z});
    }

    PxConvexMeshDesc convexDesc;
    convexDesc.points.count = vertices.size();
    convexDesc.points.stride = sizeof(PxVec3);
    convexDesc.points.data = vertices.data();
    convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX; // | PxConvexFlag::eSHIFT_VERTICES;
    convexDesc.vertexLimit = 256;

    PxDefaultMemoryOutputStream buf;
    PxConvexMeshCookingResult::Enum result;
    if (!mSimulation->mCooking->cookConvexMesh(convexDesc, buf, &result)) {
      spdlog::error("Failed to cook a mesh from file: {}", filename);
    }
    PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
    PxConvexMesh *convexMesh = mSimulation->mPhysicsSDK->createConvexMesh(input);
    meshes.push_back(convexMesh);
  }
  return meshes;
}

std::vector<physx::PxConvexMesh *> MeshManager::loadMeshGroupVHACD(const std::string &filename) {
  spdlog::info("Decomposing {} into convex shapes with VHACD...", filename);
  VHACD::IVHACD *vhacd = VHACD::CreateVHACD();

  std::vector<PxConvexMesh *> meshes;
  if (!fs::is_regular_file(filename)) {
    spdlog::error("File not found: {}", filename);
    return meshes;
  }

  std::string fullPath = fs::canonical(filename);
  auto it = mMeshGroupRegistry.find(fullPath);
  if (it != mMeshGroupRegistry.end()) {
    spdlog::info("Using loaded mesh group: {}", filename);
    for (PxConvexMesh *mesh : it->second.meshes) {
      meshes.push_back(mesh);
    }
    return meshes;
  }

  // import obj using assimp
  Assimp::Importer importer;
  uint32_t flags = aiProcess_Triangulate;
  const aiScene *scene = importer.ReadFile(filename, flags);
  if (!scene) {
    spdlog::error(importer.GetErrorString());
    return meshes;
  }

  for (uint32_t i = 0; i < scene->mNumMeshes; ++i) {
    auto mesh = scene->mMeshes[i];

    std::vector<float> points;
    points.reserve(mesh->mNumVertices * 3);
    for (uint32_t i = 0; i < mesh->mNumVertices; ++i) {
      points.push_back(mesh->mVertices[i].x);
      points.push_back(mesh->mVertices[i].y);
      points.push_back(mesh->mVertices[i].z);
    }

    std::vector<uint32_t> triangles;
    triangles.reserve(mesh->mNumFaces * 3);
    for (uint32_t i = 0; i < mesh->mNumFaces; ++i) {
      if (mesh->mFaces[i].mNumIndices != 3) {
        spdlog::error("Detected non-triangular face with {} vertices",
                      mesh->mFaces[i].mNumIndices);
        continue;
      }
      triangles.push_back(mesh->mFaces[i].mIndices[0]);
      triangles.push_back(mesh->mFaces[i].mIndices[1]);
      triangles.push_back(mesh->mFaces[i].mIndices[2]);
    }

    VHACD::IVHACD::Parameters params;
    spdlog::info("Processing mesh {}", i);
    vhacd->Compute(points.data(), mesh->mNumVertices, triangles.data(), triangles.size() / 3,
                   params);
    uint32_t hullCount = vhacd->GetNConvexHulls();

    for (uint32_t i = 0; i < hullCount; ++i) {
      VHACD::IVHACD::ConvexHull hull;
      vhacd->GetConvexHull(i, hull);

      std::vector<PxVec3> vertices;
      for (uint32_t v = 0; v < hull.m_nPoints; ++v) {
        vertices.push_back(
            PxVec3(hull.m_points[3 * v], hull.m_points[3 * v + 1], hull.m_points[3 * v + 2]));
      }
      PxConvexMeshDesc convexDesc;
      convexDesc.points.count = vertices.size();
      convexDesc.points.stride = sizeof(PxVec3);
      convexDesc.points.data = vertices.data();
      convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX; // | PxConvexFlag::eSHIFT_VERTICES;
      convexDesc.vertexLimit = 256;
      PxDefaultMemoryOutputStream buf;
      PxConvexMeshCookingResult::Enum result;
      if (!mSimulation->mCooking->cookConvexMesh(convexDesc, buf, &result)) {
        spdlog::error("Failed to cook a mesh from file: {}", filename);
      }
      PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
      PxConvexMesh *convexMesh = mSimulation->mPhysicsSDK->createConvexMesh(input);
      meshes.push_back(convexMesh);
    }
  }

  vhacd->Release();
  spdlog::info("VHACD complete");
  return meshes;
}

} // namespace sapien
