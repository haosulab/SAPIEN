#include "mesh_manager.h"
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
#include <set>

namespace sapien {
namespace fs = std::experimental::filesystem;

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
    spdlog::get("SAPIEN")->critical("Export failed: you need to build Assimp with .stl export support.");
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

  // TODO(jigu): The following codes might be written as a helper function.
  uint32_t nbfaces = pxMesh->getNbPolygons();
  // Convert into triangles
  std::vector<uint32_t> triangles;
  const uint8_t *indexBuffer = pxMesh->getIndexBuffer();
  for (uint32_t i = 0; i < nbfaces; i++) {
    PxHullPolygon polygon;
    pxMesh->getPolygonData(i, polygon);
    for (uint32_t j = 2; j < polygon.mNbVerts; j++) {
      triangles.push_back(indexBuffer[polygon.mIndexBase]);
      triangles.push_back(indexBuffer[polygon.mIndexBase + j - 1]);
      triangles.push_back(indexBuffer[polygon.mIndexBase + j]);
    }
  }

  nbfaces = triangles.size() / 3;
  scene.mMeshes[0]->mNumFaces = nbfaces;
  scene.mMeshes[0]->mFaces = new aiFace[nbfaces];

  for (uint32_t i = 0; i < nbfaces; ++i) {
    scene.mMeshes[0]->mFaces[i].mNumIndices = 3;
    scene.mMeshes[0]->mFaces[i].mIndices = new uint32_t[3];
    scene.mMeshes[0]->mFaces[i].mIndices[0] = triangles[i * 3 + 0];
    scene.mMeshes[0]->mFaces[i].mIndices[1] = triangles[i * 3 + 1];
    scene.mMeshes[0]->mFaces[i].mIndices[2] = triangles[i * 3 + 2];
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
    spdlog::get("SAPIEN")->error(importer.GetErrorString());
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
    spdlog::get("SAPIEN")->error("File not found: {}", filename);
    return nullptr;
  }

  std::string fullPath = fs::canonical(filename);
  auto it = mMeshRegistry.find(fullPath);
  if (it != mMeshRegistry.end()) {
    spdlog::get("SAPIEN")->info("Using loaded mesh: {}", filename);
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
    spdlog::get("SAPIEN")->error("Failed to cook mesh: {}", filename);
    return nullptr;
  }
  PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
  PxConvexMesh *convexMesh = mSimulation->mPhysicsSDK->createConvexMesh(input);

  spdlog::get("SAPIEN")->info("Created {} vertices from: {}", std::to_string(convexMesh->getNbVertices()),
               filename);

  if (saveCache) {
    std::string cachedFilename = getCachedFilename(filename);
    exportMeshToFile(convexMesh, cachedFilename);
    spdlog::get("SAPIEN")->info("Saved cache file: {}", cachedFilename);
  }

  mMeshRegistry[fullPath] = {/* cached */ cacheDidLoad || saveCache, /* filename */ fullPath,
                             /* mesh */ convexMesh};

  return convexMesh;
}

std::vector<std::vector<int>> splitMesh(aiMesh *mesh) {
  // build adjacency list
  spdlog::get("SAPIEN")->info("splitting mesh with {} vertices", mesh->mNumVertices);
  std::vector<std::set<int>> adj(mesh->mNumVertices);
  for (uint32_t i = 0; i < mesh->mNumFaces; ++i) {
    for (uint32_t j = 0; j < mesh->mFaces[i].mNumIndices; ++j) {
      uint32_t a = mesh->mFaces[i].mIndices[j];
      uint32_t b = mesh->mFaces[i].mIndices[(j + 1) % mesh->mFaces[i].mNumIndices];
      adj[a].insert(b);
      adj[b].insert(a);
    }
  }

  std::vector<std::vector<int>> groups;
  std::vector<int> visited(mesh->mNumVertices, 0);
  for (uint32_t i = 0; i < mesh->mNumVertices; ++i) {
    if (visited[i]) {
      continue;
    }

    // new group
    groups.emplace_back();
    groups.back().push_back(i);

    // traverse group
    visited[i] = 1;
    std::vector<int> q;
    q.push_back(i);
    while (!q.empty()) {
      int n = q.back();
      q.pop_back();
      for (auto m : adj[n]) {
        if (!visited[m]) {
          visited[m] = 1;
          groups.back().push_back(m);
          q.push_back(m);
        }
      }
    }
  }

  return groups;
}

std::vector<PxConvexMesh *> MeshManager::loadMeshGroup(const std::string &filename) {
  std::vector<PxConvexMesh *> meshes;

  if (!fs::is_regular_file(filename)) {
    spdlog::get("SAPIEN")->error("File not found: {}", filename);
    return meshes;
  }

  std::string fullPath = fs::canonical(filename);
  auto it = mMeshGroupRegistry.find(fullPath);
  if (it != mMeshGroupRegistry.end()) {
    spdlog::get("SAPIEN")->info("Using loaded mesh group: {}", filename);
    for (PxConvexMesh *mesh : it->second.meshes) {
      meshes.push_back(mesh);
    }
    return meshes;
  }

  // import obj using assimp
  Assimp::Importer importer;
  importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
                              aiComponent_NORMALS |
                              aiComponent_TEXCOORDS |
                              aiComponent_COLORS |
                              aiComponent_TANGENTS_AND_BITANGENTS | 
                              aiComponent_MATERIALS |
                              aiComponent_TEXTURES);

  uint32_t flags = aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_RemoveComponent;

  const aiScene *scene = importer.ReadFile(filename, flags);
  if (!scene) {
    spdlog::get("SAPIEN")->error(importer.GetErrorString());
    return meshes;
  }

  spdlog::get("SAPIEN")->info("Found {} meshes", scene->mNumMeshes);
  for (uint32_t i = 0; i < scene->mNumMeshes; ++i) {
    auto mesh = scene->mMeshes[i];
    auto vertexGroups = splitMesh(mesh);

    spdlog::get("SAPIEN")->info("Decomposed mesh {} into {} components", i + 1, vertexGroups.size());
    for (auto &g : vertexGroups) {
      spdlog::get("SAPIEN")->info("vertex count: {}", g.size());
      std::vector<PxVec3> vertices;
      for (auto v : g) {
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
        spdlog::get("SAPIEN")->error("Failed to cook a mesh from file: {}", filename);
      }
      PxDefaultMemoryInputData input(buf.getData(), buf.getSize());
      PxConvexMesh *convexMesh = mSimulation->mPhysicsSDK->createConvexMesh(input);
      meshes.push_back(convexMesh);
    }
  }
  return meshes;
}

} // namespace sapien
