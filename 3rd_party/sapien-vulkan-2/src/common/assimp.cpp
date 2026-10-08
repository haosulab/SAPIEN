/*
 * Copyright 2025 Hillbot Inc.
 * Copyright 2020-2024 UCSD SU Lab
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "svulkan2/common/assimp.h"
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>

namespace svulkan2 {

const char *getFormatId(std::string const &format) {
  int formatCount = aiGetExportFormatCount();
  const char *formatId = nullptr;
  for (int i = 0; i < formatCount; ++i) {
    const aiExportFormatDesc *formatDesc = aiGetExportFormatDescription(i);
    if (std::string(formatDesc->fileExtension) == format) {
      formatId = formatDesc->id;
      return formatId;
    }
  }
  throw std::runtime_error("export mesh failed: unsupported format \"" +
                           format + "\"");
}

void exportTriangleMesh(std::string const &filename,
                        std::vector<float> const &vertices,
                        std::vector<uint32_t> const &indices,
                        std::vector<float> const &normals,
                        std::vector<float> const &uvs) {
  auto dotidx = filename.find_last_of(".");
  if (dotidx == filename.npos) {
    throw std::runtime_error(
        "export mesh failed: filename does not contain a format");
  }
  const char *formatId = getFormatId(filename.substr(dotidx + 1));

  // sanity checks
  if (vertices.size() == 0) {
    throw std::runtime_error("export mesh failed: no vertices");
  }
  if (vertices.size() % 3 != 0) {
    throw std::runtime_error(
        "export mesh failed: vertex array count is not a multiple of 3");
  }
  if (indices.size() % 3 != 0) {
    throw std::runtime_error(
        "export mesh failed: index count is not a multiple of 3");
  }
  for (uint32_t index : indices) {
    if (index >= vertices.size() / 3) {
      throw std::runtime_error("export mesh failed: index out of range");
    }
  }
  if (normals.size() != 0 && normals.size() != vertices.size()) {
    throw std::runtime_error(
        "export mesh failed: normal count does not match vertex count");
  }
  if (uvs.size() != 0 && uvs.size() / 2 != vertices.size() / 3) {
    throw std::runtime_error(
        "export mesh failed: UV count does not match vertex count");
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

  uint32_t nbvertices = vertices.size() / 3;
  scene.mMeshes[0]->mNumVertices = nbvertices;
  scene.mMeshes[0]->mNormals = new aiVector3D[nbvertices];
  scene.mMeshes[0]->mVertices = new aiVector3D[nbvertices];
  scene.mMeshes[0]->mTextureCoords[0] = new aiVector3D[nbvertices];
  for (uint32_t i = 1; i < AI_MAX_NUMBER_OF_TEXTURECOORDS; ++i) {
    scene.mMeshes[0]->mTextureCoords[i] = nullptr;
  }

  for (uint32_t i = 0; i < nbvertices; ++i) {
    scene.mMeshes[0]->mVertices[i] =
        aiVector3D(vertices[3 * i], vertices[3 * i + 1], vertices[3 * i + 2]);
  }
  for (uint32_t i = 0; i < nbvertices; ++i) {
    if (normals.size()) {
      scene.mMeshes[0]->mNormals[i] =
          aiVector3D(normals[3 * i], normals[3 * i + 1], normals[3 * i + 2]);
    } else {
      scene.mMeshes[0]->mNormals[i] = aiVector3D(0, 0, 0);
    }
    if (uvs.size()) {
      scene.mMeshes[0]->mTextureCoords[0][i] =
          aiVector3D(uvs[2 * i], uvs[2 * i + 1], 0);
    } else {
      scene.mMeshes[0]->mTextureCoords[0][i] = aiVector3D(0, 0, 0);
    }
  }

  uint32_t nbfaces = indices.size() / 3;
  scene.mMeshes[0]->mNumFaces = nbfaces;
  scene.mMeshes[0]->mFaces = new aiFace[nbfaces];
  for (uint32_t i = 0; i < nbfaces; ++i) {
    scene.mMeshes[0]->mFaces[i].mNumIndices = 3;
    scene.mMeshes[0]->mFaces[i].mIndices = new uint32_t[3];
    for (uint32_t j = 0; j < 3; ++j) {
      scene.mMeshes[0]->mFaces[i].mIndices[j] = indices[3 * i + j];
    }
  }

  exporter.Export(&scene, formatId, filename);
}

} // namespace svulkan2