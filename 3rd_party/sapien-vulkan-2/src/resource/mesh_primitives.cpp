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
#include "../common/logger.h"
#include "svulkan2/common/assimp.h"
#include "svulkan2/core/context.h"
#include "svulkan2/resource/mesh.h"
#include <memory>

namespace svulkan2 {
namespace resource {
static std::shared_ptr<SVMeshRigid> makeMesh(std::vector<glm::vec3> const &vertices,
                                             std::vector<glm::ivec3> const &indices,
                                             std::vector<glm::vec3> const &normals,
                                             std::vector<glm::vec2> const &uvs) {
  auto mesh = std::make_shared<SVMeshRigid>();
  std::vector<uint32_t> indices_;
  indices_.reserve(3 * indices.size());
  std::vector<float> vertices_;
  vertices_.reserve(3 * vertices.size());
  std::vector<float> normals_;
  normals_.reserve(3 * normals.size());
  std::vector<float> uvs_;
  uvs_.reserve(2 * uvs.size());
  for (auto &index : indices) {
    indices_.push_back(index.x);
    indices_.push_back(index.y);
    indices_.push_back(index.z);
    assert(static_cast<uint32_t>(index.x) < vertices.size() &&
           static_cast<uint32_t>(index.y) < vertices.size() &&
           static_cast<uint32_t>(index.z) < vertices.size());
  }
  for (auto &vertex : vertices) {
    vertices_.push_back(vertex.x);
    vertices_.push_back(vertex.y);
    vertices_.push_back(vertex.z);
  }
  for (auto &normal : normals) {
    normals_.push_back(normal.x);
    normals_.push_back(normal.y);
    normals_.push_back(normal.z);
  }
  for (auto &uv : uvs) {
    uvs_.push_back(uv.x);
    uvs_.push_back(uv.y);
  }

  mesh->setIndices(indices_);
  mesh->setVertexAttribute("position", vertices_);
  mesh->setVertexAttribute("normal", normals_);
  mesh->setVertexAttribute("uv", uvs_);

  // TODO compute tangent and bitangent
  std::vector<float> tangents_(3 * normals.size(), 0.f);
  std::vector<float> bitangents_(3 * normals.size(), 0.f);

  mesh->setVertexAttribute("tangent", tangents_);
  mesh->setVertexAttribute("bitangent", bitangents_);

  return mesh;
}

std::shared_ptr<SVMesh> SVMesh::CreateUVSphere(int segments, int rings) {
  std::vector<glm::vec3> vertices;
  std::vector<glm::vec2> uvs;
  std::vector<glm::ivec3> indices;

  for (int s = 0; s < segments; ++s) {
    vertices.push_back({1.f, 0.f, 0.f});
    uvs.push_back({(0.5f + s) / segments, 1.f});
  }
  for (int r = 1; r < rings; ++r) {
    float theta = glm::pi<float>() * r / rings;
    float x = glm::cos(theta);
    float yz = glm::sin(theta);
    for (int s = 0; s < segments + 1; ++s) {
      float phi = glm::pi<float>() * s * 2 / segments;
      float y = yz * glm::cos(phi);
      float z = yz * glm::sin(phi);
      vertices.push_back({x, y, z});
      uvs.push_back({static_cast<float>(s) / segments, 1.f - static_cast<float>(r) / rings});
    }
  }
  for (int s = 0; s < segments; ++s) {
    vertices.push_back({-1.f, 0.f, 0.f});
    uvs.push_back({(0.5f + s) / segments, 0.f});
  }

  for (int s = 0; s < segments; ++s) {
    indices.push_back({s, s + segments, s + segments + 1});
  }

  for (int r = 0; r < rings - 2; ++r) {
    for (int s = 0; s < segments; ++s) {
      indices.push_back({
          segments + (segments + 1) * r + s,
          segments + (segments + 1) * (r + 1) + s,
          segments + (segments + 1) * (r + 1) + s + 1,
      });
      indices.push_back({segments + (segments + 1) * r + s,
                         segments + (segments + 1) * (r + 1) + s + 1,
                         segments + (segments + 1) * r + s + 1});
    }
  }
  for (int s = 0; s < segments; ++s) {
    indices.push_back({segments + (segments + 1) * (rings - 2) + s,
                       segments + (segments + 1) * (rings - 1) + s,
                       segments + (segments + 1) * (rings - 2) + s + 1});
  }

  return makeMesh(vertices, indices, vertices, uvs);
}

std::shared_ptr<SVMesh> SVMesh::CreateCapsule(float radius, float halfLength, int segments,
                                              int halfRings) {
  std::vector<glm::vec3> vertices;
  std::vector<glm::vec3> normals;
  std::vector<glm::vec2> uvs;
  std::vector<glm::ivec3> indices;

  for (int s = 0; s < segments; ++s) {
    vertices.push_back({radius + halfLength, 0.f, 0.f});
    normals.push_back({1.f, 0.f, 0.f});
    uvs.push_back({(0.5f + s) / segments, 1.f});
  }
  int rings = 2 * halfRings;
  for (int r = 1; r <= halfRings; ++r) {
    float theta = glm::pi<float>() * r / rings;
    float x = glm::cos(theta);
    float yz = glm::sin(theta);
    for (int s = 0; s < segments + 1; ++s) {
      float phi = glm::pi<float>() * s * 2 / segments;
      float y = yz * glm::cos(phi);
      float z = yz * glm::sin(phi);
      vertices.push_back(glm::vec3{x, y, z} * radius + glm::vec3{halfLength, 0, 0});
      normals.push_back({x, y, z});
      uvs.push_back(
          {static_cast<float>(s) / segments, 1.f - 0.5f * static_cast<float>(r) / rings});
    }
  }
  for (int r = halfRings; r < rings; ++r) {
    float theta = glm::pi<float>() * r / rings;
    float x = glm::cos(theta);
    float yz = glm::sin(theta);
    for (int s = 0; s < segments + 1; ++s) {
      float phi = glm::pi<float>() * s * 2 / segments;
      float y = yz * glm::cos(phi);
      float z = yz * glm::sin(phi);
      vertices.push_back(glm::vec3{x, y, z} * radius - glm::vec3{halfLength, 0, 0});
      normals.push_back({x, y, z});
      uvs.push_back(
          {static_cast<float>(s) / segments, 0.5f - 0.5f * static_cast<float>(r) / rings});
    }
  }

  for (int s = 0; s < segments; ++s) {
    vertices.push_back({-radius - halfLength, 0.f, 0.f});
    normals.push_back({-1.f, 0.f, 0.f});
    uvs.push_back({(0.5f + s) / segments, 0.f});
  }

  for (int s = 0; s < segments; ++s) {
    indices.push_back({s, s + segments, s + segments + 1});
  }

  for (int r = 0; r < rings - 1; ++r) {
    for (int s = 0; s < segments; ++s) {
      indices.push_back({
          segments + (segments + 1) * r + s,
          segments + (segments + 1) * (r + 1) + s,
          segments + (segments + 1) * (r + 1) + s + 1,
      });
      indices.push_back({segments + (segments + 1) * r + s,
                         segments + (segments + 1) * (r + 1) + s + 1,
                         segments + (segments + 1) * r + s + 1});
    }
  }
  for (int s = 0; s < segments; ++s) {
    indices.push_back({segments + (segments + 1) * (rings - 1) + s,
                       segments + (segments + 1) * (rings) + s,
                       segments + (segments + 1) * (rings - 1) + s + 1});
  }

  return makeMesh(vertices, indices, normals, uvs);
}

std::shared_ptr<SVMesh> SVMesh::CreateCone(int segments) {
  std::vector<glm::vec3> vertices;
  std::vector<glm::vec3> normals;
  std::vector<glm::vec2> uvs;
  std::vector<glm::ivec3> indices;

  float s22 = glm::sqrt(2.f) / 2.f;
  for (int s = 0; s < segments; ++s) {
    vertices.push_back({1, 0, 0});
    float theta = glm::pi<float>() * 2.f * (s + 0.5f) / segments;
    normals.push_back({s22 * glm::cos(theta), s22 * glm::sin(theta), s22});
    uvs.push_back({0.25f, 0.5f});
  }
  for (int s = 0; s < segments; ++s) {
    float theta = glm::pi<float>() * 2.f * s / segments;
    vertices.push_back({0.f, glm::cos(theta), glm::sin(theta)});
    normals.push_back({s22 * glm::cos(theta), s22 * glm::sin(theta), s22});
    uvs.push_back({glm::cos(theta) * 0.25 + 0.25, glm::sin(theta) * 0.5 + 0.5});
  }
  for (int s = 0; s < segments; ++s) {
    float theta = glm::pi<float>() * 2.f * s / segments;
    vertices.push_back({0.f, glm::cos(theta), glm::sin(theta)});
    normals.push_back({-1.f, 0.f, 0.f});
    uvs.push_back({glm::cos(theta) * 0.25 + 0.75, glm::sin(theta) * 0.5 + 0.5});
  }
  vertices.push_back({0.f, 0.f, 0.f});
  normals.push_back({-1.f, 0.f, 0.f});
  uvs.push_back({0.75f, 0.5f});

  for (int s = 0; s < segments; ++s) {
    indices.push_back({s, s + segments, segments + (s + 1) % segments});
  }
  for (int s = 0; s < segments; ++s) {
    indices.push_back({segments * 2 + (s + 1) % segments, segments * 2 + s, segments * 3});
  }

  return makeMesh(vertices, indices, normals, uvs);
}

std::shared_ptr<SVMesh> SVMesh::CreateCube() {
  std::vector<glm::vec3> vertices;
  std::vector<glm::vec3> normals;
  std::vector<glm::vec2> uvs;
  std::vector<glm::ivec3> indices;

  // +Z
  vertices.push_back({-1.f, 1.f, 1.f});
  vertices.push_back({-1.f, -1.f, 1.f});
  vertices.push_back({1.f, -1.f, 1.f});
  vertices.push_back({1.f, 1.f, 1.f});
  for (int i = 0; i < 4; ++i) {
    normals.push_back({0.f, 0.f, 1.f});
  }
  uvs.push_back({1.f / 4.f, 2.f / 3.f});
  uvs.push_back({1.f / 4.f, 1.f / 3.f});
  uvs.push_back({2.f / 4.f, 1.f / 3.f});
  uvs.push_back({2.f / 4.f, 2.f / 3.f});

  // -Z
  vertices.push_back({1.f, 1.f, -1.f});
  vertices.push_back({1.f, -1.f, -1.f});
  vertices.push_back({-1.f, -1.f, -1.f});
  vertices.push_back({-1.f, 1.f, -1.f});
  for (int i = 0; i < 4; ++i) {
    normals.push_back({0.f, 0.f, -1.f});
  }
  uvs.push_back({3.f / 4.f, 2.f / 3.f});
  uvs.push_back({3.f / 4.f, 1.f / 3.f});
  uvs.push_back({4.f / 4.f, 1.f / 3.f});
  uvs.push_back({4.f / 4.f, 2.f / 3.f});

  // +X
  vertices.push_back({1.f, 1.f, 1.f});
  vertices.push_back({1.f, -1.f, 1.f});
  vertices.push_back({1.f, -1.f, -1.f});
  vertices.push_back({1.f, 1.f, -1.f});
  for (int i = 0; i < 4; ++i) {
    normals.push_back({1.f, 0.f, 0.f});
  }
  uvs.push_back({2.f / 4.f, 2.f / 3.f});
  uvs.push_back({2.f / 4.f, 1.f / 3.f});
  uvs.push_back({3.f / 4.f, 1.f / 3.f});
  uvs.push_back({3.f / 4.f, 2.f / 3.f});

  // -X
  vertices.push_back({-1.f, 1.f, -1.f});
  vertices.push_back({-1.f, -1.f, -1.f});
  vertices.push_back({-1.f, -1.f, 1.f});
  vertices.push_back({-1.f, 1.f, 1.f});
  for (int i = 0; i < 4; ++i) {
    normals.push_back({-1.f, 0.f, 0.f});
  }
  uvs.push_back({0.f / 4.f, 2.f / 3.f});
  uvs.push_back({0.f / 4.f, 1.f / 3.f});
  uvs.push_back({1.f / 4.f, 1.f / 3.f});
  uvs.push_back({1.f / 4.f, 2.f / 3.f});

  // +Y
  vertices.push_back({-1.f, 1.f, -1.f});
  vertices.push_back({-1.f, 1.f, 1.f});
  vertices.push_back({1.f, 1.f, 1.f});
  vertices.push_back({1.f, 1.f, -1.f});
  for (int i = 0; i < 4; ++i) {
    normals.push_back({0.f, 1.f, 0.f});
  }
  uvs.push_back({1.f / 4.f, 3.f / 3.f});
  uvs.push_back({1.f / 4.f, 2.f / 3.f});
  uvs.push_back({2.f / 4.f, 2.f / 3.f});
  uvs.push_back({2.f / 4.f, 3.f / 3.f});

  // -Y
  vertices.push_back({-1.f, -1.f, 1.f});
  vertices.push_back({-1.f, -1.f, -1.f});
  vertices.push_back({1.f, -1.f, -1.f});
  vertices.push_back({1.f, -1.f, 1.f});
  for (int i = 0; i < 4; ++i) {
    normals.push_back({0.f, -1.f, 0.f});
  }
  uvs.push_back({1.f / 4.f, 1.f / 3.f});
  uvs.push_back({1.f / 4.f, 0.f / 3.f});
  uvs.push_back({2.f / 4.f, 0.f / 3.f});
  uvs.push_back({2.f / 4.f, 1.f / 3.f});

  indices.push_back({0, 1, 2});
  indices.push_back({0, 2, 3});
  indices.push_back({4, 5, 6});
  indices.push_back({4, 6, 7});
  indices.push_back({8, 9, 10});
  indices.push_back({8, 10, 11});
  indices.push_back({12, 13, 14});
  indices.push_back({12, 14, 15});
  indices.push_back({16, 17, 18});
  indices.push_back({16, 18, 19});
  indices.push_back({20, 21, 22});
  indices.push_back({20, 22, 23});

  return makeMesh(vertices, indices, normals, uvs);
}

std::shared_ptr<SVMesh> SVMesh::CreateYZPlane() {
  std::vector<glm::vec3> vertices;
  std::vector<glm::vec3> normals;
  std::vector<glm::vec2> uvs;
  std::vector<glm::ivec3> indices;

  vertices.push_back({0.f, 1.f, 1.f});
  vertices.push_back({0.f, -1.f, 1.f});
  vertices.push_back({0.f, -1.f, -1.f});
  vertices.push_back({0.f, 1.f, -1.f});
  for (int i = 0; i < 4; ++i) {
    normals.push_back({1.f, 0.f, 0.f});
  }
  uvs.push_back({0.f, 1.f});
  uvs.push_back({0.f, 0.f});
  uvs.push_back({1.f, 0.f});
  uvs.push_back({1.f, 1.f});

  indices.push_back({0, 1, 2});
  indices.push_back({0, 2, 3});

  return makeMesh(vertices, indices, normals, uvs);
}

std::shared_ptr<SVMesh> SVMesh::CreateCylinder(int segments) {
  std::vector<glm::vec3> vertices;
  std::vector<glm::vec3> normals;
  std::vector<glm::vec2> uvs;
  std::vector<glm::ivec3> indices;

  // circle 1
  vertices.push_back({1.f, 0.f, 0.f});
  normals.push_back({1.f, 0.f, 0.f});
  uvs.push_back({0, 0}); // TODO
  float step = glm::pi<float>() * 2.f / segments;
  for (int i = 0; i < segments; ++i) {
    vertices.push_back({1.f, glm::cos(step * i), glm::sin(step * i)});
    normals.push_back({1.f, 0.f, 0.f});
    uvs.push_back({0, 0}); // TODO
  }
  for (int i = 0; i < segments; ++i) {
    indices.push_back({0, i + 1, (i + 1) % segments + 1});
  }

  // circle 2
  vertices.push_back({-1.f, 0.f, 0.f});
  normals.push_back({-1.f, 0.f, 0.f});
  uvs.push_back({0, 0}); // TODO
  for (int i = 0; i < segments; ++i) {
    vertices.push_back({-1.f, glm::cos(step * i), glm::sin(step * i)});
    normals.push_back({-1.f, 0.f, 0.f});
    uvs.push_back({0, 0}); // TODO
  }
  for (int i = 0; i < segments; ++i) {
    indices.push_back({segments + 1, (i + 1) % segments + segments + 2, i + segments + 2});
  }

  int base = segments * 2 + 2;
  // make 2 rings
  for (int i = 0; i < segments; ++i) {
    vertices.push_back({1.f, glm::cos(step * i), glm::sin(step * i)});
    normals.push_back({0.f, glm::cos(step * i), glm::sin(step * i)});
    uvs.push_back({0, 0}); // TODO
  }

  for (int i = 0; i < segments; ++i) {
    vertices.push_back({-1.f, glm::cos(step * i), glm::sin(step * i)});
    normals.push_back({0.f, glm::cos(step * i), glm::sin(step * i)});
    uvs.push_back({0, 0}); // TODO
  }

  // connect 2 rings
  for (int i = 0; i < segments; ++i) {
    indices.push_back({base + i, base + i + segments, base + (i + 1) % segments});
    indices.push_back(
        {base + (i + 1) % segments, base + i + segments, base + segments + (i + 1) % segments});
  }

  return makeMesh(vertices, indices, normals, uvs);
}

std::shared_ptr<SVMesh> SVMesh::Create(std::vector<float> const &position,
                                       std::vector<uint32_t> const &index) {
  auto mesh = std::make_shared<SVMeshRigid>();
  mesh->setIndices(index);
  mesh->setVertexAttribute("position", position);
  return mesh;
}

} // namespace resource
} // namespace svulkan2