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
#pragma once
#include "svulkan2/common/layout.h"
#include "svulkan2/common/vk.h"
#include "svulkan2/core/buffer.h"
#include <mutex>
#include <unordered_map>

namespace svulkan2 {
namespace resource {
class SVMeshRigid;

class SVMesh {
public:
  uint32_t getVertexSize() const;
  uint32_t getVertexCount() const { return mVertexCount; }
  uint32_t getTriangleCount() const { return mTriangleCount; }

  bool isOnDevice() const { return mVertexBuffer != nullptr; }
  core::Buffer &getVertexBuffer();
  core::Buffer &getIndexBuffer();

  virtual void setIndices(std::vector<uint32_t> const &indices) = 0;
  virtual std::vector<uint32_t> const &getIndices() const = 0;

  virtual void setVertexAttribute(std::string const &name, std::vector<float> const &attrib) = 0;
  virtual std::vector<float> const &getVertexAttribute(std::string const &name) const = 0;
  virtual bool hasVertexAttribute(std::string const &name) const = 0;

  virtual void uploadToDevice() = 0;
  virtual void removeFromDevice() = 0;

  vk::AccelerationStructureGeometryKHR getASGeometry();

  static std::shared_ptr<SVMesh> CreateUVSphere(int segments, int rings);
  static std::shared_ptr<SVMesh> CreateCapsule(float radius, float halfLength, int segments,
                                                    int halfRings);
  static std::shared_ptr<SVMesh> CreateCone(int segments);
  static std::shared_ptr<SVMesh> CreateCube();
  static std::shared_ptr<SVMesh> CreateYZPlane();
  static std::shared_ptr<SVMesh> CreateCylinder(int segments);
  static std::shared_ptr<SVMesh> Create(std::vector<float> const &position,
                                             std::vector<uint32_t> const &index);

protected:
  size_t mTriangleCount{};
  size_t mVertexCount{};

  std::unique_ptr<core::Buffer> mVertexBuffer;
  std::unique_ptr<core::Buffer> mIndexBuffer;
  std::mutex mUploadingMutex;
};

class SVMeshRigid : public SVMesh {
public:
  SVMeshRigid();

  void setIndices(std::vector<uint32_t> const &indices) override;
  std::vector<uint32_t> const &getIndices() const override;

  void setVertexAttribute(std::string const &name, std::vector<float> const &attrib) override;
  std::vector<float> const &getVertexAttribute(std::string const &name) const override;
  bool hasVertexAttribute(std::string const &name) const override;

  void uploadToDevice() override;
  void removeFromDevice() override;

  uint32_t getTriangleCount();
  // void exportToFile(std::string const &filename) const;

private:
  std::vector<uint32_t> mIndices;
  std::unordered_map<std::string, std::vector<float>> mAttributes;
};

class SVMeshDeformable : public SVMesh {
public:
  SVMeshDeformable(uint32_t maxVertexCount = 0, uint32_t maxTriangleCount = 0);

  uint32_t getMaxVertexCount() { return mMaxVertexCount; }
  uint32_t getMaxTriangleCount() { return mMaxTriangleCount; }

  void setVertexCount(uint32_t vertexCount);
  void setTriangleCount(uint32_t triangleCount);

  void setIndices(std::vector<uint32_t> const &indices) override;
  std::vector<uint32_t> const &getIndices() const override;

  void setVertexAttribute(std::string const &name, std::vector<float> const &attrib) override;
  std::vector<float> const &getVertexAttribute(std::string const &name) const override;
  bool hasVertexAttribute(std::string const &name) const override;

  void uploadToDevice() override;
  void removeFromDevice() override;

private:
  uint32_t mMaxVertexCount{};
  uint32_t mMaxTriangleCount{};

  std::vector<uint32_t> mIndices;
  std::unordered_map<std::string, std::vector<float>> mAttributes;
};

} // namespace resource
} // namespace svulkan2