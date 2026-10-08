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
#include "svulkan2/resource/mesh.h"
#include "../common/logger.h"
#include "svulkan2/common/assimp.h"
#include "svulkan2/core/context.h"
#include <memory>

namespace svulkan2 {
namespace resource {

uint32_t SVMesh::getVertexSize() const {
  return core::Context::Get()->getResourceManager()->getVertexLayout()->getSize();
}

core::Buffer &SVMesh::getVertexBuffer() {
  if (!mVertexBuffer) {
    uploadToDevice();
  }
  return *mVertexBuffer;
}

core::Buffer &SVMesh::getIndexBuffer() {
  if (!mIndexBuffer) {
    uploadToDevice();
  }
  return *mIndexBuffer;
}

vk::AccelerationStructureGeometryKHR SVMesh::getASGeometry() {
  auto context = core::Context::Get();
  uploadToDevice();

  vk::DeviceAddress vertexAddress =
      context->getDevice().getBufferAddress({mVertexBuffer->getVulkanBuffer()});
  vk::DeviceAddress indexAddress =
      context->getDevice().getBufferAddress({mIndexBuffer->getVulkanBuffer()});

  vk::AccelerationStructureGeometryTrianglesDataKHR trianglesData(
      vk::Format::eR32G32B32Sfloat, vertexAddress,
      context->getResourceManager()->getVertexLayout()->getSize(), mVertexCount,
      vk::IndexType::eUint32, indexAddress, {});

  vk::AccelerationStructureGeometryKHR geom(vk::GeometryTypeKHR::eTriangles, trianglesData,
                                            {}); // TODO: add opaque flag?
  return geom;
}

SVMeshRigid::SVMeshRigid() {}

void SVMeshRigid::setIndices(std::vector<uint32_t> const &indices) {
  if (mIndexBuffer) {
    throw std::runtime_error("set indices failed: mesh already on device");
  }

  if (indices.size() % 3 != 0) {
    throw std::runtime_error("set indices failed: provided number is not a multiple of 3");
  }
  mIndices = indices;
  mTriangleCount = indices.size() / 3;
}

std::vector<uint32_t> const &SVMeshRigid::getIndices() const { return mIndices; }

void SVMeshRigid::setVertexAttribute(std::string const &name, std::vector<float> const &attrib) {
  if (mVertexBuffer) {
    throw std::runtime_error("set indices failed: mesh already on device");
  }
  mAttributes[name] = attrib;

  if (name == "position") {
    if (attrib.size() % 3 != 0) {
      throw std::runtime_error("failed to set attribute position");
    }
    mVertexCount = attrib.size() / 3;
  }
}

std::vector<float> const &SVMeshRigid::getVertexAttribute(std::string const &name) const {
  if (mAttributes.find(name) == mAttributes.end()) {
    throw std::runtime_error("failed to get vertex attribute: attribute " + name +
                             " does not exist");
  }
  return mAttributes.at(name);
}

bool SVMeshRigid::hasVertexAttribute(std::string const &name) const {
  return mAttributes.find(name) != mAttributes.end();
}

void SVMeshRigid::uploadToDevice() {
  std::scoped_lock lock(mUploadingMutex);
  if (mVertexBuffer) {
    return;
  }
  auto context = core::Context::Get();

  auto layout = context->getResourceManager()->getVertexLayout();

  if (!mAttributes.contains("position") || mAttributes["position"].size() == 0) {
    throw std::runtime_error("mesh upload failed: no vertex positions");
  }
  if (!mIndices.size()) {
    throw std::runtime_error("mesh upload failed: empty vertex indices");
  }

  size_t vertexSize = layout->getSize();
  size_t bufferSize = vertexSize * mVertexCount;
  size_t indexBufferSize = sizeof(uint32_t) * mTriangleCount * 3;

  std::vector<char> buffer(bufferSize, 0);
  auto elements = layout->getElementsSorted();
  uint32_t offset = 0;
  for (auto &elem : elements) {
    if (mAttributes.find(elem.name) != mAttributes.end()) {
      if (mAttributes[elem.name].size() * sizeof(float) != mVertexCount * elem.getSize()) {
        throw std::runtime_error("vertex attribute " + elem.name + " has incorrect size");
      }
      strided_memcpy(buffer.data() + offset, mAttributes[elem.name].data(), elem.getSize(),
                     mVertexCount, vertexSize);
    }
    offset += elem.getSize();
  }

  if (!mVertexBuffer) {
    vk::BufferUsageFlags deviceAddressFlag =
        context->isRayTracingAvailable()
            ? vk::BufferUsageFlagBits::eShaderDeviceAddress |
                  vk::BufferUsageFlagBits::eAccelerationStructureBuildInputReadOnlyKHR |
                  vk::BufferUsageFlagBits::eStorageBuffer
            : vk::BufferUsageFlags{};

    mVertexBuffer = core::Buffer::Create(
        bufferSize,
        deviceAddressFlag | vk::BufferUsageFlagBits::eVertexBuffer |
            vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eTransferSrc,
        VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY, VmaAllocationCreateFlags{}, true);
    mIndexBuffer = core::Buffer::Create(
        indexBufferSize,
        deviceAddressFlag | vk::BufferUsageFlagBits::eIndexBuffer |
            vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eTransferSrc,
        VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY, VmaAllocationCreateFlags{}, true);
  }
  mVertexBuffer->upload(buffer.data(), vertexSize * mVertexCount);
  mIndexBuffer->upload<uint32_t>(mIndices);
}

void SVMeshRigid::removeFromDevice() {
  mVertexBuffer.reset();
  mIndexBuffer.reset();
}

// void SVMeshRigid::exportToFile(std::string const &filename) const {
//   exportTriangleMesh(filename,
//                      mAttributes.find("position") != mAttributes.end() ?
//                      mAttributes.at("position")
//                                                                        : std::vector<float>{},
//                      mIndices,
//                      mAttributes.find("normal") != mAttributes.end() ? mAttributes.at("normal")
//                                                                      : std::vector<float>{},
//                      mAttributes.find("uv") != mAttributes.end() ? mAttributes.at("uv")
//                                                                  : std::vector<float>{});
// }

//==================== Deformable ====================//

SVMeshDeformable::SVMeshDeformable(uint32_t maxVertexCount, uint32_t maxTriangleCount)
    : mMaxVertexCount(maxVertexCount), mMaxTriangleCount(maxTriangleCount) {}

void SVMeshDeformable::setVertexCount(uint32_t vertexCount) {
  if (mMaxVertexCount == 0) {
    mMaxVertexCount = vertexCount;
  }
  if (vertexCount > mMaxVertexCount) {
    throw std::runtime_error("failed to set vertex count: it exceeds max vertex count.");
  }
  mVertexCount = vertexCount;
}

void SVMeshDeformable::setTriangleCount(uint32_t triangleCount) {
  if (mMaxTriangleCount == 0) {
    mMaxVertexCount = triangleCount;
  }
  if (triangleCount > mMaxTriangleCount) {
    throw std::runtime_error("failed to set triangle count: it exceeds max triangle count.");
  }
  mTriangleCount = triangleCount;
}

void SVMeshDeformable::setIndices(std::vector<uint32_t> const &indices) {
  throw std::runtime_error("not implemented");
}
std::vector<uint32_t> const &SVMeshDeformable::getIndices() const {
  throw std::runtime_error("not implemented");
}

void SVMeshDeformable::setVertexAttribute(std::string const &name,
                                          std::vector<float> const &attrib) {
  throw std::runtime_error("not implemented");
}
std::vector<float> const &SVMeshDeformable::getVertexAttribute(std::string const &name) const {
  throw std::runtime_error("not implemented");
}
bool SVMeshDeformable::hasVertexAttribute(std::string const &name) const {
  throw std::runtime_error("not implemented");
}

void SVMeshDeformable::uploadToDevice() {
  std::scoped_lock lock(mUploadingMutex);
  auto context = core::Context::Get();
  auto layout = context->getResourceManager()->getVertexLayout();
  size_t vertexSize = layout->getSize();

  if (mMaxVertexCount == 0 || mMaxTriangleCount == 0) {
    throw std::runtime_error("failed to upload mesh: vertex or index capacity is 0");
  }

  size_t bufferSize = vertexSize * mMaxVertexCount;
  size_t indexBufferSize = sizeof(uint32_t) * mMaxTriangleCount * 3;

  if (!mVertexBuffer) {
    vk::BufferUsageFlags deviceAddressFlag =
        context->isRayTracingAvailable()
            ? vk::BufferUsageFlagBits::eShaderDeviceAddress |
                  vk::BufferUsageFlagBits::eAccelerationStructureBuildInputReadOnlyKHR |
                  vk::BufferUsageFlagBits::eStorageBuffer
            : vk::BufferUsageFlags{};

    mVertexBuffer = core::Buffer::Create(
        bufferSize,
        deviceAddressFlag | vk::BufferUsageFlagBits::eVertexBuffer |
            vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eTransferSrc,
        VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY, VmaAllocationCreateFlags{}, true);
    mIndexBuffer = core::Buffer::Create(
        indexBufferSize,
        deviceAddressFlag | vk::BufferUsageFlagBits::eIndexBuffer |
            vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eTransferSrc,
        VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY, VmaAllocationCreateFlags{}, true);
  }
}

void SVMeshDeformable::removeFromDevice() { throw std::runtime_error("not implemented"); }

} // namespace resource
} // namespace svulkan2