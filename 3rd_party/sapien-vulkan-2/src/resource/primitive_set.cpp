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
#include "svulkan2/resource/primitive_set.h"
#include "svulkan2/core/context.h"

namespace svulkan2 {
namespace resource {

SVPrimitiveSet::SVPrimitiveSet(uint32_t capacity) : mVertexCapacity(capacity) {}

void SVPrimitiveSet::setVertexAttribute(std::string const &name,
                                        std::vector<float> const &attrib) {
  mDirty = true;
  mAttributes[name] = attrib;
  if (name == "position") {
    mVertexCount = attrib.size() / 3;
  }
  if (this->mOnDevice) {
    this->uploadToDevice();
  }
}

std::vector<float> const &SVPrimitiveSet::getVertexAttribute(std::string const &name) const {
  if (mAttributes.find(name) == mAttributes.end()) {
    throw std::runtime_error("attribute " + name + " does not exist on vertex");
  }
  return mAttributes.at(name);
}

size_t SVPrimitiveSet::getVertexSize() {
  auto layout = core::Context::Get()->getResourceManager()->getLineVertexLayout();
  return layout->getSize();
}

core::Buffer &SVPrimitiveSet::getVertexBuffer() {
  if (!mVertexBuffer) {
    uploadToDevice();
  }
  return *mVertexBuffer;
}

void SVPrimitiveSet::uploadToDevice() {
  std::scoped_lock lock(mUploadingMutex);

  if (!mDirty) {
    return;
  }
  auto context = core::Context::Get();

  auto layout = context->getResourceManager()->getLineVertexLayout();

  if (mAttributes.find("position") == mAttributes.end() || mAttributes["position"].size() == 0) {
    throw std::runtime_error("primitive set upload failed: no vertex positions");
  }

  size_t vertexSize = layout->getSize();

  if (mVertexCapacity == 0) {
    mVertexCapacity = mVertexCount;
  }

  if (mVertexCapacity < mVertexCount) {
    throw std::runtime_error("failed to upload primitive set: vertex count exceeds capacity");
  }

  size_t bufferSize = vertexSize * mVertexCapacity;

  std::vector<char> buffer(bufferSize, 0);
  auto elements = layout->getElementsSorted();
  uint32_t offset = 0;
  for (auto &elem : elements) {
    if (mAttributes.find(elem.name) != mAttributes.end()) {
      if (mAttributes[elem.name].size() * sizeof(float) > mVertexCapacity * elem.getSize()) {
        throw std::runtime_error("vertex attribute " + elem.name + " has incorrect size");
      }
      auto count = mAttributes[elem.name].size() * sizeof(float) / elem.getSize();
      strided_memcpy(buffer.data() + offset, mAttributes[elem.name].data(), elem.getSize(), count,
                     vertexSize);
    }
    offset += elem.getSize();
  }

  if (!mVertexBuffer) {
    mVertexBuffer = core::Buffer::Create(
        bufferSize,
        vk::BufferUsageFlagBits::eVertexBuffer | vk::BufferUsageFlagBits::eTransferDst |
            vk::BufferUsageFlagBits::eTransferSrc | vk::BufferUsageFlagBits::eStorageBuffer,
        VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY, VmaAllocationCreateFlags{}, true);
  }
  mVertexBuffer->upload(buffer.data(), bufferSize);
  mOnDevice = true;
  mDirty = false;
}

void SVPrimitiveSet::removeFromDevice() {
  mDirty = true;
  mOnDevice = false;
  mVertexBuffer.reset();
}

void SVPointSet::buildBLAS(bool update) {
  if (getVertexCapacity() == 0) {
    throw std::runtime_error("failed to build BLAS: unspecified vertex capacity");
  }
  struct Aabb {
    glm::vec3 min;
    glm::vec3 max;
  };

  if (!mAabbBuffer) {
    mAabbBuffer = core::Buffer::Create(
        getVertexCapacity() * sizeof(Aabb),
        vk::BufferUsageFlagBits::eShaderDeviceAddress |
            vk::BufferUsageFlagBits::eAccelerationStructureBuildInputReadOnlyKHR |
            vk::BufferUsageFlagBits::eStorageBuffer |
            vk::BufferUsageFlagBits::eAccelerationStructureStorageKHR |
            vk::BufferUsageFlagBits::eTransferSrc | vk::BufferUsageFlagBits::eTransferDst,
        VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY, VmaAllocationCreateFlags{}, true);
  }

  std::vector<Aabb> aabbs;
  auto position = mAttributes.at("position");
  auto scale = mAttributes.at("scale");
  int size = position.size() / 3;
  aabbs.reserve(size);
  for (int i = 0; i < size; ++i) {
    glm::vec3 center = glm::vec3(position[3 * i], position[3 * i + 1], position[3 * i + 2]);
    aabbs.push_back({center - glm::vec3(scale.at(i)),
                     center + glm::vec3(scale.at(i))}); // TODO use a proper scale
  }
  mAabbBuffer->upload(aabbs);

  static_assert(sizeof(Aabb) == sizeof(float) * 6);

  vk::AccelerationStructureGeometryAabbsDataKHR data(mAabbBuffer->getAddress(), sizeof(Aabb));
  vk::AccelerationStructureGeometryKHR geom(vk::GeometryTypeKHR::eAabbs, data,
                                            vk::GeometryFlagBitsKHR::eOpaque);
  vk::AccelerationStructureBuildRangeInfoKHR range(getVertexCount(), 0, 0, 0);
  mBLAS = std::make_unique<core::BLAS>(std::vector{geom}, std::vector{range},
                                       std::vector<uint32_t>{getVertexCapacity()}, false, update);
  mBLAS->build();
}

void SVPointSet::recordUpdateBLAS(vk::CommandBuffer commandBuffer) {
  vk::AccelerationStructureBuildRangeInfoKHR range(getVertexCount(), 0, 0, 0);
  mBLAS->recordUpdate(commandBuffer, {range});
}

} // namespace resource
} // namespace svulkan2