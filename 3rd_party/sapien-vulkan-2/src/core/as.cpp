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
#include "svulkan2/core/as.h"
#include "../common/logger.h"
#include "svulkan2/core/context.h"

namespace svulkan2 {
namespace core {

BLAS::BLAS(std::vector<vk::AccelerationStructureGeometryKHR> const &geometries,
           std::vector<vk::AccelerationStructureBuildRangeInfoKHR> const &buildRanges,
           std::vector<uint32_t> const &maxPrimitiveCounts, bool compaction, bool update)
    : mGeometries(geometries), mBuildRanges(buildRanges), mCompaction(compaction),
      mUpdate(update) {
  if (compaction && update) {
    logger::error(
        "Current implementation does not allow compaction for dynamic acceleration structures.");
    mCompaction = false;
  }

  if (maxPrimitiveCounts.empty()) {
    for (auto &range : mBuildRanges) {
      mMaxPrimitiveCounts.push_back(range.primitiveCount);
    }
  } else {
    // sanity checks
    if (maxPrimitiveCounts.size() != buildRanges.size()) {
      throw std::runtime_error("buildRanges and maxPrimitiveCounts must have equal size");
    }
    for (size_t i = 0; i < maxPrimitiveCounts.size(); ++i) {
      if (maxPrimitiveCounts[i] < buildRanges[i].primitiveCount) {
        throw std::runtime_error(
            "maxPrimitiveCounts must be greater or equal to primitive count in buildRanges");
      }
    }
    mMaxPrimitiveCounts = maxPrimitiveCounts;
  }
}

void BLAS::build() {
  auto context = Context::Get();

  vk::BuildAccelerationStructureFlagsKHR flags{};
  if (mCompaction) {
    flags |= vk::BuildAccelerationStructureFlagBitsKHR::eAllowCompaction;
  }
  if (mUpdate) {
    flags |= vk::BuildAccelerationStructureFlagBitsKHR::eAllowUpdate;
  }

  vk::AccelerationStructureBuildGeometryInfoKHR buildInfo(
      vk::AccelerationStructureTypeKHR::eBottomLevel, flags,
      vk::BuildAccelerationStructureModeKHR::eBuild, nullptr, vk::AccelerationStructureKHR{},
      mGeometries, nullptr, vk::DeviceOrHostAddressKHR{});
  vk::AccelerationStructureBuildSizesInfoKHR sizeInfo =
      context->getDevice().getAccelerationStructureBuildSizesKHR(
          vk::AccelerationStructureBuildTypeKHR::eDevice, buildInfo, mMaxPrimitiveCounts);
  vk::DeviceSize size = sizeInfo.accelerationStructureSize;

  auto asBuffer = core::Buffer::Create(sizeInfo.accelerationStructureSize,
                                       vk::BufferUsageFlagBits::eAccelerationStructureStorageKHR |
                                           vk::BufferUsageFlagBits::eShaderDeviceAddress,
                                       VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
  vk::AccelerationStructureCreateInfoKHR createInfo(
      {}, asBuffer->getVulkanBuffer(), {}, sizeInfo.accelerationStructureSize,
      vk::AccelerationStructureTypeKHR::eBottomLevel, {});
  auto blas = context->getDevice().createAccelerationStructureKHRUnique(createInfo);

  auto scratchBuffer = core::Buffer::Create(
      sizeInfo.buildScratchSize,
      vk::BufferUsageFlagBits::eAccelerationStructureStorageKHR |
          vk::BufferUsageFlagBits::eShaderDeviceAddress | vk::BufferUsageFlagBits::eStorageBuffer,
      VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY, VmaAllocationCreateFlags{}, false,
      Context::Get()->getAllocator().getRTPool());

  vk::DeviceAddress scratchAddress =
      context->getDevice().getBufferAddress({scratchBuffer->getVulkanBuffer()});

  auto queryPool = context->getDevice().createQueryPoolUnique(
      vk::QueryPoolCreateInfo({}, vk::QueryType::eAccelerationStructureCompactedSizeKHR, 1));
  auto commandPool = context->createCommandPool();
  auto commandBuffer = commandPool->allocateCommandBuffer();

  buildInfo.scratchData.setDeviceAddress(scratchAddress);
  buildInfo.setDstAccelerationStructure(blas.get());

  commandBuffer->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
  commandBuffer->buildAccelerationStructuresKHR(buildInfo, mBuildRanges.data());
  commandBuffer->resetQueryPool(queryPool.get(), 0, 1);

  if (mCompaction) {
    commandBuffer->writeAccelerationStructuresPropertiesKHR(
        blas.get(), vk::QueryType::eAccelerationStructureCompactedSizeKHR, queryPool.get(), 0);
  }

  commandBuffer->end();
  context->getQueue().submitAndWait(commandBuffer.get());

  if (mUpdate) {
    buildInfo.setMode(vk::BuildAccelerationStructureModeKHR::eUpdate);
    auto updateSizeInfo = context->getDevice().getAccelerationStructureBuildSizesKHR(
        vk::AccelerationStructureBuildTypeKHR::eDevice, buildInfo, mMaxPrimitiveCounts);
    mUpdateScratchBuffer =
        core::Buffer::Create(updateSizeInfo.updateScratchSize,
                             vk::BufferUsageFlagBits::eAccelerationStructureStorageKHR |
                                 vk::BufferUsageFlagBits::eShaderDeviceAddress |
                                 vk::BufferUsageFlagBits::eStorageBuffer,
                             VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY, VmaAllocationCreateFlags{},
                             false, Context::Get()->getAllocator().getRTPool());
    mUpdateScratchBufferAddress = mUpdateScratchBuffer->getAddress();
    logger::info("TLAS size {}, build scratch size {}, update scratch size {}",
                 sizeInfo.accelerationStructureSize, sizeInfo.buildScratchSize,
                 updateSizeInfo.updateScratchSize);
  }

  // find compact size
  // if compact size > original size, disable compaction
  vk::DeviceSize compactSize{0};
  if (mCompaction) {
    auto result = context->getDevice().getQueryPoolResults(
        queryPool.get(), 0, 1, sizeof(vk::DeviceSize), &compactSize, sizeof(vk::DeviceSize),
        vk::QueryResultFlagBits::eWait);
    if (result != vk::Result::eSuccess) {
      throw std::runtime_error("failed to get query pool result");
    }
    logger::info("BLAS original size {}, compact size {}", size, compactSize);
    if (compactSize > size) {
      logger::warn("compact size is greater than original size, aborting copmaction");
      mCompaction = false;
    }
  }

  if (mCompaction) {
    auto compactionCommandBuffer = commandPool->allocateCommandBuffer();
    compactionCommandBuffer->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
    auto asBufferCompact =
        core::Buffer::Create(compactSize,
                             vk::BufferUsageFlagBits::eAccelerationStructureStorageKHR |
                                 vk::BufferUsageFlagBits::eShaderDeviceAddress,
                             VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
    auto blasCompact = context->getDevice().createAccelerationStructureKHRUnique(
        vk::AccelerationStructureCreateInfoKHR(
            {}, asBufferCompact->getVulkanBuffer(), {}, compactSize,
            vk::AccelerationStructureTypeKHR::eBottomLevel, {}));
    vk::CopyAccelerationStructureInfoKHR copyInfo(blas.get(), blasCompact.get(),
                                                  vk::CopyAccelerationStructureModeKHR::eCompact);
    compactionCommandBuffer->copyAccelerationStructureKHR(copyInfo);
    compactionCommandBuffer->end();
    context->getQueue().submitAndWait(compactionCommandBuffer.get());
    mBuffer = std::move(asBufferCompact);
    mAS = std::move(blasCompact);
  } else {
    mBuffer = std::move(asBuffer);
    mAS = std::move(blas);
  }
}

void BLAS::recordUpdate(
    vk::CommandBuffer commandBuffer,
    std::vector<vk::AccelerationStructureBuildRangeInfoKHR> const &buildRanges) {
  if (!mUpdate) {
    logger::error("BLAS is not built to allow update");
  }
  vk::MemoryBarrier barrier(vk::AccessFlagBits::eTransferWrite,
                            vk::AccessFlagBits::eAccelerationStructureWriteKHR);
  commandBuffer.pipelineBarrier(vk::PipelineStageFlagBits::eTransfer,
                                vk::PipelineStageFlagBits::eAccelerationStructureBuildKHR, {},
                                barrier, nullptr, nullptr);
  vk::AccelerationStructureBuildGeometryInfoKHR buildInfo(
      vk::AccelerationStructureTypeKHR::eBottomLevel,
      vk::BuildAccelerationStructureFlagBitsKHR::eAllowUpdate,
      vk::BuildAccelerationStructureModeKHR::eUpdate, mAS.get(), mAS.get(), mGeometries, nullptr,
      mUpdateScratchBufferAddress);
  commandBuffer.buildAccelerationStructuresKHR(buildInfo, buildRanges.data());
}

vk::DeviceAddress BLAS::getAddress() {
  auto context = core::Context::Get();
  return context->getDevice().getAccelerationStructureAddressKHR({mAS.get()});
}

TLAS::TLAS(std::vector<vk::AccelerationStructureInstanceKHR> const &instances)
    : mInstances(instances) {}

void TLAS::build() {
  auto context = Context::Get();
  uint32_t instanceCount = static_cast<uint32_t>(mInstances.size());

  mInstanceBuffer = core::Buffer::Create(
      sizeof(vk::AccelerationStructureInstanceKHR) * std::max(1u, instanceCount),
      vk::BufferUsageFlagBits::eShaderDeviceAddress |
          vk::BufferUsageFlagBits::eAccelerationStructureBuildInputReadOnlyKHR,
      VmaMemoryUsage::VMA_MEMORY_USAGE_CPU_TO_GPU);
  mInstanceBuffer->upload(mInstances);

  auto commandPool = context->createCommandPool();
  auto commandBuffer = commandPool->allocateCommandBuffer();
  commandBuffer->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});

  vk::MemoryBarrier barrier(vk::AccessFlagBits::eTransferWrite,
                            vk::AccessFlagBits::eAccelerationStructureWriteKHR);
  commandBuffer->pipelineBarrier(vk::PipelineStageFlagBits::eTransfer,
                                 vk::PipelineStageFlagBits::eAccelerationStructureBuildKHR, {},
                                 barrier, nullptr, nullptr);

  mInstanceBufferAddress = mInstanceBuffer->getAddress();
  vk::AccelerationStructureGeometryInstancesDataKHR instancesData(VK_FALSE,
                                                                  mInstanceBufferAddress);
  vk::AccelerationStructureGeometryKHR tlasGeometry(vk::GeometryTypeKHR::eInstances, instancesData,
                                                    {});
  vk::AccelerationStructureBuildGeometryInfoKHR buildInfo(
      vk::AccelerationStructureTypeKHR::eTopLevel,
      vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace |
          vk::BuildAccelerationStructureFlagBitsKHR::eAllowUpdate,
      vk::BuildAccelerationStructureModeKHR::eBuild, nullptr, {}, tlasGeometry, {}, {});

  auto buildSizeInfo = context->getDevice().getAccelerationStructureBuildSizesKHR(
      vk::AccelerationStructureBuildTypeKHR::eDevice, buildInfo, instanceCount);
  auto buildScratchBuffer = core::Buffer::Create(
      buildSizeInfo.buildScratchSize,
      vk::BufferUsageFlagBits::eAccelerationStructureStorageKHR |
          vk::BufferUsageFlagBits::eShaderDeviceAddress | vk::BufferUsageFlagBits::eStorageBuffer,
      VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY, VmaAllocationCreateFlags{}, false,
      Context::Get()->getAllocator().getRTPool());

  buildInfo.setMode(vk::BuildAccelerationStructureModeKHR::eUpdate);
  auto updateSizeInfo = context->getDevice().getAccelerationStructureBuildSizesKHR(
      vk::AccelerationStructureBuildTypeKHR::eDevice, buildInfo, instanceCount);
  mUpdateScratchBuffer = core::Buffer::Create(
      updateSizeInfo.updateScratchSize,
      vk::BufferUsageFlagBits::eAccelerationStructureStorageKHR |
          vk::BufferUsageFlagBits::eShaderDeviceAddress | vk::BufferUsageFlagBits::eStorageBuffer,
      VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY, VmaAllocationCreateFlags{}, false,
      Context::Get()->getAllocator().getRTPool());
  mUpdateScratchBufferAddress = mUpdateScratchBuffer->getAddress();

  logger::info("TLAS size {}, build scratch size {}, update scratch size {}",
               buildSizeInfo.accelerationStructureSize, buildSizeInfo.buildScratchSize,
               updateSizeInfo.updateScratchSize);

  mBuffer = core::Buffer::Create(buildSizeInfo.accelerationStructureSize,
                                 vk::BufferUsageFlagBits::eAccelerationStructureStorageKHR |
                                     vk::BufferUsageFlagBits::eShaderDeviceAddress,
                                 VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);
  vk::AccelerationStructureCreateInfoKHR createInfo(
      {}, mBuffer->getVulkanBuffer(), {}, buildSizeInfo.accelerationStructureSize,
      vk::AccelerationStructureTypeKHR::eTopLevel, {});
  mAS = context->getDevice().createAccelerationStructureKHRUnique(createInfo);

  buildInfo.setMode(vk::BuildAccelerationStructureModeKHR::eBuild);
  buildInfo.setSrcAccelerationStructure(nullptr);
  buildInfo.setDstAccelerationStructure(mAS.get());
  buildInfo.scratchData.setDeviceAddress(buildScratchBuffer->getAddress());

  vk::AccelerationStructureBuildRangeInfoKHR buildRange(instanceCount, 0, 0, 0);
  commandBuffer->buildAccelerationStructuresKHR(buildInfo, &buildRange);
  commandBuffer->end();
  context->getQueue().submitAndWait(commandBuffer.get());
}

void TLAS::recordUpdate(vk::CommandBuffer commandBuffer,
                        std::vector<vk::TransformMatrixKHR> const &transforms) {
  if (mInstances.size() != transforms.size()) {
    throw std::runtime_error("failed to update TLAS: length of transforms does "
                             "not match instance count");
  }

  for (size_t i = 0; i < transforms.size(); ++i) {
    mInstances[i].setTransform(transforms[i]);
  }
  mInstanceBuffer->upload(mInstances);

  mInstanceBufferAddress = mInstanceBuffer->getAddress();
  vk::AccelerationStructureGeometryInstancesDataKHR instancesData(VK_FALSE,
                                                                  mInstanceBufferAddress);
  vk::AccelerationStructureGeometryKHR tlasGeometry(vk::GeometryTypeKHR::eInstances, instancesData,
                                                    {});

  vk::AccelerationStructureBuildGeometryInfoKHR buildInfo(
      vk::AccelerationStructureTypeKHR::eTopLevel,
      vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace |
          vk::BuildAccelerationStructureFlagBitsKHR::eAllowUpdate,
      vk::BuildAccelerationStructureModeKHR::eUpdate, mAS.get(), mAS.get(), tlasGeometry, {},
      mUpdateScratchBufferAddress);

  // wait for upload and BLAS builds
  vk::MemoryBarrier barrier(vk::AccessFlagBits::eTransferWrite |
                                vk::AccessFlagBits::eAccelerationStructureWriteKHR,
                            vk::AccessFlagBits::eAccelerationStructureWriteKHR |
                                vk::AccessFlagBits::eAccelerationStructureReadKHR);
  commandBuffer.pipelineBarrier(vk::PipelineStageFlagBits::eTransfer |
                                    vk::PipelineStageFlagBits::eAccelerationStructureBuildKHR,
                                vk::PipelineStageFlagBits::eAccelerationStructureBuildKHR, {},
                                barrier, nullptr, nullptr);

  vk::AccelerationStructureBuildRangeInfoKHR buildRange(static_cast<uint32_t>(mInstances.size()),
                                                        0, 0, 0);
  commandBuffer.buildAccelerationStructuresKHR(buildInfo, &buildRange);
}

vk::DeviceAddress TLAS::getAddress() {
  auto context = core::Context::Get();
  return context->getDevice().getAccelerationStructureAddressKHR({mAS.get()});
}

} // namespace core
} // namespace svulkan2