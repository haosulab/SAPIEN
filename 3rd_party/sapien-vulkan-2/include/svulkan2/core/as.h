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
#include "buffer.h"
#include "command_pool.h"

namespace svulkan2 {
namespace core {

class BLAS {
public:
  BLAS(std::vector<vk::AccelerationStructureGeometryKHR> const &geometries,
       std::vector<vk::AccelerationStructureBuildRangeInfoKHR> const &buildRanges,
       std::vector<uint32_t> const &maxPrimitiveCounts = {}, bool compaction = true,
       bool update = false);

  void build();
  void recordUpdate(vk::CommandBuffer commandBuffer,
                    std::vector<vk::AccelerationStructureBuildRangeInfoKHR> const &buildRanges);

  vk::DeviceAddress getAddress();

private:
  std::vector<vk::AccelerationStructureGeometryKHR> mGeometries;
  std::vector<vk::AccelerationStructureBuildRangeInfoKHR> mBuildRanges;
  std::vector<uint32_t> mMaxPrimitiveCounts;
  bool mCompaction;
  bool mUpdate;

  std::unique_ptr<Buffer> mUpdateScratchBuffer;
  vk::DeviceAddress mUpdateScratchBufferAddress;

  std::unique_ptr<Buffer> mBuffer;
  vk::UniqueAccelerationStructureKHR mAS;
};

class TLAS {

public:
  TLAS(std::vector<vk::AccelerationStructureInstanceKHR> const &instances);
  void build();
  void recordUpdate(vk::CommandBuffer commandBuffer,
                    std::vector<vk::TransformMatrixKHR> const &transforms);

  vk::DeviceAddress getAddress();

  vk::AccelerationStructureKHR getVulkanAS() const { return mAS.get(); };

private:
  std::vector<vk::AccelerationStructureInstanceKHR> mInstances;

  std::unique_ptr<Buffer> mInstanceBuffer;
  vk::DeviceAddress mInstanceBufferAddress;

  std::unique_ptr<Buffer> mUpdateScratchBuffer;
  vk::DeviceAddress mUpdateScratchBufferAddress;

  std::unique_ptr<Buffer> mBuffer;
  vk::UniqueAccelerationStructureKHR mAS;
};

} // namespace core
} // namespace svulkan2