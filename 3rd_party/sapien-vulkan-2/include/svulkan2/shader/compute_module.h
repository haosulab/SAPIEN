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
#include "base_parser.h"

namespace svulkan2 {

namespace core {
class Context;
class Buffer;
class CommandPool;
} // namespace core

namespace shader {

class ComputeModule {

public:
  ComputeModule(std::vector<uint32_t> code, int blockSizeX, int blockSizeY = 1,
                int blockSizeZ = 1);
  ComputeModule(std::string const &filename, int blockSizeX, int blockSizeY = 1,
                int blockSizeZ = 1);

  std::shared_ptr<StructDataLayout> getPushConstantLayout() const { return mPushConstantLayout; }
  std::vector<vk::UniqueDescriptorSetLayout> const &getSetLayouts() const { return mSetLayouts; }
  std::vector<DescriptorSetDescription> const &getDescriptorSetDescriptions() const {
    return mDescriptorSetDescriptions;
  }
  vk::Pipeline getPipeline() const { return mPipeline.get(); }
  vk::PipelineLayout getPipelineLayout() const { return mPipelineLayout.get(); }

  ~ComputeModule();
  ComputeModule(ComputeModule const &) = delete;
  ComputeModule &operator=(ComputeModule const &) = delete;
  ComputeModule(ComputeModule const &&) = delete;
  ComputeModule &operator=(ComputeModule const &&) = delete;

private:
  void reflect();
  void compile();

  std::shared_ptr<core::Context> mContext;
  std::vector<uint32_t> mCode;
  std::array<int, 3> mBlockSize;

  // reflection results
  std::vector<DescriptorSetDescription> mDescriptorSetDescriptions;
  std::shared_ptr<SpecializationConstantLayout> mSpecializationConstantLayout;
  std::shared_ptr<StructDataLayout> mPushConstantLayout;

  vk::UniquePipelineLayout mPipelineLayout;
  vk::UniquePipeline mPipeline;
  std::vector<vk::UniqueDescriptorSetLayout> mSetLayouts;
};

class ComputeModuleInstance {
public:
  ComputeModuleInstance(std::shared_ptr<ComputeModule> m);

  void setBuffer(std::string const &name, core::Buffer *buffer);
  template <typename T> void setPushConstant(std::string const &name, T data) {
    auto elem = mModule->getPushConstantLayout()->elements.at(name);
    if (DataTypeFor<T>::value != elem.dtype) {
      throw std::runtime_error("incompatible push constant type");
    }
    std::memcpy(mPushConstantBuffer.data() + elem.offset, &data, elem.size);
  }

  void record(vk::CommandBuffer cb, int x, int y, int z);

  void setCommandPool(std::shared_ptr<core::CommandPool> pool);
  void setGridSize(int x, int y, int z);
  void launch();

private:
  std::shared_ptr<ComputeModule> mModule;

  std::vector<vk::UniqueDescriptorSet> mSets;
  std::vector<char> mPushConstantBuffer;

  std::shared_ptr<core::CommandPool> mCommandPool;
  vk::UniqueCommandBuffer mCommandBuffer;
};

}; // namespace shader
} // namespace svulkan2