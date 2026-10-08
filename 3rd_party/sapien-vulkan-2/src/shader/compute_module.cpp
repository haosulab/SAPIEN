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
#include "svulkan2/shader/compute_module.h"
#include "../common/logger.h"
#include "./reflect.h"
#include "svulkan2/core/context.h"
#include "svulkan2/core/physical_device.h"
#include "svulkan2/shader/glsl_compiler.h"

namespace svulkan2 {
namespace shader {

static std::string summarizeResources(std::vector<DescriptorSetDescription> const &resources) {
  std::stringstream ss;
  for (uint32_t sid = 0; sid < resources.size(); ++sid) {
    auto &set = resources[sid];
    ss << "\nSet " << std::setw(2) << sid;
    if (set.type != UniformBindingType::eUnknown) {
      if (set.type == UniformBindingType::eRTCamera) {
        ss << "    Camera";
      } else if (set.type == UniformBindingType::eRTScene) {
        ss << "     Scene";
      } else if (set.type == UniformBindingType::eRTOutput) {
        ss << "    Output";
      }
    }
    ss << "\n";
    for (auto &[bid, b] : set.bindings) {
      ss << "  Binding " << std::setw(2) << bid << std::setw(20) << b.name;
      switch (b.type) {
      case vk::DescriptorType::eUniformBuffer:
        ss << " UniformBuffer\n";
        ss << "    Dim  " << b.dim << "\n";
        if (b.dim > 0) {
          ss << "    Size " << b.arraySize << "\n";
        }
        break;
      case vk::DescriptorType::eStorageBuffer:
        ss << " StorageBuffer\n";
        ss << "    Dim  " << b.dim << "\n";
        if (b.dim > 0) {
          ss << "    Size " << b.arraySize << "\n";
        }
        ss << "    " << std::setw(15) << "Field" << std::setw(15) << "offset" << std::setw(15)
           << "size" << std::setw(15) << "dim" << std::setw(15) << "type" << std::endl;
        for (auto &elem : set.buffers[b.arrayIndex]->getElementsSorted()) {
          ss << "    " << std::setw(15) << elem->name << std::setw(15) << elem->offset
             << std::setw(15) << elem->size << std::setw(15) << elem->array.size() << std::setw(15)
             << elem->dtype.typestr() << std::endl;
        }
        break;
      case vk::DescriptorType::eCombinedImageSampler:
        ss << " CombinedImageSampler\n";
        break;
      case vk::DescriptorType::eStorageImage:
        ss << " StorageImage\n";
        break;
      case vk::DescriptorType::eAccelerationStructureKHR:
        ss << " AccelerationStructure\n";
        break;
      default:
        ss << " Unknown\n";
        break;
      }
    }
  }
  return ss.str();
}

static std::string summarizeConstant(SpecializationConstantLayout const &layout) {
  std::stringstream ss;
  for (auto elem : layout.getElementsSorted()) {
    ss << elem.id;
    ss << std::setw(15) << elem.name << std::setw(10) << elem.dtype.typestr();
  }
  return ss.str();
}

ComputeModule::ComputeModule(std::vector<uint32_t> code, int blockSizeX, int blockSizeY,
                             int blockSizeZ)
    : mCode(code), mBlockSize({blockSizeX, blockSizeY, blockSizeZ}) {
  mContext = core::Context::Get();
  reflect();
  compile();
}

ComputeModule::ComputeModule(std::string const &filename, int blockSizeX, int blockSizeY,
                             int blockSizeZ)
    : mBlockSize({blockSizeX, blockSizeY, blockSizeZ}) {
  mContext = core::Context::Get();

  if (filename.ends_with(".spv") || filename.ends_with(".SPV")) {
    auto data = readFile(filename);
    if (data.size() / 4 * 4 != data.size()) {
      throw std::runtime_error("invalid spv file: " + filename);
    }
    mCode.resize(data.size() / 4);
    std::memcpy(mCode.data(), data.data(), data.size());
  } else {
    mCode = GLSLCompiler::compileGlslFileCached(vk::ShaderStageFlagBits::eCompute, filename);
  }
  reflect();
  compile();
}

void ComputeModule::reflect() {
  spirv_cross::Compiler compiler(mCode);
  auto resources = compiler.get_shader_resources();

  auto ids = getDescriptorSetIds(compiler);
  if (ids.size() == 0) {
    throw std::runtime_error("failed to load compute shader: no input buffers or images");
  }

  if (ids.size() != 1) {
    throw std::runtime_error("failed to load compute shader: currently only a single descriptor "
                             "set is supported"); // TODO: allow more
  }

  mDescriptorSetDescriptions.clear();
  for (uint32_t i = 0; i < ids.size(); ++i) {
    // ids should be consecutive integers
    if (i != ids.at(i)) {
      throw std::runtime_error("failed to load compute shader: compute shader descriptor sets "
                               "should be consecutive integers starting from 0");
    }
    mDescriptorSetDescriptions.push_back(getDescriptorSetDescription(compiler, i));
  }

  mSpecializationConstantLayout = parseSpecializationConstant(compiler);

  mPushConstantLayout = nullptr;
  for (auto &r : resources.push_constant_buffers) {
    auto const &type = compiler.get_type(r.type_id);
    if (type.basetype != spirv_cross::SPIRType::Struct) {
      throw std::runtime_error(
          "failed to load compute shader: push constant buffer must be a struct");
    }
    mPushConstantLayout = parseBuffer(compiler, type);
  }

  spirv_cross::SpecializationConstant x, y, z;
  compiler.get_work_group_size_specialization_constants(x, y, z);
  if (x.id) {
    mSpecializationConstantLayout->elements["local_size_x_id"] = {
        .name = "local_size_x_id", .id = x.constant_id, .dtype = DataType::UINT()};
  }
  if (y.id) {
    mSpecializationConstantLayout->elements["local_size_y_id"] = {
        .name = "local_size_y_id", .id = y.constant_id, .dtype = DataType::UINT()};
  }
  if (z.id) {
    mSpecializationConstantLayout->elements["local_size_z_id"] = {
        .name = "local_size_z_id", .id = z.constant_id, .dtype = DataType::UINT()};
  }
  logger::info("{}", summarizeResources(mDescriptorSetDescriptions));
  logger::info("{}", summarizeConstant(*mSpecializationConstantLayout));
}

void ComputeModule::compile() {
  auto device = mContext->getDevice();

  std::vector<vk::DescriptorSetLayout> layouts;

  for (auto &set : mDescriptorSetDescriptions) {
    std::vector<vk::DescriptorSetLayoutBinding> bindings;
    for (auto &[bid, binding] : set.bindings) {
      // TODO: handle storage image
      if (binding.type == vk::DescriptorType::eStorageBuffer) {
        // TODO: handle binding.dim != 0
        bindings.push_back(
            {bid, vk::DescriptorType::eStorageBuffer, 1, vk::ShaderStageFlagBits::eCompute});
      } else {
        throw std::runtime_error(
            "failed to load compute shader: only storage buffer is currently supported.");
      }
    }
    mSetLayouts.push_back(device.createDescriptorSetLayoutUnique({{}, bindings}));
    layouts.push_back(mSetLayouts.back().get());
  }

  std::vector<vk::PushConstantRange> pushConstantRanges;
  if (mPushConstantLayout) {
    pushConstantRanges.push_back(
        vk::PushConstantRange(vk::ShaderStageFlagBits::eCompute, 0, mPushConstantLayout->size));
  }

  mPipelineLayout = device.createPipelineLayoutUnique(
      vk::PipelineLayoutCreateInfo{{}, layouts, pushConstantRanges});

  auto elems = mSpecializationConstantLayout->getElementsSorted();
  std::vector<vk::SpecializationMapEntry> entries;
  std::vector<int> specializationData;
  if (elems.size()) {
    specializationData.resize(elems.size());
    for (uint32_t i = 0; i < elems.size(); ++i) {
      if (elems[i].name == "subgroupSize") {
        entries.emplace_back(elems[i].id, i * sizeof(int), sizeof(int));
        uint32_t v = core::Context::Get()->getPhysicalDevice2()->getSubgroupSize();
        std::memcpy(specializationData.data() + i, &v, sizeof(int));
      } else if (elems[i].name == "local_size_x_id") {
        entries.emplace_back(elems[i].id, i * sizeof(int), sizeof(int));
        std::memcpy(specializationData.data() + i, &mBlockSize[0], sizeof(int));
      } else if (elems[i].name == "local_size_y_id") {
        entries.emplace_back(elems[i].id, i * sizeof(int), sizeof(int));
        std::memcpy(specializationData.data() + i, &mBlockSize[1], sizeof(int));
      } else if (elems[i].name == "local_size_z_id") {
        entries.emplace_back(elems[i].id, i * sizeof(int), sizeof(int));
        std::memcpy(specializationData.data() + i, &mBlockSize[2], sizeof(int));
      }
    }
  }
  auto specializationInfo =
      vk::SpecializationInfo(entries.size(), entries.data(),
                             specializationData.size() * sizeof(int), specializationData.data());

  auto shaderModule = device.createShaderModuleUnique(vk::ShaderModuleCreateInfo({}, mCode));
  auto shaderStageInfo =
      vk::PipelineShaderStageCreateInfo({}, vk::ShaderStageFlagBits::eCompute, shaderModule.get(),
                                        "main", elems.size() ? &specializationInfo : nullptr);
  vk::ComputePipelineCreateInfo computePipelineCreateInfo({}, shaderStageInfo,
                                                          mPipelineLayout.get());
  auto pipelinCache = device.createPipelineCacheUnique({});
  mPipeline =
      device.createComputePipelineUnique(pipelinCache.get(), computePipelineCreateInfo).value;

  // mSets.clear();
  // for (auto layout : layouts) {
  //   mSets.push_back(core::Context::Get()->getDescriptorPool().allocateSet(layout));
  // }
}

ComputeModuleInstance::ComputeModuleInstance(std::shared_ptr<ComputeModule> m) : mModule(m) {
  if (auto layout = mModule->getPushConstantLayout()) {
    mPushConstantBuffer.resize(layout->size);
  }
  for (auto &layout : mModule->getSetLayouts()) {
    mSets.push_back(core::Context::Get()->getDescriptorPool().allocateSet(layout.get()));
  }
}
void ComputeModuleInstance::setBuffer(std::string const &name, core::Buffer *buffer) {
  auto device = core::Context::Get()->getDevice();
  for (auto &[id, binding] : mModule->getDescriptorSetDescriptions().at(0).bindings) {
    if (binding.name == name) {
      vk::DescriptorBufferInfo bufferInfo =
          vk::DescriptorBufferInfo(buffer->getVulkanBuffer(), 0, VK_WHOLE_SIZE);
      vk::WriteDescriptorSet write(mSets.at(0).get(), id, 0, vk::DescriptorType::eStorageBuffer,
                                   {}, bufferInfo);
      device.updateDescriptorSets(write, {});
    }
  }
}

void ComputeModuleInstance::setCommandPool(std::shared_ptr<core::CommandPool> pool) {
  mCommandPool = pool;
  mCommandBuffer = mCommandPool->allocateCommandBuffer();
}

void ComputeModuleInstance::setGridSize(int x, int y, int z) {
  if (!mCommandBuffer) {
    throw std::runtime_error("grid size should only be set after setting the command pool");
  }
  mCommandBuffer->begin(
      vk::CommandBufferBeginInfo(vk::CommandBufferUsageFlagBits::eSimultaneousUse));
  record(mCommandBuffer.get(), x, y, z);
  mCommandBuffer->end();
}

void ComputeModuleInstance::record(vk::CommandBuffer cb, int x, int y, int z) {
  cb.bindPipeline(vk::PipelineBindPoint::eCompute, mModule->getPipeline());
  cb.bindDescriptorSets(vk::PipelineBindPoint::eCompute, mModule->getPipelineLayout(), 0,
                        mSets.at(0).get(), {});
  if (mModule->getPushConstantLayout()) {
    // cb.pipelineBarrier(
    //     vk::PipelineStageFlagBits::eAllCommands, vk::PipelineStageFlagBits::eComputeShader,
    //     vk::DependencyFlagBits::eByRegion, {},
    //     vk::BufferMemoryBarrier(vk::AccessFlagBits::eMemoryWrite, vk::AccessFlagBits::eMemoryRead),
    //     {});
    cb.pushConstants(mModule->getPipelineLayout(), vk::ShaderStageFlagBits::eCompute, 0,
                     mPushConstantBuffer.size(), mPushConstantBuffer.data());
  }
  cb.dispatch(x, y, z);
}

void ComputeModuleInstance::launch() {
  core::Context::Get()->getQueue().submit(mCommandBuffer.get(), {});
}

ComputeModule::~ComputeModule() {}

}; // namespace shader
}; // namespace svulkan2