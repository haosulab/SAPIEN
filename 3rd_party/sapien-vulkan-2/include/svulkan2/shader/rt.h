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
#include "postprocessing.h"

namespace svulkan2 {
namespace core {
class Buffer;
}

namespace shader {

class RayTracingStageParser {
public:
  std::future<void> loadFileAsync(std::string const &filepath, vk::ShaderStageFlagBits stage);
  void reflectSPV();
  inline std::unordered_map<uint32_t, DescriptorSetDescription> const &getResources() const {
    return mResources;
  }
  inline std::shared_ptr<StructDataLayout> getPushConstantLayout() const {
    return mPushConstantLayout;
  }
  inline std::vector<uint32_t> const &getCode() const { return mSPVCode; }
  std::string summary() const;

  void setNmae(std::string const &name) { mName = name; }
  std::string getName() const { return mName; }

private:
  std::vector<uint32_t> mSPVCode;
  std::unordered_map<uint32_t, DescriptorSetDescription> mResources;
  std::shared_ptr<StructDataLayout> mPushConstantLayout;
  std::string mName;
};

class RayTracingShaderPack {
public:
  struct HitGroupParser {
    std::string name;
    std::unique_ptr<RayTracingStageParser> any;
    std::unique_ptr<RayTracingStageParser> closest;
    std::unique_ptr<RayTracingStageParser> intersect;
  };

  struct MissGroupParser {
    std::string name;
    std::unique_ptr<RayTracingStageParser> miss;
  };

  RayTracingShaderPack(std::string const &shaderDir);

  std::shared_ptr<InputDataLayout> computeCompatibleInputVertexLayout() const;
  std::shared_ptr<InputDataLayout> computePrimitiveLayout() const;

  inline std::unordered_map<uint32_t, DescriptorSetDescription> const &getResources() const {
    return mResources;
  }
  inline std::shared_ptr<StructDataLayout> getPushConstantLayout() const {
    return mPushConstantLayout;
  }

  std::string summary() const;

  RayTracingStageParser *getRaygenStageParser() const { return mRaygenStageParser.get(); };
  inline std::vector<MissGroupParser> const &getMissGroupParsers() const {
    return mMissGroupParsers;
  }
  inline std::vector<HitGroupParser> const &getHitGroupParsers() const { return mHitGroupParsers; }

  // inline std::vector<std::unique_ptr<RayTracingStageParser>> const &getAnyHitStageParsers()
  // const {
  //   return mAnyHitStageParsers;
  // }
  // inline std::vector<std::unique_ptr<RayTracingStageParser>> const &
  // getClosestHitStageParsers() const {
  //   return mClosestHitStageParsers;
  // }
  // inline std::vector<std::unique_ptr<RayTracingStageParser>> const &
  // getIntersectStageParsers() const {
  //   return mIntersectStageParsers;
  // }
  inline std::vector<std::unique_ptr<PostprocessingShaderParser>> const &
  getPostprocessingParsers() const {
    return mPostprocessingParsers;
  }

  StructDataLayout const &getMaterialBufferLayout() const;
  StructDataLayout const &getObjectBufferLayout() const;
  StructDataLayout const &getTextureIndexBufferLayout() const;
  StructDataLayout const &getGeometryInstanceBufferLayout() const;
  StructDataLayout const &getCameraBufferLayout() const;
  DescriptorSetDescription const &getOutputDescription() const;
  DescriptorSetDescription const &getSceneDescription() const;
  DescriptorSetDescription const &getCameraDescription() const;

private:
  std::unique_ptr<RayTracingStageParser> mRaygenStageParser;
  std::vector<MissGroupParser> mMissGroupParsers;
  std::vector<HitGroupParser> mHitGroupParsers;

  std::vector<std::unique_ptr<PostprocessingShaderParser>> mPostprocessingParsers;

  std::unordered_map<uint32_t, DescriptorSetDescription> mResources;
  std::shared_ptr<StructDataLayout> mPushConstantLayout;
};

struct RayTracingShaderPackInstanceDesc {
  std::string shaderDir{};
  uint32_t maxMeshes{};
  uint32_t maxMaterials{};
  uint32_t maxTextures{};
  uint32_t maxPointSets{};
};

class RayTracingShaderPackInstance {

public:
  RayTracingShaderPackInstance(RayTracingShaderPackInstanceDesc desc);
  vk::PipelineLayout getPipelineLayout();
  vk::Pipeline getPipeline();
  core::Buffer &getShaderBindingTable();
  vk::DescriptorSetLayout getOutputSetLayout();
  vk::DescriptorSetLayout getSceneSetLayout();
  vk::DescriptorSetLayout getCameraSetLayout();

  inline std::shared_ptr<RayTracingShaderPack> getShaderPack() const { return mShaderPack; }

  vk::StridedDeviceAddressRegionKHR const &getRgenRegion();
  vk::StridedDeviceAddressRegionKHR const &getMissRegion();
  vk::StridedDeviceAddressRegionKHR const &getHitRegion();
  vk::StridedDeviceAddressRegionKHR const &getCallRegion();

  inline std::vector<vk::UniqueDescriptorSetLayout> const &getPostprocessingSetLayouts() const {
    return mPostprocessingSetLayouts;
  }

  inline std::vector<vk::UniquePipeline> const &getPostprocessingPipelines() const {
    return mPostprocessingPipelines;
  }
  inline std::vector<vk::UniquePipelineLayout> const &getPostprocessingPipelineLayouts() const {
    return mPostprocessingPipelineLayouts;
  }

  RayTracingShaderPackInstance(RayTracingShaderPackInstance const &) = delete;
  RayTracingShaderPackInstance &operator=(RayTracingShaderPackInstance const &) = delete;

private:
  void initPipeline();

  void initSBT();

  RayTracingShaderPackInstanceDesc mDesc;

  // vk::UniqueShaderModule mRgenModule;
  // std::vector<vk::UniqueShaderModule> mMissModules;
  // std::vector<vk::UniqueShaderModule> mRcHitModules;
  // std::vector<vk::UniqueShaderModule> mRaHitModules;
  // std::vector<vk::UniqueShaderModule> mIntersectModules;

  vk::UniqueDescriptorSetLayout mSceneSetLayout;
  vk::UniqueDescriptorSetLayout mCameraSetLayout;
  vk::UniqueDescriptorSetLayout mOutputSetLayout;
  vk::UniquePipelineLayout mPipelineLayout;
  vk::UniquePipeline mPipeline;
  std::unique_ptr<core::Buffer> mSBTBuffer;
  vk::StridedDeviceAddressRegionKHR mRgenRegion{};
  vk::StridedDeviceAddressRegionKHR mMissRegion{};
  vk::StridedDeviceAddressRegionKHR mHitRegion{};
  vk::StridedDeviceAddressRegionKHR mCallRegion{};

  std::vector<vk::UniqueDescriptorSetLayout> mPostprocessingSetLayouts;
  std::vector<vk::UniquePipelineLayout> mPostprocessingPipelineLayouts;
  std::vector<vk::UniquePipeline> mPostprocessingPipelines;

  std::shared_ptr<RayTracingShaderPack> mShaderPack;
};

} // namespace shader
} // namespace svulkan2