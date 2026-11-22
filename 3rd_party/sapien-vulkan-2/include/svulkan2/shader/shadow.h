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
namespace shader {

class ShadowPassParser : public BaseParser {
  std::shared_ptr<InputDataLayout> mVertexInputLayout;
  std::vector<DescriptorSetDescription> mDescriptorSetDescriptions;

public:
  inline std::shared_ptr<InputDataLayout> getVertexInputLayout() const {
    return mVertexInputLayout;
  }

  inline std::shared_ptr<OutputDataLayout>
  getTextureOutputLayout() const override {
    return std::make_shared<OutputDataLayout>();
  };

  inline std::vector<std::string> getColorRenderTargetNames() const override {
    return {};
  };
  std::optional<std::string> getDepthRenderTargetName() const override;

  vk::UniqueRenderPass createRenderPass(
      vk::Device device, std::vector<vk::Format> const &colorFormats,
      vk::Format depthFormat,
      std::vector<std::pair<vk::ImageLayout, vk::ImageLayout>> const
          &colorTargetLayouts,
      std::pair<vk::ImageLayout, vk::ImageLayout> const &depthLayout,
      vk::SampleCountFlagBits sampleCount) const override;

  vk::UniquePipeline
  createPipeline(vk::Device device, vk::PipelineLayout layout,
                 vk::RenderPass renderPass, vk::CullModeFlags cullMode,
                 vk::FrontFace frontFace, bool alphaBlend,
                 vk::SampleCountFlagBits sampleCount,
                 std::map<std::string, SpecializationConstantValue> const
                     &specializationConstantInfo) const override;

  std::vector<UniformBindingType> getUniformBindingTypes() const override;

  inline std::vector<DescriptorSetDescription>
  getDescriptorSetDescriptions() const override {
    return mDescriptorSetDescriptions;
  };

private:
  void reflectSPV() override;
  void validate() const;
};

} // namespace shader
} // namespace svulkan2