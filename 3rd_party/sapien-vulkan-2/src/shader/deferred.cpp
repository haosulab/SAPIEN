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
#include "svulkan2/shader/deferred.h"
#include "../common/logger.h"
#include "reflect.h"

namespace svulkan2 {
namespace shader {

void DeferredPassParser::reflectSPV() {
  spirv_cross::Compiler vertComp(mVertSPVCode);
  spirv_cross::Compiler fragComp(mFragSPVCode);
  std::vector<DescriptorSetDescription> fragDesc;
  try {
    mSpecializationConstantLayout = parseSpecializationConstant(fragComp);
    mTextureOutputLayout = parseTextureOutput(fragComp);
    for (uint32_t i = 0; i < 4; ++i) {
      fragDesc.push_back(getDescriptorSetDescription(fragComp, i));
    }

  } catch (std::runtime_error const &err) {
    throw std::runtime_error("[frag]" + std::string(err.what()));
  }

  for (uint32_t i = 0, stop = false; i < 4; ++i) {
    if (fragDesc[i].type == UniformBindingType::eNone) {
      stop = true;
      continue;
    }
    if (stop == true) {
      throw std::runtime_error("deferred: descriptor sets should use "
                               "consecutive integers starting from 0");
    }
    mDescriptorSetDescriptions.push_back(fragDesc[i]);
  }

  validate();
}

void DeferredPassParser::validate() const {
  for (auto &desc : mDescriptorSetDescriptions) {
    if (desc.type == UniformBindingType::eScene) {
      // validate constants
      ASSERT(mSpecializationConstantLayout->elements.contains("NUM_DIRECTIONAL_LIGHTS"),
             "[frag]NUM_DIRECTIONAL_LIGHTS is a required specialization"
             "constant when using SceneBuffer");
      ASSERT(mSpecializationConstantLayout->elements.contains("NUM_POINT_LIGHTS"),
             "[frag]NUM_POINT_LIGHTS is a required specialization "
             "constant when using SceneBuffer");
      ASSERT(mSpecializationConstantLayout->elements.contains("NUM_SPOT_LIGHTS"),
             "[frag]NUM_SPOT_LIGHTS is a required specialization "
             "constant when using SceneBuffer");
    }
  }

  // validate out textures
  for (auto &elem : mTextureOutputLayout->elements) {
    ASSERT(elem.second.name.substr(0, 3) == "out",
           "[frag]all out texture variables must start with \"out\"");
  }
}

vk::UniqueRenderPass DeferredPassParser::createRenderPass(
    vk::Device device, std::vector<vk::Format> const &colorFormats, vk::Format depthFormat,
    std::vector<std::pair<vk::ImageLayout, vk::ImageLayout>> const &colorTargetLayouts,
    std::pair<vk::ImageLayout, vk::ImageLayout> const &depthLayout,
    vk::SampleCountFlagBits sampleCount) const {
  std::vector<vk::AttachmentDescription> attachmentDescriptions;
  std::vector<vk::AttachmentReference> colorAttachments;

  auto elems = mTextureOutputLayout->getElementsSorted();
  for (uint32_t i = 0; i < elems.size(); ++i) {
    colorAttachments.push_back({i, vk::ImageLayout::eColorAttachmentOptimal});
    attachmentDescriptions.push_back(vk::AttachmentDescription(
        {}, colorFormats.at(i), sampleCount, vk::AttachmentLoadOp::eDontCare,
        vk::AttachmentStoreOp::eStore, // color attachment load and store op
        vk::AttachmentLoadOp::eDontCare,
        vk::AttachmentStoreOp::eDontCare, // stencil load and store op
        colorTargetLayouts[i].first, colorTargetLayouts[i].second));
  }

  vk::SubpassDescription subpassDescription({}, vk::PipelineBindPoint::eGraphics, 0, nullptr,
                                            colorAttachments.size(), colorAttachments.data(),
                                            nullptr, nullptr);

  // ensure previous writes are done
  // TODO: compute a better dependency
  std::array<vk::SubpassDependency, 2> deps{
      vk::SubpassDependency(VK_SUBPASS_EXTERNAL, 0,
                            vk::PipelineStageFlagBits::eColorAttachmentOutput |
                                vk::PipelineStageFlagBits::eEarlyFragmentTests |
                                vk::PipelineStageFlagBits::eLateFragmentTests,
                            vk::PipelineStageFlagBits::eFragmentShader |
                                vk::PipelineStageFlagBits::eColorAttachmentOutput,
                            vk::AccessFlagBits::eColorAttachmentWrite |
                                vk::AccessFlagBits::eDepthStencilAttachmentWrite,
                            vk::AccessFlagBits::eShaderRead |
                                vk::AccessFlagBits::eColorAttachmentWrite),
      vk::SubpassDependency(0, VK_SUBPASS_EXTERNAL,
                            vk::PipelineStageFlagBits::eColorAttachmentOutput |
                                vk::PipelineStageFlagBits::eEarlyFragmentTests |
                                vk::PipelineStageFlagBits::eLateFragmentTests,
                            vk::PipelineStageFlagBits::eFragmentShader |
                                vk::PipelineStageFlagBits::eColorAttachmentOutput,
                            vk::AccessFlagBits::eColorAttachmentWrite |
                                vk::AccessFlagBits::eDepthStencilAttachmentWrite,
                            vk::AccessFlagBits::eShaderRead |
                                vk::AccessFlagBits::eColorAttachmentWrite),
  };

  return device.createRenderPassUnique(
      vk::RenderPassCreateInfo({}, attachmentDescriptions.size(), attachmentDescriptions.data(), 1,
                               &subpassDescription, 2, deps.data()));
}

vk::UniquePipeline DeferredPassParser::createPipeline(
    vk::Device device, vk::PipelineLayout layout, vk::RenderPass renderPass,
    vk::CullModeFlags cullMode, vk::FrontFace frontFace, bool alphaBlend,
    vk::SampleCountFlagBits sampleCount,
    std::map<std::string, SpecializationConstantValue> const &specializationConstantInfo) const {

  // shaders
  vk::UniquePipelineCache pipelineCache =
      device.createPipelineCacheUnique(vk::PipelineCacheCreateInfo());
  auto vsm = device.createShaderModuleUnique(
      {{}, mVertSPVCode.size() * sizeof(uint32_t), mVertSPVCode.data()});
  auto fsm = device.createShaderModuleUnique(
      {{}, mFragSPVCode.size() * sizeof(uint32_t), mFragSPVCode.data()});

  // specialization Constants
  auto elems = mSpecializationConstantLayout->getElementsSorted();
  vk::SpecializationInfo fragSpecializationInfo;
  std::vector<vk::SpecializationMapEntry> entries;
  std::vector<std::byte> specializationData(mSpecializationConstantLayout->size());
  if (elems.size()) {
    uint32_t offset = 0;
    for (uint32_t i = 0; i < elems.size(); ++i) {
      if (specializationConstantInfo.find(elems[i].name) != specializationConstantInfo.end() &&
          elems[i].dtype != specializationConstantInfo.at(elems[i].name).dtype) {
        throw std::runtime_error("Type mismatch on specialization constant " + elems[i].name +
                                 ".");
      }
      entries.emplace_back(elems[i].id, offset, elems[i].dtype.size());
      if (specializationConstantInfo.contains(elems[i].name)) {
        std::memcpy(specializationData.data() + offset,
                    specializationConstantInfo.at(elems[i].name).buffer, elems[i].dtype.size());
      } else {
        std::memcpy(specializationData.data() + offset, elems[i].buffer, elems[i].dtype.size());
      }
      offset += elems[i].dtype.size();
    }
    fragSpecializationInfo = vk::SpecializationInfo(
        entries.size(), entries.data(), specializationData.size(), specializationData.data());
  }

  std::array<vk::PipelineShaderStageCreateInfo, 2> pipelineShaderStageCreateInfos{
      vk::PipelineShaderStageCreateInfo(vk::PipelineShaderStageCreateFlags(),
                                        vk::ShaderStageFlagBits::eVertex, vsm.get(), "main",
                                        nullptr),
      vk::PipelineShaderStageCreateInfo(vk::PipelineShaderStageCreateFlags(),
                                        vk::ShaderStageFlagBits::eFragment, fsm.get(), "main",
                                        elems.size() ? &fragSpecializationInfo : nullptr)};

  // vertex input
  // drawing a single hardcoded triangle
  vk::PipelineVertexInputStateCreateInfo pipelineVertexInputStateCreateInfo;
  pipelineVertexInputStateCreateInfo.vertexBindingDescriptionCount = 0;
  pipelineVertexInputStateCreateInfo.pVertexBindingDescriptions = nullptr;
  pipelineVertexInputStateCreateInfo.vertexAttributeDescriptionCount = 0;
  pipelineVertexInputStateCreateInfo.pVertexAttributeDescriptions = nullptr;

  // input assembly
  vk::PipelineInputAssemblyStateCreateInfo pipelineInputAssemblyStateCreateInfo(
      vk::PipelineInputAssemblyStateCreateFlags(), vk::PrimitiveTopology::eTriangleList);

  // viewport
  vk::PipelineViewportStateCreateInfo pipelineViewportStateCreateInfo(
      vk::PipelineViewportStateCreateFlags(), 1, nullptr, 1, nullptr);

  // rasterization
  vk::PipelineRasterizationStateCreateInfo pipelineRasterizationStateCreateInfo(
      vk::PipelineRasterizationStateCreateFlags(), false, false, vk::PolygonMode::eFill,
      vk::CullModeFlagBits::eNone, vk::FrontFace::eCounterClockwise, false, 0.0f, 0.0f, 0.0f,
      1.0f);

  // multisample
  vk::PipelineMultisampleStateCreateInfo pipelineMultisampleStateCreateInfo{{}, sampleCount};

  // stencil
  vk::StencilOpState stencilOpState{};
  vk::PipelineDepthStencilStateCreateInfo pipelineDepthStencilStateCreateInfo(
      vk::PipelineDepthStencilStateCreateFlags(), true, true, vk::CompareOp::eLessOrEqual,
      false, // depth test enabled , depth write enabled, depth compare op,
             // depth Bounds Test Enable
      false, stencilOpState,
      stencilOpState); // stensil test enabled, stensil front, stensil back

  // blend
  uint32_t numColorAttachments = mTextureOutputLayout->elements.size();
  vk::ColorComponentFlags colorComponentFlags(
      vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG |
      vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA);
  std::vector<vk::PipelineColorBlendAttachmentState> pipelineColorBlendAttachmentStates;
  for (uint32_t i = 0; i < numColorAttachments; ++i) {
    pipelineColorBlendAttachmentStates.push_back(vk::PipelineColorBlendAttachmentState(
        false, vk::BlendFactor::eZero, vk::BlendFactor::eZero, vk::BlendOp::eAdd,
        vk::BlendFactor::eZero, vk::BlendFactor::eZero, vk::BlendOp::eAdd, colorComponentFlags));
  }
  vk::PipelineColorBlendStateCreateInfo pipelineColorBlendStateCreateInfo(
      vk::PipelineColorBlendStateCreateFlags(), false, vk::LogicOp::eNoOp, numColorAttachments,
      pipelineColorBlendAttachmentStates.data(), {{1.0f, 1.0f, 1.0f, 1.0f}});

  // dynamic
  vk::DynamicState dynamicStates[2] = {vk::DynamicState::eViewport, vk::DynamicState::eScissor};
  vk::PipelineDynamicStateCreateInfo pipelineDynamicStateCreateInfo(
      vk::PipelineDynamicStateCreateFlags(), 2, dynamicStates);

  vk::GraphicsPipelineCreateInfo graphicsPipelineCreateInfo(
      vk::PipelineCreateFlags(), pipelineShaderStageCreateInfos.size(),
      pipelineShaderStageCreateInfos.data(), &pipelineVertexInputStateCreateInfo,
      &pipelineInputAssemblyStateCreateInfo, nullptr, &pipelineViewportStateCreateInfo,
      &pipelineRasterizationStateCreateInfo, &pipelineMultisampleStateCreateInfo,
      &pipelineDepthStencilStateCreateInfo, &pipelineColorBlendStateCreateInfo,
      &pipelineDynamicStateCreateInfo, layout, renderPass);
  return device.createGraphicsPipelineUnique(pipelineCache.get(), graphicsPipelineCreateInfo)
      .value;
}

std::vector<std::string> DeferredPassParser::getColorRenderTargetNames() const {
  std::vector<std::string> result;
  auto elems = mTextureOutputLayout->getElementsSorted();
  for (auto elem : elems) {
    result.push_back(getOutTextureName(elem.name));
  }
  return result;
}

std::vector<std::string> DeferredPassParser::getInputTextureNames() const {
  std::vector<std::string> result;
  for (auto &desc : mDescriptorSetDescriptions) {
    if (desc.type == UniformBindingType::eTextures) {
      for (uint32_t i = 0; i < desc.bindings.size(); ++i) {
        result.push_back(getInTextureName(desc.bindings.at(i).name));
      }
    }
  }
  return result;
}

std::vector<UniformBindingType> DeferredPassParser::getUniformBindingTypes() const {
  std::vector<UniformBindingType> result;
  for (auto &desc : mDescriptorSetDescriptions) {
    result.push_back(desc.type);
  }
  return result;
}

} // namespace shader
} // namespace svulkan2