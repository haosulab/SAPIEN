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
#include "svulkan2/shader/gbuffer.h"
#include "../common/logger.h"
#include "reflect.h"

namespace svulkan2 {
namespace shader {

void GbufferPassParser::reflectSPV() {
  spirv_cross::Compiler vertComp(mVertSPVCode);
  std::vector<DescriptorSetDescription> vertDesc;
  std::vector<DescriptorSetDescription> fragDesc;
  try {
    mVertexInputLayout = parseVertexInput(vertComp);
    for (uint32_t i = 0; i < 4; ++i) {
      vertDesc.push_back(getDescriptorSetDescription(vertComp, i));
    }
  } catch (std::runtime_error const &err) {
    throw std::runtime_error("[vert]" + std::string(err.what()));
  }

  spirv_cross::Compiler fragComp(mFragSPVCode);
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
    if (vertDesc[i].type == UniformBindingType::eNone &&
        fragDesc[i].type == UniformBindingType::eNone) {
      stop = true;
      continue;
    }
    if (stop == true) {
      throw std::runtime_error("gbuffer: descriptor sets should use "
                               "consecutive integers starting from 0");
    }
    if (vertDesc[i].type == UniformBindingType::eNone) {
      mDescriptorSetDescriptions.push_back(fragDesc[i]);
    } else if (fragDesc[i].type == UniformBindingType::eNone) {
      mDescriptorSetDescriptions.push_back(vertDesc[i]);
    } else {
      mDescriptorSetDescriptions.push_back(vertDesc[i].merge(fragDesc[i]));
    }
  }

  validate();
}

void GbufferPassParser::validate() const {
  for (auto &elem : mTextureOutputLayout->elements) {
    ASSERT(elem.second.name.substr(0, 3) == "out",
           "[frag]all out texture variables must start with \"out\"");
  }
};

vk::UniqueRenderPass GbufferPassParser::createRenderPass(
    vk::Device device, std::vector<vk::Format> const &colorFormats, vk::Format depthFormat,
    std::vector<std::pair<vk::ImageLayout, vk::ImageLayout>> const &colorTargetLayouts,
    std::pair<vk::ImageLayout, vk::ImageLayout> const &depthLayout,
    vk::SampleCountFlagBits sampleCount) const {
  std::vector<vk::AttachmentDescription> attachmentDescriptions;
  std::vector<vk::AttachmentReference> colorAttachmentRefs;

  uint32_t attachmentIndex{0};

  auto elems = mTextureOutputLayout->getElementsSorted();
  for (uint32_t i = 0; i < elems.size(); ++i, ++attachmentIndex) {
    colorAttachmentRefs.push_back({attachmentIndex, vk::ImageLayout::eColorAttachmentOptimal});

    attachmentDescriptions.push_back(vk::AttachmentDescription(
        {}, colorFormats.at(i), sampleCount,
        colorTargetLayouts[i].first == vk::ImageLayout::eUndefined ? vk::AttachmentLoadOp::eClear
                                                                   : vk::AttachmentLoadOp::eLoad,
        vk::AttachmentStoreOp::eStore, vk::AttachmentLoadOp::eDontCare,
        vk::AttachmentStoreOp::eDontCare, colorTargetLayouts[i].first,
        colorTargetLayouts[i].second));
  }

  std::vector<vk::AttachmentReference> resolveAttachmentRefs;
  if (sampleCount != vk::SampleCountFlagBits::e1) {
    for (uint32_t i = 0; i < elems.size(); ++i, ++attachmentIndex) {
      resolveAttachmentRefs.push_back({attachmentIndex, vk::ImageLayout::eColorAttachmentOptimal});

      attachmentDescriptions.push_back(vk::AttachmentDescription(
          {}, colorFormats.at(i), vk::SampleCountFlagBits::e1, vk::AttachmentLoadOp::eDontCare,
          vk::AttachmentStoreOp::eStore, vk::AttachmentLoadOp::eDontCare,
          vk::AttachmentStoreOp::eDontCare, vk::ImageLayout::eUndefined,
          vk::ImageLayout::eTransferSrcOptimal));
    }
  }

  attachmentDescriptions.push_back(vk::AttachmentDescription(
      vk::AttachmentDescriptionFlags(), depthFormat, sampleCount,
      depthLayout.first == vk::ImageLayout::eUndefined ? vk::AttachmentLoadOp::eClear
                                                       : vk::AttachmentLoadOp::eLoad,
      vk::AttachmentStoreOp::eStore, vk::AttachmentLoadOp::eDontCare,
      vk::AttachmentStoreOp::eDontCare, depthLayout.first, depthLayout.second));

  vk::AttachmentReference depthAttachment(attachmentIndex,
                                          vk::ImageLayout::eDepthStencilAttachmentOptimal);

  vk::SubpassDescription subpassDescription({}, vk::PipelineBindPoint::eGraphics, {},
                                            colorAttachmentRefs, resolveAttachmentRefs,
                                            &depthAttachment);

  // vk::SubpassDescription subpassDescription(
  //     {}, vk::PipelineBindPoint::eGraphics, 0, nullptr,
  //     colorAttachmentRefs.size(), colorAttachmentRefs.data(), nullptr,
  //     &depthAttachment);

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
      vk::RenderPassCreateInfo({}, attachmentDescriptions, subpassDescription, deps));

  // return device.createRenderPassUnique(vk::RenderPassCreateInfo(
  //     {}, attachmentDescriptions.size(), attachmentDescriptions.data(), 1,
  //     &subpassDescription, 2, deps.data()));
}

vk::UniquePipeline GbufferPassParser::createPipeline(
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
  auto vertexInputBindingDescriptions =
      mVertexInputLayout->computeVertexInputBindingDescriptions();
  auto vertexInputAttributeDescriptions =
      mVertexInputLayout->computeVertexInputAttributesDescriptions();
  vk::PipelineVertexInputStateCreateInfo pipelineVertexInputStateCreateInfo;
  pipelineVertexInputStateCreateInfo.vertexBindingDescriptionCount =
      vertexInputBindingDescriptions.size();
  pipelineVertexInputStateCreateInfo.pVertexBindingDescriptions =
      vertexInputBindingDescriptions.data();
  pipelineVertexInputStateCreateInfo.vertexAttributeDescriptionCount =
      vertexInputAttributeDescriptions.size();
  pipelineVertexInputStateCreateInfo.pVertexAttributeDescriptions =
      vertexInputAttributeDescriptions.data();

  // input assembly
  vk::PipelineInputAssemblyStateCreateInfo pipelineInputAssemblyStateCreateInfo(
      vk::PipelineInputAssemblyStateCreateFlags(), vk::PrimitiveTopology::eTriangleList);

  // viewport
  vk::PipelineViewportStateCreateInfo pipelineViewportStateCreateInfo(
      vk::PipelineViewportStateCreateFlags(), 1, nullptr, 1, nullptr);

  // rasterization
  vk::PipelineRasterizationStateCreateInfo pipelineRasterizationStateCreateInfo(
      vk::PipelineRasterizationStateCreateFlags(), false, false, mPolygonMode, cullMode, frontFace,
      false, 0.0f, 0.0f, 0.0f, 1.0f);

  // multisample
  vk::PipelineMultisampleStateCreateInfo pipelineMultisampleStateCreateInfo{{}, sampleCount};

  // stencil
  vk::StencilOpState stencilOpState{};
  vk::PipelineDepthStencilStateCreateInfo pipelineDepthStencilStateCreateInfo(
      vk::PipelineDepthStencilStateCreateFlags(), true, true, vk::CompareOp::eLessOrEqual, false,
      false, stencilOpState, stencilOpState);

  // blend
  uint32_t numColorAttachments = mTextureOutputLayout->elements.size();
  vk::ColorComponentFlags colorComponentFlags(
      vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG |
      vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA);
  std::vector<vk::PipelineColorBlendAttachmentState> pipelineColorBlendAttachmentStates;

  auto outTextures = mTextureOutputLayout->getElementsSorted();
  for (uint32_t i = 0; i < numColorAttachments; ++i) {
    // alpha blend float textures
    if (alphaBlend && outTextures[i].dtype == DataType::FLOAT4()) {
      pipelineColorBlendAttachmentStates.push_back(vk::PipelineColorBlendAttachmentState(
          true, vk::BlendFactor::eSrcAlpha, vk::BlendFactor::eOneMinusSrcAlpha, vk::BlendOp::eAdd,
          vk::BlendFactor::eOne, vk::BlendFactor::eZero, vk::BlendOp::eAdd, colorComponentFlags));
    } else {
      pipelineColorBlendAttachmentStates.push_back(vk::PipelineColorBlendAttachmentState(
          false, vk::BlendFactor::eZero, vk::BlendFactor::eZero, vk::BlendOp::eAdd,
          vk::BlendFactor::eZero, vk::BlendFactor::eZero, vk::BlendOp::eAdd, colorComponentFlags));
    }
  }
  vk::PipelineColorBlendStateCreateInfo pipelineColorBlendStateCreateInfo(
      vk::PipelineColorBlendStateCreateFlags(), false, vk::LogicOp::eNoOp, numColorAttachments,
      pipelineColorBlendAttachmentStates.data(), {{1.0f, 1.0f, 1.0f, 1.0f}});

  // dynamic
  std::vector<vk::DynamicState> dynamicStates = {
      vk::DynamicState::eViewport, vk::DynamicState::eScissor, vk::DynamicState::eCullMode};
  vk::PipelineDynamicStateCreateInfo pipelineDynamicStateCreateInfo(
      vk::PipelineDynamicStateCreateFlags(), dynamicStates);

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

std::vector<std::string> GbufferPassParser::getColorRenderTargetNames() const {
  std::vector<std::string> result;
  auto elems = mTextureOutputLayout->getElementsSorted();
  for (auto elem : elems) {
    result.push_back(getOutTextureName(elem.name));
  }
  return result;
}

void GbufferPassParser::setDepthRenderTargetName(std::string const &name) {
  mDepthRenderTargetName = name;
}

std::optional<std::string> GbufferPassParser::getDepthRenderTargetName() const {
  if (mDepthRenderTargetName.empty()) {
    return getName() + "Depth";
  }
  return mDepthRenderTargetName;
}

std::vector<UniformBindingType> GbufferPassParser::getUniformBindingTypes() const {
  std::vector<UniformBindingType> result;
  for (auto &desc : mDescriptorSetDescriptions) {
    result.push_back(desc.type);
  }
  return result;
}

} // namespace shader
} // namespace svulkan2