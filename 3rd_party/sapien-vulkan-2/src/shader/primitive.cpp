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
#include "svulkan2/shader/primitive.h"
#include "../common/logger.h"
#include "reflect.h"

namespace svulkan2 {
namespace shader {

void PrimitivePassParser::reflectSPV() {
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
    // if (mSpecializationConstantLayout->elements.contains("RESOLUTION_SCALE"))
    // {
    //   mResolutionScale =
    //       mSpecializationConstantLayout->elements["RESOLUTION_SCALE"]
    //           .floatValue;
    // }

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
      throw std::runtime_error("line: descriptor sets should use "
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

void PrimitivePassParser::validate() const {
  for (auto &elem : mTextureOutputLayout->elements) {
    ASSERT(elem.second.name.substr(0, 3) == "out",
           "[frag]all out texture variables must start with \"out\"");
  }
};

vk::UniqueRenderPass PrimitivePassParser::createRenderPass(
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
        {}, colorFormats.at(i), sampleCount,
        colorTargetLayouts[i].first == vk::ImageLayout::eUndefined ? vk::AttachmentLoadOp::eClear
                                                                   : vk::AttachmentLoadOp::eLoad,
        vk::AttachmentStoreOp::eStore, vk::AttachmentLoadOp::eDontCare,
        vk::AttachmentStoreOp::eDontCare, colorTargetLayouts[i].first,
        colorTargetLayouts[i].second));
  }
  attachmentDescriptions.push_back(vk::AttachmentDescription(
      vk::AttachmentDescriptionFlags(), depthFormat, sampleCount,
      depthLayout.first == vk::ImageLayout::eUndefined ? vk::AttachmentLoadOp::eClear
                                                       : vk::AttachmentLoadOp::eLoad,
      vk::AttachmentStoreOp::eStore, vk::AttachmentLoadOp::eDontCare,
      vk::AttachmentStoreOp::eDontCare, depthLayout.first, depthLayout.second));

  vk::AttachmentReference depthAttachment(elems.size(),
                                          vk::ImageLayout::eDepthStencilAttachmentOptimal);

  vk::SubpassDescription subpassDescription({}, vk::PipelineBindPoint::eGraphics, 0, nullptr,
                                            colorAttachments.size(), colorAttachments.data(),
                                            nullptr, &depthAttachment);

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

vk::UniquePipeline PrimitivePassParser::createPipelineHelper(
    vk::Device device, vk::PipelineLayout layout, vk::RenderPass renderPass,
    vk::CullModeFlags cullMode, vk::FrontFace frontFace, bool alphaBlend,
    vk::SampleCountFlagBits sampleCount,
    std::map<std::string, SpecializationConstantValue> const &specializationConstantInfo,
    int primitiveType // 0 for point, 1 for line
) const {

  // shaders
  vk::UniquePipelineCache pipelineCache =
      device.createPipelineCacheUnique(vk::PipelineCacheCreateInfo());
  auto vsm = device.createShaderModuleUnique(
      {{}, mVertSPVCode.size() * sizeof(uint32_t), mVertSPVCode.data()});
  auto fsm = device.createShaderModuleUnique(
      {{}, mFragSPVCode.size() * sizeof(uint32_t), mFragSPVCode.data()});
  vk::UniqueShaderModule gsm;
  if (mGeomSPVCode.size()) {
    gsm = device.createShaderModuleUnique(
        {{}, mGeomSPVCode.size() * sizeof(uint32_t), mGeomSPVCode.data()});
  }

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

  // auto elems = mSpecializationConstantLayout->getElementsSorted();
  // vk::SpecializationInfo fragSpecializationInfo;
  // std::vector<vk::SpecializationMapEntry> entries;
  // std::vector<int> specializationData;
  // if (elems.size()) {
  //   specializationData.resize(elems.size());
  //   for (uint32_t i = 0; i < elems.size(); ++i) {
  //     if (specializationConstantInfo.find(elems[i].name) !=
  //             specializationConstantInfo.end() &&
  //         elems[i].dtype !=
  //             specializationConstantInfo.at(elems[i].name).dtype) {
  //       throw std::runtime_error("Type mismatch on specialization constant " +
  //                                elems[i].name + ".");
  //     }
  //     if (elems[i].dtype == DataType::INT()) {
  //       entries.emplace_back(elems[i].id, i * sizeof(int), sizeof(int));
  //       int v = specializationConstantInfo.find(elems[i].name) !=
  //                       specializationConstantInfo.end()
  //                   ? specializationConstantInfo.at(elems[i].name).intValue
  //                   : elems[i].intValue;
  //       std::memcpy(specializationData.data() + i, &v, sizeof(int));
  //     } else if (elems[i].dtype == DataType::FLOAT()) {
  //       entries.emplace_back(elems[i].id, i * sizeof(float), sizeof(float));
  //       float v = specializationConstantInfo.find(elems[i].name) !=
  //                         specializationConstantInfo.end()
  //                     ? specializationConstantInfo.at(elems[i].name).floatValue
  //                     : elems[i].floatValue;
  //       std::memcpy(specializationData.data() + i, &v, sizeof(float));
  //     } else {
  //       throw std::runtime_error(
  //           "only int and float are allowed specialization constants");
  //     }
  //   }
  //   fragSpecializationInfo = vk::SpecializationInfo(
  //       entries.size(), entries.data(), specializationData.size() * sizeof(int),
  //       specializationData.data());
  // }

  std::vector<vk::PipelineShaderStageCreateInfo> pipelineShaderStageCreateInfos = {
      vk::PipelineShaderStageCreateInfo(vk::PipelineShaderStageCreateFlags(),
                                        vk::ShaderStageFlagBits::eVertex, vsm.get(), "main",
                                        nullptr),
      vk::PipelineShaderStageCreateInfo(vk::PipelineShaderStageCreateFlags(),
                                        vk::ShaderStageFlagBits::eFragment, fsm.get(), "main",
                                        elems.size() ? &fragSpecializationInfo : nullptr)};
  if (gsm) {
    pipelineShaderStageCreateInfos.push_back(vk::PipelineShaderStageCreateInfo(
        vk::PipelineShaderStageCreateFlags(), vk::ShaderStageFlagBits::eGeometry, gsm.get(),
        "main", nullptr));
  }

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
      vk::PipelineInputAssemblyStateCreateFlags(),
      primitiveType == 0 ? vk::PrimitiveTopology::ePointList : vk::PrimitiveTopology::eLineList);

  // viewport
  vk::PipelineViewportStateCreateInfo pipelineViewportStateCreateInfo(
      vk::PipelineViewportStateCreateFlags(), 1, nullptr, 1, nullptr);

  // rasterization
  vk::PipelineRasterizationStateCreateInfo pipelineRasterizationStateCreateInfo(
      vk::PipelineRasterizationStateCreateFlags(), false, false, vk::PolygonMode::eFill, cullMode,
      frontFace, false, 0.0f, 0.0f, 0.f, 1.f);

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
  std::vector<vk::DynamicState> dynamicStates;

  if (primitiveType == 0) {
    dynamicStates =
        std::vector<vk::DynamicState>{vk::DynamicState::eViewport, vk::DynamicState::eScissor};
  } else {
    dynamicStates = std::vector<vk::DynamicState>{
        vk::DynamicState::eViewport, vk::DynamicState::eScissor, vk::DynamicState::eLineWidth};
  }

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

std::vector<std::string> PrimitivePassParser::getColorRenderTargetNames() const {
  std::vector<std::string> result;
  auto elems = mTextureOutputLayout->getElementsSorted();
  for (auto elem : elems) {
    result.push_back(getOutTextureName(elem.name));
  }
  return result;
}

std::optional<std::string> PrimitivePassParser::getDepthRenderTargetName() const {
  return getName() + "Depth";
}

std::vector<UniformBindingType> PrimitivePassParser::getUniformBindingTypes() const {
  std::vector<UniformBindingType> result;
  for (auto &desc : mDescriptorSetDescriptions) {
    result.push_back(desc.type);
  }
  return result;
}

vk::UniquePipeline PointPassParser::createPipeline(
    vk::Device device, vk::PipelineLayout layout, vk::RenderPass renderPass,
    vk::CullModeFlags cullMode, vk::FrontFace frontFace, bool alphaBlend,
    vk::SampleCountFlagBits sampleCount,
    std::map<std::string, SpecializationConstantValue> const &specializationConstantInfo) const {
  return createPipelineHelper(device, layout, renderPass, cullMode, frontFace, alphaBlend,
                              sampleCount, specializationConstantInfo, 0);
}

vk::UniquePipeline LinePassParser::createPipeline(
    vk::Device device, vk::PipelineLayout layout, vk::RenderPass renderPass,
    vk::CullModeFlags cullMode, vk::FrontFace frontFace, bool alphaBlend,
    vk::SampleCountFlagBits sampleCount,
    std::map<std::string, SpecializationConstantValue> const &specializationConstantInfo) const {
  return createPipelineHelper(device, layout, renderPass, cullMode, frontFace, alphaBlend,
                              sampleCount, specializationConstantInfo, 1);
}

} // namespace shader
} // namespace svulkan2