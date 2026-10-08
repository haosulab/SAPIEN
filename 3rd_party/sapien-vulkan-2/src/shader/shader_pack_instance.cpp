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
#include "svulkan2/shader/shader_pack_instance.h"
#include "../common/logger.h"
#include "svulkan2/common/launch_policy.h"
#include "svulkan2/core/context.h"
#include "svulkan2/shader/deferred.h"
#include "svulkan2/shader/gbuffer.h"
#include "svulkan2/shader/primitive.h"
#include "svulkan2/shader/primitive_shadow.h"
#include "svulkan2/shader/shadow.h"
#include <set>

namespace svulkan2 {
namespace shader {

ShaderPackInstance::ShaderPackInstance(ShaderPackInstanceDesc const &desc) : mDesc(desc) {
  // compile GLSL immediately
  mContext = core::Context::Get();
  mShaderPack = mContext->getResourceManager()->CreateShaderPack(mDesc.config->shaderDir);
}

static vk::UniqueDescriptorSetLayout
createObjectDescriptorSetLayout(vk::Device device,
                                DescriptorSetDescription const &objectSetDescription) {
  std::vector<vk::DescriptorSetLayoutBinding> bindings;
  bindings.push_back({0, vk::DescriptorType::eUniformBuffer, 1,
                      vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment});
  bindings.push_back({1, vk::DescriptorType::eUniformBuffer, 1,
                      vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment});
  for (uint32_t bid = 2; bid < objectSetDescription.bindings.size(); ++bid) {
    // TODO: verify bid is consecutive
    auto binding = objectSetDescription.bindings.at(bid);
    if (binding.type == vk::DescriptorType::eCombinedImageSampler) {
      if (binding.dim == 0) {
        bindings.push_back(
            {bid, vk::DescriptorType::eCombinedImageSampler, 1,
             vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment});
      } else {
        uint32_t arraySize = binding.arraySize;
        bindings.push_back(
            {bid, vk::DescriptorType::eCombinedImageSampler, arraySize,
             vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment});
      }
    }
  }

  return device.createDescriptorSetLayoutUnique(vk::DescriptorSetLayoutCreateInfo({}, bindings));
}

static vk::UniqueDescriptorSetLayout createCameraDescriptorSetLayout(vk::Device device) {
  auto binding = vk::DescriptorSetLayoutBinding(0, vk::DescriptorType::eUniformBuffer, 1,
                                                vk::ShaderStageFlagBits::eVertex |
                                                    vk::ShaderStageFlagBits::eFragment);
  return device.createDescriptorSetLayoutUnique(vk::DescriptorSetLayoutCreateInfo({}, binding));
}

static vk::UniqueDescriptorSetLayout
createSceneDescriptorSetLayout(vk::Device device,
                               DescriptorSetDescription const &sceneSetDescription) {
  std::vector<vk::DescriptorSetLayoutBinding> bindings;
  std::vector<vk::DescriptorBindingFlags> bindingFlags;
  for (uint32_t bindingIndex = 0; bindingIndex < sceneSetDescription.bindings.size();
       ++bindingIndex) {
    if (sceneSetDescription.bindings.at(bindingIndex).type == vk::DescriptorType::eUniformBuffer) {
      bindings.push_back({bindingIndex, vk::DescriptorType::eUniformBuffer, 1,
                          vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment});
      bindingFlags.push_back({});
    } else if (sceneSetDescription.bindings.at(bindingIndex).type ==
               vk::DescriptorType::eCombinedImageSampler) {
      if (sceneSetDescription.bindings.at(bindingIndex).dim == 0) {
        bindings.push_back(
            {bindingIndex, vk::DescriptorType::eCombinedImageSampler, 1,
             vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment});
        bindingFlags.push_back({});
      } else {
        uint32_t arraySize = sceneSetDescription.bindings.at(bindingIndex).arraySize;
        bindings.push_back(
            {bindingIndex, vk::DescriptorType::eCombinedImageSampler, arraySize,
             vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment});
        bindingFlags.push_back(vk::DescriptorBindingFlagBits::ePartiallyBound);
      }
    } else {
      throw std::runtime_error("invalid scene descriptor set");
    }
  }
  vk::DescriptorSetLayoutBindingFlagsCreateInfo flagInfo(bindingFlags);
  vk::DescriptorSetLayoutCreateInfo layoutInfo({}, bindings);
  layoutInfo.setPNext(&flagInfo);
  return device.createDescriptorSetLayoutUnique(layoutInfo);
}

static vk::UniqueDescriptorSetLayout createLightDescriptorSetLayout(vk::Device device) {
  auto binding = vk::DescriptorSetLayoutBinding(0, vk::DescriptorType::eUniformBuffer, 1,
                                                vk::ShaderStageFlagBits::eVertex);
  return device.createDescriptorSetLayoutUnique(vk::DescriptorSetLayoutCreateInfo({}, binding));
}

static std::vector<vk::UniqueDescriptorSetLayout>
createTextureDescriptorSetLayouts(vk::Device device,
                                  std::vector<std::shared_ptr<BaseParser>> const &passes) {
  std::vector<vk::UniqueDescriptorSetLayout> textureLayouts;
  for (auto &p : passes) {
    auto types = p->getUniformBindingTypes();
    vk::UniqueDescriptorSetLayout layout{};
    for (auto type : types) {
      if (type == shader::UniformBindingType::eTextures) {
        std::vector<vk::DescriptorSetLayoutBinding> bindings;
        auto texs = p->getInputTextureNames();
        for (uint32_t i = 0; i < texs.size(); ++i) {
          bindings.push_back(
              vk::DescriptorSetLayoutBinding(i, vk::DescriptorType::eCombinedImageSampler, 1,
                                             vk::ShaderStageFlagBits::eFragment));
        }
        layout = device.createDescriptorSetLayoutUnique(
            vk::DescriptorSetLayoutCreateInfo({}, bindings));
        break;
      }
    }
    textureLayouts.push_back(std::move(layout));
  }
  return textureLayouts;
}

static RenderTargetOperation getPrevOperation(optable_t optable, std::string texName,
                                              std::shared_ptr<BaseParser> pass) {
  for (int i = pass->getIndex() - 1; i >= 0; i--) {
    auto op = optable.at(texName)[i];
    if (op != RenderTargetOperation::eNoOp) {
      return op;
    }
  }
  return RenderTargetOperation::eNoOp;
}

static RenderTargetOperation getNextOperation(optable_t optable, std::string texName,
                                              std::shared_ptr<BaseParser> pass) {
  auto ops = optable.at(texName);
  for (uint32_t i = pass->getIndex() + 1; i < ops.size(); i++) {
    auto op = ops.at(i);
    if (op != RenderTargetOperation::eNoOp) {
      return op;
    }
  }
  return RenderTargetOperation::eNoOp;
}

static RenderTargetOperation getLastOperation(optable_t optable, std::string texName) {
  auto ops = optable.at(texName);
  for (int i = ops.size() - 1; i >= 0; i--) {
    auto op = ops.at(i);
    if (op != RenderTargetOperation::eNoOp) {
      return op;
    }
  }
  throw std::runtime_error("invalid last operation on " + texName);
}

static std::vector<std::pair<vk::ImageLayout, vk::ImageLayout>>
getColorAttachmentLayoutsForPass(optable_t optable, std::shared_ptr<BaseParser> pass) {
  auto elems = pass->getTextureOutputLayout()->getElementsSorted();
  std::vector<std::pair<vk::ImageLayout, vk::ImageLayout>> result(elems.size());
  for (uint32_t i = 0; i < elems.size(); ++i) {
    ASSERT(elems[i].location == i, "output textures must have consecutive binding locations");
    auto texName = getOutTextureName(elems[i].name);
    auto prevOp = getPrevOperation(optable, texName, pass);
    auto nextOp = getNextOperation(optable, texName, pass);
    if (prevOp == RenderTargetOperation::eNoOp) {
      result[i].first = vk::ImageLayout::eUndefined;
    } else if (prevOp == RenderTargetOperation::eRead) {
      result[i].first = vk::ImageLayout::eShaderReadOnlyOptimal;
    } else {
      result[i].first = vk::ImageLayout::eColorAttachmentOptimal;
    }
    switch (nextOp) {
    case RenderTargetOperation::eNoOp:
      result[i].second = vk::ImageLayout::eColorAttachmentOptimal;
      break;
    case RenderTargetOperation::eRead:
      result[i].second = vk::ImageLayout::eShaderReadOnlyOptimal;
      break;
    case RenderTargetOperation::eColorWrite:
      result[i].second = vk::ImageLayout::eColorAttachmentOptimal;
      break;
    case RenderTargetOperation::eDepthWrite:
      throw std::runtime_error("invalid depth attachment layout");
    }
  }
  return result;
}

static std::pair<vk::ImageLayout, vk::ImageLayout>
getDepthAttachmentLayoutsForPass(optable_t optable, std::shared_ptr<BaseParser> pass,
                                 std::optional<std::string> const &depthName) {
  if (!depthName.has_value()) {
    return {vk::ImageLayout::eUndefined, vk::ImageLayout::eUndefined};
  }
  std::string texName = depthName.value();
  auto prevOp = getPrevOperation(optable, texName, pass);
  auto nextOp = getNextOperation(optable, texName, pass);
  vk::ImageLayout prev = vk::ImageLayout::eUndefined;
  vk::ImageLayout next = vk::ImageLayout::eUndefined;
  switch (prevOp) {
  case RenderTargetOperation::eNoOp:
    prev = vk::ImageLayout::eUndefined;
    break;
  case RenderTargetOperation::eRead:
    prev = vk::ImageLayout::eShaderReadOnlyOptimal;
    break;
  case RenderTargetOperation::eDepthWrite:
    prev = vk::ImageLayout::eDepthStencilAttachmentOptimal;
    break;
  case RenderTargetOperation::eColorWrite:
    throw std::runtime_error("invalid depth attachment layout");
  }
  switch (nextOp) {
  case RenderTargetOperation::eNoOp:
    next = vk::ImageLayout::eDepthStencilAttachmentOptimal;
    break;
  case RenderTargetOperation::eRead:
    next = vk::ImageLayout::eShaderReadOnlyOptimal;
    break;
  case RenderTargetOperation::eDepthWrite:
    next = vk::ImageLayout::eDepthStencilAttachmentOptimal;
    break;
  case RenderTargetOperation::eColorWrite:
    throw std::runtime_error("invalid depth attachment layout");
  }
  return {prev, next};
}

static vk::Format getRenderTargetFormat(RendererConfig const &config,
                                        OutputDataLayout::Element const &element) {
  auto texname = getOutTextureName(element.name);
  if (config.textureFormat.contains(texname)) {
    return config.textureFormat.at(texname);
  }

  if (element.dtype.kind == TypeKind::eFloat) {
    if (element.dtype.bytes == 1) {
      return config.colorFormat1;
    }
    if (element.dtype.bytes == 4) {
      return config.colorFormat4;
    }
  } else if (element.dtype.kind == TypeKind::eInt) {
    if (element.dtype.bytes == 1) {
      return vk::Format::eR32Sint;
    }
    if (element.dtype.bytes == 4) {
      return vk::Format::eR32G32B32A32Sint;
    }
  } else if (element.dtype.kind == TypeKind::eUint) {
    if (element.dtype.bytes == 1) {
      return vk::Format::eR32Uint;
    }
    if (element.dtype.bytes == 4) {
      return vk::Format::eR32G32B32A32Uint;
    }
  }
  throw std::runtime_error(
      "invalid output attachment format (supported: float, float4, int, int4, uint, uint4)");
}

std::future<void> ShaderPackInstance::loadAsync() {
  if (mLoaded) {
    return std::async(std::launch::deferred, []() {});
  }
  return std::async(LAUNCH_ASYNC, [this]() {
    std::lock_guard<std::mutex> lock(mLoadingMutex);
    if (mLoaded) {
      return;
    }

    vk::Device device = mContext->getDevice();

    auto textureOperationTable = generateTextureOperationTable();
    auto inputLayouts = mShaderPack->getShaderInputLayouts();
    auto passes = mShaderPack->getNonShadowPasses();

    // descriptor set layout
    mSceneSetLayout = createSceneDescriptorSetLayout(device, inputLayouts->sceneSetDescription);
    mObjectSetLayout = createObjectDescriptorSetLayout(device, inputLayouts->objectSetDescription);
    mCameraSetLayout = createCameraDescriptorSetLayout(device);
    mLightSetLayout = createLightDescriptorSetLayout(device);
    mTextureSetLayouts = createTextureDescriptorSetLayouts(device, passes);

    // enable msaa only in the forward pipeline
    vk::SampleCountFlagBits msaa = vk::SampleCountFlagBits::e1;
    if (!mShaderPack->hasDeferredPass()) {
      msaa = mDesc.config->msaa;
    }

    // non shadow
    for (uint32_t passIdx = 0; passIdx < passes.size(); ++passIdx) {
      auto pass = passes[passIdx];
      std::vector<vk::DescriptorSetLayout> descriptorSetLayouts;
      for (auto type : pass->getUniformBindingTypes()) {
        switch (type) {
        case UniformBindingType::eCamera:
          descriptorSetLayouts.push_back(mCameraSetLayout.get());
          break;
        case UniformBindingType::eObject:
          descriptorSetLayouts.push_back(mObjectSetLayout.get());
          break;
        case UniformBindingType::eScene:
          descriptorSetLayouts.push_back(mSceneSetLayout.get());
          break;
        case UniformBindingType::eMaterial:
          descriptorSetLayouts.push_back(mContext->getMetallicDescriptorSetLayout());
          break;
        case UniformBindingType::eTextures:
          descriptorSetLayouts.push_back(mTextureSetLayouts[passIdx].get());
          break;
        default:
          throw std::runtime_error("ShaderManager::createPipelines: not "
                                   "implemented uniform binding type");
        }
      }
      std::vector<vk::Format> formats;
      for (auto layout : pass->getTextureOutputLayout()->getElementsSorted()) {
        formats.push_back(getRenderTargetFormat(*mDesc.config, layout));
      }

      // determine whether to use alpha blend
      // TODO: enable alpha based on config
      bool alpha = false;
      if (auto p = std::dynamic_pointer_cast<GbufferPassParser>(pass)) {
        if (!mShaderPack->hasDeferredPass() || p != mShaderPack->getGbufferPasses().at(0)) {
          alpha = true;
        }
      }

      PipelineResources res;
      res.layout = pass->createPipelineLayout(device, descriptorSetLayouts);
      res.renderPass =
          pass->createRenderPass(device, formats, mDesc.config->depthFormat,
                                 getColorAttachmentLayoutsForPass(textureOperationTable, pass),
                                 getDepthAttachmentLayoutsForPass(textureOperationTable, pass,
                                                                  getDepthRenderTargetName(*pass)),
                                 msaa);
      res.pipeline = pass->createPipeline(
          device, res.layout.get(), res.renderPass.get(), vk::CullModeFlagBits::eBack,
          vk::FrontFace::eCounterClockwise, alpha, msaa, mDesc.specializationConstants);

      mNonShadowPassResources.push_back(std::move(res));
    }

    // shadow
    auto shadowPass = mShaderPack->getShadowPass();
    if (shadowPass) {
      std::vector<vk::DescriptorSetLayout> descriptorSetLayouts;
      for (auto type : shadowPass->getUniformBindingTypes()) {
        switch (type) {
        case UniformBindingType::eObject:
          descriptorSetLayouts.push_back(mObjectSetLayout.get());
          break;
        case UniformBindingType::eLight:
          descriptorSetLayouts.push_back(mLightSetLayout.get());
          break;
        case UniformBindingType::eMaterial:
        case UniformBindingType::eCamera:
        case UniformBindingType::eScene:
          throw std::runtime_error("shadow pass may only use object and light buffers");
        default:
          throw std::runtime_error("ShaderManager::createPipelines: not "
                                   "implemented uniform binding type");
        }
      }

      PipelineResources res;
      res.layout = shadowPass->createPipelineLayout(device, descriptorSetLayouts);
      res.renderPass = shadowPass->createRenderPass(
          device, {}, mDesc.config->depthFormat, {},
          {vk::ImageLayout::eUndefined, vk::ImageLayout::eShaderReadOnlyOptimal},
          vk::SampleCountFlagBits::e1);
      res.pipeline = shadowPass->createPipeline(
          device, res.layout.get(), res.renderPass.get(), vk::CullModeFlagBits::eFront,
          vk::FrontFace::eCounterClockwise, false, msaa, mDesc.specializationConstants);

      mShadowPassResources = std::move(res);
    }

    // point shadow
    auto pointShadowPass = mShaderPack->getPointShadowPass();
    if (pointShadowPass) {
      std::vector<vk::DescriptorSetLayout> descriptorSetLayouts;
      for (auto type : pointShadowPass->getUniformBindingTypes()) {
        switch (type) {
        case UniformBindingType::eObject:
          descriptorSetLayouts.push_back(mObjectSetLayout.get());
          break;
        case UniformBindingType::eLight:
          descriptorSetLayouts.push_back(mLightSetLayout.get());
          break;
        case UniformBindingType::eMaterial:
        case UniformBindingType::eCamera:
        case UniformBindingType::eScene:
          throw std::runtime_error("pointShadow pass may only use object and light buffers");
        default:
          throw std::runtime_error("ShaderManager::createPipelines: not "
                                   "implemented uniform binding type");
        }
      }

      std::pair<vk::ImageLayout, vk::ImageLayout> layouts;
      if (shadowPass) {
        layouts = {vk::ImageLayout::eShaderReadOnlyOptimal,
                   vk::ImageLayout::eShaderReadOnlyOptimal};
      } else {
        layouts = {vk::ImageLayout::eUndefined, vk::ImageLayout::eShaderReadOnlyOptimal};
      }

      PipelineResources res;
      res.layout = pointShadowPass->createPipelineLayout(device, descriptorSetLayouts);
      res.renderPass = pointShadowPass->createRenderPass(device, {}, mDesc.config->depthFormat, {},
                                                         layouts, vk::SampleCountFlagBits::e1);
      res.pipeline = pointShadowPass->createPipeline(
          device, res.layout.get(), res.renderPass.get(), vk::CullModeFlagBits::eFront,
          vk::FrontFace::eCounterClockwise, false, msaa, mDesc.specializationConstants);

      mPointShadowPassResources = std::move(res);
    }

    // generate render target formats
    std::unordered_map<std::string, vk::Format> formats;
    for (auto pass : passes) {
      auto depthName = getDepthRenderTargetName(*pass);
      if (depthName.has_value()) {
        mRenderTargetFormats[depthName.value()] = mDesc.config->depthFormat;
      }
      for (auto &elem : pass->getTextureOutputLayout()->elements) {
        std::string texName = getOutTextureName(elem.second.name);
        if (texName.ends_with("Depth")) {
          throw std::runtime_error("You are not allowed to name your texture \"*Depth\"");
        }
        vk::Format texFormat = getRenderTargetFormat(*mDesc.config, elem.second);

        if (mRenderTargetFormats.find(texName) != mRenderTargetFormats.end()) {
          if (mRenderTargetFormats.at(texName) != texFormat) {
            std::runtime_error("Inconsistent texture format for \"" + texName + "\"");
          }
        }
        mRenderTargetFormats[texName] = texFormat;
      }
    }

    // generate render target final layouts
    for (auto tex : mRenderTargetFormats) {
      auto op = getLastOperation(textureOperationTable, tex.first);
      switch (op) {
      case RenderTargetOperation::eRead:
        mRenderTargetFinalLayouts[tex.first] = vk::ImageLayout::eShaderReadOnlyOptimal;
        break;
      case RenderTargetOperation::eColorWrite:
        mRenderTargetFinalLayouts[tex.first] = vk::ImageLayout::eColorAttachmentOptimal;
        break;
      case RenderTargetOperation::eDepthWrite:
        mRenderTargetFinalLayouts[tex.first] = vk::ImageLayout::eDepthStencilAttachmentOptimal;
        break;
      case RenderTargetOperation::eNoOp:
        throw std::runtime_error("invalid render target");
      }
    }

    mLoaded = true;
  });
}

optable_t ShaderPackInstance::generateTextureOperationTable() const {
  optable_t operationTable;

  // TODO texture format
  std::set<std::string> textureNames;
  std::vector<std::string> textureNamesOrdered;

  auto passes = mShaderPack->getNonShadowPasses();
  for (auto pass : passes) {
    auto depthName = getDepthRenderTargetName(*pass);
    if (depthName.has_value()) {
      if (!textureNames.contains(depthName.value())) {
        textureNames.insert(depthName.value());
        textureNamesOrdered.push_back(depthName.value());
      }
    }
    for (auto &elem : pass->getTextureOutputLayout()->elements) {
      std::string texName = getOutTextureName(elem.second.name);
      if (texName.ends_with("Depth")) {
        throw std::runtime_error("Naming a texture \"*Depth\" is not allowed in a shader pack");
      }
      if (!textureNames.contains(texName)) {
        textureNames.insert(texName);
        textureNamesOrdered.push_back(texName);
      }
    }
  }

  // init op table
  for (auto name : textureNames) {
    operationTable[name] =
        std::vector<RenderTargetOperation>(passes.size(), RenderTargetOperation::eNoOp);
  }

  for (uint32_t passIdx = 0; passIdx < passes.size(); ++passIdx) {
    auto pass = passes[passIdx];
    for (auto name : pass->getInputTextureNames()) {
      if (operationTable.find(name) != operationTable.end()) {
        operationTable.at(name)[passIdx] = RenderTargetOperation::eRead;
      }
    }
    for (auto &name : pass->getColorRenderTargetNames()) {
      operationTable.at(name)[passIdx] = RenderTargetOperation::eColorWrite;
    }
    auto name = getDepthRenderTargetName(*pass);
    if (name.has_value()) {
      operationTable.at(name.value())[passIdx] = RenderTargetOperation::eDepthWrite;
    }
  }

  std::stringstream ss;
  ss << "Operation Table" << std::endl;
  ss << std::setw(20) << ""
     << " ";
  for (auto pass : passes) {
    ss << std::setw(15) << pass->getName() << " ";
  }
  ss << std::endl;

  for (auto &tex : textureNamesOrdered) {
    auto ops = operationTable.at(tex);
    ss << std::setw(20) << tex << " ";
    for (auto &op : ops) {
      if (op == RenderTargetOperation::eNoOp) {
        ss << std::setw(15) << " "
           << " ";
      } else if (op == RenderTargetOperation::eRead) {
        ss << std::setw(15) << "R"
           << " ";
      } else if (op == RenderTargetOperation::eColorWrite) {
        ss << std::setw(15) << "W"
           << " ";
      } else if (op == RenderTargetOperation::eDepthWrite) {
        ss << std::setw(15) << "D"
           << " ";
      }
    }
    ss << std::endl;
  }
  logger::info(ss.str());

  return operationTable;
}

std::optional<std::string>
ShaderPackInstance::getDepthRenderTargetName(BaseParser const &pass) const {
  if (mDesc.config->shareGbufferDepths && dynamic_cast<GbufferPassParser const *>(&pass)) {
    return "GbufferDepth";
  }
  return pass.getDepthRenderTargetName();
}

} // namespace shader
} // namespace svulkan2