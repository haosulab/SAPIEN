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
#include "svulkan2/renderer/rt_renderer.h"
#include "../common/logger.h"
#include "denoiser.h"
#include "svulkan2/core/context.h"
#include "svulkan2/shader/rt.h"

namespace svulkan2 {
namespace renderer {
RTRenderer::RTRenderer(std::string const &shaderDir) : mShaderDir(shaderDir) {
  mContext = core::Context::Get();
  if (!mContext->isVulkanAvailable()) {
    return;
  }
  if (!mContext->isRayTracingAvailable()) {
    logger::error("The selected GPU does not support ray tracing");
    return;
  }
  mShaderPack = mContext->getResourceManager()->CreateRTShaderPack(shaderDir);

  mMaterialBufferLayout = mShaderPack->getMaterialBufferLayout();
  mTextureIndexBufferLayout = mShaderPack->getTextureIndexBufferLayout();
  mGeometryInstanceBufferLayout = mShaderPack->getGeometryInstanceBufferLayout();
  mCameraBufferLayout = mShaderPack->getCameraBufferLayout();
  mObjectBufferLayout = mShaderPack->getObjectBufferLayout();

  // set default values
  mCustomPropertiesInt["spp"] = 4;
  mCustomPropertiesInt["maxDepth"] = 3;
  mCustomPropertiesInt["russianRoulette"] = 0;
  mCustomPropertiesInt["russianRouletteMinBounces"] = 2;
  mCustomPropertiesFloat["exposure"] = 1.f;
  mCustomPropertiesFloat["aperture"] = 0.f;
  mCustomPropertiesFloat["focusPlane"] = 1.f;

  mSceneAccessFence =
      mContext->getDevice().createFenceUnique({vk::FenceCreateFlagBits::eSignaled});
}

void RTRenderer::resize(int width, int height) {
  if (mWidth != width || mHeight != height) {
    mWidth = width;
    mHeight = height;
    mRequiresRebuild = true;
  }
}

void RTRenderer::setScene(std::shared_ptr<scene::Scene> scene) {
  if (mScene == scene) {
    return;
  }
  if (mScene) {
    mScene->unregisterAccessFence(mSceneAccessFence.get());
  }

  mScene = scene;
  mSceneVersion = 0l;
  mSceneRenderVersion = 0l;

  if (mScene) {
    mScene->registerAccessFence(mSceneAccessFence.get());
  }
}

void RTRenderer::prepareObjects() {
  auto objects = mScene->getObjects();
  // auto objects = mScene->getVisibleObjects();
  mObjectBuffer = core::Buffer::Create(
      std::max(size_t(1), objects.size()) * mObjectBufferLayout.size,
      vk::BufferUsageFlagBits::eStorageBuffer | vk::BufferUsageFlagBits::eTransferDst,
      VMA_MEMORY_USAGE_CPU_TO_GPU);
}

void RTRenderer::updateObjects() {
  auto objects = mScene->getObjects();
  // auto objects = mScene->getVisibleObjects();

  uint32_t segOffset = mObjectBufferLayout.elements.at("segmentation").offset;
  uint32_t transparencyOffset = mObjectBufferLayout.elements.at("transparency").offset;
  uint32_t shadeFlatOffset = mObjectBufferLayout.elements.at("shadeFlat").offset;

  uint8_t *memory = static_cast<uint8_t *>(mObjectBuffer->map());
  uint32_t stride = mObjectBufferLayout.size;

  uint32_t count = 0;
  for (auto obj : objects) {
    int shadeFlat = obj->getShadeFlat();
    glm::uvec4 seg = obj->getSegmentation();
    float transparency = obj->getTransparency();

    std::memcpy(memory + stride * count + segOffset, &seg, sizeof(glm::uvec4));
    std::memcpy(memory + stride * count + shadeFlatOffset, &shadeFlat, sizeof(int));
    std::memcpy(memory + stride * count + transparencyOffset, &transparency, sizeof(float));

    ++count;
  }
  mObjectBuffer->unmap();
}

void RTRenderer::prepareRender(scene::Camera &camera) {
  if (mEnvironmentMap != mScene->getEnvironmentMap()) {
    mEnvironmentMap = mScene->getEnvironmentMap();
    mEnvironmentMap->load();
    mRequiresRebuild = true;
  }

  auto objects = camera.getScene()->getObjects();
  // auto objects = camera.getScene()->getVisibleObjects();
  auto scene = camera.getScene();
  if (mSceneVersion != scene->getVersion()) {
    mRequiresRebuild = true;
  }

  if (mRequiresRebuild) {
    mFrameCount = 0;

    scene->buildRTResources(mMaterialBufferLayout, mTextureIndexBufferLayout,
                            mGeometryInstanceBufferLayout);

    mShaderPackInstance = std::make_shared<shader::RayTracingShaderPackInstance>(
        shader::RayTracingShaderPackInstanceDesc{
            .shaderDir = mShaderDir,
            .maxMeshes = static_cast<uint32_t>(mScene->getRTVertexBuffers().size()),
            .maxMaterials = static_cast<uint32_t>(mScene->getRTMaterialBuffers().size()),
            .maxTextures = static_cast<uint32_t>(mScene->getRTTextures().size()),
            .maxPointSets = static_cast<uint32_t>(mScene->getRTPointSetBuffers().size())});
    prepareOutput();
    prepareCamera();
    prepareScene();
    preparePostprocessing();

#ifdef SVULKAN2_CUDA_INTEROP
    if (mDenoiser) {
      if (static_cast<int>(mDenoiser->getWidth()) != mWidth ||
          static_cast<int>(mDenoiser->getHeight()) != mHeight) {
        mDenoiser->allocate(mWidth, mHeight);
      }
    }
#endif

    mSceneVersion = scene->getVersion();
  }

  if (mSceneRenderVersion != mScene->getRenderVersion() || mRequiresRebuild) {
    updateObjects();

    mScene->updateRTResources();                                // update TLAS
    camera.uploadToDevice(*mCameraBuffer, mCameraBufferLayout); // update camera
    mFrameCount = 0;
    mSceneRenderVersion = mScene->getRenderVersion();
  } else {
    mFrameCount += 1;
  }

  updatePushConstant();

  recordRender();
  recordPostprocess();

  mRequiresRebuild = false;
}

void RTRenderer::updatePushConstant() {
  auto pushConstantLayout = mShaderPack->getPushConstantLayout();
  mPushConstantBuffer.resize(pushConstantLayout->size);

  auto layout = mShaderPack->getPushConstantLayout();
  for (auto &[name, value] : mCustomPropertiesInt) {
    auto it = layout->elements.find(name);
    if (it != layout->elements.end() && it->second.dtype == DataType::INT()) {
      std::memcpy(mPushConstantBuffer.data() + it->second.offset, &value, it->second.size);
    }
  }
  for (auto &[name, value] : mCustomPropertiesFloat) {
    auto it = layout->elements.find(name);
    if (it != layout->elements.end() && it->second.dtype == DataType::FLOAT()) {
      std::memcpy(mPushConstantBuffer.data() + it->second.offset, &value, it->second.size);
    }
  }
  for (auto &[name, value] : mCustomPropertiesVec3) {
    auto it = layout->elements.find(name);
    if (it != layout->elements.end() && it->second.dtype == DataType::FLOAT3()) {
      std::memcpy(mPushConstantBuffer.data() + it->second.offset, &value, it->second.size);
    }
  }
  for (auto &[name, value] : mCustomPropertiesVec4) {
    auto it = layout->elements.find(name);
    if (it != layout->elements.end() && it->second.dtype == DataType::FLOAT4()) {
      std::memcpy(mPushConstantBuffer.data() + it->second.offset, &value, it->second.size);
    }
  }

  if (layout->elements.contains("pointLightCount")) {
    auto &elem = layout->elements.at("pointLightCount");
    if (elem.dtype != DataType::INT()) {
      throw std::runtime_error("pointLightCount must be type int");
    }
    uint32_t v = mScene->getPointLights().size();
    std::memcpy(mPushConstantBuffer.data() + elem.offset, &v, elem.size);
  }

  if (layout->elements.contains("directionalLightCount")) {
    auto &elem = layout->elements.at("directionalLightCount");
    if (elem.dtype != DataType::INT()) {
      throw std::runtime_error("directionalLightCount must be type int");
    }
    uint32_t v = mScene->getDirectionalLights().size();
    std::memcpy(mPushConstantBuffer.data() + elem.offset, &v, elem.size);
  }

  if (layout->elements.contains("spotLightCount")) {
    auto &elem = layout->elements.at("spotLightCount");
    if (elem.dtype != DataType::INT()) {
      throw std::runtime_error("spotLightCount must be type int");
    }
    uint32_t v = mScene->getSpotLights().size() + mScene->getTexturedLights().size();
    std::memcpy(mPushConstantBuffer.data() + elem.offset, &v, elem.size);
  }

  if (layout->elements.contains("parallelogramLightCount")) {
    auto &elem = layout->elements.at("parallelogramLightCount");
    if (elem.dtype != DataType::INT()) {
      throw std::runtime_error("parallelogramLightCount must be type int");
    }
    uint32_t v = mScene->getParallelogramLights().size();
    std::memcpy(mPushConstantBuffer.data() + elem.offset, &v, elem.size);
  }

  if (layout->elements.contains("frameCount")) {
    auto &elem = layout->elements.at("frameCount");
    if (elem.dtype != DataType::INT()) {
      throw std::runtime_error("frameCount must be type int");
    }
    std::memcpy(mPushConstantBuffer.data() + elem.offset, &mFrameCount, elem.size);
  }

  if (layout->elements.contains("envmap")) {
    auto &elem = layout->elements.at("envmap");
    if (elem.dtype != DataType::INT()) {
      throw std::runtime_error("envmap must be type int");
    }
    int envmap = mScene->getEnvironmentMap() != nullptr;
    std::memcpy(mPushConstantBuffer.data() + elem.offset, &envmap, elem.size);
  }

  if (layout->elements.contains("ambientLight")) {
    auto &elem = layout->elements.at("ambientLight");
    if (elem.dtype != DataType::FLOAT3()) {
      throw std::runtime_error("ambientLight must be type vec3");
    }
    glm::vec3 ambientLight = mScene->getAmbientLight();
    std::memcpy(mPushConstantBuffer.data() + elem.offset, &ambientLight, elem.size);
  }
}

void RTRenderer::recordRender() {
  if (!mRenderCommandPool) {
    mRenderCommandPool = mContext->createCommandPool();
    mRenderCommandBuffer = mRenderCommandPool->allocateCommandBuffer();
    mPostprocessCommandBuffer = mRenderCommandPool->allocateCommandBuffer();
  }

  mRenderCommandBuffer->reset();

  auto pushConstantLayout = mShaderPack->getPushConstantLayout();

  mRenderCommandBuffer->begin(vk::CommandBufferBeginInfo({}, {}));

  {
    // make sure AS building is done
    vk::MemoryBarrier barrier(vk::AccessFlagBits::eAccelerationStructureWriteKHR,
                              vk::AccessFlagBits::eAccelerationStructureReadKHR);
    mRenderCommandBuffer->pipelineBarrier(
        vk::PipelineStageFlagBits::eAccelerationStructureBuildKHR,
        vk::PipelineStageFlagBits::eRayTracingShaderKHR, {}, barrier, {}, {});
  }

  for (auto &image : mRTImages) {
    // make sure everything is done
    image->getImage().transitionLayout(
        mRenderCommandBuffer.get(), vk::ImageLayout::eGeneral, vk::ImageLayout::eGeneral,
        vk::AccessFlagBits::eMemoryRead | vk::AccessFlagBits::eMemoryWrite,
        vk::AccessFlagBits::eShaderRead | vk::AccessFlagBits::eShaderWrite,
        vk::PipelineStageFlagBits::eAllCommands, vk::PipelineStageFlagBits::eRayTracingShaderKHR);
  }

  mRenderCommandBuffer->pushConstants(
      mShaderPackInstance->getPipelineLayout(),
      vk::ShaderStageFlagBits::eRaygenKHR | vk::ShaderStageFlagBits::eMissKHR |
          vk::ShaderStageFlagBits::eClosestHitKHR | vk::ShaderStageFlagBits::eAnyHitKHR |
          vk::ShaderStageFlagBits::eCompute,
      0, pushConstantLayout->size, mPushConstantBuffer.data());
  mRenderCommandBuffer->bindPipeline(vk::PipelineBindPoint::eRayTracingKHR,
                                     mShaderPackInstance->getPipeline());
  std::vector<vk::DescriptorSet> descriptorSets;
  auto const &resources = mShaderPack->getResources();
  for (uint32_t sid = 0; sid < resources.size(); ++sid) {
    if (resources.at(sid).type == shader::UniformBindingType::eRTOutput) {
      descriptorSets.push_back(mOutputSet.get());
    } else if (resources.at(sid).type == shader::UniformBindingType::eRTScene) {
      descriptorSets.push_back(mSceneSet.get());
    } else if (resources.at(sid).type == shader::UniformBindingType::eRTCamera) {
      descriptorSets.push_back(mCameraSet.get());
    } else {
      throw std::runtime_error("unrecognized descriptor set");
    }
  }

  mRenderCommandBuffer->bindDescriptorSets(vk::PipelineBindPoint::eRayTracingKHR,
                                           mShaderPackInstance->getPipelineLayout(), 0,
                                           descriptorSets, {});

  mRenderCommandBuffer->traceRaysKHR(mShaderPackInstance->getRgenRegion(),
                                     mShaderPackInstance->getMissRegion(),
                                     mShaderPackInstance->getHitRegion(),
                                     mShaderPackInstance->getCallRegion(), mWidth, mHeight, 1);

  // make sure ray tracing is done

#ifdef SVULKAN2_CUDA_INTEROP
  if (mDenoiser) {
    mRenderImages.at(mDenoiseColorName)
        ->getImage()
        .transitionLayout(
            mRenderCommandBuffer.get(), vk::ImageLayout::eGeneral, vk::ImageLayout::eGeneral,
            vk::AccessFlagBits::eShaderWrite, vk::AccessFlagBits::eTransferRead,
            vk::PipelineStageFlagBits::eRayTracingShaderKHR, vk::PipelineStageFlagBits::eTransfer);
    mRenderImages.at(mDenoiseAlbedoName)
        ->getImage()
        .transitionLayout(
            mRenderCommandBuffer.get(), vk::ImageLayout::eGeneral, vk::ImageLayout::eGeneral,
            vk::AccessFlagBits::eShaderWrite, vk::AccessFlagBits::eTransferRead,
            vk::PipelineStageFlagBits::eRayTracingShaderKHR, vk::PipelineStageFlagBits::eTransfer);
    mRenderImages.at(mDenoiseNormalName)
        ->getImage()
        .transitionLayout(
            mRenderCommandBuffer.get(), vk::ImageLayout::eGeneral, vk::ImageLayout::eGeneral,
            vk::AccessFlagBits::eShaderWrite, vk::AccessFlagBits::eTransferRead,
            vk::PipelineStageFlagBits::eRayTracingShaderKHR, vk::PipelineStageFlagBits::eTransfer);
  }
#endif

  mRenderCommandBuffer->end();
}

void RTRenderer::prepareOutput() {
  mRenderImages.clear();
  mRTImages.clear();

  auto &set = mShaderPack->getOutputDescription();
  if (mWidth <= 0 || mHeight <= 0) {
    return;
  }

  // create images
  std::vector<std::shared_ptr<resource::SVStorageImage>> images;
  for (auto &[bid, binding] : set.bindings) {
    if (binding.type == vk::DescriptorType::eStorageImage && binding.name.starts_with("out")) {
      std::string texName = binding.name.substr(3);
      auto image = mRenderImages[texName] =
          std::make_shared<resource::SVStorageImage>(texName, mWidth, mHeight, binding.format);
      mRTImages.push_back(image);
      images.push_back(image);
      image->createDeviceResources();
    }
  }

  auto pool = mContext->createCommandPool();
  auto commandBuffer = pool->allocateCommandBuffer();
  commandBuffer->begin(vk::CommandBufferBeginInfo(vk::CommandBufferUsageFlagBits::eOneTimeSubmit));
  for (auto image : images) {
    image->getImage().transitionLayout(
        commandBuffer.get(), vk::ImageLayout::eUndefined, vk::ImageLayout::eGeneral, {},
        vk::AccessFlagBits::eShaderRead | vk::AccessFlagBits::eShaderWrite,
        vk::PipelineStageFlagBits::eTopOfPipe, vk::PipelineStageFlagBits::eRayTracingShaderKHR);
  }
  commandBuffer->end();
  mContext->getQueue().submitAndWait(commandBuffer.get());

  mOutputSet.reset();

  // make descriptor set
  mOutputPool = std::make_unique<core::DynamicDescriptorPool>(std::vector<vk::DescriptorPoolSize>{
      {vk::DescriptorType::eStorageImage, static_cast<uint32_t>(images.size())}});
  mOutputSet = mOutputPool->allocateSet(mShaderPackInstance->getOutputSetLayout());

  // write descriptor sets
  std::vector<vk::WriteDescriptorSet> writeDescriptorSets;
  std::vector<vk::DescriptorImageInfo> imageInfos;
  for (auto img : images) {
    imageInfos.push_back(
        vk::DescriptorImageInfo(VK_NULL_HANDLE, img->getImageView(), vk::ImageLayout::eGeneral));
  }
  for (uint32_t binding = 0; binding < imageInfos.size(); ++binding) {
    writeDescriptorSets.push_back(vk::WriteDescriptorSet(
        mOutputSet.get(), binding, 0, vk::DescriptorType::eStorageImage, imageInfos.at(binding)));
  }
  mContext->getDevice().updateDescriptorSets(writeDescriptorSets, {});
}

void RTRenderer::prepareScene() {
  prepareObjects();

  auto &set = mShaderPack->getSceneDescription();
  uint32_t asCount{0};
  uint32_t storageBufferCount{0};
  uint32_t textureCount{0};
  for (auto &[bid, binding] : set.bindings) {
    switch (binding.type) {
    case vk::DescriptorType::eAccelerationStructureKHR:
      ++asCount;
      break;
    case vk::DescriptorType::eStorageBuffer:
      ++storageBufferCount;
      break;
    case vk::DescriptorType::eCombinedImageSampler:
      ++textureCount;
      break;
    default:
      throw std::runtime_error("scene set should only contain acceleration "
                               "structures, storage buffers, and textures");
    }
  }

  std::vector<vk::WriteDescriptorSet> writeDescriptorSets;
  auto as = mScene->getTLAS()->getVulkanAS();
  vk::WriteDescriptorSetAccelerationStructureKHR asWrite(as);

  vk::DescriptorBufferInfo objectBufferInfo(mObjectBuffer->getVulkanBuffer(), 0, VK_WHOLE_SIZE);
  vk::DescriptorBufferInfo geometryInstanceBufferInfo(mScene->getRTGeometryInstanceBuffer(), 0,
                                                      VK_WHOLE_SIZE);
  vk::DescriptorBufferInfo textureIndexBufferInfo(mScene->getRTTextureIndexBuffer(), 0,
                                                  VK_WHOLE_SIZE);
  vk::DescriptorBufferInfo pointInstanceBufferInfo(mScene->getRTPointInstanceBuffer(), 0,
                                                   VK_WHOLE_SIZE);

  std::vector<vk::DescriptorBufferInfo> materialBufferInfos;
  for (auto buffer : mScene->getRTMaterialBuffers()) {
    materialBufferInfos.push_back({buffer, 0, VK_WHOLE_SIZE});
  }
  std::vector<vk::DescriptorImageInfo> textureInfos;
  for (auto [view, sampler] : mScene->getRTTextures()) {
    textureInfos.push_back(
        vk::DescriptorImageInfo(sampler, view, vk::ImageLayout::eShaderReadOnlyOptimal));
  }
  std::vector<vk::DescriptorBufferInfo> vertexBufferInfos;
  for (auto buffer : mScene->getRTVertexBuffers()) {
    vertexBufferInfos.push_back({buffer, 0, VK_WHOLE_SIZE});
  }
  std::vector<vk::DescriptorBufferInfo> indexBufferInfos;
  for (auto buffer : mScene->getRTIndexBuffers()) {
    indexBufferInfos.push_back({buffer, 0, VK_WHOLE_SIZE});
  }
  std::vector<vk::DescriptorBufferInfo> pointsetBufferInfos;
  for (auto buffer : mScene->getRTPointSetBuffers()) {
    pointsetBufferInfos.push_back({buffer, 0, VK_WHOLE_SIZE});
  }

  mSceneSet.reset();
  mScenePool = std::make_unique<core::DynamicDescriptorPool>(std::vector<vk::DescriptorPoolSize>{
      {vk::DescriptorType::eAccelerationStructureKHR, asCount},
      {vk::DescriptorType::eStorageBuffer,
       storageBufferCount +
           static_cast<uint32_t>(materialBufferInfos.size() + vertexBufferInfos.size() +
                                 indexBufferInfos.size())},
      {vk::DescriptorType::eCombinedImageSampler,
       textureCount + static_cast<uint32_t>(textureInfos.size())}});
  mSceneSet = mScenePool->allocateSet(mShaderPackInstance->getSceneSetLayout());

  vk::DescriptorBufferInfo pointLightBufferInfo(mScene->getRTPointLightBuffer(), 0, VK_WHOLE_SIZE);
  vk::DescriptorBufferInfo directionalLightBufferInfo(mScene->getRTDirectionalLightBuffer(), 0,
                                                      VK_WHOLE_SIZE);
  vk::DescriptorBufferInfo spotLightBufferInfo(mScene->getRTSpotLightBuffer(), 0, VK_WHOLE_SIZE);
  vk::DescriptorBufferInfo parallelogramLightBufferInfo(mScene->getRTParallelogramLightBuffer(), 0,
                                                        VK_WHOLE_SIZE);

  auto cube = mEnvironmentMap;
  if (!cube) {
    cube = mContext->getResourceManager()->getDefaultCubemap();
  }
  cube->uploadToDevice();
  vk::DescriptorImageInfo cubeMapInfo(cube->getSampler(), cube->getImageView(),
                                      vk::ImageLayout::eShaderReadOnlyOptimal);

  for (auto &[bid, binding] : set.bindings) {
    if (binding.name == "tlas") {
      writeDescriptorSets.push_back(vk::WriteDescriptorSet(
          mSceneSet.get(), bid, 0, 1, vk::DescriptorType::eAccelerationStructureKHR, {}, {}, {},
          &asWrite));
    } else if (binding.name == "Objects") {
      writeDescriptorSets.push_back(vk::WriteDescriptorSet(
          mSceneSet.get(), bid, 0, vk::DescriptorType::eStorageBuffer, {}, objectBufferInfo, {}));
    } else if (binding.name == "GeometryInstances") {
      writeDescriptorSets.push_back(vk::WriteDescriptorSet(mSceneSet.get(), bid, 0,
                                                           vk::DescriptorType::eStorageBuffer, {},
                                                           geometryInstanceBufferInfo, {}));
    } else if (binding.name == "TextureIndices") {
      writeDescriptorSets.push_back(vk::WriteDescriptorSet(mSceneSet.get(), bid, 0,
                                                           vk::DescriptorType::eStorageBuffer, {},
                                                           textureIndexBufferInfo, {}));
    } else if (binding.name == "Materials") {
      if (materialBufferInfos.size()) {
        writeDescriptorSets.push_back(vk::WriteDescriptorSet(mSceneSet.get(), bid, 0,
                                                             vk::DescriptorType::eStorageBuffer,
                                                             {}, materialBufferInfos, {}));
      }
    } else if (binding.name == "textures") {
      if (textureInfos.size()) {
        writeDescriptorSets.push_back(vk::WriteDescriptorSet(
            mSceneSet.get(), bid, 0, vk::DescriptorType::eCombinedImageSampler, textureInfos, {},
            {}));
      }
    } else if (binding.name == "Vertices") {
      if (vertexBufferInfos.size()) {
        writeDescriptorSets.push_back(vk::WriteDescriptorSet(mSceneSet.get(), bid, 0,
                                                             vk::DescriptorType::eStorageBuffer,
                                                             {}, vertexBufferInfos, {}));
      }
    } else if (binding.name == "Indices") {
      if (indexBufferInfos.size()) {
        writeDescriptorSets.push_back(vk::WriteDescriptorSet(mSceneSet.get(), bid, 0,
                                                             vk::DescriptorType::eStorageBuffer,
                                                             {}, indexBufferInfos, {}));
      }
    } else if (binding.name == "PointLights") {
      writeDescriptorSets.push_back(vk::WriteDescriptorSet(mSceneSet.get(), bid, 0,
                                                           vk::DescriptorType::eStorageBuffer, {},
                                                           pointLightBufferInfo, {}));
    } else if (binding.name == "DirectionalLights") {
      writeDescriptorSets.push_back(vk::WriteDescriptorSet(mSceneSet.get(), bid, 0,
                                                           vk::DescriptorType::eStorageBuffer, {},
                                                           directionalLightBufferInfo, {}));
    } else if (binding.name == "SpotLights") {
      writeDescriptorSets.push_back(vk::WriteDescriptorSet(mSceneSet.get(), bid, 0,
                                                           vk::DescriptorType::eStorageBuffer, {},
                                                           spotLightBufferInfo, {}));
    } else if (binding.name == "ParallelogramLights") {
      writeDescriptorSets.push_back(vk::WriteDescriptorSet(mSceneSet.get(), bid, 0,
                                                           vk::DescriptorType::eStorageBuffer, {},
                                                           parallelogramLightBufferInfo, {}));
    } else if (binding.name == "samplerEnvironment") {
      writeDescriptorSets.push_back(
          vk::WriteDescriptorSet(mSceneSet.get(), bid, 0,
                                 vk::DescriptorType::eCombinedImageSampler, cubeMapInfo, {}, {}));
    }

    else if (binding.name == "PointInstances") {
      writeDescriptorSets.push_back(vk::WriteDescriptorSet(mSceneSet.get(), bid, 0,
                                                           vk::DescriptorType::eStorageBuffer, {},
                                                           pointInstanceBufferInfo, {}));
    } else if (binding.name == "Points") {
      if (pointsetBufferInfos.size()) {
        writeDescriptorSets.push_back(vk::WriteDescriptorSet(mSceneSet.get(), bid, 0,
                                                             vk::DescriptorType::eStorageBuffer,
                                                             {}, pointsetBufferInfos, {}));
      }
    }

    else {
      throw std::runtime_error("unrecognized scene set binding " + binding.name);
    }
  }
  mContext->getDevice().updateDescriptorSets(writeDescriptorSets, {});
}

void RTRenderer::prepareCamera() {
  auto &set = mShaderPack->getCameraDescription();
  for (auto &[bid, binding] : set.bindings) {
    if (binding.name == "CameraBuffer") {
      mCameraBuffer = core::Buffer::CreateUniform(set.buffers.at(binding.arrayIndex)->size);
      if (bid != 0) {
        throw std::runtime_error("CameraBuffer must have binding 0");
      }
    }
  }

  mCameraSet.reset(); // TODO: manage
  mCameraPool = std::make_unique<core::DynamicDescriptorPool>(
      std::vector<vk::DescriptorPoolSize>{{vk::DescriptorType::eUniformBuffer, 1}}); // TODO adapt
  mCameraSet = mCameraPool->allocateSet(mShaderPackInstance->getCameraSetLayout());

  // write
  vk::DescriptorBufferInfo bufferInfo(mCameraBuffer->getVulkanBuffer(), 0, VK_WHOLE_SIZE);
  vk::WriteDescriptorSet write(mCameraSet.get(), 0, 0, vk::DescriptorType::eUniformBuffer, {},
                               bufferInfo);
  mContext->getDevice().updateDescriptorSets(write, {});
}

void RTRenderer::preparePostprocessing() {
  mPostprocessImages.clear();
  if (mWidth <= 0 || mHeight <= 0) {
    return;
  }

  auto &layouts = mShaderPackInstance->getPostprocessingSetLayouts();
  if (layouts.size() == 0) {
    return;
  }

  logger::info("Postprocessing passes {}", layouts.size());

  // create postprocessing images
  auto pool = mContext->createCommandPool();
  auto commandBuffer = pool->allocateCommandBuffer();
  commandBuffer->begin(vk::CommandBufferBeginInfo(vk::CommandBufferUsageFlagBits::eOneTimeSubmit));
  for (auto &parser : mShaderPack->getPostprocessingParsers()) {
    for (auto &[sid, set] : parser->getResources()) {
      for (auto &[bid, binding] : set.bindings) {
        if (binding.type == vk::DescriptorType::eStorageImage) {
          std::string texName = binding.name;
          if (mRenderImages.contains(texName)) {
            if (mRenderImages.at(texName)->getFormat() != binding.format) {
              throw std::runtime_error("image format mismatch in a postprocessing shader");
            }
          } else {
            auto image = mRenderImages[texName] = std::make_shared<resource::SVStorageImage>(
                texName, mWidth, mHeight, binding.format);
            image->createDeviceResources();
            image->getImage().transitionLayout(
                commandBuffer.get(), vk::ImageLayout::eUndefined, vk::ImageLayout::eGeneral, {},
                vk::AccessFlagBits::eShaderRead | vk::AccessFlagBits::eShaderWrite,
                vk::PipelineStageFlagBits::eTopOfPipe, vk::PipelineStageFlagBits::eComputeShader);
          }
          mPostprocessImages.push_back(mRenderImages.at(texName));
        }
      }
    }
  }

  commandBuffer->end();
  mContext->getQueue().submitAndWait(commandBuffer.get());

  mPostprocessingSets.clear(); // delete sets before deleting pool

  // create descriptor sets, bind images to them
  mPostprocessingPool = std::make_unique<core::DynamicDescriptorPool>(
      std::vector<vk::DescriptorPoolSize>{{vk::DescriptorType::eStorageBuffer, 10}}); // TODO adapt

  auto &parsers = mShaderPack->getPostprocessingParsers();
  for (uint32_t pid = 0; pid < parsers.size(); ++pid) {
    auto &layout = layouts.at(pid);
    mPostprocessingSets.push_back(mPostprocessingPool->allocateSet(layout.get()));
    auto desc = parsers.at(pid)->getResources().begin()->second;

    std::vector<vk::WriteDescriptorSet> writeDescriptorSets;
    std::vector<vk::DescriptorImageInfo> imageInfos;
    for (uint32_t binding = 0; binding < desc.bindings.size(); ++binding) {
      auto texName = desc.bindings.at(binding).name;
      imageInfos.push_back(vk::DescriptorImageInfo(
          VK_NULL_HANDLE, mRenderImages.at(texName)->getImageView(), vk::ImageLayout::eGeneral));
    }
    for (uint32_t binding = 0; binding < desc.bindings.size(); ++binding) {
      writeDescriptorSets.push_back(
          vk::WriteDescriptorSet(mPostprocessingSets.at(pid).get(), binding, 0,
                                 vk::DescriptorType::eStorageImage, imageInfos.at(binding)));
    }
    mContext->getDevice().updateDescriptorSets(writeDescriptorSets, {});
  }
}

void RTRenderer::render(scene::Camera &camera, std::vector<vk::Semaphore> const &waitSemaphores,
                        std::vector<vk::PipelineStageFlags> const &waitStages,
                        std::vector<vk::Semaphore> const &signalSemaphores, vk::Fence fence) {
  if (!mContext->isVulkanAvailable()) {
    return;
  }
  if (!mScene) {
    throw std::runtime_error("setScene must be called before rendering");
  }

  prepareRender(camera);

  mContext->getDevice().resetFences(mSceneAccessFence.get());
  mContext->getQueue().submit(mRenderCommandBuffer.get(), {}, {}, {}, mSceneAccessFence.get());

#ifdef SVULKAN2_CUDA_INTEROP
  if (mDenoiser) {
    mDenoiser->denoise(mRenderImages.at(mDenoiseColorName)->getImage(),
                       &mRenderImages.at(mDenoiseAlbedoName)->getImage(),
                       &mRenderImages.at(mDenoiseNormalName)->getImage());
  }
#endif
  mContext->getQueue().submit(mPostprocessCommandBuffer.get(), waitSemaphores, waitStages,
                              signalSemaphores, fence);
}

void RTRenderer::render(
    scene::Camera &camera, vk::ArrayProxyNoTemporaries<vk::Semaphore const> const &waitSemaphores,
    vk::ArrayProxyNoTemporaries<vk::PipelineStageFlags const> const &waitStageMasks,
    vk::ArrayProxyNoTemporaries<uint64_t const> const &waitValues,
    vk::ArrayProxyNoTemporaries<vk::Semaphore const> const &signalSemaphores,
    vk::ArrayProxyNoTemporaries<uint64_t const> const &signalValues) {
  if (!mContext->isVulkanAvailable()) {
    return;
  }
  if (!mScene) {
    throw std::runtime_error("setScene must be called before rendering");
  }

  prepareRender(camera);

  mContext->getQueue().submit(mRenderCommandBuffer.get(), {}, {}, {}, {});
#ifdef SVULKAN2_CUDA_INTEROP
  if (mDenoiser) {
    mDenoiser->denoise(mRenderImages.at(mDenoiseColorName)->getImage(),
                       &mRenderImages.at(mDenoiseAlbedoName)->getImage(),
                       &mRenderImages.at(mDenoiseNormalName)->getImage());
  }
#endif
  mContext->getQueue().submit(mPostprocessCommandBuffer.get(), waitSemaphores, waitStageMasks,
                              waitValues, signalSemaphores, signalValues, {});
}

void RTRenderer::display(std::string const &imageName, vk::Image backBuffer, vk::Format format,
                         uint32_t width, uint32_t height,
                         std::vector<vk::Semaphore> const &waitSemaphores,
                         std::vector<vk::PipelineStageFlags> const &waitStages,
                         std::vector<vk::Semaphore> const &signalSemaphores, vk::Fence fence) {
  if (!mContext->isPresentAvailable()) {
    throw std::runtime_error("Display failed: present is not enabled.");
  }

  if (!mDisplayCommandBuffer) {
    mDisplayCommandPool = mContext->createCommandPool();
    mDisplayCommandBuffer = mDisplayCommandPool->allocateCommandBuffer();
  }
  mDisplayCommandBuffer->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});

  auto renderTarget = mRenderImages.at(imageName);
  auto targetFormat = renderTarget->getFormat();
  if (targetFormat != vk::Format::eR8G8B8A8Unorm &&
      targetFormat != vk::Format::eR32G32B32A32Sfloat) {
    throw std::runtime_error("failed to display: only color textures are supported in display");
  };

  renderTarget->getImage().transitionLayout(
      mDisplayCommandBuffer.get(), vk::ImageLayout::eGeneral, vk::ImageLayout::eGeneral,
      vk::AccessFlagBits::eMemoryWrite, vk::AccessFlagBits::eTransferRead,
      vk::PipelineStageFlagBits::eComputeShader | vk::PipelineStageFlagBits::eTransfer,
      vk::PipelineStageFlagBits::eTransfer);

  {
    vk::ImageSubresourceRange imageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1);
    vk::ImageMemoryBarrier barrier({}, vk::AccessFlagBits::eTransferWrite,
                                   vk::ImageLayout::eUndefined,
                                   vk::ImageLayout::eTransferDstOptimal, VK_QUEUE_FAMILY_IGNORED,
                                   VK_QUEUE_FAMILY_IGNORED, backBuffer, imageSubresourceRange);
    mDisplayCommandBuffer->pipelineBarrier(vk::PipelineStageFlagBits::eTopOfPipe,
                                           vk::PipelineStageFlagBits::eTransfer, {}, nullptr,
                                           nullptr, barrier);
  }
  vk::ImageSubresourceLayers imageSubresourceLayers(vk::ImageAspectFlagBits::eColor, 0, 0, 1);
  vk::ImageBlit imageBlit(imageSubresourceLayers,
                          {{vk::Offset3D{0, 0, 0}, vk::Offset3D{mWidth, mHeight, 1}}},
                          imageSubresourceLayers,
                          {{vk::Offset3D{0, 0, 0},
                            vk::Offset3D{static_cast<int>(width), static_cast<int>(height), 1}}});
  mDisplayCommandBuffer->blitImage(
      renderTarget->getImage().getVulkanImage(), vk::ImageLayout::eGeneral, backBuffer,
      vk::ImageLayout::eTransferDstOptimal, imageBlit, vk::Filter::eNearest);

  // transfer swap chain back
  {
    vk::ImageSubresourceRange imageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1);
    vk::ImageMemoryBarrier barrier(
        vk::AccessFlagBits::eTransferWrite, vk::AccessFlagBits::eMemoryRead,
        vk::ImageLayout::eTransferDstOptimal, vk::ImageLayout::eColorAttachmentOptimal,
        VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED, backBuffer, imageSubresourceRange);
    mDisplayCommandBuffer->pipelineBarrier(vk::PipelineStageFlagBits::eTransfer,
                                           vk::PipelineStageFlagBits::eAllCommands, {}, nullptr,
                                           nullptr, barrier);
  }

  mDisplayCommandBuffer->end();
  mContext->getQueue().submit(mDisplayCommandBuffer.get(), waitSemaphores, waitStages,
                              signalSemaphores, fence);
}

std::vector<std::string> RTRenderer::getDisplayTargetNames() const {
  std::vector<std::string> names;
  for (auto &[bid, binding] : mShaderPack->getOutputDescription().bindings) {
    if (binding.type == vk::DescriptorType::eStorageImage && binding.name.starts_with("out")) {
      std::string texName = binding.name.substr(3);
      if (binding.format == vk::Format::eR32G32B32A32Sfloat) {
        names.push_back(texName);
      }
    }
  }
  for (auto &p : mShaderPack->getPostprocessingParsers()) {
    for (auto &[id, resource] : p->getResources()) {
      for (auto &[id, binding] : resource.bindings) {
        if (binding.type == vk::DescriptorType::eStorageImage &&
            std::find(names.begin(), names.end(), binding.name) == names.end()) {
          if (binding.format == vk::Format::eR32G32B32A32Sfloat) {
            names.push_back(binding.name);
          }
        }
      }
    }
  }
  return names;
}

std::vector<std::string> RTRenderer::getRenderTargetNames() const {
  std::vector<std::string> names;
  for (auto &[bid, binding] : mShaderPack->getOutputDescription().bindings) {
    if (binding.type == vk::DescriptorType::eStorageImage && binding.name.starts_with("out")) {
      std::string texName = binding.name.substr(3);
      names.push_back(texName);
    }
  }
  for (auto &p : mShaderPack->getPostprocessingParsers()) {
    for (auto &[id, resource] : p->getResources()) {
      for (auto &[id, binding] : resource.bindings) {
        if (binding.type == vk::DescriptorType::eStorageImage &&
            std::find(names.begin(), names.end(), binding.name) == names.end()) {
          if (binding.format == vk::Format::eR32G32B32A32Sfloat) {
            names.push_back(binding.name);
          }
        }
      }
    }
  }
  return names;
}

void RTRenderer::enableDenoiser(DenoiserType type, std::string const &colorName,
                                std::string const &albedoName, std::string const &normalName) {
#ifdef SVULKAN2_CUDA_INTEROP
  if (getDenoiserType() == type) {
    return;
  }
  mRequiresRebuild = true;

  if (type == DenoiserType::eNONE) {
    disableDenoiser();
    return;
  }

  if (type == DenoiserType::eOPTIX) {
    mDenoiser = std::make_unique<DenoiserOptix>();
  } else {
    mDenoiser = std::make_unique<DenoiserOidn>();
  }
  if (!mDenoiser->init(true, true, true)) {
    logger::error("Failed to initialize denoiser");
    mDenoiser.reset();
    return;
  }

  mDenoiseColorName = colorName;
  mDenoiseAlbedoName = albedoName;
  mDenoiseNormalName = normalName;
#endif
}

RTRenderer::DenoiserType RTRenderer::getDenoiserType() const {
#ifdef SVULKAN2_CUDA_INTEROP
  if (!mDenoiser) {
    return DenoiserType::eNONE;
  }
  if (dynamic_cast<DenoiserOptix *>(mDenoiser.get())) {
    return DenoiserType::eOPTIX;
  }
  return DenoiserType::eOIDN;
#else
  return DenoiserType::eNONE;
#endif
}

void RTRenderer::disableDenoiser() {
#ifdef SVULKAN2_CUDA_INTEROP
  mRequiresRebuild = true;
  mDenoiser.reset();
#endif
}

void RTRenderer::recordPostprocess() {
  mPostprocessCommandBuffer->reset();
  mPostprocessCommandBuffer->begin(vk::CommandBufferBeginInfo({}, {}));

  for (auto image : mPostprocessImages) {
    // TODO: find a better access flag
    image->getImage().transitionLayout(
        mPostprocessCommandBuffer.get(), vk::ImageLayout::eGeneral, vk::ImageLayout::eGeneral,
        vk::AccessFlagBits::eMemoryWrite,
        vk::AccessFlagBits::eShaderRead | vk::AccessFlagBits::eShaderWrite,
        vk::PipelineStageFlagBits::eRayTracingShaderKHR | vk::PipelineStageFlagBits::eTransfer,
        vk::PipelineStageFlagBits::eComputeShader);
  }

  auto pushConstantLayout = mShaderPack->getPushConstantLayout();

  // TODO insert memory barrier between render passes
  // currently it only works for a single pass
  for (uint32_t i = 0; i < mShaderPackInstance->getPostprocessingPipelines().size(); ++i) {
    mPostprocessCommandBuffer->pushConstants(
        mShaderPackInstance->getPostprocessingPipelineLayouts().at(i).get(),
        vk::ShaderStageFlagBits::eRaygenKHR | vk::ShaderStageFlagBits::eMissKHR |
            vk::ShaderStageFlagBits::eClosestHitKHR | vk::ShaderStageFlagBits::eAnyHitKHR |
            vk::ShaderStageFlagBits::eCompute,
        0, pushConstantLayout->size, mPushConstantBuffer.data());
    mPostprocessCommandBuffer->bindPipeline(
        vk::PipelineBindPoint::eCompute,
        mShaderPackInstance->getPostprocessingPipelines().at(i).get());
    mPostprocessCommandBuffer->bindDescriptorSets(
        vk::PipelineBindPoint::eCompute,
        mShaderPackInstance->getPostprocessingPipelineLayouts().at(i).get(), 0,
        mPostprocessingSets.at(i).get(), {});
    mPostprocessCommandBuffer->dispatch(mWidth, mHeight, 1);
  }
  mPostprocessCommandBuffer->end();
}

RTRenderer::~RTRenderer() {
  if (mScene && mSceneAccessFence) {
    mScene->unregisterAccessFence(mSceneAccessFence.get());
  }
}

} // namespace renderer
} // namespace svulkan2