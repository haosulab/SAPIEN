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
#include "svulkan2/renderer/renderer.h"
#include "../common/logger.h"
#include "svulkan2/common/profiler.h"
#include "svulkan2/core/command_pool.h"
#include "svulkan2/shader/deferred.h"
#include "svulkan2/shader/gbuffer.h"
#include "svulkan2/shader/primitive.h"
#include "svulkan2/shader/primitive_shadow.h"
#include "svulkan2/shader/shadow.h"

namespace svulkan2 {
namespace renderer {

static void
updateArrayDescriptorSets(vk::Device device, vk::DescriptorSet descriptorSet,
                          std::vector<std::tuple<vk::ImageView, vk::Sampler>> textureData,
                          uint32_t binding) {
  std::vector<vk::WriteDescriptorSet> writeDescriptorSets;
  writeDescriptorSets.reserve(textureData.size());

  std::vector<vk::DescriptorImageInfo> imageInfos;
  imageInfos.reserve(textureData.size());
  for (auto const &tex : textureData) {
    imageInfos.push_back(vk::DescriptorImageInfo(std::get<1>(tex), std::get<0>(tex),
                                                 vk::ImageLayout::eShaderReadOnlyOptimal));
  }
  for (uint32_t idx = 0; idx < imageInfos.size(); ++idx) {
    writeDescriptorSets.push_back(vk::WriteDescriptorSet(descriptorSet, binding, idx, 1,
                                                         vk::DescriptorType::eCombinedImageSampler,
                                                         &imageInfos[idx]));
  }
  device.updateDescriptorSets(writeDescriptorSets, nullptr);
}

static void updateDescriptorSets(vk::Device device,
                                 vk::DescriptorSet descriptorSet,
                                 std::vector<std::tuple<vk::DescriptorType, vk::Buffer, vk::BufferView>> const &bufferData,
                                 std::vector<std::tuple<vk::ImageView, vk::Sampler>> textureData,
                                 uint32_t bindingOffset) {
  std::vector<vk::WriteDescriptorSet> writeDescriptorSets;
  writeDescriptorSets.reserve(bufferData.size() + textureData.size());

  std::vector<vk::DescriptorBufferInfo> bufferInfos;
  bufferInfos.reserve(bufferData.size());
  for (auto const &bd : bufferData) {
    bufferInfos.push_back(vk::DescriptorBufferInfo(std::get<1>(bd), 0, VK_WHOLE_SIZE));
    writeDescriptorSets.push_back(
        vk::WriteDescriptorSet(descriptorSet, bindingOffset++, 0, 1, std::get<0>(bd), nullptr,
                               &bufferInfos.back(), std::get<2>(bd) ? &std::get<2>(bd) : nullptr));
  }
  std::vector<vk::DescriptorImageInfo> imageInfos;
  imageInfos.reserve(textureData.size());
  for (auto const &tex : textureData) {
    imageInfos.push_back(vk::DescriptorImageInfo(std::get<1>(tex), std::get<0>(tex),
                                                 vk::ImageLayout::eShaderReadOnlyOptimal));
    writeDescriptorSets.push_back(
        vk::WriteDescriptorSet(descriptorSet, bindingOffset++, 0, 1,
                               vk::DescriptorType::eCombinedImageSampler, &imageInfos.back()));
  }
  device.updateDescriptorSets(writeDescriptorSets, nullptr);
}

Renderer::Renderer(std::shared_ptr<RendererConfig> config) {
  mConfig = std::make_shared<RendererConfig>(*config);

  mContext = core::Context::Get();
  if (!mContext->isVulkanAvailable()) {
    return;
  }

  // preload the shader pack to get vertex layouts
  mShaderPack = mContext->getResourceManager()->CreateShaderPack(config->shaderDir);

  // make sure only forward renderer uses msaa
  if (mShaderPack->hasDeferredPass()) {
    mConfig->msaa = vk::SampleCountFlagBits::e1;
  }

  vk::DescriptorPoolSize pool_sizes[] = {
      {vk::DescriptorType::eCombinedImageSampler,
       100}, // render targets and input textures TODO: configure instead of 100
#ifdef VK_USE_PLATFORM_MACOS_MVK
      {vk::DescriptorType::eUniformBuffer, 100},
#endif
  };
  auto info = vk::DescriptorPoolCreateInfo(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet,
#ifdef VK_USE_PLATFORM_MACOS_MVK
                                           200, 2,
#else
                                           100, 1,
#endif
                                           pool_sizes);
  mDescriptorPool = mContext->getDevice().createDescriptorPoolUnique(info);

  mObjectPool = std::make_unique<core::DynamicDescriptorPool>(
      std::vector<vk::DescriptorPoolSize>{{vk::DescriptorType::eUniformBuffer, 1024}});
}

void Renderer::prepareRenderTargets(uint32_t width, uint32_t height) {
  if (!mContext->isVulkanAvailable()) {
    return;
  }

  auto renderTargetFormats = mShaderPackInstance->getRenderTargetFormats();

  for (auto &[name, format] : renderTargetFormats) {
    float scale = 1.f; // renderTargetScales[name];
    mRenderTargets[name] = std::make_shared<resource::SVRenderTarget>(
        name, static_cast<uint32_t>(width * scale), static_cast<uint32_t>(height * scale), format);
    mRenderTargets[name]->createDeviceResources();
  }

  // TODO: remove single sample depth target
  if (mConfig->msaa != vk::SampleCountFlagBits::e1) {
    for (auto &[name, format] : renderTargetFormats) {
      float scale = 1.f;
      mMultisampledTargets[name];
      mMultisampledTargets[name] = std::make_shared<resource::SVRenderTarget>(
          name, static_cast<uint32_t>(width * scale), static_cast<uint32_t>(height * scale),
          format, mConfig->msaa);
      mMultisampledTargets[name]->createDeviceResources();
    }
  }

  mRenderTargetFinalLayouts = mShaderPackInstance->getRenderTargetFinalLayouts();
}

void Renderer::prepareShadowRenderTargets() {
  if (!mContext->isVulkanAvailable()) {
    return;
  }

  auto format = mConfig->depthFormat;

  mDirectionalShadowWriteTargets.clear();
  mPointShadowWriteTargets.clear();
  mSpotShadowWriteTargets.clear();
  mTexturedLightShadowWriteTargets.clear();

  mDirectionalShadowReadTargets.clear();
  mPointShadowReadTargets.clear();
  mSpotShadowReadTargets.clear();
  mTexturedLightShadowReadTargets.clear();

  // TODO: this pool may not be necessary
  auto pool = mContext->createCommandPool();
  auto commandBuffer = pool->allocateCommandBuffer();

  commandBuffer->begin(vk::CommandBufferBeginInfo(vk::CommandBufferUsageFlagBits::eOneTimeSubmit));

  vk::ComponentMapping componentMapping(vk::ComponentSwizzle::eR, vk::ComponentSwizzle::eG,
                                        vk::ComponentSwizzle::eB, vk::ComponentSwizzle::eA);

  // point light
  {
    for (uint32_t size : mPointLightShadowSizes) {
      auto pointShadowImage = std::make_shared<core::Image>(
          vk::ImageType::e2D, vk::Extent3D{size, size, 1}, format,
          vk::ImageUsageFlagBits::eDepthStencilAttachment | vk::ImageUsageFlagBits::eSampled,
          VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY, vk::SampleCountFlagBits::e1, 1, 6,
          vk::ImageTiling::eOptimal, vk::ImageCreateFlagBits::eCubeCompatible);

      // HACK: transition to shader read to stop validation from complaining
      // dummy texture is not shader read
      pointShadowImage->transitionLayout(
          commandBuffer.get(), vk::ImageLayout::eUndefined,
          vk::ImageLayout::eShaderReadOnlyOptimal, {}, vk::AccessFlagBits::eShaderRead,
          vk::PipelineStageFlagBits::eBottomOfPipe, vk::PipelineStageFlagBits::eFragmentShader);

      // read targets
      vk::ImageViewCreateInfo viewInfo(
          {}, pointShadowImage->getVulkanImage(), vk::ImageViewType::eCube, format,
          componentMapping,
          vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eDepth, 0, 1, 0, 6));
      auto imageView = mContext->getDevice().createImageViewUnique(viewInfo);
      auto sampler = mContext->createSampler(vk::SamplerCreateInfo(
          {}, vk::Filter::eNearest, vk::Filter::eNearest, vk::SamplerMipmapMode::eNearest,
          vk::SamplerAddressMode::eClampToBorder, vk::SamplerAddressMode::eClampToBorder,
          vk::SamplerAddressMode::eClampToBorder, 0.f, false, 0.f, false, vk::CompareOp::eNever,
          0.f, 0.f, vk::BorderColor::eFloatOpaqueWhite));
      mPointShadowReadTargets.push_back(std::make_shared<resource::SVRenderTarget>(
          "PointShadow", size, size, pointShadowImage, std::move(imageView), sampler));

      // write targets
      for (uint32_t faceIndex = 0; faceIndex < 6; ++faceIndex) {
        vk::ImageViewCreateInfo viewInfo(
            {}, pointShadowImage->getVulkanImage(), vk::ImageViewType::e2D, format,
            componentMapping,
            vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eDepth, 0, 1, faceIndex, 1));
        auto imageView = mContext->getDevice().createImageViewUnique(viewInfo);
        mPointShadowWriteTargets.push_back(std::make_shared<resource::SVRenderTarget>(
            "PointShadow", size, size, pointShadowImage, std::move(imageView), vk::Sampler{}));
      }
    }
  }

  // directional light
  {
    for (uint32_t size : mDirectionalLightShadowSizes) {
      auto directionalShadowImage = std::make_shared<core::Image>(
          vk::ImageType::e2D, vk::Extent3D{size, size, 1}, format,
          vk::ImageUsageFlagBits::eDepthStencilAttachment | vk::ImageUsageFlagBits::eSampled,
          VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY, vk::SampleCountFlagBits::e1, 1, 1);

      directionalShadowImage->transitionLayout(
          commandBuffer.get(), vk::ImageLayout::eUndefined,
          vk::ImageLayout::eShaderReadOnlyOptimal, {}, vk::AccessFlagBits::eShaderRead,
          vk::PipelineStageFlagBits::eBottomOfPipe, vk::PipelineStageFlagBits::eFragmentShader);

      {
        vk::ImageViewCreateInfo viewInfo(
            {}, directionalShadowImage->getVulkanImage(), vk::ImageViewType::e2D, format,
            componentMapping,
            vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eDepth, 0, 1, 0, 1));
        auto imageView = mContext->getDevice().createImageViewUnique(viewInfo);
        auto sampler = mContext->createSampler(vk::SamplerCreateInfo(
            {}, vk::Filter::eNearest, vk::Filter::eNearest, vk::SamplerMipmapMode::eNearest,
            vk::SamplerAddressMode::eClampToBorder, vk::SamplerAddressMode::eClampToBorder,
            vk::SamplerAddressMode::eClampToBorder, 0.f, false, 0.f, false, vk::CompareOp::eNever,
            0.f, 0.f, vk::BorderColor::eFloatOpaqueWhite));
        mDirectionalShadowReadTargets.push_back(std::make_shared<resource::SVRenderTarget>(
            "DirectionalShadow", size, size, directionalShadowImage, std::move(imageView),
            sampler));
      }

      {
        vk::ImageViewCreateInfo viewInfo(
            {}, directionalShadowImage->getVulkanImage(), vk::ImageViewType::e2D, format,
            componentMapping,
            vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eDepth, 0, 1, 0, 1));
        auto imageView = mContext->getDevice().createImageViewUnique(viewInfo);
        mDirectionalShadowWriteTargets.push_back(std::make_shared<resource::SVRenderTarget>(
            "DirectionalShadow", size, size, directionalShadowImage, std::move(imageView),
            vk::Sampler{}));
      }
    }
  }

  // spot lights
  {
    for (uint32_t size : mSpotLightShadowSizes) {
      auto shadowImage = std::make_shared<core::Image>(
          vk::ImageType::e2D, vk::Extent3D{size, size, 1}, format,
          vk::ImageUsageFlagBits::eDepthStencilAttachment | vk::ImageUsageFlagBits::eSampled,
          VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY, vk::SampleCountFlagBits::e1, 1, 1);

      shadowImage->transitionLayout(
          commandBuffer.get(), vk::ImageLayout::eUndefined,
          vk::ImageLayout::eShaderReadOnlyOptimal, {}, vk::AccessFlagBits::eShaderRead,
          vk::PipelineStageFlagBits::eBottomOfPipe, vk::PipelineStageFlagBits::eFragmentShader);

      {
        vk::ImageViewCreateInfo viewInfo(
            {}, shadowImage->getVulkanImage(), vk::ImageViewType::e2D, format, componentMapping,
            vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eDepth, 0, 1, 0, 1));
        auto imageView = mContext->getDevice().createImageViewUnique(viewInfo);
        auto sampler = mContext->createSampler(vk::SamplerCreateInfo(
            {}, vk::Filter::eNearest, vk::Filter::eNearest, vk::SamplerMipmapMode::eNearest,
            vk::SamplerAddressMode::eClampToBorder, vk::SamplerAddressMode::eClampToBorder,
            vk::SamplerAddressMode::eClampToBorder, 0.f, false, 0.f, false, vk::CompareOp::eNever,
            0.f, 0.f, vk::BorderColor::eFloatOpaqueWhite));
        mSpotShadowReadTargets.push_back(std::make_shared<resource::SVRenderTarget>(
            "SpotShadow", size, size, shadowImage, std::move(imageView), sampler));
      }
      {
        vk::ImageViewCreateInfo viewInfo(
            {}, shadowImage->getVulkanImage(), vk::ImageViewType::e2D, format, componentMapping,
            vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eDepth, 0, 1, 0, 1));
        auto imageView = mContext->getDevice().createImageViewUnique(viewInfo);
        mSpotShadowWriteTargets.push_back(std::make_shared<resource::SVRenderTarget>(
            "SpotShadow", size, size, shadowImage, std::move(imageView), vk::Sampler{}));
      }
    }
  }

  // textured lights
  {
    for (uint32_t size : mTexturedLightShadowSizes) {
      auto shadowImage = std::make_shared<core::Image>(
          vk::ImageType::e2D, vk::Extent3D{size, size, 1}, format,
          vk::ImageUsageFlagBits::eDepthStencilAttachment | vk::ImageUsageFlagBits::eSampled,
          VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY, vk::SampleCountFlagBits::e1, 1, 1);

      shadowImage->transitionLayout(
          commandBuffer.get(), vk::ImageLayout::eUndefined,
          vk::ImageLayout::eShaderReadOnlyOptimal, {}, vk::AccessFlagBits::eShaderRead,
          vk::PipelineStageFlagBits::eBottomOfPipe, vk::PipelineStageFlagBits::eFragmentShader);

      {
        vk::ImageViewCreateInfo viewInfo(
            {}, shadowImage->getVulkanImage(), vk::ImageViewType::e2D, format, componentMapping,
            vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eDepth, 0, 1, 0, 1));
        auto imageView = mContext->getDevice().createImageViewUnique(viewInfo);
        auto sampler = mContext->createSampler(vk::SamplerCreateInfo(
            {}, vk::Filter::eNearest, vk::Filter::eNearest, vk::SamplerMipmapMode::eNearest,
            vk::SamplerAddressMode::eClampToBorder, vk::SamplerAddressMode::eClampToBorder,
            vk::SamplerAddressMode::eClampToBorder, 0.f, false, 0.f, false, vk::CompareOp::eNever,
            0.f, 0.f, vk::BorderColor::eFloatOpaqueWhite));
        mTexturedLightShadowReadTargets.push_back(std::make_shared<resource::SVRenderTarget>(
            "TexturedLightShadow", size, size, shadowImage, std::move(imageView), sampler));
      }
      {
        vk::ImageViewCreateInfo viewInfo(
            {}, shadowImage->getVulkanImage(), vk::ImageViewType::e2D, format, componentMapping,
            vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eDepth, 0, 1, 0, 1));
        auto imageView = mContext->getDevice().createImageViewUnique(viewInfo);
        mTexturedLightShadowWriteTargets.push_back(std::make_shared<resource::SVRenderTarget>(
            "TexturedLightShadow", size, size, shadowImage, std::move(imageView), vk::Sampler{}));
      }
    }
  }

  commandBuffer->end();
  mContext->getQueue().submitAndWait(commandBuffer.get());
}

void Renderer::preparePipelines() {
  if (!mContext->isVulkanAvailable()) {
    return;
  }

  shader::ShaderPackInstanceDesc desc;
  desc.config = mConfig;
  desc.specializationConstants = mSpecializationConstants;
  mShaderPackInstance = mContext->getResourceManager()->CreateShaderPackInstance(desc);
  mShaderPackInstance->loadAsync().wait();
  if (mShaderPackInstance->getShaderPack() != mShaderPack) {
    throw std::runtime_error("renderer corrupted! impossible error in shader pack.");
  }
}

void Renderer::prepareFramebuffers(uint32_t width, uint32_t height) {
  if (!mContext->isVulkanAvailable()) {
    return;
  }
  mFramebuffers.clear();

  auto passes = mShaderPack->getNonShadowPasses();

  for (uint32_t i = 0; i < passes.size(); ++i) {
    float scale = 1.0f; // TODO support scale
    auto names = passes[i]->getColorRenderTargetNames();
    std::vector<vk::ImageView> attachments;

    // color attachments and resolve attachments
    if (mConfig->msaa != vk::SampleCountFlagBits::e1) {
      for (auto &name : names) {
        attachments.push_back(mMultisampledTargets.at(name)->getImageView());
      }
    }
    for (auto &name : names) {
      attachments.push_back(mRenderTargets.at(name)->getImageView());
    }

    auto depthName = mShaderPackInstance->getDepthRenderTargetName(*passes[i]);
    if (depthName.has_value()) {
      if (mConfig->msaa != vk::SampleCountFlagBits::e1) {
        // TODO delete the other unused depth completely
        attachments.push_back(mMultisampledTargets.at(depthName.value())->getImageView());
      } else {
        attachments.push_back(mRenderTargets.at(depthName.value())->getImageView());
      }
    }

    vk::FramebufferCreateInfo info(
        {}, mShaderPackInstance->getNonShadowPassResources().at(i).renderPass.get(),
        attachments.size(), attachments.data(), static_cast<uint32_t>(scale * width),
        static_cast<uint32_t>(scale * height), 1);
    mFramebuffers.push_back(mContext->getDevice().createFramebufferUnique(info));
  }
}

void Renderer::prepareShadowFramebuffers() {
  if (!mContext->isVulkanAvailable()) {
    return;
  }
  mShadowFramebuffers.clear();
  mShadowSizes.clear();
  std::vector<std::shared_ptr<resource::SVRenderTarget>> targets;
  targets.insert(targets.end(), mDirectionalShadowWriteTargets.begin(),
                 mDirectionalShadowWriteTargets.begin() + mDirectionalLightShadowSizes.size());
  for (uint32_t i = 0; i < mDirectionalLightShadowSizes.size(); ++i) {
    mShadowSizes.push_back(mDirectionalLightShadowSizes[i]);
  }

  targets.insert(targets.end(), mPointShadowWriteTargets.begin(),
                 mPointShadowWriteTargets.begin() + mPointLightShadowSizes.size() * 6);
  for (uint32_t i = 0; i < mPointLightShadowSizes.size(); ++i) {
    for (uint32_t j = 0; j < 6; ++j) {
      mShadowSizes.push_back(mPointLightShadowSizes[i]);
    }
  }

  targets.insert(targets.end(), mSpotShadowWriteTargets.begin(),
                 mSpotShadowWriteTargets.begin() + mSpotLightShadowSizes.size());
  for (uint32_t i = 0; i < mSpotLightShadowSizes.size(); ++i) {
    mShadowSizes.push_back(mSpotLightShadowSizes[i]);
  }

  targets.insert(targets.end(), mTexturedLightShadowWriteTargets.begin(),
                 mTexturedLightShadowWriteTargets.begin() + mTexturedLightShadowSizes.size());
  for (uint32_t i = 0; i < mTexturedLightShadowSizes.size(); ++i) {
    mShadowSizes.push_back(mTexturedLightShadowSizes[i]);
  }

  for (auto &target : targets) {
    vk::ImageView view = target->getImageView();
    vk::FramebufferCreateInfo info({},
                                   mShaderPackInstance->getShadowPassResources().renderPass.get(),
                                   1, &view, target->getWidth(), target->getHeight(), 1);
    mShadowFramebuffers.push_back(mContext->getDevice().createFramebufferUnique(info));
  }
  assert(mShadowSizes.size() == mShadowFramebuffers.size());
}

void Renderer::resize(int width, int height) {
  if (width <= 0 || height <= 0) {
    throw std::runtime_error("failed to resize: width and height must be positive.");
  }
  mWidth = width;
  mHeight = height;
  mRequiresRebuild = true;
}

void Renderer::recordUpload() {
  if (!mContext->isVulkanAvailable()) {
    return;
  }

  mUploadCommandBuffer.reset();
  mUploadCommandPool = mContext->createCommandPool();
  mUploadCommandBuffer = mUploadCommandPool->allocateCommandBuffer();

  mUploadCommandBuffer->begin(vk::CommandBufferBeginInfo({}, {}));
  vk::BufferCopy region(0, 0, mCameraBufferCpu->getSize());
  mUploadCommandBuffer->copyBuffer(mCameraBufferCpu->getVulkanBuffer(),
                                   mCameraBuffer->getVulkanBuffer(), region);

  vk::MemoryBarrier barrier(vk::AccessFlagBits::eTransferWrite, vk::AccessFlagBits::eShaderRead);
  mUploadCommandBuffer->pipelineBarrier(vk::PipelineStageFlagBits::eTransfer,
                                        vk::PipelineStageFlagBits::eVertexShader |
                                            vk::PipelineStageFlagBits::eFragmentShader,
                                        {}, barrier, {}, {});

  mUploadCommandBuffer->end();
}

void Renderer::recordShadows() {
  if (!mContext->isVulkanAvailable()) {
    return;
  }

  mShadowCommandBuffer.reset();
  mShadowCommandPool = mContext->createCommandPool();
  mShadowCommandBuffer = mShadowCommandPool->allocateCommandBuffer();

  mShadowCommandBuffer->begin(vk::CommandBufferBeginInfo({}, {}));

  // render shadow passes
  if (mShaderPack->getShadowPass()) {
    auto objects = mScene->getObjects();
    auto shadowPass = mShaderPack->getShadowPass();

    std::vector<vk::ClearValue> clearValues;
    clearValues.push_back(vk::ClearDepthStencilValue(1.0f, 0));

    for (uint32_t shadowIdx = 0;
         shadowIdx < mDirectionalLightShadowSizes.size() + 6 * mPointLightShadowSizes.size() +
                         mSpotLightShadowSizes.size() + mTexturedLightShadowSizes.size();
         ++shadowIdx) {
      uint32_t size = mShadowSizes[shadowIdx];

      vk::Viewport viewport{0.f, 0.f, static_cast<float>(size), static_cast<float>(size),
                            0.f, 1.f};
      vk::Rect2D scissor{vk::Offset2D{0u, 0u},
                         vk::Extent2D{static_cast<uint32_t>(size), static_cast<uint32_t>(size)}};

      vk::RenderPassBeginInfo renderPassBeginInfo{
          mShaderPackInstance->getShadowPassResources().renderPass.get(),
          mShadowFramebuffers[shadowIdx].get(),
          vk::Rect2D({0, 0}, {static_cast<uint32_t>(size), static_cast<uint32_t>(size)}),
          static_cast<uint32_t>(clearValues.size()), clearValues.data()};
      mShadowCommandBuffer->beginRenderPass(renderPassBeginInfo, vk::SubpassContents::eInline);
      mShadowCommandBuffer->bindPipeline(
          vk::PipelineBindPoint::eGraphics,
          mShaderPackInstance->getShadowPassResources().pipeline.get());

      mShadowCommandBuffer->setViewport(0, viewport);
      mShadowCommandBuffer->setScissor(0, scissor);

      int objectSetIdx = -1;
      auto types = shadowPass->getUniformBindingTypes();
      for (uint32_t setIdx = 0; setIdx < types.size(); ++setIdx) {
        switch (types[setIdx]) {
        case shader::UniformBindingType::eObject:
          objectSetIdx = setIdx;
          break;
        case shader::UniformBindingType::eLight:
          mShadowCommandBuffer->bindDescriptorSets(
              vk::PipelineBindPoint::eGraphics,
              mShaderPackInstance->getShadowPassResources().layout.get(), setIdx,
              mLightSets[shadowIdx].get(), nullptr);
          break;
        default:
          throw std::runtime_error("shadow pass may only use object and light buffer");
        }
      }

      for (uint32_t objIdx = 0; objIdx < objects.size(); ++objIdx) {
        if (objects[objIdx]->getTransparency() >= 1 || !objects[objIdx]->getCastShadow()) {
          continue;
        }

        for (auto &shape : objects[objIdx]->getModel()->getShapes()) {
          if (objectSetIdx >= 0) {
            mShadowCommandBuffer->bindDescriptorSets(
                vk::PipelineBindPoint::eGraphics,
                mShaderPackInstance->getShadowPassResources().layout.get(), objectSetIdx,
                mObjectSet[objIdx].get(), nullptr);
          }
          mShadowCommandBuffer->bindVertexBuffers(0,
                                                  shape->mesh->getVertexBuffer().getVulkanBuffer(),
                                                  std::vector<vk::DeviceSize>(1, 0));
          mShadowCommandBuffer->bindIndexBuffer(shape->mesh->getIndexBuffer().getVulkanBuffer(), 0,
                                                vk::IndexType::eUint32);
          mShadowCommandBuffer->drawIndexed(shape->mesh->getTriangleCount() * 3, 1, 0, 0, 0);
        }
      }
      mShadowCommandBuffer->endRenderPass();
    }
  }

  if (mShaderPack->getPointShadowPass()) {
    auto objects = mScene->getPointObjects();
    auto pointShadowPass = mShaderPack->getPointShadowPass();
    std::vector<vk::ClearValue> clearValues;
    clearValues.push_back(vk::ClearDepthStencilValue(1.0f, 0));

    for (uint32_t shadowIdx = 0;
         shadowIdx < mDirectionalLightShadowSizes.size() + 6 * mPointLightShadowSizes.size() +
                         mSpotLightShadowSizes.size() + mTexturedLightShadowSizes.size();
         ++shadowIdx) {
      uint32_t size = mShadowSizes[shadowIdx];

      vk::Viewport viewport{0.f, 0.f, static_cast<float>(size), static_cast<float>(size),
                            0.f, 1.f};
      vk::Rect2D scissor{vk::Offset2D{0u, 0u},
                         vk::Extent2D{static_cast<uint32_t>(size), static_cast<uint32_t>(size)}};

      vk::RenderPassBeginInfo renderPassBeginInfo{
          mShaderPackInstance->getPointShadowPassResources().renderPass.get(),
          mShadowFramebuffers[shadowIdx].get(),
          vk::Rect2D({0, 0}, {static_cast<uint32_t>(size), static_cast<uint32_t>(size)}),
          static_cast<uint32_t>(clearValues.size()), clearValues.data()};
      mShadowCommandBuffer->beginRenderPass(renderPassBeginInfo, vk::SubpassContents::eInline);
      mShadowCommandBuffer->bindPipeline(
          vk::PipelineBindPoint::eGraphics,
          mShaderPackInstance->getPointShadowPassResources().pipeline.get());

      mShadowCommandBuffer->setViewport(0, viewport);
      mShadowCommandBuffer->setScissor(0, scissor);

      int objectSetIdx = -1;
      auto types = pointShadowPass->getUniformBindingTypes();
      for (uint32_t setIdx = 0; setIdx < types.size(); ++setIdx) {
        switch (types[setIdx]) {
        case shader::UniformBindingType::eObject:
          objectSetIdx = setIdx;
          break;
        case shader::UniformBindingType::eLight:
          mShadowCommandBuffer->bindDescriptorSets(
              vk::PipelineBindPoint::eGraphics,
              mShaderPackInstance->getPointShadowPassResources().layout.get(), setIdx,
              mLightSets[shadowIdx].get(), nullptr);
          break;
        default:
          throw std::runtime_error("point shadow pass may only use object and light buffer");
        }
      }

      for (uint32_t objIdx = 0; objIdx < objects.size(); ++objIdx) {
        if (objects[objIdx]->getTransparency() >= 1) {
          continue;
        }
        if (objectSetIdx >= 0) {
          mShadowCommandBuffer->bindDescriptorSets(
              vk::PipelineBindPoint::eGraphics,
              mShaderPackInstance->getPointShadowPassResources().layout.get(), objectSetIdx,
              mObjectSet[mPointObjectIndex + objIdx].get(), nullptr);
        }
        mShadowCommandBuffer->bindVertexBuffers(
            0, objects[objIdx]->getPointSet()->getVertexBuffer().getVulkanBuffer(),
            vk::DeviceSize(0));
        mShadowCommandBuffer->draw(objects[objIdx]->getVertexCount(), 1, 0, 0);
      }
      mShadowCommandBuffer->endRenderPass();
    }
  }

  mShadowCommandBuffer->end();
}

void Renderer::prepareObjects() {
  SVULKAN2_PROFILE_BLOCK_BEGIN("Prepare objects");
  auto objects = mScene->getObjects();
  auto lineObjects = mScene->getLineObjects();
  auto pointObjects = mScene->getPointObjects();
  auto size = objects.size();

  if (mShaderPack->hasLinePass()) {
    mLineObjectIndex = size;
    size += lineObjects.size();
  }
  if (mShaderPack->hasPointPass()) {
    mPointObjectIndex = size;
    size += pointObjects.size();
  }

  prepareObjectBuffers(size);

  SVULKAN2_PROFILE_BLOCK_END;

  // load objects to CPU, if not already loaded
  {
    SVULKAN2_PROFILE_BLOCK("Load objects to CPU");
    std::vector<std::future<void>> futures;
    for (auto obj : objects) {
      futures.push_back(obj->getModel()->loadAsync());
    }
    for (auto &f : futures) {
      f.get();
    }
  }

  {
    SVULKAN2_PROFILE_BLOCK("Upload objects to GPU");
    // upload objects to GPU, if not up-to-date
    for (auto obj : objects) {
      for (auto shape : obj->getModel()->getShapes()) {
        shape->material->uploadToDevice();
        shape->mesh->uploadToDevice();
      }
    }
    if (mShaderPack->hasLinePass()) {
      for (auto obj : lineObjects) {
        obj->getLineSet()->uploadToDevice();
      }
    }
    if (mShaderPack->hasPointPass()) {
      for (auto obj : pointObjects) {
        obj->getPointSet()->uploadToDevice();
      }
    }
  }

  // update bindings for custom textures
  {
    auto setDesc = mShaderPack->getShaderInputLayouts()->objectSetDescription;
    for (uint32_t bid = 1; bid < setDesc.bindings.size(); ++bid) {
      auto binding = setDesc.bindings.at(bid);
      if (binding.type == vk::DescriptorType::eCombinedImageSampler &&
          binding.name.substr(0, 7) == "sampler") {
        auto name = binding.name.substr(7);
        if (binding.arraySize == 0) {
          // sampler
          std::vector<vk::DescriptorImageInfo> imageInfo;
          for (uint32_t objId = 0; objId < objects.size(); ++objId) {
            auto t = objects[objId]->getCustomTexture(name);
            if (!t) {
              if (binding.imageDim == 1) {
                t = mContext->getResourceManager()->getDefaultTexture1D();
              } else if (binding.imageDim == 2) {
                t = mContext->getResourceManager()->getDefaultTexture();
              } else {
                t = mContext->getResourceManager()->getDefaultTexture3D();
              }
            }

            t->loadAsync().get();
            t->uploadToDevice();

            imageInfo.push_back(
                {t->getSampler(), t->getImageView(), vk::ImageLayout::eShaderReadOnlyOptimal});
          }

          std::vector<vk::WriteDescriptorSet> writeDescriptorSets;
          for (uint32_t objId = 0; objId < objects.size(); ++objId) {
            writeDescriptorSets.push_back(vk::WriteDescriptorSet(
                mObjectSet[objId].get(), bid, 0, vk::DescriptorType::eCombinedImageSampler,
                imageInfo[objId]));
          }
          mContext->getDevice().updateDescriptorSets(writeDescriptorSets, nullptr);
        } else {
          // sampler array
          std::vector<std::vector<vk::DescriptorImageInfo>> imageInfo;
          for (uint32_t objId = 0; objId < objects.size(); ++objId) {
            auto ta = objects[objId]->getCustomTextureArray(name);
            if (static_cast<int>(ta.size()) > binding.arraySize) {
              ta.resize(binding.arraySize);
            } else {
              std::shared_ptr<resource::SVTexture> t;
              if (binding.imageDim == 1) {
                t = mContext->getResourceManager()->getDefaultTexture1D();
              } else if (binding.imageDim == 2) {
                t = mContext->getResourceManager()->getDefaultTexture();
              } else {
                t = mContext->getResourceManager()->getDefaultTexture3D();
              }

              ta.resize(binding.arraySize, t);
            }
            for (auto t : ta) {
              t->loadAsync().get();
              t->uploadToDevice();
            }

            imageInfo.emplace_back();
            for (auto t : ta) {
              imageInfo.back().push_back(
                  {t->getSampler(), t->getImageView(), vk::ImageLayout::eShaderReadOnlyOptimal});
            }
          }

          std::vector<vk::WriteDescriptorSet> writeDescriptorSets;
          for (uint32_t objId = 0; objId < objects.size(); ++objId) {
            writeDescriptorSets.push_back(vk::WriteDescriptorSet(
                mObjectSet[objId].get(), bid, 0, vk::DescriptorType::eCombinedImageSampler,
                imageInfo[objId]));
          }

          mContext->getDevice().updateDescriptorSets(writeDescriptorSets, nullptr);
        }
      }
    }
  }
}

void Renderer::recordRenderPasses() {
  mModelCache.clear();
  mLineSetCache.clear();
  mPointSetCache.clear();

  mRenderCommandBuffer.reset();
  mRenderCommandPool = mContext->createCommandPool();
  mRenderCommandBuffer = mRenderCommandPool->allocateCommandBuffer();

  mRenderCommandBuffer->begin(vk::CommandBufferBeginInfo({}, {}));

  // classify shapes
  uint32_t numGbufferPasses = mShaderPack->getGbufferPasses().size();

  std::vector<std::vector<std::shared_ptr<resource::SVShape>>> shapes(numGbufferPasses);

  std::vector<std::vector<uint32_t>> shapeObjectIndex(numGbufferPasses);

  int transparencyShadingMode = numGbufferPasses > 1 ? 1 : 0;

  auto objects = mScene->getObjects();
  for (uint32_t objectIndex = 0; objectIndex < objects.size(); ++objectIndex) {
    int shadingMode = objects[objectIndex]->getShadingMode();
    if (static_cast<uint32_t>(shadingMode) >= shapes.size()) {
      continue; // do not render
    }
    if (shadingMode == 0 && objects[objectIndex]->getTransparency() != 0) {
      shadingMode = transparencyShadingMode;
    }
    if (objects[objectIndex]->getTransparency() >= 1) {
      continue;
    }
    /* HACK: hold onto the models to make sure the underlying buffers are not
     * released until command buffer reset */
    mModelCache.insert(objects[objectIndex]->getModel());

    for (auto shape : objects[objectIndex]->getModel()->getShapes()) {
      int shapeShadingMode = shadingMode;
      if (shape->material->getOpacity() == 0) {
        continue;
      }
      if (shape->material->getOpacity() != 1 && shadingMode == 0) {
        shapeShadingMode = transparencyShadingMode;
      }
      shapes[shapeShadingMode].push_back(shape);
      shapeObjectIndex[shapeShadingMode].push_back(objectIndex);
    }
  }

  auto linesetObjects = mScene->getLineObjects();

  // classify point sets
  uint32_t numPointPasses = mShaderPack->getPointPasses().size();

  auto pointsetObjects = mScene->getPointObjects();
  std::vector<std::vector<uint32_t>> pointsets(numPointPasses);
  for (uint32_t objectIndex = 0; objectIndex < pointsetObjects.size(); ++objectIndex) {
    auto mode = pointsetObjects[objectIndex]->getShadingMode();
    if (static_cast<uint32_t>(mode) >= pointsets.size()) {
      continue;
    }
    pointsets[mode].push_back(objectIndex);
  }

  uint32_t gbufferIndex = 0;
  uint32_t pointIndex = 0;
  auto passes = mShaderPack->getNonShadowPasses();
  for (uint32_t pass_index = 0; pass_index < passes.size(); ++pass_index) {
    SVULKAN2_PROFILE_BLOCK("Record render pass" + std::to_string(pass_index));

    auto pass = passes[pass_index];

    float scale = 1.f; // TODO: fix
    // float scale = pass->getResolutionScale();
    uint32_t passWidth = static_cast<uint32_t>(mWidth * scale);
    uint32_t passHeight = static_cast<uint32_t>(mHeight * scale);

    vk::Viewport viewport{0.f, 0.f, static_cast<float>(passWidth), static_cast<float>(passHeight),
                          0.f, 1.f};
    vk::Rect2D scissor{vk::Offset2D{0u, 0u}, vk::Extent2D{static_cast<uint32_t>(passWidth),
                                                          static_cast<uint32_t>(passHeight)}};

    uint32_t colorCount = pass->getTextureOutputLayout()->elements.size();
    std::vector<vk::ClearValue> clearValues(
        mConfig->msaa == vk::SampleCountFlagBits::e1 ? colorCount : colorCount * 2,
        vk::ClearColorValue(std::array<float, 4>{0.f, 0.f, 0.f, 0.f}));

    clearValues.push_back(vk::ClearDepthStencilValue(1.0f, 0));
    vk::RenderPassBeginInfo renderPassBeginInfo{
        mShaderPackInstance->getNonShadowPassResources().at(pass_index).renderPass.get(),
        mFramebuffers[pass_index].get(),
        vk::Rect2D({0, 0}, {static_cast<uint32_t>(passWidth), static_cast<uint32_t>(passHeight)}),
        static_cast<uint32_t>(clearValues.size()), clearValues.data()};
    mRenderCommandBuffer->beginRenderPass(renderPassBeginInfo, vk::SubpassContents::eInline);
    mRenderCommandBuffer->bindPipeline(
        vk::PipelineBindPoint::eGraphics,
        mShaderPackInstance->getNonShadowPassResources().at(pass_index).pipeline.get());
    mRenderCommandBuffer->setViewport(0, viewport);
    mRenderCommandBuffer->setScissor(0, scissor);

    int objectBinding = -1;
    int materialBinding = -1;
    auto types = pass->getUniformBindingTypes();

    auto layout = mShaderPackInstance->getNonShadowPassResources().at(pass_index).layout.get();

    for (uint32_t i = 0; i < types.size(); ++i) {
      switch (types[i]) {
      case shader::UniformBindingType::eCamera:
        mRenderCommandBuffer->bindDescriptorSets(vk::PipelineBindPoint::eGraphics, layout, i,
                                                 mCameraSet.get(), nullptr);
        break;
      case shader::UniformBindingType::eScene:
        mRenderCommandBuffer->bindDescriptorSets(vk::PipelineBindPoint::eGraphics, layout, i,
                                                 mSceneSet.get(), nullptr);
        break;
      case shader::UniformBindingType::eTextures:
        mRenderCommandBuffer->bindDescriptorSets(vk::PipelineBindPoint::eGraphics, layout, i,
                                                 mInputTextureSets[pass_index].get(), nullptr);
        break;
      case shader::UniformBindingType::eObject:
        objectBinding = i;
        break;
      case shader::UniformBindingType::eMaterial:
        materialBinding = i;
        break;
      default:
        throw std::runtime_error("not implemented");
      }
    }

    if (auto gbufferPass = std::dynamic_pointer_cast<shader::GbufferPassParser>(pass)) {
      for (uint32_t i = 0; i < shapes[gbufferIndex].size(); ++i) {
        auto shape = shapes[gbufferIndex][i];
        uint32_t objectIndex = shapeObjectIndex[gbufferIndex][i];
        if (objectBinding >= 0) {
          mRenderCommandBuffer->bindDescriptorSets(vk::PipelineBindPoint::eGraphics, layout,
                                                   objectBinding, mObjectSet[objectIndex].get(),
                                                   nullptr);
        }
        if (materialBinding >= 0) {
          mRenderCommandBuffer->bindDescriptorSets(vk::PipelineBindPoint::eGraphics, layout,
                                                   materialBinding,
                                                   shape->material->getDescriptorSet(), nullptr);
        }
        mRenderCommandBuffer->bindVertexBuffers(0,
                                                shape->mesh->getVertexBuffer().getVulkanBuffer(),
                                                std::vector<vk::DeviceSize>(1, 0));
        mRenderCommandBuffer->bindIndexBuffer(shape->mesh->getIndexBuffer().getVulkanBuffer(), 0,
                                              vk::IndexType::eUint32);
        mRenderCommandBuffer->setCullMode(shape->material->getCullMode());
        mRenderCommandBuffer->drawIndexed(shape->mesh->getTriangleCount() * 3, 1, 0, 0, 0);
      }
      gbufferIndex++;
    } else if (auto linePass = std::dynamic_pointer_cast<shader::LinePassParser>(pass)) {
      for (uint32_t index = 0; index < linesetObjects.size(); ++index) {
        auto &lineObj = linesetObjects[index];
        if (objectBinding >= 0) {
          mRenderCommandBuffer->bindDescriptorSets(
              vk::PipelineBindPoint::eGraphics, layout, objectBinding,
              mObjectSet[mLineObjectIndex + index].get(), nullptr);
        }
        if (lineObj->getTransparency() < 1) {
          mLineSetCache.insert(lineObj->getLineSet());
          mRenderCommandBuffer->setLineWidth(lineObj->getLineWidth());
          mRenderCommandBuffer->bindVertexBuffers(
              0, lineObj->getLineSet()->getVertexBuffer().getVulkanBuffer(), vk::DeviceSize(0));
          mRenderCommandBuffer->draw(lineObj->getLineSet()->getVertexCount(), 1, 0, 0);
        }
      }
    } else if (auto pointPass = std::dynamic_pointer_cast<shader::PointPassParser>(pass)) {
      for (uint32_t i = 0; i < pointsets[pointIndex].size(); ++i) {
        uint32_t objectIndex = pointsets[pointIndex][i];
        auto &pointObj = pointsetObjects[objectIndex];
        if (objectBinding >= 0) {
          mRenderCommandBuffer->bindDescriptorSets(
              vk::PipelineBindPoint::eGraphics, layout, objectBinding,
              mObjectSet[mPointObjectIndex + i].get(), nullptr);
        }
        if (pointObj->getTransparency() < 1) {
          mPointSetCache.insert(pointObj->getPointSet());
          mRenderCommandBuffer->bindVertexBuffers(
              0, pointObj->getPointSet()->getVertexBuffer().getVulkanBuffer(), vk::DeviceSize(0));
          mRenderCommandBuffer->draw(pointObj->getVertexCount(), 1, 0, 0);
        }
      }
      pointIndex++;
    } else {
      mRenderCommandBuffer->draw(3, 1, 0, 0);
    }
    mRenderCommandBuffer->endRenderPass();
  }
  mRenderCommandBuffer->end();
}

void Renderer::prepareRender(scene::Camera &camera) {
  if (mLastVersion != mScene->getVersion()) {
    logger::info("Scene updated");
    mRequiresRecord = true;
  }

  if (mEnvironmentMap != mScene->getEnvironmentMap()) {
    mEnvironmentMap = mScene->getEnvironmentMap();
    mEnvironmentMap->load();
    mRequiresRebuild = true;
  }

  if (mWidth <= 0 || mHeight <= 0) {
    throw std::runtime_error("failed to render: resize must be called before rendering.");
  }

  if (mRequiresRecord) {
    auto pointLights = mScene->getPointLights();
    auto directionalLights = mScene->getDirectionalLights();
    auto spotLights = mScene->getSpotLights();
    auto texturedLights = mScene->getTexturedLights();

    int numPointLights = pointLights.size();
    int numDirectionalLights = directionalLights.size();
    int numSpotLights = spotLights.size();

    mPointLightShadowSizes.clear();
    mDirectionalLightShadowSizes.clear();
    mSpotLightShadowSizes.clear();
    mTexturedLightShadowSizes.clear();

    for (auto l : pointLights) {
      if (l->isShadowEnabled()) {
        mPointLightShadowSizes.push_back(l->getShadowMapSize());
      }
    }
    for (auto l : directionalLights) {
      if (l->isShadowEnabled()) {
        mDirectionalLightShadowSizes.push_back(l->getShadowMapSize());
      }
    }
    for (auto l : spotLights) {
      if (l->isShadowEnabled()) {
        mSpotLightShadowSizes.push_back(l->getShadowMapSize());
      }
    }
    for (auto l : texturedLights) {
      mTexturedLightShadowSizes.push_back(l->getShadowMapSize());
    }

    {
      // load custom textures
      std::vector<std::future<void>> futures;
      for (auto t : mCustomTextures) {
        futures.push_back(t.second->loadAsync());
      }
      for (auto a : mCustomTextureArray) {
        for (auto t : a.second) {
          futures.push_back(t->loadAsync());
        }
      }
      for (auto t : mCustomCubemaps) {
        futures.push_back(t.second->loadAsync());
      }
      for (auto l : mScene->getTexturedLights()) {
        if (l->getTexture()) {
          futures.push_back(l->getTexture()->loadAsync());
        }
      }
      for (auto &f : futures) {
        f.get();
      }
      for (auto l : mScene->getTexturedLights()) {
        if (l->getTexture()) {
          l->getTexture()->uploadToDevice();
        }
      }
    }

    setSpecializationConstant<int>("NUM_POINT_LIGHTS", numPointLights);
    setSpecializationConstant<int>("NUM_DIRECTIONAL_LIGHTS", numDirectionalLights);
    setSpecializationConstant<int>("NUM_SPOT_LIGHTS", numSpotLights);

    setSpecializationConstant<int>("NUM_POINT_LIGHT_SHADOWS", mPointLightShadowSizes.size());
    setSpecializationConstant<int>("NUM_DIRECTIONAL_LIGHT_SHADOWS",
                                   mDirectionalLightShadowSizes.size());
    setSpecializationConstant<int>("NUM_SPOT_LIGHT_SHADOWS", mSpotLightShadowSizes.size());
    setSpecializationConstant<int>("NUM_TEXTURED_LIGHT_SHADOWS", mTexturedLightShadowSizes.size());
  }

  if (mRequiresRebuild || mSpecializationConstantsChanged) {
    SVULKAN2_PROFILE_BLOCK("Rebuilding Pipeline");
    mRequiresRecord = true;

    preparePipelines();

    prepareRenderTargets(mWidth, mHeight);
    if (mShaderPack->getShadowPass()) {
      prepareShadowRenderTargets();
      prepareShadowFramebuffers();
    }

    prepareFramebuffers(mWidth, mHeight);
    prepareInputTextureDescriptorSets();
    mSpecializationConstantsChanged = false;
    mRequiresRebuild = false;

    if (mShaderPack->getShadowPass()) {
      prepareLightBuffers();
    }
    prepareSceneBuffer();
    prepareCameraBuffer();

    if (camera.getWidth() != mWidth || camera.getHeight() != mHeight) {
      throw std::runtime_error("Camera size and renderer size does not match. "
                               "Please resize camera first.");
    }
  }

  if (mRequiresRecord) {
    prepareObjects();

    recordUpload();

    {
      SVULKAN2_PROFILE_BLOCK("Record shadow draw calls");
      recordShadows();
    }
    {
      SVULKAN2_PROFILE_BLOCK("Record render draw calls");
      recordRenderPasses();
    }

    uploadGpuResources(camera);
    mContext->getQueue().waitIdle();
  }

  mRequiresRecord = false;
  mLastVersion = mScene->getVersion();

  for (auto &[name, target] : mRenderTargets) {
    target->getImage().setCurrentLayout(mRenderTargetFinalLayouts[name]);
  }

  // when using multisampling, the original layout is always transfer src
  for (auto &[name, target] : mMultisampledTargets) {
    mRenderTargets.at(name)->getImage().setCurrentLayout(vk::ImageLayout::eTransferSrcOptimal);
    target->getImage().setCurrentLayout(mRenderTargetFinalLayouts[name]);
  }
}

void Renderer::uploadGpuResources(scene::Camera &camera) {
  {
    SVULKAN2_PROFILE_BLOCK("Update camera & scene");
    // update camera
    camera.uploadToDevice(*mCameraBufferCpu,
                          *mShaderPack->getShaderInputLayouts()->cameraBufferLayout);

    // update scene
    mScene->uploadToDevice(*mSceneBuffer,
                           *mShaderPack->getShaderInputLayouts()->sceneBufferLayout);

    if (mShaderPack->getShadowPass()) {
      mScene->uploadShadowToDevice(*mShadowBuffer, mLightBuffers,
                                   *mShaderPack->getShaderInputLayouts()->shadowBufferLayout);
    }
  }

  {
    SVULKAN2_PROFILE_BLOCK("Update objects");
    auto bufferSize = mShaderPack->getShaderInputLayouts()->objectDataBufferLayout->getAlignedSize(
        mContext->getPhysicalDeviceLimits().minUniformBufferOffsetAlignment);

    // update objects
    mScene->uploadObjectTransforms();

    mObjectDataBuffer->map();
    auto objects = mScene->getObjects();

    {
      auto layout = mShaderPack->getShaderInputLayouts()->objectDataBufferLayout;
      int segmentationOffset = layout->elements.at("segmentation").offset;
      int transparencyOffset = layout->elements.find("transparency") == layout->elements.end()
                                   ? -1
                                   : layout->elements.at("transparency").offset;
      int shadeFlatOffset = layout->elements.find("shadeFlat") == layout->elements.end()
                                ? -1
                                : layout->elements.at("shadeFlat").offset;

      for (uint32_t i = 0; i < objects.size(); ++i) {
        auto segmentation = objects[i]->getSegmentation();
        float transparency = objects[i]->getTransparency();
        int shadeFlat = static_cast<int>(objects[i]->getShadeFlat());

        assert(objects[i]->getInternalGpuIndex() == i);

        mObjectDataBuffer->upload(&segmentation, 16, i * bufferSize + segmentationOffset);

        if (transparencyOffset >= 0) {
          mObjectDataBuffer->upload(&transparency, 4, i * bufferSize + transparencyOffset);
        }
        if (shadeFlatOffset >= 0) {
          mObjectDataBuffer->upload(&shadeFlat, 4, i * bufferSize + shadeFlatOffset);
        }

        for (auto &[name, value] : objects[i]->getCustomData()) {
          if (layout->elements.find(name) != layout->elements.end()) {
            auto &elem = layout->elements.at(name);
            if (elem.dtype != value.dtype) {
              throw std::runtime_error("Upload object failed: object attribute \"" + name +
                                       "\" does not match declared type.");
            }
            mObjectDataBuffer->upload(&value.floatValue, elem.size, i * bufferSize + elem.offset);
          }
        }
      }
    }

    if (mShaderPack->hasLinePass()) {
      auto lineObjects = mScene->getLineObjects();

      for (uint32_t i = 0; i < lineObjects.size(); ++i) {
        assert(lineObjects[i]->getInternalGpuIndex() == mLineObjectIndex + i);
        lineObjects[i]->uploadToDevice(
            *mObjectDataBuffer, (mLineObjectIndex + i) * bufferSize,
            *mShaderPack->getShaderInputLayouts()->objectDataBufferLayout);
      }
    }
    if (mShaderPack->hasPointPass()) {
      auto pointObjects = mScene->getPointObjects();
      for (uint32_t i = 0; i < pointObjects.size(); ++i) {
        assert(pointObjects[i]->getInternalGpuIndex() == mPointObjectIndex + i);
        pointObjects[i]->uploadToDevice(
            *mObjectDataBuffer, (mPointObjectIndex + i) * bufferSize,
            *mShaderPack->getShaderInputLayouts()->objectDataBufferLayout);
      }
    }
    mObjectDataBuffer->unmap();
  }

  mContext->getQueue().submit(mUploadCommandBuffer.get(), {});
}

void Renderer::forceUploadCameraBuffer(scene::Camera &camera) {
  if (mCameraBufferCpu && mUploadCommandBuffer) {
    camera.uploadToDevice(*mCameraBufferCpu,
                          *mShaderPack->getShaderInputLayouts()->cameraBufferLayout);
    mContext->getQueue().submitAndWait(mUploadCommandBuffer.get());
  }
}

void Renderer::render(scene::Camera &camera, std::vector<vk::Semaphore> const &waitSemaphores,
                      std::vector<vk::PipelineStageFlags> const &waitStages,
                      std::vector<vk::Semaphore> const &signalSemaphores, vk::Fence fence) {
  if (!mContext->isVulkanAvailable()) {
    return;
  }
  if (!mScene) {
    throw std::runtime_error("setScene must be called before rendering");
  }
  SVULKAN2_PROFILE_BLOCK("Record & Submit");
  prepareRender(camera);

  if (mAutoUpload) {
    uploadGpuResources(camera);
  }

  std::vector<vk::CommandBuffer> cbs = {mShadowCommandBuffer.get(), mRenderCommandBuffer.get()};
  mContext->getQueue().submit(cbs, waitSemaphores, waitStages, signalSemaphores, fence);
}

void Renderer::render(
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
  SVULKAN2_PROFILE_BLOCK("Record & Submit");
  prepareRender(camera);

  if (mAutoUpload) {
    uploadGpuResources(camera);
  }

  std::vector<vk::CommandBuffer> cbs = {mShadowCommandBuffer.get(), mRenderCommandBuffer.get()};
  mContext->getQueue().submit(cbs, waitSemaphores, waitStageMasks, waitValues, signalSemaphores,
                              signalValues, {});
}

std::vector<std::string> Renderer::getDisplayTargetNames() const {
  if (!mContext->isVulkanAvailable()) {
    return {};
  }

  std::unordered_set<std::string> names;
  for (auto pass : mShaderPack->getNonShadowPasses()) {
    for (auto &elem : pass->getTextureOutputLayout()->elements) {
      std::string texName = shader::getOutTextureName(elem.second.name);
      if (texName.ends_with("Depth")) {
        throw std::runtime_error("You are not allowed to name your texture \"*Depth\"");
      }
      if (elem.second.dtype == DataType::FLOAT4()) {
        names.insert(texName);
      }
    }
  }

  return std::vector(names.begin(), names.end());
}

std::vector<std::string> Renderer::getRenderTargetNames() const {
  if (!mContext->isVulkanAvailable()) {
    return {};
  }

  std::unordered_set<std::string> names;
  for (auto pass : mShaderPack->getNonShadowPasses()) {
    auto depthName = mShaderPackInstance->getDepthRenderTargetName(*pass);
    if (depthName.has_value()) {
      names.insert(depthName.value());
    }
    for (auto &elem : pass->getTextureOutputLayout()->elements) {
      std::string texName = shader::getOutTextureName(elem.second.name);
      if (texName.ends_with("Depth")) {
        throw std::runtime_error("You are not allowed to name your texture \"*Depth\"");
      }
      names.insert(texName);
    }
  }

  return std::vector(names.begin(), names.end());
}

void Renderer::display(std::string const &renderTargetName, vk::Image backBuffer,
                       vk::Format format, uint32_t width, uint32_t height,
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

  if (!mRenderTargets.contains(renderTargetName)) {
    throw std::runtime_error("failed to find render target with name " + renderTargetName +
                             ". Did you forget to take picture?");
  }

  auto renderTarget = mRenderTargets.at(renderTargetName);

  auto targetFormat = renderTarget->getFormat();
  if (targetFormat != vk::Format::eR8G8B8A8Unorm &&
      targetFormat != vk::Format::eR32G32B32A32Sfloat) {
    throw std::runtime_error("failed to display: only color textures are supported in display");
  };
  auto layout = renderTarget->getImage().getCurrentLayout(0);

  if (layout != vk::ImageLayout::eTransferSrcOptimal) {
    vk::AccessFlags sourceAccessMask;
    vk::PipelineStageFlags sourceStage;
    if (layout == vk::ImageLayout::eColorAttachmentOptimal) {
      sourceAccessMask = vk::AccessFlagBits::eColorAttachmentWrite;
      sourceStage = vk::PipelineStageFlagBits::eColorAttachmentOutput;
    } else if (layout == vk::ImageLayout::eShaderReadOnlyOptimal) {
      sourceAccessMask = {};
      sourceStage = vk::PipelineStageFlagBits::eTopOfPipe;
    } else {
      throw std::runtime_error("invalid layout");
    }
    // transfer render target
    renderTarget->getImage().transitionLayout(mDisplayCommandBuffer.get(), layout,
                                              vk::ImageLayout::eTransferSrcOptimal,
                                              sourceAccessMask, vk::AccessFlagBits::eTransferRead,
                                              sourceStage, vk::PipelineStageFlagBits::eTransfer);
  }

  // transfer swap chain
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
      renderTarget->getImage().getVulkanImage(), vk::ImageLayout::eTransferSrcOptimal, backBuffer,
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

  renderTarget->getImage().setCurrentLayout(vk::ImageLayout::eTransferSrcOptimal);
}

void Renderer::prepareSceneBuffer() {
  if (mShaderPack->getShadowPass()) {
    mShadowBuffer = core::Buffer::CreateUniform(
        mShaderPack->getShaderInputLayouts()->shadowBufferLayout->size);
  }
  mSceneBuffer =
      core::Buffer::CreateUniform(mShaderPack->getShaderInputLayouts()->sceneBufferLayout->size);
  auto layout = mShaderPackInstance->getSceneDescriptorSetLayout();

  mSceneSet = std::move(mContext->getDevice()
                            .allocateDescriptorSetsUnique(
                                vk::DescriptorSetAllocateInfo(mDescriptorPool.get(), 1, &layout))
                            .front());

  auto setDesc = mShaderPack->getShaderInputLayouts()->sceneSetDescription;

  for (uint32_t bindingIndex = 0; bindingIndex < setDesc.bindings.size(); ++bindingIndex) {
    auto binding = setDesc.bindings.at(bindingIndex);
    if (binding.name == "SceneBuffer") {
      updateDescriptorSets(
          mContext->getDevice(), mSceneSet.get(),
          {{vk::DescriptorType::eUniformBuffer, mSceneBuffer->getVulkanBuffer(), nullptr}}, {},
          bindingIndex);
    } else if (binding.name == "ShadowBuffer") {
      updateDescriptorSets(
          mContext->getDevice(), mSceneSet.get(),
          {{vk::DescriptorType::eUniformBuffer, mShadowBuffer->getVulkanBuffer(), nullptr}}, {},
          bindingIndex);
    } else if (binding.name == "samplerPointLightDepths") {
      std::vector<std::tuple<vk::ImageView, vk::Sampler>> textures;
      for (auto t : mPointShadowReadTargets) {
        textures.push_back({t->getImageView(), t->getSampler()});
      }
      updateArrayDescriptorSets(mContext->getDevice(), mSceneSet.get(), textures, bindingIndex);
    } else if (binding.name == "samplerDirectionalLightDepths") {
      std::vector<std::tuple<vk::ImageView, vk::Sampler>> textures;
      for (auto t : mDirectionalShadowReadTargets) {
        textures.push_back({t->getImageView(), t->getSampler()});
      }
      updateArrayDescriptorSets(mContext->getDevice(), mSceneSet.get(), textures, bindingIndex);
    } else if (binding.name == "samplerSpotLightDepths") {
      std::vector<std::tuple<vk::ImageView, vk::Sampler>> textures;
      for (auto t : mSpotShadowReadTargets) {
        textures.push_back({t->getImageView(), t->getSampler()});
      }
      updateArrayDescriptorSets(mContext->getDevice(), mSceneSet.get(), textures, bindingIndex);
    } else if (binding.name == "samplerTexturedLightDepths") {
      std::vector<std::tuple<vk::ImageView, vk::Sampler>> textures;
      for (auto t : mTexturedLightShadowReadTargets) {
        textures.push_back({t->getImageView(), t->getSampler()});
      }
      updateArrayDescriptorSets(mContext->getDevice(), mSceneSet.get(), textures, bindingIndex);
    } else if (binding.name == "samplerTexturedLightTextures") {
      auto lights = mScene->getTexturedLights();
      std::vector<std::tuple<vk::ImageView, vk::Sampler>> textures;
      for (auto l : lights) {
        auto t = l->getTexture();
        if (!t) {
          logger::error("A textured light does not have a texture attached");
          t = mContext->getResourceManager()->getDefaultTexture();
          t->loadAsync().get();
          t->uploadToDevice();
        }
        textures.push_back({t->getImageView(), t->getSampler()});
      }
      updateArrayDescriptorSets(mContext->getDevice(), mSceneSet.get(), textures, bindingIndex);
    } else if (binding.type == vk::DescriptorType::eCombinedImageSampler &&
               binding.name.substr(0, 7) == "sampler") {
      std::string customTextureName = binding.name.substr(7);
      if (customTextureName.substr(0, 6) == "Random") {
        auto randomTex = mContext->getResourceManager()->CreateRandomTexture(customTextureName);
        randomTex->uploadToDevice();
        updateDescriptorSets(mContext->getDevice(), mSceneSet.get(), {},
                             {{randomTex->getImageView(), randomTex->getSampler()}}, bindingIndex);
      } else if (mCustomTextures.find(customTextureName) != mCustomTextures.end()) {
        mCustomTextures[customTextureName]->uploadToDevice();
        updateDescriptorSets(mContext->getDevice(), mSceneSet.get(), {},
                             {{mCustomTextures[customTextureName]->getImageView(),
                               mCustomTextures[customTextureName]->getSampler()}},
                             bindingIndex);
      } else if (mCustomTextureArray.contains(customTextureName)) {
        auto textures = mCustomTextureArray.at(customTextureName);
        std::vector<std::tuple<vk::ImageView, vk::Sampler>> ts;
        for (auto t : textures) {
          t->uploadToDevice();
          ts.push_back({t->getImageView(), t->getSampler()});
        }
        updateDescriptorSets(mContext->getDevice(), mSceneSet.get(), {}, ts, bindingIndex);
      } else if (customTextureName == "BRDFLUT") {
        // generate if BRDFLUT is not supplied
        auto tex = mContext->getResourceManager()->getDefaultBRDFLUT();
        updateDescriptorSets(mContext->getDevice(), mSceneSet.get(), {},
                             {{tex->getImageView(), tex->getSampler()}}, bindingIndex);
      } else if (mCustomCubemaps.find(customTextureName) != mCustomCubemaps.end()) {
        mCustomCubemaps[customTextureName]->uploadToDevice();
        updateDescriptorSets(mContext->getDevice(), mSceneSet.get(), {},
                             {{mCustomCubemaps[customTextureName]->getImageView(),
                               mCustomCubemaps[customTextureName]->getSampler()}},
                             bindingIndex);
      } else if (customTextureName == "Environment") {
        auto cube = mEnvironmentMap;
        if (!cube) {
          cube = mContext->getResourceManager()->getDefaultCubemap();
        }
        cube->uploadToDevice();
        updateDescriptorSets(mContext->getDevice(), mSceneSet.get(), {},
                             {{cube->getImageView(), cube->getSampler()}}, bindingIndex);
      } else {
        throw std::runtime_error("custom sampler \"" + customTextureName +
                                 "\" is not set in the renderer");
      }
    } else {
      throw std::runtime_error("unrecognized uniform binding in scene \"" + binding.name + "\"");
    }
  }
} // namespace renderer

void Renderer::prepareObjectBuffers(uint32_t numObjects) {

  auto transformBuffer = mScene->getObjectTransformBuffer();

  if (mObjectTransformBuffer == transformBuffer) {
    // current object set and buffer bindings are still good
    return;
  }

  // the obejct buffer is outdated
  auto bufferSize = mShaderPack->getShaderInputLayouts()->objectDataBufferLayout->getAlignedSize(
      mContext->getPhysicalDeviceLimits().minUniformBufferOffsetAlignment);

  mObjectTransformBuffer = transformBuffer;

  size_t transformSize = mScene->getGpuTransformBufferSize();

  uint32_t newSize = mObjectTransformBuffer->getSize() / transformSize;
  assert(newSize >= numObjects);

  // reallocate data buffer
  mObjectDataBuffer = core::Buffer::CreateUniform(bufferSize * newSize, false);

  // expand or shrink sets
  if (mObjectSet.size() < newSize) {
    auto layout = mShaderPackInstance->getObjectDescriptorSetLayout();
    for (uint32_t i = mObjectSet.size(); i < newSize; ++i) {
      mObjectSet.push_back(mObjectPool->allocateSet(layout));
    }
  } else {
    mObjectSet.resize(newSize);
  }

  std::vector<vk::WriteDescriptorSet> writeDescriptorSets;
  writeDescriptorSets.reserve(mObjectSet.size() * 2);
  std::vector<std::array<vk::DescriptorBufferInfo, 2>> bufferInfos(mObjectSet.size());

  for (uint32_t i = 0; i < mObjectSet.size(); ++i) {
    auto buffer = mObjectDataBuffer->getVulkanBuffer();
    bufferInfos[i] = {vk::DescriptorBufferInfo(transformBuffer->getVulkanBuffer(),
                                               transformSize * i, transformSize),
                      vk::DescriptorBufferInfo(buffer, bufferSize * i, bufferSize)};
    // TODO: these 2 can merge into 1?
    writeDescriptorSets.push_back(vk::WriteDescriptorSet(
        mObjectSet[i].get(), 0, 0, vk::DescriptorType::eUniformBuffer, {}, bufferInfos[i][0], {}));
    writeDescriptorSets.push_back(vk::WriteDescriptorSet(
        mObjectSet[i].get(), 1, 0, vk::DescriptorType::eUniformBuffer, {}, bufferInfos[i][1], {}));
  }
  mContext->getDevice().updateDescriptorSets(writeDescriptorSets, nullptr);

  // bool updated{false};

  // // make sure object buffer can be created
  // if (numObjects == 0) {
  //   numObjects = 1;
  // }

  // // shrink
  // if (numObjects * 2 < mObjectSet.size()) {
  //   updated = true;
  //   uint32_t newSize = numObjects;

  //   // reallocate buffer
  //   mObjectDataBuffer = core::Buffer::CreateUniform(bufferSize * newSize, false);

  //   mObjectSet.resize(newSize);
  // }
  // // expand
  // if (numObjects > mObjectSet.size()) {
  //   updated = true;
  //   uint32_t newSize = std::max(numObjects, 2 * static_cast<uint32_t>(mObjectSet.size()));

  //   // reallocate buffer
  //   mObjectDataBuffer = core::Buffer::CreateUniform(bufferSize * newSize, false);

  //   auto layout = mShaderPackInstance->getObjectDescriptorSetLayout();
  //   for (uint32_t i = mObjectSet.size(); i < newSize; ++i) {
  //     mObjectSet.push_back(mObjectPool->allocateSet(layout));
  //   }
  // }

  // if (updated) {
  //   std::vector<vk::WriteDescriptorSet> writeDescriptorSets;
  //   // std::vector<vk::DescriptorBufferInfo> bufferInfos(mObjectSet.size());
  //   std::vector<vk::DescriptorBufferInfo> bufferInfos;
  //   bufferInfos.reserve(mObjectSet.size() * 2);

  //   for (uint32_t i = 0; i < mObjectSet.size(); ++i) {
  //     auto buffer = mObjectDataBuffer->getVulkanBuffer();
  //     bufferInfos[i] = vk::DescriptorBufferInfo(buffer, bufferSize * i, bufferSize);
  //     writeDescriptorSets.push_back(vk::WriteDescriptorSet(
  //         mObjectSet[i].get(), 0, 0, vk::DescriptorType::eUniformBuffer, {}, bufferInfos[i],
  //         {}));
  //   }
  //   mContext->getDevice().updateDescriptorSets(writeDescriptorSets, nullptr);
  // }
}

void Renderer::prepareLightBuffers() {
  auto lightBufferLayout = mShaderPack->getShaderInputLayouts()->lightBufferLayout;

  uint32_t numShadows = mPointLightShadowSizes.size() * 6 + mDirectionalLightShadowSizes.size() +
                        mSpotLightShadowSizes.size() + mTexturedLightShadowSizes.size();

  // too many shadow sets
  if (numShadows * 2 < mLightSets.size()) {
    uint32_t newSize = numShadows;
    mLightSets.resize(newSize);
    mLightBuffers.resize(newSize);
  }

  // too few shadow sets
  for (uint32_t i = mLightSets.size(); i < numShadows; ++i) {
    auto layout = mShaderPackInstance->getLightDescriptorSetLayout();
    mLightBuffers.push_back(core::Buffer::CreateUniform(lightBufferLayout->size));
    auto shadowSet = std::move(mContext->getDevice()
                                   .allocateDescriptorSetsUnique(vk::DescriptorSetAllocateInfo(
                                       mDescriptorPool.get(), 1, &layout))
                                   .front());
    mLightSets.push_back(std::move(shadowSet));
    updateDescriptorSets(
        mContext->getDevice(), mLightSets.back().get(),
        {{vk::DescriptorType::eUniformBuffer, mLightBuffers.back()->getVulkanBuffer(), nullptr}},
        {}, 0);
  }
}

void Renderer::prepareInputTextureDescriptorSets() {
  auto layouts = mShaderPackInstance->getInputTextureLayouts();
  auto passes = mShaderPack->getNonShadowPasses();

  mInputTextureSets.clear();
  for (uint32_t i = 0; i < passes.size(); ++i) {
    auto layout = layouts[i];
    if (layout) {
      mInputTextureSets.push_back(
          std::move(mContext->getDevice()
                        .allocateDescriptorSetsUnique(
                            vk::DescriptorSetAllocateInfo(mDescriptorPool.get(), 1, &layout))
                        .front()));

      std::vector<std::tuple<vk::ImageView, vk::Sampler>> textureData;
      auto names = passes[i]->getInputTextureNames();
      for (auto name : names) {
        if (mRenderTargets.find(name) != mRenderTargets.end()) {
          textureData.push_back(
              {mRenderTargets[name]->getImageView(), mRenderTargets[name]->getSampler()});
        } else if (name.substr(0, 6) == "Random") {
          auto randomTex = mContext->getResourceManager()->CreateRandomTexture(name);
          randomTex->uploadToDevice();
          textureData.push_back({randomTex->getImageView(), randomTex->getSampler()});
        } else if (mCustomTextures.find(name) != mCustomTextures.end()) {
          mCustomTextures[name]->uploadToDevice();
          textureData.push_back(
              {mCustomTextures[name]->getImageView(), mCustomTextures[name]->getSampler()});
        } else if (name == "BRDFLUT") {
          // generate if BRDFLUT is not supplied
          auto tex = mContext->getResourceManager()->getDefaultBRDFLUT();
          textureData.push_back({tex->getImageView(), tex->getSampler()});
        } else if (mCustomCubemaps.find(name) != mCustomCubemaps.end()) {
          mCustomCubemaps[name]->uploadToDevice();
          textureData.push_back(
              {mCustomCubemaps[name]->getImageView(), mCustomCubemaps[name]->getSampler()});
        } else if (name == "Environment") {
          auto cube = mEnvironmentMap;
          if (!cube) {
            cube = mContext->getResourceManager()->getDefaultCubemap();
          }
          cube->uploadToDevice();
          textureData.push_back({cube->getImageView(), cube->getSampler()});
        } else {
          throw std::runtime_error("custom sampler \"" + name + "\" is not set in the renderer");
        }
      }
      updateDescriptorSets(mContext->getDevice(), mInputTextureSets.back().get(), {}, textureData,
                           0);
    } else {
      mInputTextureSets.push_back({});
    }
  }
}

void Renderer::prepareCameraBuffer() {
  if (mCameraBuffer) {
    // camera buffer can always be reused
    return;
  }

  mCameraBufferCpu =
      core::Buffer::Create(mShaderPack->getShaderInputLayouts()->cameraBufferLayout->size,
                           vk::BufferUsageFlagBits::eTransferSrc, VMA_MEMORY_USAGE_CPU_ONLY);
  mCameraBuffer = core::Buffer::CreateUniform(
      mShaderPack->getShaderInputLayouts()->cameraBufferLayout->size, true, true);

  auto layout = mShaderPackInstance->getCameraDescriptorSetLayout();
  mCameraSet = std::move(mContext->getDevice()
                             .allocateDescriptorSetsUnique(
                                 vk::DescriptorSetAllocateInfo(mDescriptorPool.get(), 1, &layout))
                             .front());
  updateDescriptorSets(
      mContext->getDevice(), mCameraSet.get(),
      {{vk::DescriptorType::eUniformBuffer, mCameraBuffer->getVulkanBuffer(), nullptr}}, {}, 0);
}

void Renderer::setCustomTextureArray(std::string const &name,
                                     std::vector<std::shared_ptr<resource::SVTexture>> textures) {
  mCustomTextureArray[name] = textures;
}
void Renderer::setCustomTexture(std::string const &name,
                                std::shared_ptr<resource::SVTexture> texture) {
  mCustomTextures[name] = texture;
}

void Renderer::setCustomCubemap(std::string const &name,
                                std::shared_ptr<resource::SVCubemap> cubemap) {
  mCustomCubemaps[name] = cubemap;
}

int Renderer::getCustomPropertyInt(std::string const &name) const {
  if (mSpecializationConstants.contains(name)) {
    auto c = mSpecializationConstants.at(name);
    if (c.dtype == DataType::INT()) {
      int v;
      std::memcpy(&v, c.buffer, sizeof(int));
      return v;
    }
  }
  throw std::runtime_error("invalid property " + name);
}

float Renderer::getCustomPropertyFloat(std::string const &name) const {
  if (mSpecializationConstants.contains(name)) {
    auto c = mSpecializationConstants.at(name);
    if (c.dtype == DataType::FLOAT()) {
      float v;
      std::memcpy(&v, c.buffer, sizeof(float));
      return v;
    }
  }
  throw std::runtime_error("invalid property " + name);
}

glm::vec3 Renderer::getCustomPropertyVec3(std::string const &name) const {
  if (mSpecializationConstants.contains(name)) {
    auto c = mSpecializationConstants.at(name);
    if (c.dtype == DataType::FLOAT3()) {
      glm::vec3 v;
      std::memcpy(&v[0], c.buffer, sizeof(float) * 3);
      return v;
    }
  }
  throw std::runtime_error("invalid property " + name);
}
glm::vec4 Renderer::getCustomPropertyVec4(std::string const &name) const {
  if (mSpecializationConstants.contains(name)) {
    auto c = mSpecializationConstants.at(name);
    if (c.dtype == DataType::FLOAT3()) {
      glm::vec4 v;
      std::memcpy(&v[0], c.buffer, sizeof(float) * 4);
      return v;
    }
  }
  throw std::runtime_error("invalid property " + name);
}

void Renderer::setCustomProperty(std::string const &name, int p) {
  setSpecializationConstant(name, p);
}

void Renderer::setCustomProperty(std::string const &name, float p) {
  setSpecializationConstant(name, p);
}

void Renderer::setCustomProperty(std::string const &name, glm::vec3 p) {
  setSpecializationConstant(name, p);
}

void Renderer::setCustomProperty(std::string const &name, glm::vec4 p) {
  setSpecializationConstant(name, p);
}

std::shared_ptr<resource::SVRenderTarget>
Renderer::getRenderTarget(std::string const &name) const {
  return mRenderTargets.at(name);
}

core::Buffer &Renderer::getCameraBuffer() {
  prepareCameraBuffer();
  return *mCameraBuffer;
}

void Renderer::setAutoUploadEnabled(bool enable) { mAutoUpload = enable; }

} // namespace renderer
} // namespace svulkan2