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
#include "svulkan2/shader/compute.h"
#include "svulkan2/common/fs.h"
#include "svulkan2/core/context.h"
#include "svulkan2/shader/brdf_lut.h"
#include "svulkan2/shader/latlong_to_cubemap.h"
#include "svulkan2/shader/prefilter.h"

namespace svulkan2 {
namespace shader {

std::unique_ptr<core::Image> generateBRDFLUT(uint32_t size) {
  auto context = core::Context::Get();
  auto image = std::make_unique<core::Image>(
      vk::ImageType::e2D, vk::Extent3D{size, size, 1}, vk::Format::eR16G16Sfloat,
      vk::ImageUsageFlagBits::eSampled | vk::ImageUsageFlagBits::eTransferSrc |
          vk::ImageUsageFlagBits::eStorage,
      VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY);

  auto device = context->getDevice();

  vk::DescriptorSetLayoutBinding binding(0, vk::DescriptorType::eStorageImage, 1,
                                         vk::ShaderStageFlagBits::eCompute);
  auto descriptorSetLayout =
      device.createDescriptorSetLayoutUnique(vk::DescriptorSetLayoutCreateInfo({}, binding));
  auto pipelineLayout = device.createPipelineLayoutUnique(
      vk::PipelineLayoutCreateInfo({}, descriptorSetLayout.get()));

  auto descriptorSet = context->getDescriptorPool().allocateSet(descriptorSetLayout.get());

  vk::ImageViewCreateInfo viewInfo(
      {}, image->getVulkanImage(), vk::ImageViewType::e2D, image->getFormat(),
      vk::ComponentMapping(vk::ComponentSwizzle::eR, vk::ComponentSwizzle::eG,
                           vk::ComponentSwizzle::eB, vk::ComponentSwizzle::eA),
      vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1));
  auto imageView = device.createImageViewUnique(viewInfo);

  auto imageInfo = vk::DescriptorImageInfo({}, imageView.get(), vk::ImageLayout::eGeneral);
  vk::WriteDescriptorSet write(descriptorSet.get(), 0, 0, vk::DescriptorType::eStorageImage,
                               imageInfo);
  device.updateDescriptorSets(write, {});

  std::vector<uint32_t> code(reinterpret_cast<const uint32_t *>(brdf_lut_code),
                             reinterpret_cast<const uint32_t *>(brdf_lut_code) +
                                 brdf_lut_size / 4);

  auto shaderModule = device.createShaderModuleUnique(vk::ShaderModuleCreateInfo({}, code));
  auto shaderStageInfo = vk::PipelineShaderStageCreateInfo({}, vk::ShaderStageFlagBits::eCompute,
                                                           shaderModule.get(), "main");
  vk::ComputePipelineCreateInfo computePipelineCreateInfo({}, shaderStageInfo,
                                                          pipelineLayout.get());
  auto pipelinCache = device.createPipelineCacheUnique({});
  auto pipeline =
      device.createComputePipelineUnique(pipelinCache.get(), computePipelineCreateInfo).value;

  auto pool = context->createCommandPool();
  auto cb = pool->allocateCommandBuffer();
  cb->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
  image->transitionLayout(cb.get(), vk::ImageLayout::eUndefined, vk::ImageLayout::eGeneral, {},
                          vk::AccessFlagBits::eShaderWrite, vk::PipelineStageFlagBits::eTopOfPipe,
                          vk::PipelineStageFlagBits::eComputeShader);

  cb->bindPipeline(vk::PipelineBindPoint::eCompute, pipeline.get());
  cb->bindDescriptorSets(vk::PipelineBindPoint::eCompute, pipelineLayout.get(), 0,
                         descriptorSet.get(), {});
  cb->dispatch(size, size, 1);
  image->transitionLayout(
      cb.get(), vk::ImageLayout::eGeneral, vk::ImageLayout::eShaderReadOnlyOptimal,
      vk::AccessFlagBits::eShaderWrite, vk::AccessFlagBits::eShaderRead,
      vk::PipelineStageFlagBits::eComputeShader, vk::PipelineStageFlagBits::eFragmentShader);
  cb->end();
  image->setCurrentLayout(vk::ImageLayout::eShaderReadOnlyOptimal);

  context->getQueue().submitAndWait(cb.get());
  return image;
}

void prefilterCubemap(core::Image &image) {
  if (image.getMipLevels() == 1) {
    return;
  }

  auto context = core::Context::Get();
  auto device = context->getDevice();

  auto binding0 = vk::DescriptorSetLayoutBinding(0, vk::DescriptorType::eCombinedImageSampler, 1,
                                                 vk::ShaderStageFlagBits::eCompute);
  auto binding1 = vk::DescriptorSetLayoutBinding(0, vk::DescriptorType::eStorageImage, 1,
                                                 vk::ShaderStageFlagBits::eCompute);
  auto descriptorSetLayout0 =
      device.createDescriptorSetLayoutUnique(vk::DescriptorSetLayoutCreateInfo({}, binding0));
  auto descriptorSetLayout1 =
      device.createDescriptorSetLayoutUnique(vk::DescriptorSetLayoutCreateInfo({}, binding1));
  std::vector<vk::DescriptorSetLayout> layouts = {descriptorSetLayout0.get(),
                                                  descriptorSetLayout1.get()};
  auto pushConstantRange =
      vk::PushConstantRange(vk::ShaderStageFlagBits::eCompute, 0, sizeof(float));
  auto pipelineLayout = device.createPipelineLayoutUnique(
      vk::PipelineLayoutCreateInfo({}, layouts, pushConstantRange));

  // resources for the base sampler
  auto descriptorSet = context->getDescriptorPool().allocateSet(descriptorSetLayout0.get());

  vk::ImageViewCreateInfo viewInfo(
      {}, image.getVulkanImage(), vk::ImageViewType::eCube, image.getFormat(),
      vk::ComponentSwizzle::eIdentity,
      vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 6));
  auto imageView = device.createImageViewUnique(viewInfo);
  auto sampler = device.createSamplerUnique(vk::SamplerCreateInfo(
      {}, vk::Filter::eLinear, vk::Filter::eLinear, vk::SamplerMipmapMode::eNearest,
      vk::SamplerAddressMode::eClampToEdge, vk::SamplerAddressMode::eClampToEdge,
      vk::SamplerAddressMode::eClampToEdge, 0.f, false, 0.f, false, vk::CompareOp::eNever, 0.f,
      0.f, vk::BorderColor::eFloatOpaqueWhite));
  auto imageInfo = vk::DescriptorImageInfo(sampler.get(), imageView.get(),
                                           vk::ImageLayout::eShaderReadOnlyOptimal);
  vk::WriteDescriptorSet write(descriptorSet.get(), 0, 0,
                               vk::DescriptorType::eCombinedImageSampler, imageInfo);
  device.updateDescriptorSets(write, {});

  std::vector<uint32_t> code(reinterpret_cast<const uint32_t *>(prefilter_code),
                             reinterpret_cast<const uint32_t *>(prefilter_code) +
                                 prefilter_size / 4);

  auto shaderModule = device.createShaderModuleUnique(vk::ShaderModuleCreateInfo({}, code));
  auto shaderStageInfo = vk::PipelineShaderStageCreateInfo({}, vk::ShaderStageFlagBits::eCompute,
                                                           shaderModule.get(), "main");
  vk::ComputePipelineCreateInfo computePipelineCreateInfo({}, shaderStageInfo,
                                                          pipelineLayout.get());
  auto pipelinCache = device.createPipelineCacheUnique({});
  auto pipeline =
      device.createComputePipelineUnique(pipelinCache.get(), computePipelineCreateInfo).value;

  std::vector<vk::UniqueImageView> levelViews;
  std::vector<vk::UniqueDescriptorSet> levelSets;

  uint32_t size = image.getExtent().width;
  uint32_t levels = image.getMipLevels();
  for (uint32_t level = 1; level < levels; ++level) {
    size /= 2;
    vk::ImageViewCreateInfo viewInfo(
        {}, image.getVulkanImage(), vk::ImageViewType::eCube, image.getFormat(),
        vk::ComponentSwizzle::eIdentity,
        vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, level, 1, 0, 6));
    levelViews.push_back(device.createImageViewUnique(viewInfo));
    levelSets.push_back(context->getDescriptorPool().allocateSet(descriptorSetLayout1.get()));

    auto imageInfo =
        vk::DescriptorImageInfo({}, levelViews.back().get(), vk::ImageLayout::eGeneral);
    vk::WriteDescriptorSet write(levelSets.back().get(), 0, 0, vk::DescriptorType::eStorageImage,
                                 imageInfo);
    device.updateDescriptorSets(write, {});
  }

  auto pool = context->createCommandPool();
  auto cb = pool->allocateCommandBuffer();

  cb->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});

  // transition mipmap levels for compute shader write
  auto barrier = vk::ImageMemoryBarrier(
      {}, vk::AccessFlagBits::eShaderWrite, vk::ImageLayout::eUndefined, vk::ImageLayout::eGeneral,
      VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED, image.getVulkanImage(),
      vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 1, levels - 1, 0, 6));
  cb->pipelineBarrier(vk::PipelineStageFlagBits::eTopOfPipe,
                      vk::PipelineStageFlagBits::eComputeShader, {}, {}, {}, barrier);

  // execute compute
  size = image.getExtent().width;
  for (uint32_t level = 1; level < levels; ++level) {
    size /= 2;
    float roughness = static_cast<float>(level) / (levels - 1);
    cb->bindPipeline(vk::PipelineBindPoint::eCompute, pipeline.get());
    cb->bindDescriptorSets(vk::PipelineBindPoint::eCompute, pipelineLayout.get(), 0,
                           descriptorSet.get(), {});
    cb->pushConstants(pipelineLayout.get(), vk::ShaderStageFlagBits::eCompute, 0, sizeof(float),
                      &roughness);
    cb->bindDescriptorSets(vk::PipelineBindPoint::eCompute, pipelineLayout.get(), 1,
                           levelSets[level - 1].get(), {});
    cb->dispatch(size, size, 6);

    // if compute takes too long, it can cause device lost, so we split it
    cb->end();
    context->getQueue().submitAndWait(cb.get());
    cb->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
  }

  // transition mipmaps for shader read
  barrier = vk::ImageMemoryBarrier(
      vk::AccessFlagBits::eShaderWrite, vk::AccessFlagBits::eShaderRead, vk::ImageLayout::eGeneral,
      vk::ImageLayout::eShaderReadOnlyOptimal, VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED,
      image.getVulkanImage(),
      vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 1, levels - 1, 0, 6));
  cb->pipelineBarrier(vk::PipelineStageFlagBits::eComputeShader,
                      vk::PipelineStageFlagBits::eFragmentShader, {}, {}, {}, barrier);

  image.setCurrentLayout(vk::ImageLayout::eShaderReadOnlyOptimal);
  cb->end();
  context->getQueue().submitAndWait(cb.get());
}

std::unique_ptr<core::Image> latlongToCube(core::Image &latlong, int mipLevels) {
  uint32_t width = latlong.getExtent().width;
  uint32_t height = latlong.getExtent().height;
  if (width != height * 2) {
    throw std::runtime_error("invalid latlong map");
  }

  auto context = core::Context::Get();
  auto device = context->getDevice();

  auto cubemap = std::make_unique<core::Image>(
      vk::ImageType::e2D, vk::Extent3D{height, height, 1}, latlong.getFormat(),
      vk::ImageUsageFlagBits::eTransferSrc | vk::ImageUsageFlagBits::eTransferDst |
          vk::ImageUsageFlagBits::eSampled | vk::ImageUsageFlagBits::eStorage,
      VMA_MEMORY_USAGE_GPU_ONLY, vk::SampleCountFlagBits::e1, mipLevels, 6,
      vk::ImageTiling::eOptimal, vk::ImageCreateFlagBits::eCubeCompatible);

  auto binding0 = vk::DescriptorSetLayoutBinding(0, vk::DescriptorType::eCombinedImageSampler, 1,
                                                 vk::ShaderStageFlagBits::eCompute);
  auto binding1 = vk::DescriptorSetLayoutBinding(0, vk::DescriptorType::eStorageImage, 1,
                                                 vk::ShaderStageFlagBits::eCompute);
  auto descriptorSetLayout0 =
      device.createDescriptorSetLayoutUnique(vk::DescriptorSetLayoutCreateInfo({}, binding0));
  auto descriptorSetLayout1 =
      device.createDescriptorSetLayoutUnique(vk::DescriptorSetLayoutCreateInfo({}, binding1));
  std::vector<vk::DescriptorSetLayout> layouts = {descriptorSetLayout0.get(),
                                                  descriptorSetLayout1.get()};
  auto pipelineLayout =
      device.createPipelineLayoutUnique(vk::PipelineLayoutCreateInfo({}, layouts));

  // 2D image
  auto descriptorSet = context->getDescriptorPool().allocateSet(descriptorSetLayout0.get());
  vk::ImageViewCreateInfo viewInfo(
      {}, latlong.getVulkanImage(), vk::ImageViewType::e2D, latlong.getFormat(),
      vk::ComponentSwizzle::eIdentity,
      vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1));
  auto imageView = device.createImageViewUnique(viewInfo);
  auto sampler = device.createSamplerUnique(vk::SamplerCreateInfo(
      {}, vk::Filter::eLinear, vk::Filter::eLinear, vk::SamplerMipmapMode::eNearest,
      vk::SamplerAddressMode::eRepeat, vk::SamplerAddressMode::eRepeat,
      vk::SamplerAddressMode::eRepeat, 0.f, false, 0.f, false, vk::CompareOp::eNever, 0.f, 0.f,
      vk::BorderColor::eFloatOpaqueWhite));
  auto imageInfo = vk::DescriptorImageInfo(sampler.get(), imageView.get(),
                                           vk::ImageLayout::eShaderReadOnlyOptimal);
  vk::WriteDescriptorSet write(descriptorSet.get(), 0, 0,
                               vk::DescriptorType::eCombinedImageSampler, imageInfo);
  device.updateDescriptorSets(write, {});

  std::vector<uint32_t> code(reinterpret_cast<const uint32_t *>(latlong_to_cubemap_code),
                             reinterpret_cast<const uint32_t *>(latlong_to_cubemap_code) +
                                 latlong_to_cubemap_size / 4);
  auto shaderModule = device.createShaderModuleUnique(vk::ShaderModuleCreateInfo({}, code));
  auto shaderStageInfo = vk::PipelineShaderStageCreateInfo({}, vk::ShaderStageFlagBits::eCompute,
                                                           shaderModule.get(), "main");
  vk::ComputePipelineCreateInfo computePipelineCreateInfo({}, shaderStageInfo,
                                                          pipelineLayout.get());
  auto pipelinCache = device.createPipelineCacheUnique({});
  auto pipeline =
      device.createComputePipelineUnique(pipelinCache.get(), computePipelineCreateInfo).value;

  // cube image
  vk::ImageViewCreateInfo cubeBaseViewInfo(
      {}, cubemap->getVulkanImage(), vk::ImageViewType::eCube, cubemap->getFormat(),
      vk::ComponentSwizzle::eIdentity,
      vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, 1, 0, 6));
  auto cubeView = device.createImageViewUnique(cubeBaseViewInfo);
  auto cubeSet = context->getDescriptorPool().allocateSet(descriptorSetLayout1.get());
  auto cubeImageInfo = vk::DescriptorImageInfo({}, cubeView.get(), vk::ImageLayout::eGeneral);
  vk::WriteDescriptorSet write2(cubeSet.get(), 0, 0, vk::DescriptorType::eStorageImage,
                                cubeImageInfo);
  device.updateDescriptorSets(write2, {});

  auto pool = context->createCommandPool();
  auto cb = pool->allocateCommandBuffer();

  cb->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});

  cubemap->transitionLayout(cb.get(), vk::ImageLayout::eUndefined, vk::ImageLayout::eGeneral, {},
                            vk::AccessFlagBits::eShaderWrite,
                            vk::PipelineStageFlagBits::eTopOfPipe,
                            vk::PipelineStageFlagBits::eComputeShader);

  cb->bindPipeline(vk::PipelineBindPoint::eCompute, pipeline.get());
  cb->bindDescriptorSets(vk::PipelineBindPoint::eCompute, pipelineLayout.get(), 0,
                         descriptorSet.get(), {});
  cb->bindDescriptorSets(vk::PipelineBindPoint::eCompute, pipelineLayout.get(), 1, cubeSet.get(),
                         {});
  cb->dispatch(height, height, 6);

  cubemap->transitionLayout(
      cb.get(), vk::ImageLayout::eGeneral, vk::ImageLayout::eShaderReadOnlyOptimal,
      vk::AccessFlagBits::eShaderWrite, vk::AccessFlagBits::eShaderRead,
      vk::PipelineStageFlagBits::eComputeShader, vk::PipelineStageFlagBits::eComputeShader);

  cb->end();
  context->getQueue().submitAndWait(cb.get());

  return cubemap;
}

} // namespace shader
} // namespace svulkan2