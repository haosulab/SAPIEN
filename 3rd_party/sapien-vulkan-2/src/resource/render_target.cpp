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
#include "svulkan2/resource/render_target.h"
#include "svulkan2/core/context.h"

namespace svulkan2 {
namespace resource {

SVRenderTarget::SVRenderTarget(std::string const &name, uint32_t width, uint32_t height,
                               vk::Format format, vk::SampleCountFlagBits sampleCount)
    : mName(name), mFormat(format), mWidth(width), mHeight(height), mSampleCount(sampleCount) {}

SVRenderTarget::SVRenderTarget(std::string const &name, uint32_t width, uint32_t height,
                               std::shared_ptr<core::Image> image, vk::UniqueImageView imageView,
                               vk::Sampler sampler)
    : mName(name), mFormat(image->getFormat()), mWidth(width), mHeight(height), mImage(image),
      mImageView(std::move(imageView)), mSampler(sampler) {}

void SVRenderTarget::createDeviceResources() {
  bool isDepth = false;
  vk::ImageUsageFlags usage;
  if (mFormat == vk::Format::eR8G8B8A8Unorm ||

      mFormat == vk::Format::eR32G32B32A32Sfloat || mFormat == vk::Format::eR16G16B16A16Sfloat ||

      mFormat == vk::Format::eR32G32B32A32Uint || mFormat == vk::Format::eR32G32B32A32Sint ||

      mFormat == vk::Format::eR16G16B16A16Uint || mFormat == vk::Format::eR16G16B16A16Sint ||

      mFormat == vk::Format::eR32Sfloat || mFormat == vk::Format::eR16Sfloat) {
    usage = vk::ImageUsageFlagBits::eSampled | vk::ImageUsageFlagBits::eColorAttachment |
            vk::ImageUsageFlagBits::eTransferSrc;
  } else if (mFormat == vk::Format::eD32Sfloat) {
    usage = vk::ImageUsageFlagBits::eSampled | vk::ImageUsageFlagBits::eDepthStencilAttachment |
            vk::ImageUsageFlagBits::eTransferSrc;
    isDepth = true;
  } else {
    throw std::runtime_error("failed to create image resources: unsupported image format.");
  }

  mContext = core::Context::Get();
  mImage = std::make_shared<core::Image>(vk::ImageType::e2D, vk::Extent3D{mWidth, mHeight, 1},
                                         mFormat, usage, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY,
                                         mSampleCount, 1);
  vk::ComponentMapping componentMapping(vk::ComponentSwizzle::eR, vk::ComponentSwizzle::eG,
                                        vk::ComponentSwizzle::eB, vk::ComponentSwizzle::eA);

  vk::ImageViewCreateInfo info({}, mImage->getVulkanImage(), vk::ImageViewType::e2D, mFormat,
                               componentMapping,
                               vk::ImageSubresourceRange(isDepth ? vk::ImageAspectFlagBits::eDepth
                                                                 : vk::ImageAspectFlagBits::eColor,
                                                         0, 1, 0, 1));
  mImageView = mContext->getDevice().createImageViewUnique(info);
  mSampler = mContext->createSampler(vk::SamplerCreateInfo(
      {}, vk::Filter::eNearest, vk::Filter::eNearest, vk::SamplerMipmapMode::eNearest,
      vk::SamplerAddressMode::eClampToEdge, vk::SamplerAddressMode::eClampToEdge,
      vk::SamplerAddressMode::eClampToEdge, 0.f, false, 0.f, false, vk::CompareOp::eNever, 0.f,
      0.f, vk::BorderColor::eFloatOpaqueBlack));
}

} // namespace resource
} // namespace svulkan2