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
#include "svulkan2/resource/storage_image.h"
#include "svulkan2/core/context.h"

namespace svulkan2 {
namespace resource {

SVStorageImage::SVStorageImage(std::string const &name, uint32_t width, uint32_t height,
                               vk::Format format)
    : mName(name), mFormat(format), mWidth(width), mHeight(height) {}

void SVStorageImage::createDeviceResources() {
  bool isDepth = false;
  vk::ImageUsageFlags usage;
  if (mFormat == vk::Format::eR8G8B8A8Unorm || mFormat == vk::Format::eR32G32B32A32Sfloat ||
      mFormat == vk::Format::eR32G32B32A32Uint || mFormat == vk::Format::eR32Sfloat) {
    usage = vk::ImageUsageFlagBits::eStorage | vk::ImageUsageFlagBits::eTransferSrc |
            vk::ImageUsageFlagBits::eTransferDst | vk::ImageUsageFlagBits::eSampled;
  } else if (mFormat == vk::Format::eD32Sfloat) {
    usage = vk::ImageUsageFlagBits::eStorage | vk::ImageUsageFlagBits::eTransferSrc |
            vk::ImageUsageFlagBits::eTransferDst | vk::ImageUsageFlagBits::eSampled;
    isDepth = true;
  } else {
    throw std::runtime_error(
        "failed to create image resources: unsupported image format. Currently "
        "supported formats are R32Sfloat, R32G32B32A32Sfloat, R8G8B8A8Unorm, "
        "R32G32B32A32Uint, D32Sfloat, D24UnormS8Uint");
  }

  mContext = core::Context::Get();
  mImage = std::make_shared<core::Image>(vk::ImageType::e2D, vk::Extent3D{mWidth, mHeight, 1},
                                         mFormat, usage, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY,
                                         vk::SampleCountFlagBits::e1, 1);
  vk::ComponentMapping componentMapping(vk::ComponentSwizzle::eR, vk::ComponentSwizzle::eG,
                                        vk::ComponentSwizzle::eB, vk::ComponentSwizzle::eA);

  vk::ImageViewCreateInfo info({}, mImage->getVulkanImage(), vk::ImageViewType::e2D, mFormat,
                               componentMapping,
                               vk::ImageSubresourceRange(isDepth ? vk::ImageAspectFlagBits::eDepth
                                                                 : vk::ImageAspectFlagBits::eColor,
                                                         0, 1, 0, 1));
  mImageView = mContext->getDevice().createImageViewUnique(info);
}

} // namespace resource
} // namespace svulkan2