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
#include "svulkan2/resource/texture.h"
#include "../common/logger.h"
#include "svulkan2/common/launch_policy.h"
#include "svulkan2/core/context.h"
#include "svulkan2/resource/manager.h"

namespace svulkan2 {
namespace resource {

std::shared_ptr<SVTexture> SVTexture::FromFile(std::string const &filename, uint32_t mipLevels,
                                               vk::Filter magFilter, vk::Filter minFilter,
                                               vk::SamplerAddressMode addressModeU,
                                               vk::SamplerAddressMode addressModeV, bool srgb,
                                               uint32_t desiredChannels) {
  auto texture = std::shared_ptr<SVTexture>(new SVTexture);
  texture->mDescription = SVTextureDescription{.source = SVTextureDescription::SourceType::eFILE,
                                               .filename = filename,
                                               .desiredChannels = desiredChannels,
                                               .mipLevels = mipLevels,
                                               .magFilter = magFilter,
                                               .minFilter = minFilter,
                                               .addressModeU = addressModeU,
                                               .addressModeV = addressModeV,
                                               .srgb = srgb,
                                               .dim = 2};
  return texture;
}

std::shared_ptr<SVTexture> SVTexture::FromRawData(uint32_t width, uint32_t height, uint32_t depth,
                                                  vk::Format format, std::vector<char> const &data,
                                                  int dim, uint32_t mipLevels,
                                                  vk::Filter magFilter, vk::Filter minFilter,
                                                  vk::SamplerAddressMode addressModeU,
                                                  vk::SamplerAddressMode addressModeV,
                                                  vk::SamplerAddressMode addressModeW, bool srgb) {
  if (srgb && !getFormatSupportSrgb(format)) {
    throw std::runtime_error("format does not support srgb");
  }

  auto texture = std::shared_ptr<SVTexture>(new SVTexture);
  texture->mDescription = {.source = SVTextureDescription::SourceType::eCUSTOM,
                           .format = format,
                           .filename = {},
                           .mipLevels = mipLevels,
                           .magFilter = magFilter,
                           .minFilter = minFilter,
                           .addressModeU = addressModeU,
                           .addressModeV = addressModeV,
                           .addressModeW = addressModeW,
                           .srgb = srgb,
                           .dim = dim};
  if (dim <= 0 || dim > 3) {
    throw std::runtime_error("texture dimension must be 1, 2 or 3");
  }
  if (dim == 1 && (height != 1 || depth != 1)) {
    throw std::runtime_error("1D texture must have height = 1 and depth = 1");
  }
  if (dim == 2 && (depth != 1)) {
    throw std::runtime_error("2D texture must have depth = 1");
  }

  vk::ImageType type =
      dim == 1 ? vk::ImageType::e1D : (dim == 2 ? vk::ImageType::e2D : vk::ImageType::e3D);

  texture->mImage = SVImage::FromRawData(type, width, height, depth, format, {data}, mipLevels);
  texture->mLoaded = true;
  return texture;
}

std::shared_ptr<SVTexture> SVTexture::FromImage(std::shared_ptr<SVImage> image,
                                                vk::UniqueImageView imageView,
                                                vk::Sampler sampler) {
  auto texture = std::shared_ptr<SVTexture>(new SVTexture);
  texture->mContext = core::Context::Get();
  texture->mDescription = {.source = SVTextureDescription::SourceType::eCUSTOM};
  texture->mImage = image;
  texture->mImageView = std::move(imageView);
  texture->mSampler = sampler;
  texture->mLoaded = true;
  return texture;
}

void SVTexture::uploadToDevice() {
  std::scoped_lock lock(mUploadingMutex);
  if (mOnDevice) {
    return;
  }
  mContext = core::Context::Get();

  if (!mImage->isOnDevice()) {
    mImage->uploadToDevice();
  }
  if (!mImageView) {
    auto format = mImage->getDeviceImage()->getFormat();
    if (mDescription.srgb) {
      if (format == vk::Format::eR8G8B8A8Unorm) {
        format = vk::Format::eR8G8B8A8Srgb;
      } else if (format == vk::Format::eR8Unorm) {
        format = vk::Format::eR8Srgb;
      }
    }

    vk::ImageViewType viewType = vk::ImageViewType::e2D;
    if (mDescription.dim == 1) {
      viewType = vk::ImageViewType::e1D;
    } else if (mDescription.dim == 2) {
      viewType = vk::ImageViewType::e2D;
    } else {
      viewType = vk::ImageViewType::e3D;
    }

    mImageView = mContext->getDevice().createImageViewUnique(
        vk::ImageViewCreateInfo({}, mImage->getDeviceImage()->getVulkanImage(), viewType, format,
                                vk::ComponentSwizzle::eIdentity,
                                vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0,
                                                          mDescription.mipLevels, 0, 1)));
  }
  if (!mSampler) {
    mSampler = mContext->createSampler(vk::SamplerCreateInfo(
        {}, mDescription.magFilter, mDescription.minFilter, vk::SamplerMipmapMode::eLinear,
        mDescription.addressModeU, mDescription.addressModeV, mDescription.addressModeW, 0.f,
        false, 0.f, false, vk::CompareOp::eNever, 0.f, static_cast<float>(mDescription.mipLevels),
        vk::BorderColor::eFloatOpaqueBlack));
  }
  mOnDevice = true;
}

void SVTexture::removeFromDevice() {
  mOnDevice = false;
  mImageView.reset();
}

std::future<void> SVTexture::loadAsync() {
  if (mLoaded) {
    return std::async(std::launch::deferred, []() {});
  }

  auto context = core::Context::Get();
  auto manager = context->getResourceManager();

  logger::info("Loading: {}", mDescription.filename);
  if (mDescription.source != SVTextureDescription::SourceType::eFILE) {
    throw std::runtime_error("failed to load texture: the texture is not specified by a file");
  }
  return std::async(LAUNCH_ASYNC, [this, manager]() {
    std::scoped_lock lock(mLoadingMutex);
    if (mLoaded) {
      return;
    }
    mImage = manager->CreateImageFromFile(mDescription.filename, mDescription.mipLevels,
                                          mDescription.desiredChannels);
    mImage->loadAsync().get();
    mLoaded = true;
    logger::info("Loaded: {}", mDescription.filename);
  });
}

} // namespace resource
} // namespace svulkan2