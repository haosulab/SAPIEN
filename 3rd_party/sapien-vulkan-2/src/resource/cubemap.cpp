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
#include "svulkan2/resource/cubemap.h"
#include "../common/logger.h"
#include "svulkan2/common/launch_policy.h"
#include "svulkan2/core/context.h"
#include "svulkan2/resource/manager.h"
#include "svulkan2/shader/compute.h"
#include <ktx.h>

namespace svulkan2 {
namespace resource {

std::shared_ptr<SVCubemap> SVCubemap::FromFile(std::string const &filename, uint32_t mipLevels,
                                               vk::Filter magFilter, vk::Filter minFilter,
                                               bool srgb) {
  auto texture = std::shared_ptr<SVCubemap>(new SVCubemap);
  auto source = (filename.ends_with(".ktx") || filename.ends_with(".KTX"))
                    ? SVCubemapDescription::SourceType::eKTX
                    : SVCubemapDescription::SourceType::eLATLONG;
  texture->mDescription = {.source = source,
                           .filenames = {filename, "", "", "", "", ""},
                           .mipLevels = mipLevels,
                           .magFilter = magFilter,
                           .minFilter = minFilter,
                           .srgb = srgb};
  return texture;
}

std::shared_ptr<SVCubemap> SVCubemap::FromFile(std::array<std::string, 6> const &filenames,
                                               uint32_t mipLevels, vk::Filter magFilter,
                                               vk::Filter minFilter, bool srgb) {
  auto texture = std::shared_ptr<SVCubemap>(new SVCubemap);
  texture->mDescription = {.source = SVCubemapDescription::SourceType::eFACES,
                           .filenames = filenames,
                           .mipLevels = mipLevels,
                           .magFilter = magFilter,
                           .minFilter = minFilter,
                           .srgb = srgb};
  return texture;
}

std::shared_ptr<SVCubemap> SVCubemap::FromData(uint32_t size, vk::Format format,
                                               std::array<std::vector<char>, 6> const &data,
                                               uint32_t mipLevels, vk::Filter magFilter,
                                               vk::Filter minFilter, bool srgb) {
  auto texture = std::shared_ptr<SVCubemap>(new SVCubemap);
  texture->mDescription = {.source = SVCubemapDescription::SourceType::eCUSTOM,
                           .filenames = {},
                           .mipLevels = mipLevels,
                           .magFilter = magFilter,
                           .minFilter = minFilter,
                           .srgb = srgb};
  std::vector<std::vector<char>> vdata(data.begin(), data.end());
  texture->mImage =
      SVImage::FromRawData(vk::ImageType::e2D, size, size, 1, format, vdata, mipLevels);
  texture->mImage->setCreateFlags(vk::ImageCreateFlagBits::eCubeCompatible);
  texture->mLoaded = true;
  return texture;
}

std::shared_ptr<SVCubemap> SVCubemap::FromImage(std::shared_ptr<SVImage> image,
                                                vk::UniqueImageView imageView,
                                                vk::Sampler sampler) {
  auto texture = std::shared_ptr<SVCubemap>(new SVCubemap);
  texture->mContext = core::Context::Get();
  texture->mDescription = {.source = SVCubemapDescription::SourceType::eCUSTOM};
  texture->mImage = image;
  texture->mImageView = std::move(imageView);
  texture->mSampler = sampler;
  texture->mLoaded = true;

  if (image->isOnDevice()) {
    texture->mOnDevice = true;
  }

  return texture;
}

void SVCubemap::uploadToDevice() {
  std::scoped_lock lock(mUploadingMutex);
  mContext = core::Context::Get();
  if (mOnDevice) {
    return;
  }

  if (!mImage && !mLatLongImage) {
    throw std::runtime_error("failed to upload cubemap: not loaded");
  }

  // latlong
  if (!mImage) {
    mLatLongImage->uploadToDevice();
    mImage = resource::SVImage::FromDeviceImage(
        shader::latlongToCube(*mLatLongImage->getDeviceImage(), mDescription.mipLevels));
  }

  if (!mImage->isOnDevice()) {
    mImage->setUsage(vk::ImageUsageFlagBits::eSampled | vk::ImageUsageFlagBits::eTransferDst |
                     vk::ImageUsageFlagBits::eTransferSrc | vk::ImageUsageFlagBits::eStorage);
    mImage->uploadToDevice(false);
  }

  auto viewFormat = mImage->getDeviceImage()->getFormat();
  if (mDescription.srgb) {
    switch (viewFormat) {
    case vk::Format::eR8G8B8A8Unorm:
      viewFormat = vk::Format::eR8G8B8A8Srgb;
      break;
    case vk::Format::eR8G8B8Unorm:
      viewFormat = vk::Format::eR8G8B8Srgb;
      break;
    case vk::Format::eR8G8Unorm:
      viewFormat = vk::Format::eR8G8Srgb;
      break;
    case vk::Format::eR8Unorm:
      viewFormat = vk::Format::eR8Srgb;
      break;
    default:
      break;
    }
  }

  vk::ImageViewUsageCreateInfo usageInfo(vk::ImageUsageFlagBits::eSampled);
  vk::ImageViewCreateInfo viewInfo(
      {}, mImage->getDeviceImage()->getVulkanImage(), vk::ImageViewType::eCube, viewFormat,
      vk::ComponentSwizzle::eIdentity,
      vk::ImageSubresourceRange(vk::ImageAspectFlagBits::eColor, 0, mDescription.mipLevels, 0, 6));
  viewInfo.setPNext(&usageInfo);

  mImageView = mContext->getDevice().createImageViewUnique(viewInfo);

  mSampler = mContext->createSampler(vk::SamplerCreateInfo(
      {}, mDescription.magFilter, mDescription.minFilter, vk::SamplerMipmapMode::eLinear,
      vk::SamplerAddressMode::eRepeat, vk::SamplerAddressMode::eRepeat,
      vk::SamplerAddressMode::eRepeat, 0.f, false, 0.f, false, vk::CompareOp::eNever, 0.f,
      static_cast<float>(mDescription.mipLevels), vk::BorderColor::eFloatOpaqueBlack));

  if (mDescription.mipLevels > 1 && !mImage->mipmapIsLoaded()) {
    if (mImage->mipmapIsLoaded()) {
      logger::info("Cube map mipmaps are loaded, no prefiltering required.");
    } else {
      logger::info("Prefiltering cube map...");
      shader::prefilterCubemap(*mImage->getDeviceImage());
      logger::info("Prefiltering cube map completed");
    }
  }
  mImage->getDeviceImage()->setCurrentLayout(vk::ImageLayout::eShaderReadOnlyOptimal);
  mOnDevice = true;
}

void SVCubemap::removeFromDevice() {
  mOnDevice = false;
  mImageView.reset();
}

std::future<void> SVCubemap::loadAsync() {
  if (mLoaded) {
    return std::async(std::launch::deferred, []() {});
  }
  switch (mDescription.source) {
  case SVCubemapDescription::SourceType::eFACES:
    for (auto f : mDescription.filenames) {
      logger::info("Loading: {}", f);
    }
    break;
  case (SVCubemapDescription::SourceType::eKTX):
  case (SVCubemapDescription::SourceType::eLATLONG):
    logger::info("Loading: {}", mDescription.filenames[0]);
    break;
  default:
    throw std::runtime_error("failed to load cubemap: not specified by a file");
  }

  return std::async(LAUNCH_ASYNC, [this]() {
    std::scoped_lock lock(mLoadingMutex);
    if (mLoaded) {
      return;
    }
    switch (mDescription.source) {
    case SVCubemapDescription::SourceType::eFACES: {
      auto vfiles =
          std::vector<std::string>(mDescription.filenames.begin(), mDescription.filenames.end());
      mImage = SVImage::FromFile(vfiles, mDescription.mipLevels, 4);
      mImage->setCreateFlags(vk::ImageCreateFlagBits::eCubeCompatible);
      mImage->loadAsync().get();
      break;
    }
    case SVCubemapDescription::SourceType::eKTX: {
      mImage = SVImage::FromFile({mDescription.filenames[0]}, mDescription.mipLevels, 4);
      mImage->setCreateFlags(vk::ImageCreateFlagBits::eCubeCompatible);
      mImage->loadAsync().get();
      break;
    }
    case SVCubemapDescription::SourceType::eLATLONG: {
      mLatLongImage = SVImage::FromFile({mDescription.filenames[0]}, 1, 4);
      mLatLongImage->loadAsync().get();
      break;
    }
    default:
      throw std::runtime_error("failed to load cubemap: not specified by a file.");
    };

    mLoaded = true;
  });
}

void SVCubemap::load() { loadAsync().get(); }

void SVCubemap::exportKTX(std::string const &filename) {
  if (!mOnDevice) {
    throw std::runtime_error("failed to export KTX, uploadToDevice should be called first.");
  }

  auto img = mImage->getDeviceImage();
  auto extent = img->getExtent();

  if (img->getFormat() != vk::Format::eR8G8B8A8Unorm) {
    throw std::runtime_error("exporting to ktx only supports R8G8B8A8Unorm texture for now");
  }

  ktxTexture2 *texture;
  ktxTextureCreateInfo createInfo;

  createInfo.vkFormat = VK_FORMAT_R8G8B8A8_UNORM;
  createInfo.baseWidth = extent.width;
  createInfo.baseHeight = extent.height;
  createInfo.baseDepth = 1;
  createInfo.numDimensions = 2;
  createInfo.numLevels = img->getMipLevels();
  createInfo.numLayers = 1;
  createInfo.numFaces = 6;
  createInfo.isArray = KTX_FALSE;
  createInfo.generateMipmaps = KTX_FALSE;

  ktxTexture2_Create(&createInfo, KTX_TEXTURE_CREATE_ALLOC_STORAGE, &texture);
  std::vector<uint8_t> data(extent.width * extent.height * 4);

  for (uint32_t face = 0; face < 6; ++face) {
    uint32_t width = extent.width;
    uint32_t height = extent.height;
    for (uint32_t level = 0; level < img->getMipLevels(); ++level) {
      uint32_t size = width * height * 4;
      img->download(data.data(), size, {0, 0, 0}, {width, height, 1}, face, level);
      ktxTexture_SetImageFromMemory(ktxTexture(texture), level, 0, face, data.data(), size);
      width /= 2;
      height /= 2;
      if (width == 0) {
        width = 1;
      }
      if (height == 0) {
        height = 1;
      }
    }
  }
  ktxTexture_WriteToNamedFile(ktxTexture(texture), filename.c_str());
  ktxTexture_Destroy(ktxTexture(texture));

  auto context = core::Context::Get();
  auto pool = context->createCommandPool();
  auto buffer = pool->allocateCommandBuffer();
  buffer->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
  img->transitionLayout(buffer.get(), vk::ImageLayout::eTransferSrcOptimal,
                        vk::ImageLayout::eShaderReadOnlyOptimal, vk::AccessFlagBits::eTransferRead,
                        vk::AccessFlagBits::eShaderRead, vk::PipelineStageFlagBits::eTransfer,
                        vk::PipelineStageFlagBits::eFragmentShader);
  buffer->end();
  context->getQueue().submitAndWait(buffer.get());
}

} // namespace resource
} // namespace svulkan2