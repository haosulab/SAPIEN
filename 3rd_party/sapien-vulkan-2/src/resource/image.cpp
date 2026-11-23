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
#include "svulkan2/resource/image.h"
#include "../common/logger.h"
#include "svulkan2/common/image.h"
#include "svulkan2/common/launch_policy.h"
#include "svulkan2/core/context.h"
#include <filesystem>

namespace fs = std::filesystem;
namespace svulkan2 {
namespace resource {

std::shared_ptr<SVImage> SVImage::FromRawData(vk::ImageType type, uint32_t width, uint32_t height,
                                              uint32_t depth, vk::Format format,
                                              std::vector<std::vector<char>> const &data,
                                              uint32_t mipLevels) {
  size_t imageSize = width * height * depth;
  for (auto &d : data) {
    if (imageSize * getFormatSize(format) != d.size()) {
      throw std::runtime_error(
          "failed to create image: image data size does not match image dimensions");
    }
  }
  auto image = std::shared_ptr<SVImage>(new SVImage);
  image->mDescription = {.source = SVImageDescription::SourceType::eCUSTOM,
                         .format = format,
                         .filenames = {},
                         .mipLevels = mipLevels};

  image->mType = type;
  image->mFormat = format;
  image->mWidth = width;
  image->mHeight = height;
  image->mDepth = depth;
  image->mRawData = data;
  image->mLoaded = true;
  return image;
}

std::shared_ptr<SVImage> SVImage::FromFile(std::vector<std::string> const &filenames,
                                           uint32_t mipLevels, uint32_t desiredChannels) {
  auto image = std::shared_ptr<SVImage>(new SVImage);
  image->mDescription = SVImageDescription{.source = SVImageDescription::SourceType::eFILE,
                                           .filenames = filenames,
                                           .desiredChannels = desiredChannels,
                                           .mipLevels = mipLevels};
  return image;
}

std::shared_ptr<SVImage> SVImage::FromDeviceImage(std::unique_ptr<core::Image> image) {
  auto result = std::shared_ptr<SVImage>(new SVImage);
  result->mDescription = SVImageDescription{.source = SVImageDescription::SourceType::eDEVICE};
  result->mType = image->getType();
  result->mFormat = image->getFormat();
  result->mWidth = image->getExtent().width;
  result->mHeight = image->getExtent().height;
  result->mDepth = image->getExtent().depth;
  result->mImage = std::move(image);
  result->mLoaded = true;
  result->mOnDevice = true;
  return result;
}

void SVImage::setUsage(vk::ImageUsageFlags usage) {
  if (mOnDevice) {
    throw std::runtime_error("failed to set usage: device image already created");
  }
  mUsage = usage;
}

void SVImage::setCreateFlags(vk::ImageCreateFlags flags) {
  if (mOnDevice) {
    throw std::runtime_error("failed to set create flags: device image already created");
  }
  mCreateFlags = flags;
}

void SVImage::uploadToDevice(bool generateMipmaps) {
  std::scoped_lock lock(mUploadingMutex);
  if (mOnDevice) {
    return;
  }
  auto context = core::Context::Get();
  if (!mLoaded) {
    throw std::runtime_error("failed to upload to device: image does not exist in memory");
  }

  if (mFormat == vk::Format::eUndefined) {
    throw std::runtime_error("failed to upload to device: image format is not determined");
  }

  mImage = std::make_unique<core::Image>(mType, vk::Extent3D{mWidth, mHeight, mDepth}, mFormat,
                                         mUsage, VmaMemoryUsage::VMA_MEMORY_USAGE_GPU_ONLY,
                                         vk::SampleCountFlagBits::e1, mDescription.mipLevels,
                                         mRawData.size(), vk::ImageTiling::eOptimal,
                                         mCreateFlags | vk::ImageCreateFlagBits::eMutableFormat);

  if (mMipLoaded) {
    // upload all levels if mipmap is provided
    auto pool = context->createCommandPool();
    auto cb = pool->allocateCommandBuffer();
    cb->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
    mImage->transitionLayout(
        cb.get(), vk::ImageLayout::eUndefined, vk::ImageLayout::eTransferDstOptimal, {},
        vk::AccessFlagBits::eTransferWrite, vk::PipelineStageFlagBits::eTopOfPipe,
        vk::PipelineStageFlagBits::eTransfer);
    cb->end();
    context->getQueue().submitAndWait(cb.get());

    for (uint32_t layer = 0; layer < mRawData.size(); ++layer) {
      uint32_t idx = 0;
      for (uint32_t l = 0; l < mDescription.mipLevels; ++l) {
        auto size = computeMipLevelSize({mWidth, mHeight, mDepth}, l) * getFormatSize(mFormat);
        mImage->uploadLevel(mRawData[layer].data() + idx, size, layer, l);
        idx += size;
      }
    }

    cb = pool->allocateCommandBuffer();
    cb->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});

    vk::PipelineStageFlags dstStage = vk::PipelineStageFlagBits::eFragmentShader;
    if (core::Context::Get()->isRayTracingAvailable()) {
      dstStage |= vk::PipelineStageFlagBits::eRayTracingShaderKHR;
    }
    mImage->transitionLayout(cb.get(), vk::ImageLayout::eTransferDstOptimal,
                             vk::ImageLayout::eShaderReadOnlyOptimal,
                             vk::AccessFlagBits::eTransferWrite, vk::AccessFlagBits::eShaderRead,
                             vk::PipelineStageFlagBits::eTransfer, dstStage);
    cb->end();
    context->getQueue().submitAndWait(cb.get());
  } else {
    for (uint32_t layer = 0; layer < mRawData.size(); ++layer) {
      mImage->upload(mRawData[layer].data(), mRawData[layer].size(), layer, generateMipmaps);
    }
  }
  mOnDevice = true;
}

void SVImage::removeFromDevice() {
  mOnDevice = false;
  mImage.reset();
}

std::future<void> SVImage::loadAsync() {
  if (mLoaded) {
    return std::async(std::launch::deferred, []() {});
  }
  return std::async(LAUNCH_ASYNC, [this]() {
    std::scoped_lock lock(mLoadingMutex);
    if (mLoaded) {
      return;
    }
    if (mDescription.source != SVImageDescription::SourceType::eFILE) {
      throw std::runtime_error("failed to load image: the image is not specified with a file");
    }

    mWidth = 0;
    mHeight = 0;
    // mChannels = 4;

    // load ktx file
    if (mDescription.filenames.size() && mDescription.filenames[0].ends_with(".ktx")) {
      int width, height, levels, faces, layers;
      vk::Format format;
      auto data =
          loadKTXImage(mDescription.filenames[0], width, height, levels, faces, layers, format);
      if (format != vk::Format::eR8G8B8A8Unorm) {
        throw std::runtime_error("Only R8G8B8A8Unorm format is currently supported for ktx");
      }

      mFormat = format;
      mWidth = width;
      mHeight = height;

      size_t stride = 0;
      for (uint32_t l = 0; l < static_cast<uint32_t>(levels); ++l) {
        stride += width * height;
        width /= 2;
        height /= 2;
        if (width == 0) {
          width = 1;
        }
        if (height == 0) {
          height = 1;
        }
      }
      stride *= getFormatSize(format);

      if (mDescription.mipLevels != static_cast<uint32_t>(levels)) {
        // levels do not match, only take base level
        logger::warn("ktx texture has {} levels but {} levels requested", levels,
                     mDescription.mipLevels);
        for (uint32_t i = 0; i < static_cast<uint32_t>(faces * layers); ++i) {
          std::vector<uint8_t> rawData(data.begin() + i * stride,
                                       data.begin() +
                                           (i * stride + width * height * getFormatSize(format)));
          mRawData.push_back(toRawBytes(rawData));
        }
      } else {
        for (uint32_t i = 0; i < static_cast<uint32_t>(faces * layers); ++i) {
          std::vector<uint8_t> rawData(data.begin() + i * stride,
                                       data.begin() + ((i + 1) * stride));
          mRawData.push_back(toRawBytes(rawData));
        }
        mMipLoaded = true;
      }
      mLoaded = true;
    } else {
      for (uint32_t i = 0; i < mDescription.filenames.size(); ++i) {
        int width, height, channels;

        vk::Format format{vk::Format::eUndefined};
        if (mDescription.filenames[i].ends_with(".exr")) {
          // exr
          auto rawData = loadExrImage(mDescription.filenames[i], width, height);
          format = vk::Format::eR16G16B16A16Sfloat;
          channels = 4;

          mRawData.push_back(toRawBytes(rawData));
        } else {
          // png, jpg
          std::vector<uint8_t> rawData = loadImage(mDescription.filenames[i], width, height,
                                                   channels, mDescription.desiredChannels);

          switch (channels) {
          case 1:
            format = vk::Format::eR8Unorm;
            break;
          case 2:
            format = vk::Format::eR8G8Unorm;
            break;
          case 3:
            format = vk::Format::eR8G8B8Unorm;
            break;
          case 4:
            format = vk::Format::eR8G8B8A8Unorm;
            break;
          default:
            throw std::runtime_error("invalid image channels");
          }

          mRawData.push_back(toRawBytes(rawData));
        }

        if ((mWidth != 0 && mWidth != static_cast<uint32_t>(width)) ||
            (mHeight != 0 && mHeight != static_cast<uint32_t>(height)) ||
            (mFormat != vk::Format::eUndefined && mFormat != format)) {
          throw std::runtime_error(
              "image load failed: provided files have different sizes or formats");
        }

        mFormat = format;
        mWidth = width;
        mHeight = height;
      }
      mLoaded = true;
    }
  });
}

} // namespace resource
} // namespace svulkan2