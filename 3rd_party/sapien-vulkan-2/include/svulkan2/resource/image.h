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
#pragma once
#include "svulkan2/core/image.h"
#include <future>

namespace svulkan2 {
namespace resource {

struct SVImageDescription {
  enum class SourceType { eFILE, eCUSTOM, eDEVICE } source{SourceType::eCUSTOM};
  vk::Format format{vk::Format::eUndefined};
  std::vector<std::string> filenames{};
  uint32_t desiredChannels{0};
  uint32_t mipLevels{1};

  inline bool operator==(SVImageDescription const &other) const {
    return source == other.source && filenames == other.filenames &&
           mipLevels == other.mipLevels && format == other.format;
  }
};

class SVImage {

  SVImageDescription mDescription{};
  std::unique_ptr<core::Image> mImage{};

  vk::ImageType mType{vk::ImageType::e2D};
  vk::Format mFormat{vk::Format::eUndefined};
  uint32_t mWidth{1};
  uint32_t mHeight{1};
  uint32_t mDepth{1};
  vk::ImageCreateFlags mCreateFlags{};
  vk::ImageUsageFlags mUsage{vk::ImageUsageFlagBits::eSampled |
                             vk::ImageUsageFlagBits::eTransferDst |
                             vk::ImageUsageFlagBits::eTransferSrc};

  std::vector<std::vector<char>> mRawData;

  /** the image is on the host */
  bool mLoaded{};
  bool mOnDevice{};

  /** mipmap levels are provided on host */
  bool mMipLoaded{false};

  std::mutex mLoadingMutex;
  std::mutex mUploadingMutex;

public:
  static std::shared_ptr<SVImage> FromRawData(vk::ImageType type, uint32_t width, uint32_t height,
                                              uint32_t depth, vk::Format format,
                                              std::vector<std::vector<char>> const &data,
                                              uint32_t mipLevels = 1);

  static std::shared_ptr<SVImage> FromFile(std::vector<std::string> const &filenames,
                                           uint32_t mipLevels, uint32_t desiredChannels);

  static std::shared_ptr<SVImage> FromDeviceImage(std::unique_ptr<core::Image> image);

  void setUsage(vk::ImageUsageFlags usage);
  void setCreateFlags(vk::ImageCreateFlags flags);

  void uploadToDevice(bool generateMipmaps = true);
  void removeFromDevice();
  inline bool isLoaded() const { return mLoaded; }
  inline bool isOnDevice() const { return mOnDevice; }

  /** load the image to host memory */
  std::future<void> loadAsync();

  inline core::Image *getDeviceImage() const { return mImage.get(); }

  inline SVImageDescription const &getDescription() const { return mDescription; }

  std::vector<std::vector<char>> const &getRawData() const { return mRawData; }

  inline uint32_t getWidth() const { return mWidth; }
  inline uint32_t getHeight() const { return mHeight; }
  inline uint32_t getDepth() const { return mDepth; }
  inline uint32_t getChannels() const { return getFormatChannels(mFormat); }
  inline vk::Format getFormat() const { return mFormat; }

  /** Indicate that the image loads mipmap into the data and does not require
   * generation */
  bool mipmapIsLoaded() const { return mMipLoaded; }

private:
  SVImage() = default;
};

} // namespace resource
} // namespace svulkan2