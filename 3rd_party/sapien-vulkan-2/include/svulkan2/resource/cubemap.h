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
#include "image.h"
#include <memory>
#include <vector>

namespace svulkan2 {
namespace resource {

struct SVCubemapDescription {
  enum class SourceType { eFACES, eKTX, eLATLONG, eCUSTOM } source{SourceType::eCUSTOM};
  std::array<std::string, 6> filenames{};
  uint32_t mipLevels{1};
  vk::Filter magFilter{vk::Filter::eLinear};
  vk::Filter minFilter{vk::Filter::eLinear};
  bool srgb{false};

  inline bool operator==(SVCubemapDescription const &other) const {
    return source == other.source && filenames == other.filenames &&
           mipLevels == other.mipLevels && magFilter == other.magFilter &&
           minFilter == other.minFilter && srgb == other.srgb;
  }
};

class SVCubemap {
  std::shared_ptr<core::Context> mContext; // keep alive for sampler and image view
  SVCubemapDescription mDescription;
  std::shared_ptr<SVImage> mImage;
  std::shared_ptr<SVImage> mLatLongImage;

  bool mOnDevice{};
  vk::UniqueImageView mImageView;
  vk::Sampler mSampler;

  bool mLoaded{};

  std::mutex mUploadingMutex;
  std::mutex mLoadingMutex;

public:
  static std::shared_ptr<SVCubemap> FromFile(std::string const &filename, uint32_t mipLevels,
                                             vk::Filter magFilter = vk::Filter::eLinear,
                                             vk::Filter minFilter = vk::Filter::eLinear,
                                             bool srgb = false);

  static std::shared_ptr<SVCubemap> FromFile(std::array<std::string, 6> const &filenames,
                                             uint32_t mipLevels,
                                             vk::Filter magFilter = vk::Filter::eLinear,
                                             vk::Filter minFilter = vk::Filter::eLinear,
                                             bool srgb = false);

  static std::shared_ptr<SVCubemap>
  FromData(uint32_t size, vk::Format format, std::array<std::vector<char>, 6> const &data,
           uint32_t mipLevels = 1, vk::Filter magFilter = vk::Filter::eLinear,
           vk::Filter minFilter = vk::Filter::eLinear, bool srgb = false);

  static std::shared_ptr<SVCubemap> FromImage(std::shared_ptr<SVImage> image,
                                              vk::UniqueImageView imageView, vk::Sampler sampler);

  void uploadToDevice();
  void removeFromDevice();

  inline bool isOnDevice() const { return mOnDevice && mImage->isOnDevice(); }
  inline bool isLoaded() const { return mLoaded; }
  std::future<void> loadAsync();
  void load();

  inline SVCubemapDescription const &getDescription() const { return mDescription; }

  inline std::shared_ptr<SVImage> getImage() const { return mImage; }
  inline vk::ImageView getImageView() const { return mImageView.get(); }
  inline vk::Sampler getSampler() const { return mSampler; }

  void exportKTX(std::string const &filename);

private:
  SVCubemap() = default;
};

} // namespace resource
} // namespace svulkan2