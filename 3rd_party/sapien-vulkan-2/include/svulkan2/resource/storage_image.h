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
#include <memory>
#include <vector>

namespace svulkan2 {
namespace resource {

class SVStorageImage {
  std::shared_ptr<core::Context> mContext;
  std::string mName;

  vk::Format mFormat;
  uint32_t mWidth{};
  uint32_t mHeight{};

  std::shared_ptr<core::Image> mImage{};
  vk::UniqueImageView mImageView;

  bool mOnDevice{};

public:
  SVStorageImage(std::string const &name, uint32_t width, uint32_t height,
                 vk::Format format);

  void createDeviceResources();
  template <typename T> std::vector<T> download() {
    return mImage->download<T>();
  }

  inline uint32_t getWidth() const { return mWidth; }
  inline uint32_t getHeight() const { return mHeight; }

  inline core::Image &getImage() const { return *mImage; }
  inline vk::ImageView getImageView() const { return mImageView.get(); };
  inline vk::Format getFormat() { return mFormat; };
};

} // namespace resource
} // namespace svulkan2