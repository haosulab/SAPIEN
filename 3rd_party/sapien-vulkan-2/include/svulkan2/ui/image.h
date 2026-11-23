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
#include "widget.h"

namespace svulkan2 {
namespace core {
class CommandPool;
}

namespace ui {

UI_CLASS(DisplayImage) {
  UI_DECLARE_LABEL(DisplayImage);

  // TODO: own core::Image
  std::shared_ptr<DisplayImage> Image(core::Image &);
  std::shared_ptr<DisplayImage> Clear();
  UI_ATTRIBUTE(DisplayImage, glm::vec2, Size);

  void build() override;

  ~DisplayImage();

protected:
  std::unique_ptr<core::CommandPool> mCommandPool;
  vk::UniqueCommandBuffer mCommandBuffer;

  core::Image *mImage{};

  vk::Sampler mSampler;
  vk::UniqueImageView mImageView;

  VkDescriptorSet mDS{};
};

} // namespace ui
} // namespace svulkan2