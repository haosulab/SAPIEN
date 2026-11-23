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
#include "svulkan2/ui/image.h"
#include "svulkan2/core/context.h"
#include <imgui.h>
#include <imgui_impl_vulkan.h>

namespace svulkan2 {
namespace ui {

std::shared_ptr<DisplayImage> DisplayImage::Image(core::Image &image) {
  if (mImage == &image) {
    return std::static_pointer_cast<DisplayImage>(shared_from_this());
  }

  core::Context::Get()->getDevice().waitIdle();
  if (mDS) {
    ImGui_ImplVulkan_RemoveTexture(mDS);
    mDS = {};
  }

  mImage = &image;

  if (!mCommandPool) {
    mCommandPool = core::Context::Get()->createCommandPool();
    mCommandBuffer = mCommandPool->allocateCommandBuffer();
  }

  auto context = core::Context::Get();
  mSampler = context->createSampler(vk::SamplerCreateInfo(
      {}, vk::Filter::eNearest, vk::Filter::eNearest, vk::SamplerMipmapMode::eNearest,
      vk::SamplerAddressMode::eClampToBorder, vk::SamplerAddressMode::eClampToBorder,
      vk::SamplerAddressMode::eClampToBorder, 0.f, false, 0.f, false, vk::CompareOp::eNever, 0.f,
      0.f, vk::BorderColor::eFloatOpaqueWhite));

  auto format = image.getFormat();
  vk::ComponentMapping mapping(vk::ComponentSwizzle::eR, vk::ComponentSwizzle::eG,
                               vk::ComponentSwizzle::eB, vk::ComponentSwizzle::eA);
  if (getFormatChannels(format) == 1) {
    mapping = vk::ComponentMapping(vk::ComponentSwizzle::eR, vk::ComponentSwizzle::eR,
                                   vk::ComponentSwizzle::eR, vk::ComponentSwizzle::eR);
  }

  mImageView = context->getDevice().createImageViewUnique(vk::ImageViewCreateInfo(
      {}, image.getVulkanImage(), vk::ImageViewType::e2D, image.getFormat(), mapping,
      vk::ImageSubresourceRange(getFormatAspectFlags(format), 0, 1, 0, 1)));

  mDS = ImGui_ImplVulkan_AddTexture(
      mSampler, mImageView.get(),
      // HACK: assume general layout image will always use general layout
      image.getCurrentLayout(0) == vk::ImageLayout::eGeneral
          ? VK_IMAGE_LAYOUT_GENERAL
          : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

  return std::static_pointer_cast<DisplayImage>(shared_from_this());
}

std::shared_ptr<DisplayImage> DisplayImage::Clear() {
  core::Context::Get()->getDevice().waitIdle();
  if (mDS) {
    ImGui_ImplVulkan_RemoveTexture(mDS);
    mDS = {};
  }

  mImageView = {};
  mSampler = vk::Sampler{};
  mImage = {};

  return std::static_pointer_cast<DisplayImage>(shared_from_this());
}

void DisplayImage::build() {
  if (mImage) {
    mCommandBuffer->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
    if (mImage->getCurrentLayout(0) == vk::ImageLayout::eGeneral) {
      mImage->transitionLayout(
          mCommandBuffer.get(), vk::ImageLayout::eGeneral, vk::ImageLayout::eGeneral,
          vk::AccessFlagBits::eMemoryWrite, vk::AccessFlagBits::eShaderRead,
          vk::PipelineStageFlagBits::eAllCommands, vk::PipelineStageFlagBits::eFragmentShader);
    } else {
      mImage->transitionLayout(mCommandBuffer.get(), mImage->getCurrentLayout(0),
                               vk::ImageLayout::eShaderReadOnlyOptimal,
                               vk::AccessFlagBits::eMemoryWrite, vk::AccessFlagBits::eShaderRead,
                               vk::PipelineStageFlagBits::eAllCommands,
                               vk::PipelineStageFlagBits::eFragmentShader);
    }

    mCommandBuffer->end();

    // TODO: async
    core::Context::Get()->getQueue().submitAndWait(mCommandBuffer.get());
    mCommandBuffer->reset();

    if (mSize[0] <= 0 && mSize[1] <= 0) {
      mSize[0] = ImGui::GetWindowContentRegionWidth();
      mSize[1] = mSize[0] / mImage->getExtent().width * mImage->getExtent().height;
    }

    ImGui::Image((ImTextureID)mDS, ImVec2(mSize[0], mSize[1]));
  }
}

DisplayImage::~DisplayImage() {
  core::Context::Get()->getDevice().waitIdle();
  mCommandBuffer.reset();
  if (mDS) {
    ImGui_ImplVulkan_RemoveTexture(mDS);
    mDS = {};
  }
}

} // namespace ui
} // namespace svulkan2