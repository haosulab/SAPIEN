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
#include "svulkan2/core/image.h"
#include "svulkan2/common/profiler.h"
#include "svulkan2/common/image.h"
#include "svulkan2/core/allocator.h"
#include "svulkan2/core/buffer.h"
#include "svulkan2/core/context.h"

#include <ktxvulkan.h>

#ifdef SVULKAN2_CUDA_INTEROP
#include "../common/cuda_helper.h"
#endif

namespace svulkan2 {
namespace core {

static void assertImageTypeExtent(vk::ImageType type, vk::Extent3D extent) {
  if (extent.depth > 1 && type != vk::ImageType::e3D) {
    throw std::runtime_error("incompatible image type and extent");
  }

  if (extent.height > 1 && type == vk::ImageType::e1D) {
    throw std::runtime_error("incompatible image type and extent");
  }
}

Image::Image(vk::ImageType type, vk::Extent3D extent, vk::Format format,
             vk::ImageUsageFlags usageFlags, VmaMemoryUsage memoryUsage,
             vk::SampleCountFlagBits sampleCount, uint32_t mipLevels, uint32_t arrayLayers,
             vk::ImageTiling tiling, vk::ImageCreateFlags flags)
    : mType(type), mExtent(extent), mFormat(format), mUsageFlags(usageFlags),
      mSampleCount(sampleCount), mMipLevels(mipLevels), mArrayLayers(arrayLayers),
      mTiling(tiling) {
  assertImageTypeExtent(type, extent);
  vk::ImageCreateInfo imageInfo(flags, type, format, extent, mipLevels, arrayLayers, sampleCount,
                                tiling, usageFlags);
  VmaAllocationCreateInfo memoryInfo{};
  memoryInfo.usage = memoryUsage;

  mContext = Context::Get();

  mCurrentLayerLayout.resize(mArrayLayers, vk::ImageLayout::eUndefined);

  if (vmaCreateImage(mContext->getAllocator().getVmaAllocator(),
                     reinterpret_cast<VkImageCreateInfo *>(&imageInfo), &memoryInfo,
                     reinterpret_cast<VkImage *>(&mImage), &mAllocation,
                     &mAllocationInfo) != VK_SUCCESS) {
    throw std::runtime_error("cannot create image");
  }

  VkMemoryPropertyFlags memFlags;
  vmaGetMemoryTypeProperties(mContext->getAllocator().getVmaAllocator(),
                             mAllocationInfo.memoryType, &memFlags);
}

void Image::setCurrentLayout(vk::ImageLayout layout) {
  for (auto &l : mCurrentLayerLayout) {
    l = layout;
  }
}

void Image::setCurrentLayout(uint32_t layer, vk::ImageLayout layout) {
  mCurrentLayerLayout.at(layer) = layout;
}

vk::ImageLayout Image::getCurrentLayout(uint32_t layer) const {
  return mCurrentLayerLayout.at(layer);
}

Image::Image(std::unique_ptr<ktxVulkanTexture> tex) : mKtxTexture(std::move(tex)) {
  mContext = Context::Get();
  mImage = mKtxTexture->image;
  setCurrentLayout(vk::ImageLayout(mKtxTexture->imageLayout));
  mTiling = vk::ImageTiling::eOptimal;
  mMipLevels = mKtxTexture->levelCount;
  mArrayLayers = mKtxTexture->layerCount;
  mSampleCount = vk::SampleCountFlagBits::e1;
  mUsageFlags = vk::ImageUsageFlagBits::eSampled;
  mFormat = vk::Format(mKtxTexture->imageFormat);
  mExtent = vk::Extent3D{mKtxTexture->width, mKtxTexture->height, mKtxTexture->depth};
  mType = vk::ImageType::e2D;
}

Image::~Image() {
#ifdef SVULKAN2_CUDA_INTEROP
  if (mCudaArray) {
    cudaFreeMipmappedArray(mCudaArray);
    cudaDestroyExternalMemory(mCudaMem);
  }
#endif

  if (mKtxTexture) {
    ktxVulkanTexture_Destruct(mKtxTexture.get(), mContext->getDevice(), nullptr);
  } else {
    vmaDestroyImage(mContext->getAllocator().getVmaAllocator(), mImage, mAllocation);
  }
}

void Image::uploadLevel(void const *data, size_t size, uint32_t arrayLayer, uint32_t mipLevel) {
  // TODO: this function requires the image to be in transferdst layout

  auto extent = computeMipLevelExtent(mExtent, mipLevel);
  size_t imageSize = extent.width * extent.height * extent.depth * getFormatSize(mFormat);
  if (size != imageSize) {
    throw std::runtime_error("image upload failed: expecting size " + std::to_string(imageSize) +
                             ", got " + std::to_string(size));
  }
  auto stagingBuffer = Buffer::CreateStaging(size);
  stagingBuffer->upload(data, size);

  vk::BufferImageCopy copyRegion(
      0, extent.width, extent.height,
      vk::ImageSubresourceLayers(getFormatAspectFlags(mFormat), mipLevel, arrayLayer, 1),
      vk::Offset3D(0, 0, 0), extent);

  auto pool = mContext->createCommandPool();
  auto cb = pool->allocateCommandBuffer();
  cb->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});

  cb->copyBufferToImage(stagingBuffer->getVulkanBuffer(), mImage,
                        vk::ImageLayout::eTransferDstOptimal, copyRegion);

  cb->end();
  mContext->getQueue().submitAndWait(cb.get());
}

void Image::upload(void const *data, size_t size, uint32_t arrayLayer, bool mipmaps) {
  size_t imageSize = mExtent.width * mExtent.height * mExtent.depth * getFormatSize(mFormat);
  if (size != imageSize) {
    throw std::runtime_error("image upload failed: expecting size " + std::to_string(imageSize) +
                             ", got " + std::to_string(size));
  }
  auto stagingBuffer = Buffer::CreateStaging(size);
  stagingBuffer->upload(data, size);

  vk::BufferImageCopy copyRegion(
      0, mExtent.width, mExtent.height,
      vk::ImageSubresourceLayers(getFormatAspectFlags(mFormat), 0, arrayLayer, 1),
      vk::Offset3D(0, 0, 0), mExtent);

  auto pool = mContext->createCommandPool();
  auto cb = pool->allocateCommandBuffer();
  cb->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
  transitionLayout(cb.get(), vk::ImageLayout::eUndefined, vk::ImageLayout::eTransferDstOptimal, {},
                   vk::AccessFlagBits::eTransferWrite, vk::PipelineStageFlagBits::eTopOfPipe,
                   vk::PipelineStageFlagBits::eTransfer, arrayLayer);
  cb->copyBufferToImage(stagingBuffer->getVulkanBuffer(), mImage,
                        vk::ImageLayout::eTransferDstOptimal, copyRegion);
  if (mipmaps) {
    generateMipmaps(cb.get(), arrayLayer);
  } else {
    vk::ImageMemoryBarrier barrier;
    barrier.setImage(mImage);
    barrier.setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
    barrier.setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
    barrier.subresourceRange.setAspectMask(vk::ImageAspectFlagBits::eColor);
    barrier.subresourceRange.setBaseArrayLayer(arrayLayer);
    barrier.subresourceRange.setLayerCount(1);
    barrier.subresourceRange.setBaseMipLevel(0);
    barrier.subresourceRange.setLevelCount(1);
    barrier.setOldLayout(vk::ImageLayout::eTransferDstOptimal);
    barrier.setNewLayout(vk::ImageLayout::eShaderReadOnlyOptimal);
    barrier.setSrcAccessMask(vk::AccessFlagBits::eTransferWrite);
    barrier.setDstAccessMask(vk::AccessFlagBits::eShaderRead);
    cb->pipelineBarrier(vk::PipelineStageFlagBits::eTransfer,
                        vk::PipelineStageFlagBits::eFragmentShader, {}, {}, {}, barrier);
  }
  cb->end();
  mContext->getQueue().submitAndWait(cb.get());
}

void Image::generateMipmaps(vk::CommandBuffer cb, uint32_t arrayLayer) {
  vk::ImageMemoryBarrier barrier;
  barrier.setImage(mImage);
  barrier.setSrcQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
  barrier.setDstQueueFamilyIndex(VK_QUEUE_FAMILY_IGNORED);
  barrier.subresourceRange.setAspectMask(vk::ImageAspectFlagBits::eColor);
  barrier.subresourceRange.setBaseArrayLayer(arrayLayer);
  barrier.subresourceRange.setLayerCount(1);
  barrier.subresourceRange.setLevelCount(1);

  int32_t mipWidth = mExtent.width;
  int32_t mipHeight = mExtent.height;
  uint32_t i = 1;
  for (; i < mMipLevels; ++i) {
    // current level to next level
    barrier.subresourceRange.setBaseMipLevel(i - 1);
    barrier.setOldLayout(vk::ImageLayout::eTransferDstOptimal);
    barrier.setNewLayout(vk::ImageLayout::eTransferSrcOptimal);
    barrier.setSrcAccessMask(vk::AccessFlagBits::eTransferWrite);
    barrier.setDstAccessMask(vk::AccessFlagBits::eTransferRead);
    cb.pipelineBarrier(vk::PipelineStageFlagBits::eTransfer, vk::PipelineStageFlagBits::eTransfer,
                       {}, {}, {}, barrier);
    vk::ImageBlit blit(
        {vk::ImageAspectFlagBits::eColor, i - 1, 0, 1},
        {vk::Offset3D{0, 0, 0}, vk::Offset3D{mipWidth, mipHeight, 1}},
        {vk::ImageAspectFlagBits::eColor, i, 0, 1},
        {vk::Offset3D{0, 0, 0},
         vk::Offset3D{mipWidth > 1 ? mipWidth / 2 : 1, mipHeight > 1 ? mipHeight / 2 : 1, 1}});
    cb.blitImage(mImage, vk::ImageLayout::eTransferSrcOptimal, mImage,
                 vk::ImageLayout::eTransferDstOptimal, blit, vk::Filter::eLinear);

    // transition current level to shader read
    barrier.setOldLayout(vk::ImageLayout::eTransferSrcOptimal);
    barrier.setNewLayout(vk::ImageLayout::eShaderReadOnlyOptimal);
    barrier.setSrcAccessMask(vk::AccessFlagBits::eTransferRead);
    barrier.setDstAccessMask(vk::AccessFlagBits::eShaderRead);
    cb.pipelineBarrier(vk::PipelineStageFlagBits::eTransfer,
                       vk::PipelineStageFlagBits::eFragmentShader, {}, {}, {}, barrier);
    if (mipWidth > 1) {
      mipWidth /= 2;
    }
    if (mipHeight > 1) {
      mipHeight /= 2;
    }
  }

  // transition last level to shader read
  barrier.subresourceRange.setBaseMipLevel(i - 1);
  barrier.setOldLayout(vk::ImageLayout::eTransferDstOptimal);
  barrier.setNewLayout(vk::ImageLayout::eShaderReadOnlyOptimal);
  barrier.setSrcAccessMask(vk::AccessFlagBits::eTransferWrite);
  barrier.setDstAccessMask(vk::AccessFlagBits::eShaderRead);
  cb.pipelineBarrier(vk::PipelineStageFlagBits::eTransfer,
                     vk::PipelineStageFlagBits::eFragmentShader, {}, {}, {}, barrier);

  setCurrentLayout(arrayLayer, vk::ImageLayout::eShaderReadOnlyOptimal);
}

void Image::recordCopyToBuffer(vk::CommandBuffer cb, vk::ImageLayout layout, vk::Buffer buffer,
                               size_t bufferOffset, size_t size, vk::Offset3D offset,
                               vk::Extent3D extent, uint32_t arrayLayer) const {
  size_t imageSize = extent.width * extent.height * extent.depth * getFormatSize(mFormat);

  if (size != imageSize) {
    throw std::runtime_error("copy to buffer failed: expecting size " + std::to_string(imageSize) +
                             ", got " + std::to_string(size));
  }

  vk::ImageLayout finalLayout = vk::ImageLayout::eTransferSrcOptimal;
  if (layout == vk::ImageLayout::eGeneral) {
    vk::ImageSubresourceRange imageSubresourceRange(getFormatAspectFlags(mFormat), 0, mMipLevels,
                                                    arrayLayer, 1);
    vk::ImageMemoryBarrier barrier(vk::AccessFlagBits::eMemoryWrite,
                                   vk::AccessFlagBits::eTransferRead, vk::ImageLayout::eGeneral,
                                   vk::ImageLayout::eGeneral, VK_QUEUE_FAMILY_IGNORED,
                                   VK_QUEUE_FAMILY_IGNORED, mImage, imageSubresourceRange);
    cb.pipelineBarrier(vk::PipelineStageFlagBits::eAllCommands,
                       vk::PipelineStageFlagBits::eTransfer, {}, nullptr, nullptr, barrier);
    finalLayout = vk::ImageLayout::eGeneral;
  } else if (layout != vk::ImageLayout::eTransferSrcOptimal) {
    // guess what to wait and transition to TransferSrcOptimal
    vk::ImageLayout sourceLayout = vk::ImageLayout::eUndefined;
    vk::AccessFlags sourceAccessFlag{};
    vk::PipelineStageFlags sourceStage{};

    switch (layout) {
    case vk::ImageLayout::eColorAttachmentOptimal:
      sourceLayout = vk::ImageLayout::eColorAttachmentOptimal;
      sourceAccessFlag = vk::AccessFlagBits::eColorAttachmentWrite;
      sourceStage = vk::PipelineStageFlagBits::eColorAttachmentOutput;
      break;
    case vk::ImageLayout::eDepthStencilAttachmentOptimal:
      sourceLayout = vk::ImageLayout::eDepthStencilAttachmentOptimal;
      sourceAccessFlag = vk::AccessFlagBits::eDepthStencilAttachmentWrite;
      sourceStage = vk::PipelineStageFlagBits::eEarlyFragmentTests |
                    vk::PipelineStageFlagBits::eLateFragmentTests;
      break;
    case vk::ImageLayout::eShaderReadOnlyOptimal:
      sourceLayout = vk::ImageLayout::eShaderReadOnlyOptimal;
      sourceAccessFlag = vk::AccessFlagBits::eShaderRead;
      sourceStage = vk::PipelineStageFlagBits::eFragmentShader;
      break;
    case vk::ImageLayout::eTransferSrcOptimal:
      break;
    default:
      throw std::runtime_error("failed to download image: invalid layout.");
    }

    vk::ImageSubresourceRange imageSubresourceRange(getFormatAspectFlags(mFormat), 0, mMipLevels,
                                                    arrayLayer, 1);
    vk::ImageMemoryBarrier barrier(sourceAccessFlag, vk::AccessFlagBits::eTransferRead,
                                   sourceLayout, vk::ImageLayout::eTransferSrcOptimal,
                                   VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED, mImage,
                                   imageSubresourceRange);
    cb.pipelineBarrier(sourceStage, vk::PipelineStageFlagBits::eTransfer, {}, nullptr, nullptr,
                       barrier);
  }
  vk::ImageAspectFlags aspect = getFormatAspectFlags(mFormat);
  vk::BufferImageCopy copyRegion(bufferOffset, mExtent.width, mExtent.height, {aspect, 0, 0, 1},
                                 offset, extent);
  cb.copyImageToBuffer(mImage, finalLayout, buffer, copyRegion);
}

void Image::recordCopyToBuffer(vk::CommandBuffer cb, vk::Buffer buffer, size_t bufferOffset,
                               size_t size, vk::Offset3D offset, vk::Extent3D extent,
                               uint32_t arrayLayer) {
  size_t imageSize = extent.width * extent.height * extent.depth * getFormatSize(mFormat);

  if (size != imageSize) {
    throw std::runtime_error("copy to buffer failed: expecting size " + std::to_string(imageSize) +
                             ", got " + std::to_string(size));
  }

  vk::ImageLayout layout = getCurrentLayout(arrayLayer);
  if (layout == vk::ImageLayout::eGeneral) {
    // wait for everything in general layout
    transitionLayout(cb, vk::ImageLayout::eGeneral, vk::ImageLayout::eGeneral,
                     vk::AccessFlagBits::eMemoryWrite, vk::AccessFlagBits::eTransferRead,
                     vk::PipelineStageFlagBits::eAllCommands, vk::PipelineStageFlagBits::eTransfer,
                     arrayLayer);
  } else if (layout != vk::ImageLayout::eTransferSrcOptimal) {
    // guess what to wait and transition to TransferSrcOptimal
    vk::ImageLayout sourceLayout = vk::ImageLayout::eUndefined;
    vk::AccessFlags sourceAccessFlag{};
    vk::PipelineStageFlags sourceStage{};

    switch (layout) {
    case vk::ImageLayout::eColorAttachmentOptimal:
      sourceLayout = vk::ImageLayout::eColorAttachmentOptimal;
      sourceAccessFlag = vk::AccessFlagBits::eColorAttachmentWrite;
      sourceStage = vk::PipelineStageFlagBits::eColorAttachmentOutput;
      break;
    case vk::ImageLayout::eDepthStencilAttachmentOptimal:
      sourceLayout = vk::ImageLayout::eDepthStencilAttachmentOptimal;
      sourceAccessFlag = vk::AccessFlagBits::eDepthStencilAttachmentWrite;
      sourceStage = vk::PipelineStageFlagBits::eEarlyFragmentTests |
                    vk::PipelineStageFlagBits::eLateFragmentTests;
      break;
    case vk::ImageLayout::eShaderReadOnlyOptimal:
      sourceLayout = vk::ImageLayout::eShaderReadOnlyOptimal;
      sourceAccessFlag = vk::AccessFlagBits::eShaderRead;
      sourceStage = vk::PipelineStageFlagBits::eFragmentShader;
      break;
    case vk::ImageLayout::eTransferSrcOptimal:
      break;
    default:
      throw std::runtime_error("failed to download image: invalid layout.");
    }
    transitionLayout(cb, sourceLayout, vk::ImageLayout::eTransferSrcOptimal, sourceAccessFlag,
                     vk::AccessFlagBits::eTransferRead, sourceStage,
                     vk::PipelineStageFlagBits::eTransfer, arrayLayer);
  }

  vk::ImageAspectFlags aspect = getFormatAspectFlags(mFormat);

  vk::BufferImageCopy copyRegion(bufferOffset, mExtent.width, mExtent.height, {aspect, 0, 0, 1},
                                 offset, extent);
  cb.copyImageToBuffer(mImage, getCurrentLayout(arrayLayer), buffer, copyRegion);
}

void Image::recordCopyFromBuffer(vk::CommandBuffer cb, vk::Buffer buffer, size_t bufferOffset,
                                 size_t size, vk::Offset3D offset, vk::Extent3D extent,
                                 uint32_t arrayLayer) {
  size_t imageSize = extent.width * extent.height * extent.depth * getFormatSize(mFormat);

  if (size != imageSize) {
    throw std::runtime_error("buffer copy to image failed: expecting size " +
                             std::to_string(imageSize) + ", got " + std::to_string(size));
  }

  vk::ImageLayout layout = getCurrentLayout(arrayLayer);
  if (layout != vk::ImageLayout::eGeneral && layout != vk::ImageLayout::eTransferDstOptimal) {

    vk::ImageLayout sourceLayout = vk::ImageLayout::eUndefined;
    vk::AccessFlags sourceAccessFlag{};
    vk::PipelineStageFlags sourceStage{};

    switch (layout) {
    case vk::ImageLayout::eColorAttachmentOptimal:
      sourceLayout = vk::ImageLayout::eColorAttachmentOptimal;
      sourceAccessFlag = vk::AccessFlagBits::eColorAttachmentWrite;
      sourceStage = vk::PipelineStageFlagBits::eColorAttachmentOutput;
      break;
    case vk::ImageLayout::eDepthStencilAttachmentOptimal:
      sourceLayout = vk::ImageLayout::eDepthStencilAttachmentOptimal;
      sourceAccessFlag = vk::AccessFlagBits::eDepthStencilAttachmentWrite;
      sourceStage = vk::PipelineStageFlagBits::eEarlyFragmentTests |
                    vk::PipelineStageFlagBits::eLateFragmentTests;
      break;
    case vk::ImageLayout::eShaderReadOnlyOptimal:
      sourceLayout = vk::ImageLayout::eShaderReadOnlyOptimal;
      sourceAccessFlag = vk::AccessFlagBits::eShaderRead;
      sourceStage = vk::PipelineStageFlagBits::eFragmentShader;
      break;
    case vk::ImageLayout::eTransferSrcOptimal:
      sourceLayout = vk::ImageLayout::eTransferSrcOptimal;
      sourceAccessFlag = vk::AccessFlagBits::eTransferWrite;
      sourceStage = vk::PipelineStageFlagBits::eTransfer;
      break;
    case vk::ImageLayout::eTransferDstOptimal:
      break;
    default:
      throw std::runtime_error("failed to download image: invalid layout.");
    }
    transitionLayout(cb, sourceLayout, vk::ImageLayout::eTransferDstOptimal, sourceAccessFlag,
                     vk::AccessFlagBits::eTransferWrite, sourceStage,
                     vk::PipelineStageFlagBits::eTransfer, arrayLayer);
  }

  vk::ImageAspectFlags aspect = getFormatAspectFlags(mFormat);

  vk::BufferImageCopy copyRegion(bufferOffset, mExtent.width, mExtent.height, {aspect, 0, 0, 1},
                                 offset, extent);
  cb.copyBufferToImage(buffer, mImage, getCurrentLayout(arrayLayer), copyRegion);
}

void Image::copyToBuffer(vk::Buffer buffer, size_t size, vk::Offset3D offset, vk::Extent3D extent,
                         uint32_t arrayLayer) {
  auto pool = mContext->createCommandPool();
  auto cb = pool->allocateCommandBuffer();
  cb->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
  recordCopyToBuffer(cb.get(), buffer, 0, size, offset, extent, arrayLayer);
  cb->end();
  vk::Result result = mContext->getQueue().submitAndWait(cb.get());
  if (result != vk::Result::eSuccess) {
    throw std::runtime_error("failed to wait for fence");
  }
}

void Image::download(void *data, size_t size, vk::Offset3D offset, vk::Extent3D extent,
                     uint32_t arrayLayer, uint32_t mipLevel) {
  SVULKAN2_PROFILE_FUNCTION;

  size_t imageSize = extent.width * extent.height * extent.depth * getFormatSize(mFormat);

  if (size != imageSize) {
    throw std::runtime_error("image download failed: expecting size " + std::to_string(imageSize) +
                             ", got " + std::to_string(size));
  }

  vk::ImageLayout sourceLayout = vk::ImageLayout::eUndefined;
  vk::AccessFlags sourceAccessFlag;
  vk::PipelineStageFlags sourceStage;

  vk::ImageAspectFlags aspect = getFormatAspectFlags(mFormat);

  SVULKAN2_PROFILE_BLOCK_BEGIN("Record command buffer");
  auto pool = mContext->createCommandPool();
  auto cb = pool->allocateCommandBuffer();
  cb->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});

  vk::ImageLayout layout = getCurrentLayout(arrayLayer);
  if (layout == vk::ImageLayout::eGeneral) {
    // wait for everything in general layout
    transitionLayout(cb.get(), vk::ImageLayout::eGeneral, vk::ImageLayout::eGeneral,
                     vk::AccessFlagBits::eMemoryWrite, vk::AccessFlagBits::eTransferRead,
                     vk::PipelineStageFlagBits::eAllCommands, vk::PipelineStageFlagBits::eTransfer,
                     arrayLayer);
  } else if (layout != vk::ImageLayout::eTransferSrcOptimal) {
    switch (layout) {
    case vk::ImageLayout::eColorAttachmentOptimal:
      sourceLayout = vk::ImageLayout::eColorAttachmentOptimal;
      sourceAccessFlag = vk::AccessFlagBits::eColorAttachmentWrite;
      sourceStage = vk::PipelineStageFlagBits::eColorAttachmentOutput;
      break;
    case vk::ImageLayout::eDepthStencilAttachmentOptimal:
      sourceLayout = vk::ImageLayout::eDepthStencilAttachmentOptimal;
      sourceAccessFlag = vk::AccessFlagBits::eDepthStencilAttachmentWrite;
      sourceStage = vk::PipelineStageFlagBits::eEarlyFragmentTests |
                    vk::PipelineStageFlagBits::eLateFragmentTests;
      break;
    case vk::ImageLayout::eShaderReadOnlyOptimal:
      sourceLayout = vk::ImageLayout::eShaderReadOnlyOptimal;
      sourceAccessFlag = {};
      sourceStage = vk::PipelineStageFlagBits::eFragmentShader;
      break;
    case vk::ImageLayout::eTransferSrcOptimal:
      break;
    default:
      throw std::runtime_error("failed to download image: invalid layout.");
    }

    transitionLayout(cb.get(), sourceLayout, vk::ImageLayout::eTransferSrcOptimal,
                     sourceAccessFlag, vk::AccessFlagBits::eTransferRead, sourceStage,
                     vk::PipelineStageFlagBits::eTransfer, arrayLayer);
  }

  SVULKAN2_PROFILE_BLOCK_BEGIN("Allocating staging buffer");
  auto stagingBuffer = Buffer::CreateStaging(size, true);
  SVULKAN2_PROFILE_BLOCK_END;
  vk::BufferImageCopy copyRegion(0, extent.width, extent.height, {aspect, mipLevel, arrayLayer, 1},
                                 offset, extent);
  cb->copyImageToBuffer(mImage, getCurrentLayout(arrayLayer), stagingBuffer->getVulkanBuffer(),
                        copyRegion);
  cb->end();
  SVULKAN2_PROFILE_BLOCK_END;

  SVULKAN2_PROFILE_BLOCK_BEGIN("Submit and wait");
  vk::Result result = mContext->getQueue().submitAndWait(cb.get());

  if (result != vk::Result::eSuccess) {
    throw std::runtime_error("failed to wait for fence");
  }
  SVULKAN2_PROFILE_BLOCK_END;

  SVULKAN2_PROFILE_BLOCK_BEGIN("Copy data to CPU");
  std::memcpy(data, stagingBuffer->map(), size);
  stagingBuffer->unmap();
  SVULKAN2_PROFILE_BLOCK_END;
}

void Image::download(void *data, size_t size, uint32_t arrayLayer) {
  download(data, size, {0, 0, 0}, mExtent, arrayLayer);
}

void Image::downloadPixel(void *data, size_t pixelSize, vk::Offset3D offset, uint32_t arrayLayer) {
  download(data, pixelSize, offset, {1, 1, 1}, arrayLayer);
}

void Image::transitionLayout(vk::CommandBuffer commandBuffer, vk::ImageLayout oldImageLayout,
                             vk::ImageLayout newImageLayout, vk::AccessFlags sourceAccessMask,
                             vk::AccessFlags destAccessMask, vk::PipelineStageFlags sourceStage,
                             vk::PipelineStageFlags destStage, uint32_t arrayLayer) {
  vk::ImageSubresourceRange imageSubresourceRange(getFormatAspectFlags(mFormat), 0, mMipLevels,
                                                  arrayLayer, 1);
  vk::ImageMemoryBarrier barrier(sourceAccessMask, destAccessMask, oldImageLayout, newImageLayout,
                                 VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED, mImage,
                                 imageSubresourceRange);
  commandBuffer.pipelineBarrier(sourceStage, destStage, {}, nullptr, nullptr, barrier);
  setCurrentLayout(arrayLayer, newImageLayout);
}

void Image::transitionLayout(vk::CommandBuffer commandBuffer, vk::ImageLayout oldImageLayout,
                             vk::ImageLayout newImageLayout, vk::AccessFlags sourceAccessMask,
                             vk::AccessFlags destAccessMask, vk::PipelineStageFlags sourceStage,
                             vk::PipelineStageFlags destStage) {
  // TODO: check the old layout matches

  vk::ImageSubresourceRange imageSubresourceRange(getFormatAspectFlags(mFormat), 0, mMipLevels, 0,
                                                  mArrayLayers);
  vk::ImageMemoryBarrier barrier(sourceAccessMask, destAccessMask, oldImageLayout, newImageLayout,
                                 VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED, mImage,
                                 imageSubresourceRange);
  commandBuffer.pipelineBarrier(sourceStage, destStage, {}, nullptr, nullptr, barrier);
  setCurrentLayout(newImageLayout);
}

#ifdef SVULKAN2_CUDA_INTEROP
vk::DeviceSize Image::getRowPitch() {
  vk::ImageSubresource subresource(vk::ImageAspectFlagBits::eColor, 0, 0);
  vk::SubresourceLayout layout =
      mContext->getDevice().getImageSubresourceLayout(mImage, subresource);
  return layout.rowPitch;
}

cudaMipmappedArray_t Image::getCudaArray() {
  if (mCudaArray) {
    return mCudaArray;
  }
  mCudaDeviceId = getCudaDeviceIdFromPhysicalDevice(mContext->getPhysicalDevice());
  if (mCudaDeviceId < 0) {
    throw std::runtime_error(
        "Vulkan Device is not visible to CUDA. You probably need to unset the "
        "CUDA_VISIBLE_DEVICES variable. Or you can try other "
        "CUDA_VISIBLE_DEVICES until you find a working one.");
  }

  checkCudaErrors(cudaSetDevice(mCudaDeviceId));
  cudaExternalMemoryHandleDesc externalMemoryHandleDesc = {};
  externalMemoryHandleDesc.type = cudaExternalMemoryHandleTypeOpaqueFd;
  externalMemoryHandleDesc.size = mAllocationInfo.offset + mAllocationInfo.size; // TODO check

  vk::MemoryGetFdInfoKHR vkMemoryGetFdInfoKHR;
  vkMemoryGetFdInfoKHR.setPNext(nullptr);
  vkMemoryGetFdInfoKHR.setMemory(mAllocationInfo.deviceMemory);
  vkMemoryGetFdInfoKHR.setHandleType(vk::ExternalMemoryHandleTypeFlagBits::eOpaqueFd);

  auto cudaFd = mContext->getDevice().getMemoryFdKHR(vkMemoryGetFdInfoKHR);
  externalMemoryHandleDesc.handle.fd = cudaFd;

  checkCudaErrors(cudaImportExternalMemory(&mCudaMem, &externalMemoryHandleDesc));

  cudaExternalMemoryMipmappedArrayDesc desc = {};
  desc.extent.width = mExtent.width;
  desc.extent.height = mExtent.height;
  desc.extent.depth = mExtent.depth;
  desc.numLevels = mMipLevels;
  desc.offset = mAllocationInfo.offset;
  desc.flags = 0;
  if (mFormat == vk::Format::eR32G32B32A32Sfloat) {
    desc.formatDesc.x = 32;
    desc.formatDesc.y = 32;
    desc.formatDesc.z = 32;
    desc.formatDesc.w = 32;
    desc.formatDesc.f = cudaChannelFormatKindFloat;
  }

  // TODO: free!!!
  checkCudaErrors(cudaExternalMemoryGetMappedMipmappedArray(&mCudaArray, mCudaMem, &desc));
  return mCudaArray;
}

int Image::getCudaDeviceId() {
  if (!mCudaArray) {
    return getCudaDeviceIdFromPhysicalDevice(mContext->getPhysicalDevice());
  }
  return mCudaDeviceId;
}

#endif

} // namespace core
} // namespace svulkan2