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
#include "svulkan2/core/buffer.h"
#include "../common/logger.h"
#include "svulkan2/core/allocator.h"
#include "svulkan2/core/context.h"
#include "svulkan2/core/device.h"
#include "svulkan2/core/physical_device.h"
#include "svulkan2/common/profiler.h"

#ifdef SVULKAN2_CUDA_INTEROP
#include "../common/cuda_helper.h"
#endif

namespace svulkan2 {
namespace core {

std::unique_ptr<Buffer> Buffer::CreateStaging(vk::DeviceSize size, bool readback) {
  if (readback) {
    return std::make_unique<Buffer>(Context::Get()->getDevice2(), size,
                                    vk::BufferUsageFlagBits::eTransferSrc |
                                        vk::BufferUsageFlagBits::eTransferDst,
                                    VMA_MEMORY_USAGE_GPU_TO_CPU);
  }
  return std::make_unique<Buffer>(Context::Get()->getDevice2(), size,
                                  vk::BufferUsageFlagBits::eTransferSrc |
                                      vk::BufferUsageFlagBits::eTransferDst,
                                  VMA_MEMORY_USAGE_CPU_ONLY);
}

std::unique_ptr<Buffer> Buffer::CreateUniform(vk::DeviceSize size, bool deviceOnly,
                                              bool external) {
  if (deviceOnly) {
    return std::make_unique<Buffer>(
        Context::Get()->getDevice2(), size,
        vk::BufferUsageFlagBits::eUniformBuffer | vk::BufferUsageFlagBits::eTransferDst,
        VMA_MEMORY_USAGE_GPU_ONLY, VmaAllocationCreateFlags{}, external);
  } else {
    return std::make_unique<Buffer>(
        Context::Get()->getDevice2(), size,
        vk::BufferUsageFlagBits::eUniformBuffer | vk::BufferUsageFlagBits::eTransferDst,
        VMA_MEMORY_USAGE_CPU_TO_GPU, VmaAllocationCreateFlags{}, external);
  }
}

std::unique_ptr<Buffer> Buffer::Create(vk::DeviceSize size, vk::BufferUsageFlags usageFlags,
                                       VmaMemoryUsage memoryUsage,
                                       VmaAllocationCreateFlags allocationFlags, bool external,
                                       VmaPool pool) {
  return std::make_unique<Buffer>(Context::Get()->getDevice2(), size, usageFlags, memoryUsage,
                                  allocationFlags, external, pool);
}

Buffer::Buffer(std::shared_ptr<Device> device, vk::DeviceSize size,
               vk::BufferUsageFlags usageFlags, VmaMemoryUsage memoryUsage,
               VmaAllocationCreateFlags allocationFlags, bool external, VmaPool pool)
    : mDevice(device), mSize(size), mExternal(external) {
  if (!device) {
    throw std::runtime_error("failed to create buffer: invalid device");
  }
  if (external && pool) {
    throw std::runtime_error(
        "failed to create buffer: external buffer should not specify memory pool");
  }
  if (size == 0) {
    throw std::runtime_error("failed to create buffer: buffer size must not be 0.");
  }

  vk::BufferCreateInfo bufferInfo({}, size, usageFlags);
#if !defined(VK_USE_PLATFORM_MACOS_MVK)
  vk::ExternalMemoryBufferCreateInfo externalMemoryBufferInfo(
      vk::ExternalMemoryHandleTypeFlagBits::eOpaqueFd);
#endif

  VmaAllocationCreateInfo memoryInfo{};
  memoryInfo.usage = memoryUsage;
  memoryInfo.flags = allocationFlags;

  if (external) {
#if !defined(VK_USE_PLATFORM_MACOS_MVK)
    bufferInfo.setPNext(&externalMemoryBufferInfo);
#endif
    if (memoryUsage != VMA_MEMORY_USAGE_GPU_ONLY) {
      throw std::runtime_error("external buffer can only be device local");
    }
    memoryInfo.pool = mDevice->getAllocator().getExternalPool();
  } else if (pool) {
    memoryInfo.pool = pool;
  }

  if (vmaCreateBuffer(mDevice->getAllocator().getVmaAllocator(),
                      reinterpret_cast<VkBufferCreateInfo *>(&bufferInfo), &memoryInfo,
                      reinterpret_cast<VkBuffer *>(&mBuffer), &mAllocation,
                      &mAllocationInfo) != VK_SUCCESS) {
    throw std::runtime_error("cannot create buffer");
  }

  VkMemoryPropertyFlags memFlags;
  vmaGetMemoryTypeProperties(mDevice->getAllocator().getVmaAllocator(), mAllocationInfo.memoryType,
                             &memFlags);
  mHostVisible = (memFlags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) != 0;
  mHostCoherent = (memFlags & VK_MEMORY_PROPERTY_HOST_COHERENT_BIT) != 0;
}

Buffer::~Buffer() {
#ifdef SVULKAN2_CUDA_INTEROP
  if (mCudaPtr) {
    checkCudaErrors(cudaFree(mCudaPtr));
    checkCudaErrors(cudaDestroyExternalMemory(mCudaMem));
  }
#endif
  vmaDestroyBuffer(mDevice->getAllocator().getVmaAllocator(), mBuffer, mAllocation);
}

void *Buffer::map() {
  if (!mMapped) {
    auto result =
        vmaMapMemory(mDevice->getAllocator().getVmaAllocator(), mAllocation, &mMappedData);
    if (result != VK_SUCCESS) {
      throw std::runtime_error("failed to map buffer memory");
    }
    mMapped = true;
  }
  return mMappedData;
}

void Buffer::unmap() {
  if (mMapped) {
    vmaUnmapMemory(mDevice->getAllocator().getVmaAllocator(), mAllocation);
    mMapped = false;
  }
}

void Buffer::flush() {
  vmaFlushAllocation(mDevice->getAllocator().getVmaAllocator(), mAllocation, 0, mSize);
}

void Buffer::upload(void const *data, size_t size, size_t offset) {
  if (size == 0) {
    return;
  }

  if (offset + size > mSize) {
    throw std::runtime_error("failed to upload buffer: upload size exceeds buffer size");
  }

  if (mHostVisible) {
    if (mMapped) {
      std::memcpy(reinterpret_cast<uint8_t *>(mMappedData) + offset, data, size);
    } else {
      map();
      std::memcpy(reinterpret_cast<uint8_t *>(mMappedData) + offset, data, size);
      unmap();
    }
    if (!mHostCoherent) {
      flush();
    }
  } else {
    auto stagingBuffer = Buffer::CreateStaging(size);
    stagingBuffer->upload(data, size);

    // TODO: find a better way
    auto pool = mDevice->createCommandPool();
    auto cb = pool->allocateCommandBuffer();
    cb->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
    cb->copyBuffer(stagingBuffer->mBuffer, mBuffer, vk::BufferCopy(0, offset, size));
    cb->end();
    mDevice->getQueue().submitAndWait(cb.get());
  }
}

void Buffer::download(void *data, size_t size, size_t offset) {
  if (size == 0) {
    return;
  }

  if (offset + size > mSize) {
    throw std::runtime_error("failed to download buffer: download size exceeds buffer size");
  }

  if (mHostVisible) {
    map();
    std::memcpy(data, reinterpret_cast<uint8_t *>(mMappedData) + offset, size);
    unmap();
  } else {
    auto stagingBuffer = Buffer::CreateStaging(size);

    auto pool = mDevice->createCommandPool();
    auto cb = pool->allocateCommandBuffer();
    cb->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
    cb->copyBuffer(mBuffer, stagingBuffer->mBuffer, vk::BufferCopy(offset, 0, size));
    cb->end();
    mDevice->getQueue().submitAndWait(cb.get());
    stagingBuffer->download(data, size);
  }
}

vk::DeviceAddress Buffer::getAddress() const {
  return mDevice->getInternal().getBufferAddress({mBuffer});
}

#ifdef SVULKAN2_CUDA_INTEROP
void *Buffer::getCudaPtr() {
  // FIXME: technically we need to transfer ownership to VK_QUEUE_FAMILY_EXTERNAL
  // Current design is undefined behavior, but all NVIDIA's sample code do this
  if (!mExternal) {
    throw std::runtime_error("failed to get cuda pointer, \"external\" must be "
                             "passed at buffer creation");
  }

  if (mCudaPtr) {
    return mCudaPtr;
  }
  mCudaDeviceId = getCudaDeviceIdFromPhysicalDevice(mDevice->getPhysicalDevice()->getInternal());
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

  // According to https://docs.nvidia.com/pdf/CUDA_Runtime_API.pdf
  // Performing any operations on the file descriptor after it is imported
  // results in undefined behavior
  auto cudaFd = mDevice->getInternal().getMemoryFdKHR(vkMemoryGetFdInfoKHR);
  externalMemoryHandleDesc.handle.fd = cudaFd;

  checkCudaErrors(cudaImportExternalMemory(&mCudaMem, &externalMemoryHandleDesc));

  cudaExternalMemoryBufferDesc externalMemBufferDesc = {};
  externalMemBufferDesc.offset = mAllocationInfo.offset;
  externalMemBufferDesc.size = mAllocationInfo.size;
  externalMemBufferDesc.flags = 0;

  checkCudaErrors(cudaExternalMemoryGetMappedBuffer(&mCudaPtr, mCudaMem, &externalMemBufferDesc));
  return mCudaPtr;
}

int Buffer::getCudaDeviceId() {
  if (!mCudaPtr) {
    return getCudaDeviceIdFromPhysicalDevice(mDevice->getPhysicalDevice()->getInternal());
  }
  return mCudaDeviceId;
}
#endif

} // namespace core
} // namespace svulkan2