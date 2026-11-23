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
#include "svulkan2/core/allocator.h"
#include "svulkan2/core/device.h"
#include "svulkan2/core/instance.h"
#include "svulkan2/core/physical_device.h"
#include <bit>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wparentheses"
#pragma GCC diagnostic ignored "-Wunused-variable"
#define VMA_BUFFER_DEVICE_ADDRESS 1
#define VMA_EXTERNAL_MEMORY 1
#define VMA_STATIC_VULKAN_FUNCTIONS 0
#define VMA_IMPLEMENTATION
#include "svulkan2/common/vk_mem_alloc.h"
#pragma GCC diagnostic pop

namespace svulkan2 {
namespace core {

Allocator::Allocator(Device &device) {
  VmaAllocatorCreateInfo allocatorInfo = {};

  auto physicalDevice = device.getPhysicalDevice();
  auto instance = physicalDevice->getInstance();

  allocatorInfo.vulkanApiVersion = instance->getApiVersion();
  allocatorInfo.instance = instance->getInternal();
  allocatorInfo.physicalDevice = physicalDevice->getInternal();
  allocatorInfo.device = device.getInternal();

  VmaVulkanFunctions vulkanFunctions{};
  vulkanFunctions.vkGetInstanceProcAddr = VULKAN_HPP_DEFAULT_DISPATCHER.vkGetInstanceProcAddr;
  vulkanFunctions.vkGetDeviceProcAddr = VULKAN_HPP_DEFAULT_DISPATCHER.vkGetDeviceProcAddr;
  vulkanFunctions.vkGetPhysicalDeviceProperties =
      VULKAN_HPP_DEFAULT_DISPATCHER.vkGetPhysicalDeviceProperties;
  vulkanFunctions.vkGetPhysicalDeviceMemoryProperties =
      VULKAN_HPP_DEFAULT_DISPATCHER.vkGetPhysicalDeviceMemoryProperties;
  vulkanFunctions.vkGetPhysicalDeviceMemoryProperties2KHR =
      VULKAN_HPP_DEFAULT_DISPATCHER.vkGetPhysicalDeviceMemoryProperties2KHR;
  vulkanFunctions.vkAllocateMemory = VULKAN_HPP_DEFAULT_DISPATCHER.vkAllocateMemory;
  vulkanFunctions.vkFreeMemory = VULKAN_HPP_DEFAULT_DISPATCHER.vkFreeMemory;
  vulkanFunctions.vkMapMemory = VULKAN_HPP_DEFAULT_DISPATCHER.vkMapMemory;
  vulkanFunctions.vkUnmapMemory = VULKAN_HPP_DEFAULT_DISPATCHER.vkUnmapMemory;
  vulkanFunctions.vkFlushMappedMemoryRanges =
      VULKAN_HPP_DEFAULT_DISPATCHER.vkFlushMappedMemoryRanges;
  vulkanFunctions.vkInvalidateMappedMemoryRanges =
      VULKAN_HPP_DEFAULT_DISPATCHER.vkInvalidateMappedMemoryRanges;
  vulkanFunctions.vkBindBufferMemory = VULKAN_HPP_DEFAULT_DISPATCHER.vkBindBufferMemory;
  vulkanFunctions.vkBindImageMemory = VULKAN_HPP_DEFAULT_DISPATCHER.vkBindImageMemory;
  vulkanFunctions.vkGetBufferMemoryRequirements =
      VULKAN_HPP_DEFAULT_DISPATCHER.vkGetBufferMemoryRequirements;
  vulkanFunctions.vkGetImageMemoryRequirements =
      VULKAN_HPP_DEFAULT_DISPATCHER.vkGetImageMemoryRequirements;
  vulkanFunctions.vkCreateBuffer = VULKAN_HPP_DEFAULT_DISPATCHER.vkCreateBuffer;
  vulkanFunctions.vkDestroyBuffer = VULKAN_HPP_DEFAULT_DISPATCHER.vkDestroyBuffer;
  vulkanFunctions.vkCreateImage = VULKAN_HPP_DEFAULT_DISPATCHER.vkCreateImage;
  vulkanFunctions.vkDestroyImage = VULKAN_HPP_DEFAULT_DISPATCHER.vkDestroyImage;
  vulkanFunctions.vkCmdCopyBuffer = VULKAN_HPP_DEFAULT_DISPATCHER.vkCmdCopyBuffer;
  vulkanFunctions.vkGetBufferMemoryRequirements2KHR =
      VULKAN_HPP_DEFAULT_DISPATCHER.vkGetBufferMemoryRequirements2KHR;
  vulkanFunctions.vkGetImageMemoryRequirements2KHR =
      VULKAN_HPP_DEFAULT_DISPATCHER.vkGetImageMemoryRequirements2KHR;
  vulkanFunctions.vkBindBufferMemory2KHR = VULKAN_HPP_DEFAULT_DISPATCHER.vkBindBufferMemory2KHR;
  vulkanFunctions.vkBindImageMemory2KHR = VULKAN_HPP_DEFAULT_DISPATCHER.vkBindImageMemory2KHR;
  allocatorInfo.pVulkanFunctions = &vulkanFunctions;

  if (physicalDevice->getPickedDeviceInfo().rayTracing) {
    allocatorInfo.flags = VMA_ALLOCATOR_CREATE_BUFFER_DEVICE_ADDRESS_BIT;
  }

  if (vmaCreateAllocator(&allocatorInfo, &mMemoryAllocator) != VK_SUCCESS) {
    throw std::runtime_error("failed to create VmaAllocator");
  }

  // find GPU memory suitable for external buffer
  // VkPhysicalDeviceMemoryProperties properties;
  vk::PhysicalDeviceMemoryProperties properties;
  physicalDevice->getInternal().getMemoryProperties(&properties);

  vk::BufferCreateInfo bufferInfo({}, 64,
                                  vk::BufferUsageFlagBits::eVertexBuffer |
                                      vk::BufferUsageFlagBits::eIndexBuffer |
                                      vk::BufferUsageFlagBits::eTransferDst);
#if !defined(VK_USE_PLATFORM_MACOS_MVK)
  vk::ExternalMemoryBufferCreateInfo externalMemoryBufferInfo(
      vk::ExternalMemoryHandleTypeFlagBits::eOpaqueFd);
  bufferInfo.setPNext(&externalMemoryBufferInfo);
#endif

  vk::MemoryRequirements memReq;
  {
    auto buffer = device.getInternal().createBufferUnique(bufferInfo);
    memReq = device.getInternal().getBufferMemoryRequirements(buffer.get());
  }

  int index = -1;
  for (uint32_t i = 0; i < properties.memoryTypeCount; ++i) {
    if ((memReq.memoryTypeBits & (1 << i)) &&
        (properties.memoryTypes[i].propertyFlags & vk::MemoryPropertyFlagBits::eDeviceLocal)) {
      index = i;
      break;
    }
  }
  if (index == -1) {
    throw std::runtime_error("Failed to find a suitable memory type for external memory pool");
  }

  {
    VmaPoolCreateInfo poolInfo{};
    poolInfo.memoryTypeIndex = static_cast<uint32_t>(index);
    // auto limits = physicalDevice->getInternal().getProperties().limits;
    // poolInfo.minAllocationAlignment = std::max(
    //     std::max(limits.minStorageBufferOffsetAlignment, limits.minTexelBufferOffsetAlignment),
    //     limits.minUniformBufferOffsetAlignment);
#if !defined(VK_USE_PLATFORM_MACOS_MVK)
    mExternalAllocInfo =
        vk::ExportMemoryAllocateInfo(vk::ExternalMemoryHandleTypeFlagBits::eOpaqueFd);
    poolInfo.pMemoryAllocateNext = &mExternalAllocInfo;
#endif
    vmaCreatePool(mMemoryAllocator, &poolInfo, &mExternalMemoryPool);
  }

  if (physicalDevice->getPickedDeviceInfo().rayTracing) {
    auto rtprops = physicalDevice->getRayTracingProperties();
    vk::DeviceSize sbtAlign = rtprops.shaderGroupBaseAlignment;
    auto asprops = physicalDevice->getASProperties();
    vk::DeviceSize asAlign = asprops.minAccelerationStructureScratchOffsetAlignment;
    vk::DeviceSize alignment = std::bit_ceil(std::max(sbtAlign, asAlign));

    VmaPoolCreateInfo poolInfo{};
    poolInfo.memoryTypeIndex = static_cast<uint32_t>(index);
    poolInfo.minAllocationAlignment = alignment;
    vmaCreatePool(mMemoryAllocator, &poolInfo, &mRTPool);
  }
}

Allocator::~Allocator() {
  if (mRTPool) {
    vmaDestroyPool(mMemoryAllocator, mRTPool);
  }
  if (mExternalMemoryPool) {
    vmaDestroyPool(mMemoryAllocator, mExternalMemoryPool);
  }
  vmaDestroyAllocator(mMemoryAllocator);
}

} // namespace core
} // namespace svulkan2