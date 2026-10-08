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
#include "svulkan2/core/device.h"
#include "../common/logger.h"
#include "svulkan2/core/allocator.h"
#include "svulkan2/core/instance.h"
#include "svulkan2/core/physical_device.h"
#include "svulkan2/core/queue.h"
#include <openvr.h>

namespace svulkan2 {
namespace core {

// https://github.com/ValveSoftware/openvr/blob/f51d87ecf8f7903e859b0aa4d617ff1e5f33db5a/samples/hellovr_vulkan/hellovr_vulkan_main.cpp#L719
static bool GetVRDeviceExtensionsRequired(VkPhysicalDevice pPhysicalDevice,
                                          std::vector<std::string> &outDeviceExtensionList) {
  if (!vr::VRCompositor()) {
    return false;
  }

  outDeviceExtensionList.clear();
  uint32_t nBufferSize = vr::VRCompositor()->GetVulkanDeviceExtensionsRequired(
      (VkPhysicalDevice_T *)pPhysicalDevice, nullptr, 0);
  if (nBufferSize > 0) {
    // Allocate memory for the space separated list and query for it
    char *pExtensionStr = new char[nBufferSize];
    pExtensionStr[0] = 0;
    vr::VRCompositor()->GetVulkanDeviceExtensionsRequired((VkPhysicalDevice_T *)pPhysicalDevice,
                                                          pExtensionStr, nBufferSize);

    // Break up the space separated list into entries on the CUtlStringList
    std::string curExtStr;
    uint32_t nIndex = 0;
    while (pExtensionStr[nIndex] != 0 && (nIndex < nBufferSize)) {
      if (pExtensionStr[nIndex] == ' ') {
        outDeviceExtensionList.push_back(curExtStr);
        curExtStr.clear();
      } else {
        curExtStr += pExtensionStr[nIndex];
      }
      nIndex++;
    }
    if (curExtStr.size() > 0) {
      outDeviceExtensionList.push_back(curExtStr);
    }

    delete[] pExtensionStr;
  }

  return true;
}

Device::Device(std::shared_ptr<PhysicalDevice> physicalDevice) : mPhysicalDevice(physicalDevice) {
  if (!physicalDevice) {
    throw std::runtime_error("failed to create device: invalid physical device");
  }

  float queuePriority = 0.0f;
  int queueIndex = mPhysicalDevice->getPickedDeviceInfo().queueIndex;
  vk::DeviceQueueCreateInfo deviceQueueCreateInfo({}, queueIndex, 1, &queuePriority);
  std::vector<const char *> deviceExtensions{};

  vk::PhysicalDeviceFeatures2 features{};
  vk::PhysicalDeviceDescriptorIndexingFeatures descriptorFeatures{};
  vk::PhysicalDeviceTimelineSemaphoreFeatures timelineSemaphoreFeatures{};

  features.setPNext(&descriptorFeatures);
  descriptorFeatures.setPNext(&timelineSemaphoreFeatures);

  features.features.setIndependentBlend(true);
  features.features.setWideLines(physicalDevice->getPickedDeviceInfo().features.wideLines);
  features.features.setGeometryShader(
      physicalDevice->getPickedDeviceInfo().features.geometryShader);
  descriptorFeatures.setDescriptorBindingPartiallyBound(true);
  timelineSemaphoreFeatures.setTimelineSemaphore(true);
  descriptorFeatures.setPNext(&timelineSemaphoreFeatures);

  vk::PhysicalDeviceExtendedDynamicStateFeaturesEXT extendedDynamicStateFeature;
  extendedDynamicStateFeature.setExtendedDynamicState(true);
  timelineSemaphoreFeatures.setPNext(&extendedDynamicStateFeature);

  vk::PhysicalDeviceAccelerationStructureFeaturesKHR asFeature;
  asFeature.setAccelerationStructure(true);
  vk::PhysicalDeviceRayTracingPipelineFeaturesKHR rtFeature;
  rtFeature.setRayTracingPipeline(true);
  vk::PhysicalDeviceBufferDeviceAddressFeaturesKHR addrFeature;
  addrFeature.setBufferDeviceAddress(true);
  vk::PhysicalDeviceShaderClockFeaturesKHR clockFeature;
  clockFeature.setShaderDeviceClock(true);
  clockFeature.setShaderSubgroupClock(true);

#ifdef VK_VALIDATION
  deviceExtensions.push_back(VK_KHR_SHADER_NON_SEMANTIC_INFO_EXTENSION_NAME);
#endif

  if (mPhysicalDevice->getPickedDeviceInfo().rayTracing) {
    deviceExtensions.push_back(VK_KHR_RAY_TRACING_PIPELINE_EXTENSION_NAME);
    deviceExtensions.push_back(VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME);
    deviceExtensions.push_back(VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME);
    deviceExtensions.push_back(VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME);
    deviceExtensions.push_back(VK_KHR_SHADER_CLOCK_EXTENSION_NAME);
    descriptorFeatures.setRuntimeDescriptorArray(true);
    descriptorFeatures.setShaderStorageBufferArrayNonUniformIndexing(true);
    descriptorFeatures.setShaderSampledImageArrayNonUniformIndexing(true);
    extendedDynamicStateFeature.setPNext(&asFeature);
    asFeature.setPNext(&rtFeature);
    rtFeature.setPNext(&addrFeature);
    addrFeature.setPNext(&clockFeature);
    features.features.setShaderInt64(true);
  }

#ifdef VK_USE_PLATFORM_MACOS_MVK
  deviceExtensions.push_back(VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME);
#endif

  deviceExtensions.push_back(VK_EXT_EXTENDED_DYNAMIC_STATE_EXTENSION_NAME);

  if (mPhysicalDevice->getPickedDeviceInfo().cudaId >= 0) {
#ifdef SVULKAN2_CUDA_INTEROP
    deviceExtensions.push_back(VK_KHR_EXTERNAL_MEMORY_EXTENSION_NAME);
    deviceExtensions.push_back(VK_KHR_EXTERNAL_SEMAPHORE_EXTENSION_NAME);
#ifndef _WIN64
    deviceExtensions.push_back(VK_KHR_EXTERNAL_MEMORY_FD_EXTENSION_NAME);
    deviceExtensions.push_back(VK_KHR_EXTERNAL_SEMAPHORE_FD_EXTENSION_NAME);
#endif
#endif
  }

  // try to enable VR
  bool vrEnabled{false};
  std::vector<std::string> vrExtensions;
  if (GetVRDeviceExtensionsRequired(mPhysicalDevice->getInternal(), vrExtensions)) {
    for (auto const &e : vrExtensions) {
      deviceExtensions.push_back(e.c_str());
    }
    vrEnabled = true;
    logger::info("VR enabled");
  }

  if ((mPhysicalDevice->getPickedDeviceInfo().present &&
       mPhysicalDevice->getInstance()->isGLFWEnabled()) ||
      vrEnabled) {
    deviceExtensions.push_back(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
  }

  vk::DeviceCreateInfo deviceInfo({}, deviceQueueCreateInfo, {}, deviceExtensions);
  deviceInfo.setPNext(&features);
  mDevice = mPhysicalDevice->getPickedDeviceInfo().device.createDeviceUnique(deviceInfo);
  VULKAN_HPP_DEFAULT_DISPATCHER.init(mDevice.get());

  mQueue = std::make_unique<Queue>(*this, queueIndex);

  auto instance = mPhysicalDevice->getInstance();
  mAllocator = std::make_unique<Allocator>(*this);
}

uint32_t Device::getGraphicsQueueFamilyIndex() const {
  return mPhysicalDevice->getPickedDeviceInfo().queueIndex;
}

std::unique_ptr<CommandPool> Device::createCommandPool() {
  return std::make_unique<CommandPool>(shared_from_this());
}

Device::~Device() { mDevice->waitIdle(); }

} // namespace core
} // namespace svulkan2