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
#include "svulkan2/core/physical_device.h"
#include "../common/cuda_helper.h"
#include "../common/logger.h"
#include "svulkan2/core/device.h"
#include <GLFW/glfw3.h>
#include <iomanip>
#include <sstream>

namespace svulkan2 {
namespace core {

PhysicalDevice::PhysicalDevice(std::shared_ptr<Instance> instance,
                               PhysicalDeviceInfo const &deviceInfo)
    : mInstance(instance), mPickedDeviceInfo(deviceInfo) {
  mPickedDeviceLimits = mPickedDeviceInfo.device.getProperties().limits;
}

std::shared_ptr<Device> PhysicalDevice::createDevice() {
  return std::make_shared<Device>(shared_from_this());
}

vk::PhysicalDeviceRayTracingPipelinePropertiesKHR PhysicalDevice::getRayTracingProperties() const {
  if (!mPickedDeviceInfo.rayTracing) {
    throw std::runtime_error("the physical device does not support ray tracing");
  }

  auto properties =
      mPickedDeviceInfo.device.getProperties2<vk::PhysicalDeviceProperties2,
                                              vk::PhysicalDeviceRayTracingPipelinePropertiesKHR>();
  return properties.get<vk::PhysicalDeviceRayTracingPipelinePropertiesKHR>();
}

vk::PhysicalDeviceAccelerationStructurePropertiesKHR PhysicalDevice::getASProperties() const {
  if (!mPickedDeviceInfo.rayTracing) {
    throw std::runtime_error("the physical device does not support ray tracing");
  }
  auto properties = mPickedDeviceInfo.device.getProperties2<
      vk::PhysicalDeviceProperties2, vk::PhysicalDeviceAccelerationStructurePropertiesKHR>();
  return properties.get<vk::PhysicalDeviceAccelerationStructurePropertiesKHR>();
}

uint32_t PhysicalDevice::getMaxWorkGroupInvocations() const {
  return std::min(mPickedDeviceLimits.maxComputeWorkGroupInvocations,
                  mPickedDeviceInfo.subgroupSize * mPickedDeviceInfo.subgroupSize);
}
uint32_t PhysicalDevice::getSubgroupSize() const { return mPickedDeviceInfo.subgroupSize; }

PhysicalDevice::~PhysicalDevice() {}

} // namespace core
} // namespace svulkan2