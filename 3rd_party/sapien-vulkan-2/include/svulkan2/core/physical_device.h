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
#include "./instance.h"
#include "svulkan2/common/vk.h"
#include <memory>

namespace svulkan2 {
namespace core {

class Instance;
class Device;

class PhysicalDevice : public std::enable_shared_from_this<PhysicalDevice> {
public:
  PhysicalDevice(std::shared_ptr<Instance> instance, PhysicalDeviceInfo const &deviceInfo);

  inline vk::PhysicalDevice getInternal() const { return mPickedDeviceInfo.device; }
  inline PhysicalDeviceInfo const &getPickedDeviceInfo() const { return mPickedDeviceInfo; }
  inline vk::PhysicalDeviceLimits const &getPickedDeviceLimits() const {
    return mPickedDeviceLimits;
  }

  vk::PhysicalDeviceRayTracingPipelinePropertiesKHR getRayTracingProperties() const;
  vk::PhysicalDeviceAccelerationStructurePropertiesKHR getASProperties() const;

  inline std::shared_ptr<Instance> getInstance() const { return mInstance; }
  std::shared_ptr<Device> createDevice();

  uint32_t getMaxWorkGroupInvocations() const;
  uint32_t getSubgroupSize() const;

  ~PhysicalDevice();

  PhysicalDevice(PhysicalDevice const &other) = delete;
  PhysicalDevice &operator=(PhysicalDevice const &other) = delete;
  PhysicalDevice(PhysicalDevice const &&other) = delete;
  PhysicalDevice &operator=(PhysicalDevice const &&other) = delete;

  // static std::vector<DeviceInfo> summarizeDeviceInfo(Instance const &instance);
  // std::vector<DeviceInfo> summarizeDeviceInfo() const;

private:
  std::shared_ptr<Instance> mInstance;
  PhysicalDeviceInfo mPickedDeviceInfo{};
  vk::PhysicalDeviceLimits mPickedDeviceLimits{};
};

} // namespace core
} // namespace svulkan2