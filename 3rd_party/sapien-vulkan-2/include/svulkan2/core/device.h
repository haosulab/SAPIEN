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
#include "command_pool.h"
#include "svulkan2/common/vk.h"
#include <memory>

namespace svulkan2 {
namespace core {
class Instance;
class PhysicalDevice;
class Queue;
class Allocator;

class Device : public std::enable_shared_from_this<Device> {
public:
  Device(std::shared_ptr<PhysicalDevice> physicalDevice);
  inline std::shared_ptr<PhysicalDevice> getPhysicalDevice() const { return mPhysicalDevice; }
  inline vk::Device getInternal() const { return mDevice.get(); }
  inline Queue &getQueue() const { return *mQueue; }
  inline Allocator &getAllocator() const { return *mAllocator; }

  uint32_t getGraphicsQueueFamilyIndex() const;

  std::unique_ptr<CommandPool> createCommandPool();

  ~Device();

  Device(Device const &other) = delete;
  Device &operator=(Device const &other) = delete;
  Device(Device const &&other) = delete;
  Device &operator=(Device const &&other) = delete;

private:
  std::shared_ptr<PhysicalDevice> mPhysicalDevice;
  vk::UniqueDevice mDevice;
  std::unique_ptr<Queue> mQueue;
  std::unique_ptr<Allocator> mAllocator;
};

} // namespace core
} // namespace svulkan2