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
#include "svulkan2/common/vk.h"
#include <memory>

namespace svulkan2 {
namespace core {

class Device;

class DynamicDescriptorPool {
  std::shared_ptr<Device> mDevice;
  std::vector<vk::DescriptorPoolSize> mSizes;
  std::vector<vk::UniqueDescriptorPool> mPools;

public:
  // using Context singleton
  DynamicDescriptorPool(std::vector<vk::DescriptorPoolSize> const &sizes);

  DynamicDescriptorPool(std::shared_ptr<Device> device,
                        std::vector<vk::DescriptorPoolSize> const &sizes);
  DynamicDescriptorPool(DynamicDescriptorPool &other) = delete;
  DynamicDescriptorPool(DynamicDescriptorPool &&other) = default;
  DynamicDescriptorPool &operator=(DynamicDescriptorPool &other) = delete;
  DynamicDescriptorPool &operator=(DynamicDescriptorPool &&other) = default;

  vk::UniqueDescriptorSet allocateSet(vk::DescriptorSetLayout const &layout);

private:
  void expand();
};

} // namespace core
} // namespace svulkan2