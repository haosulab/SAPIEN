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
#include "svulkan2/core/descriptor_pool.h"
#include "svulkan2/core/context.h"
#include "svulkan2/core/device.h"

namespace svulkan2 {
namespace core {

DynamicDescriptorPool::DynamicDescriptorPool(std::vector<vk::DescriptorPoolSize> const &sizes)
    : DynamicDescriptorPool(core::Context::Get()->getDevice2(), sizes) {}

DynamicDescriptorPool::DynamicDescriptorPool(std::shared_ptr<Device> device,
                                             std::vector<vk::DescriptorPoolSize> const &sizes)
    : mDevice(device), mSizes(sizes) {
  expand();
}

void DynamicDescriptorPool::expand() {
  std::vector<vk::DescriptorPoolSize> sizes = mSizes;
  uint32_t count = 0;
  for (auto &size : mSizes) {
    size.setDescriptorCount(size.descriptorCount * (mPools.size() + 1));
    count += size.descriptorCount;
  }
  auto info = vk::DescriptorPoolCreateInfo(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet,
                                           count, sizes.size(), sizes.data());
  mPools.push_back(mDevice->getInternal().createDescriptorPoolUnique(info));
}

vk::UniqueDescriptorSet DynamicDescriptorPool::allocateSet(vk::DescriptorSetLayout const &layout) {
  for (auto &pool : mPools) {
    try {
      return std::move(
          Context::Get()
              ->getDevice()
              .allocateDescriptorSetsUnique(vk::DescriptorSetAllocateInfo(pool.get(), 1, &layout))
              .front());
    } catch (vk::OutOfPoolMemoryError const &) {
    }
  }
  expand();
  return std::move(mDevice->getInternal()
                       .allocateDescriptorSetsUnique(
                           vk::DescriptorSetAllocateInfo(mPools.back().get(), 1, &layout))
                       .front());
}

} // namespace core
} // namespace svulkan2