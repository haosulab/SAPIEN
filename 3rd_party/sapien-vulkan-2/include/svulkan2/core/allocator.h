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

class Allocator {
public:
  Allocator(Device &device);
  Allocator(Allocator &other) = delete;
  Allocator(Allocator &&other) = delete;
  Allocator &operator=(Allocator &other) = delete;
  Allocator &operator=(Allocator &&other) = delete;

  VmaAllocator getVmaAllocator() const { return mMemoryAllocator; }
  VmaPool getExternalPool() const { return mExternalMemoryPool; };
  VmaPool getRTPool() const {
    if (mRTPool) {
      return mRTPool;
    }
    throw std::runtime_error("this physical device does not support ray tracing");
  };

  ~Allocator();

private:
  VmaAllocator mMemoryAllocator;
  VmaPool mExternalMemoryPool{};
  vk::ExportMemoryAllocateInfo mExternalAllocInfo;

  // AS & SBT in ray tracing requires special alignment
  // we allocate them from this pool
  VmaPool mRTPool{};
};

} // namespace core
} // namespace svulkan2