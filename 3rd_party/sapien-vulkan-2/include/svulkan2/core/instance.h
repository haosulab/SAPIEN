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

class PhysicalDevice;

struct PhysicalDeviceInfo {
  std::string name;
  vk::PhysicalDevice device{};
  bool present{};
  bool supported{};
  int cudaId{-1};
  std::array<uint32_t, 4> pci;
  int queueIndex{-1};
  bool rayTracing{};
  int cudaComputeMode{-1};
  vk::PhysicalDeviceType deviceType{};
  uint32_t subgroupSize{0};

  struct Features {
    bool wideLines{false};
    bool geometryShader{false};
  } features;
};

class Instance : public std::enable_shared_from_this<Instance> {
public:
  // static std::shared_ptr<Instance> Get(uint32_t appVersion, uint32_t engineVersion,
  //                                      uint32_t apiVersion = VK_API_VERSION_1_2);

  Instance(uint32_t appVersion, uint32_t engineVersion, uint32_t apiVersion,
           bool enableVR = false);
  ~Instance();

  inline bool isGLFWEnabled() const { return mGLFWSupported; }
  vk::Instance getInternal() const { return mInstance.get(); }
  inline uint32_t getApiVersion() const { return mApiVersion; }

  auto getProcAddr(char const *name) const { return mInstance->getProcAddr(name); }

  std::vector<PhysicalDeviceInfo> summarizePhysicalDevices() const;
  std::shared_ptr<PhysicalDevice> createPhysicalDevice(std::string const &hint);

  // HACK: shutdown VR should be called before device shutdown
  void shutdownVR() const;

  Instance(Instance const &other) = delete;
  Instance(Instance const &&other) = delete;
  Instance &operator=(Instance const &other) = delete;
  Instance &operator=(Instance const &&other) = delete;

private:
  uint32_t mApiVersion;
  std::unique_ptr<vk::DynamicLoader> mDynamicLoader;
  vk::UniqueInstance mInstance{};
  bool mGLFWSupported{};
  bool mVRSupported{};
  std::vector<PhysicalDeviceInfo> mPhysicalDeviceInfo;

#ifdef VK_VALIDATION
  vk::DebugReportCallbackEXT mDebugCallbackHandle;
#endif
};

} // namespace core
} // namespace svulkan2