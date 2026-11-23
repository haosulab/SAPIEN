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
#include "allocator.h"
#include "command_pool.h"
#include "queue.h"
#include "svulkan2/common/vk.h"
#include "svulkan2/renderer/gui.h"
#include "svulkan2/resource/manager.h"
#include <future>

namespace svulkan2 {

namespace logger {

void setLogLevel(std::string_view level);
std::string getLogLevel();

}; // namespace logger

namespace core {
#ifdef SAPIEN_MACOS
struct VkSamplerCreateInfoCompare {
  bool operator()(const vk::SamplerCreateInfo& lhs, const vk::SamplerCreateInfo& rhs) const {
    return std::tie(
      lhs.sType, lhs.pNext, lhs.flags, lhs.magFilter, lhs.minFilter, lhs.mipmapMode,
      lhs.addressModeU, lhs.addressModeV, lhs.addressModeW, lhs.mipLodBias, lhs.anisotropyEnable,
      lhs.maxAnisotropy, lhs.compareEnable, lhs.compareOp, lhs.minLod, lhs.maxLod, lhs.borderColor,
      lhs.unnormalizedCoordinates) <
      std::tie(rhs.sType, rhs.pNext, rhs.flags, rhs.magFilter, rhs.minFilter, rhs.mipmapMode,
      rhs.addressModeU, rhs.addressModeV, rhs.addressModeW, rhs.mipLodBias, rhs.anisotropyEnable,
      rhs.maxAnisotropy, rhs.compareEnable, rhs.compareOp, rhs.minLod, rhs.maxLod, rhs.borderColor,
      rhs.unnormalizedCoordinates);
  }
};
#endif
class Instance;
class PhysicalDevice;
class Device;
class Allocator;

class Context : public std::enable_shared_from_this<Context> {
public:
  static std::shared_ptr<Context> Get();
  static std::shared_ptr<Context> Create(uint32_t maxNumMaterials = 5000,
                                         uint32_t maxNumTextures = 5000,
                                         uint32_t defaultMipLevels = 1,
                                         bool doNotLoadTexture = false, std::string device = "",
                                         bool enableVR = false);
  std::shared_ptr<resource::SVResourceManager> createResourceManager();

  ~Context();
  inline bool isVulkanAvailable() const { return mInstance && mPhysicalDevice; }

  bool isPresentAvailable() const;
  bool isRayTracingAvailable() const;

  Queue &getQueue() const;
  Allocator &getAllocator();

  std::unique_ptr<CommandPool> createCommandPool() const;

  uint32_t getGraphicsQueueFamilyIndex() const;

  vk::Instance getInstance() const;
  vk::Device getDevice() const;
  vk::PhysicalDevice getPhysicalDevice() const;

  std::shared_ptr<Device> getDevice2() const { return mDevice; };
  std::shared_ptr<Instance> getInstance2() const { return mInstance; }
  std::shared_ptr<PhysicalDevice> getPhysicalDevice2() const { return mPhysicalDevice; }

  vk::PhysicalDeviceLimits const &getPhysicalDeviceLimits() const;

  inline std::mutex &getGlobalLock() { return mGlobalLock; }

  inline DynamicDescriptorPool &getDescriptorPool() const { return *mDescriptorPool; }

  inline vk::DescriptorSetLayout getMetallicDescriptorSetLayout() const {
    return mMetallicDescriptorSetLayout.get();
  }

  std::shared_ptr<resource::SVResourceManager> getResourceManager() const;

  std::unique_ptr<renderer::GuiWindow> createWindow(uint32_t width, uint32_t height);

  std::shared_ptr<resource::SVMesh> createTriangleMesh(std::vector<glm::vec3> const &vertices,
                                                       std::vector<uint32_t> const &indices,
                                                       std::vector<glm::vec3> const &normals = {},
                                                       std::vector<glm::vec2> const &uvs = {});

  inline bool shouldNotLoadTexture() const { return mDoNotLoadTexture; }

  vk::UniqueSemaphore createTimelineSemaphore(uint64_t initialValue);

  vk::Sampler createSampler(vk::SamplerCreateInfo const &info);

private:
  std::shared_ptr<Instance> mInstance;
  std::shared_ptr<PhysicalDevice> mPhysicalDevice;
  std::shared_ptr<Device> mDevice;

  std::mutex mGlobalLock{};
  std::unique_ptr<DynamicDescriptorPool> mDescriptorPool;

  uint32_t mMaxNumMaterials;
  uint32_t mMaxNumTextures;
  uint32_t mDefaultMipLevels;
  bool mDoNotLoadTexture;

  std::weak_ptr<resource::SVResourceManager> mResourceManager;

  vk::UniqueDescriptorSetLayout mMetallicDescriptorSetLayout;

  Context(uint32_t maxNumMaterials = 5000, uint32_t maxNumTextures = 5000,
          uint32_t defaultMipLevels = 1, bool doNotLoadTexture = false, std::string device = "",
          bool enableVR = false);

  void createDescriptorPool();

  std::mutex mSamplerLock{};
#ifdef SAPIEN_MACOS
  std::map<vk::SamplerCreateInfo, vk::UniqueSampler, VkSamplerCreateInfoCompare> mSamplerRegistry;
#else
  std::map<vk::SamplerCreateInfo, vk::UniqueSampler> mSamplerRegistry;
#endif
};

} // namespace core
} // namespace svulkan2