#pragma once
#include "common.h"
#include "safe_map.h"
#include "sapien/thread_pool.hpp"
#include <memory>
#include <shared_mutex>
#include <svulkan2/core/context.h>
#include <svulkan2/renderer/renderer.h>
#include <svulkan2/resource/manager.h>
#include <svulkan2/resource/material.h>
#include <svulkan2/scene/scene.h>
#include <unordered_map>

namespace grpc {
class Server;
}

namespace sapien {
namespace Renderer {
namespace server {

class RenderServiceImpl;

extern std::string gDefaultShaderDirectory;
void setDefaultShaderDirectory(std::string const &dir);

class VulkanCudaBuffer {
public:
  VulkanCudaBuffer(vk::Device device, vk::PhysicalDevice physicalDevice, std::string const &type,
                   std::vector<int> const &shape);
  ~VulkanCudaBuffer();

  inline std::vector<int> const &getShape() const { return mShape; }
  inline std::string getType() const { return mType; }
  inline vk::Buffer getBuffer() const { return mBuffer.get(); }

  vk::DeviceSize getSize() const { return mSize; }

#ifdef SAPIEN_CUDA
  inline void *getCudaPtr() const { return mCudaPtr; }
#endif

private:
  uint32_t findMemoryType(uint32_t typeFilter, vk::MemoryPropertyFlags properties);

  vk::Device mDevice;
  vk::PhysicalDevice mPhysicalDevice;
  std::string mType{};
  std::vector<int> mShape;
  vk::DeviceSize mSize;

  vk::UniqueBuffer mBuffer;
  vk::UniqueDeviceMemory mMemory;

#ifdef SAPIEN_CUDA
  int mCudaDeviceId{-1};
  void *mCudaPtr{};
  cudaExternalMemory_t mCudaMem{};
#endif
};

class RenderServer {
public:
  RenderServer(uint32_t maxNumMaterials, uint32_t maxNumTextures, uint32_t defaultMipLevels,
               std::string const &device, bool doNotLoadTexture);

  void start(std::string const &address);
  void stop();

  // attempt to allocate buffers based on current scenes and cameras
  // NOTE: it must be not be called concurrently with child processes running!
  std::vector<VulkanCudaBuffer *> autoAllocateBuffers(std::vector<std::string> renderTargets);

  bool waitAll(uint64_t timeout);
  bool waitScenes(std::vector<int> const &list, uint64_t timeout);

  std::string summary() const;

  ~RenderServer();

private:
  VulkanCudaBuffer *allocateBuffer(std::string const &type, std::vector<int> const &shape);

  std::unique_ptr<RenderServiceImpl> mService;
  std::unique_ptr<grpc::Server> mServer;

  std::shared_ptr<svulkan2::core::Context> mContext;
  std::shared_ptr<svulkan2::resource::SVResourceManager> mResourceManager;

  std::vector<std::unique_ptr<VulkanCudaBuffer>> mBuffers;
};

} // namespace server
} // namespace Renderer
} // namespace sapien
