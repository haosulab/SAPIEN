#pragma once
#include "common.h"
#include "renderer/server/protos/render_server.grpc.pb.h"
#include "safe_map.h"
#include "sapien/thread_pool.hpp"
#include <grpc/grpc.h>
#include <grpcpp/grpcpp.h>
#include <memory>
#include <shared_mutex>
#include <svulkan2/core/context.h>
#include <svulkan2/renderer/renderer.h>
#include <svulkan2/resource/manager.h>
#include <svulkan2/resource/material.h>
#include <svulkan2/scene/scene.h>
#include <unordered_map>

namespace sapien {
namespace Renderer {
namespace server {

extern std::string gDefaultShaderDirectory;
void setDefaultShaderDirectory(std::string const &dir);

using grpc::ServerContext;
using grpc::Status;

class RenderServiceImpl final : public proto::RenderService::Service {

  // NOTE: Important assumption
  // Functions to modify the same scene are not called concurrently!

  // ========== Renderer ==========//
  Status CreateScene(ServerContext *c, const proto::Index *req, proto::Id *res) override;
  Status RemoveScene(ServerContext *c, const proto::Id *req, proto::Empty *res) override;
  Status CreateMaterial(ServerContext *c, const proto::Empty *req, proto::Id *res) override;
  Status RemoveMaterial(ServerContext *c, const proto::Id *req, proto::Empty *res) override;
  // ========== Scene ==========//
  Status AddBodyMesh(ServerContext *c, const proto::AddBodyMeshReq *req, proto::Id *res) override;
  Status AddBodyPrimitive(ServerContext *c, const proto::AddBodyPrimitiveReq *req,
                          proto::Id *res) override;
  Status RemoveBody(ServerContext *c, const proto::RemoveBodyReq *req, proto::Empty *res) override;
  Status AddCamera(ServerContext *c, const proto::AddCameraReq *req, proto::Id *res) override;
  Status SetAmbientLight(ServerContext *c, const proto::IdVec3 *req, proto::Empty *res) override;
  Status AddPointLight(ServerContext *c, const proto::AddPointLightReq *req,
                       proto::Id *res) override;
  Status AddDirectionalLight(ServerContext *c, const proto::AddDirectionalLightReq *req,
                             proto::Id *res) override;
  Status SetEntityOrder(ServerContext *c, const proto::EntityOrderReq *req,
                        proto::Empty *res) override;
  Status UpdateRender(ServerContext *c, const proto::UpdateRenderReq *req,
                      proto::Empty *res) override;
  Status UpdateRenderAndTakePictures(ServerContext *c,
                                     const proto::UpdateRenderAndTakePicturesReq *req,
                                     proto::Empty *res) override;
  // ========== Material ==========//
  Status SetBaseColor(ServerContext *c, const proto::IdVec4 *req, proto::Empty *res) override;
  Status SetRoughness(ServerContext *c, const proto::IdFloat *req, proto::Empty *res) override;
  Status SetSpecular(ServerContext *c, const proto::IdFloat *req, proto::Empty *res) override;
  Status SetMetallic(ServerContext *c, const proto::IdFloat *req, proto::Empty *res) override;
  // ========== Body ==========//
  Status SetUniqueId(ServerContext *c, const proto::BodyIdReq *req, proto::Empty *res) override;
  Status SetSegmentationId(ServerContext *c, const proto::BodyIdReq *req,
                           proto::Empty *res) override;
  Status SetVisibility(ServerContext *c, const proto::BodyFloat32Req *req,
                       proto::Empty *res) override;
  Status GetShapeCount(ServerContext *c, const proto::BodyReq *req, proto::Uint32 *res) override;
  // ========== Shape ==========//
  Status GetShapeMaterial(ServerContext *c, const proto::BodyUint32Req *req,
                          proto::Id *res) override;
  // ========== Camera ==========//
  Status TakePicture(ServerContext *c, const proto::TakePictureReq *req,
                     proto::Empty *res) override;
  Status SetCameraParameters(ServerContext *c, const proto::CameraParamsReq *req,
                             proto::Empty *res) override;

public:
  RenderServiceImpl(std::shared_ptr<svulkan2::core::Context> context,
                    std::shared_ptr<svulkan2::resource::SVResourceManager> manager);

  friend class RenderServer;

private:
  inline uint64_t generateId() { return mIdGenerator++; }

  std::shared_ptr<svulkan2::core::Context> mContext;
  std::shared_ptr<svulkan2::resource::SVResourceManager> mResourceManager;

  std::atomic<uint64_t> mIdGenerator{0};

  struct CameraInfo {
    uint64_t cameraIndex;
    svulkan2::scene::Camera *camera;
    std::unique_ptr<svulkan2::renderer::Renderer> renderer;
    uint64_t frameCounter{};
    vk::UniqueSemaphore semaphore;

    std::unique_ptr<svulkan2::core::CommandPool> commandPool;
    vk::UniqueCommandBuffer commandBuffer;

    std::vector<std::tuple<std::string, vk::Buffer, vk::DeviceSize>> fillInfo;
  };

  struct SceneInfo {
    uint64_t sceneIndex;
    uint64_t sceneId;
    std::shared_ptr<svulkan2::scene::Scene> scene;

    std::unordered_map<rs_id_t, std::shared_ptr<CameraInfo>> cameraMap;
    std::vector<std::shared_ptr<CameraInfo>> cameraList;

    std::unordered_map<rs_id_t, svulkan2::scene::Object *> objectMap;

    // store material list of an object
    std::unordered_map<rs_id_t, std::vector<rs_id_t>> objectMaterialIdMap;

    // ordered objects and cameras for pose update
    std::vector<svulkan2::scene::Object *> orderedObjects;
    std::vector<svulkan2::scene::Camera *> orderedCameras;

    std::unique_ptr<ThreadPool> threadRunner;
  };

  // store materials on an object
  ts_unordered_map<rs_id_t, std::weak_ptr<svulkan2::resource::SVMetallicMaterial>>
      mObjectMaterialMap;
  ts_unordered_map<rs_id_t, std::shared_ptr<svulkan2::resource::SVMetallicMaterial>> mMaterialMap;
  ts_unordered_map<rs_id_t, std::shared_ptr<SceneInfo>> mSceneMap;

  std::shared_ptr<svulkan2::resource::SVMetallicMaterial> getMaterial(rs_id_t id);

  // refresh the object material map to remove expired weak ptr
  void updateObjectMaterialMap();

  std::shared_mutex mSceneListLock;
  std::vector<std::shared_ptr<SceneInfo>> mSceneList;

  std::shared_ptr<svulkan2::resource::SVMesh> mCubeMesh;
  std::shared_ptr<svulkan2::resource::SVMesh> mSphereMesh;
  std::shared_ptr<svulkan2::resource::SVMesh> mPlaneMesh;

  // HACK: store info for filling camera fill info
  std::vector<std::tuple<std::string, vk::Buffer, size_t>>
  getCameraFillInfo(uint64_t sceneIndex, uint64_t cameraIndex) {
    std::vector<std::tuple<std::string, vk::Buffer, size_t>> result;
    for (size_t i = 0; i < mRenderTargets.size(); ++i) {
      std::string target = mRenderTargets.at(i);
      vk::Buffer buffer = mRenderTargetBuffers.at(i);
      size_t stride = mRenderTargetStrides.at(i);
      size_t offset = (sceneIndex * mMaxCameraCount + cameraIndex) * stride;
      result.push_back({target, buffer, offset});
    }
    return result;
  }
  size_t mMaxCameraCount{};
  std::vector<std::string> mRenderTargets;
  std::vector<vk::Buffer> mRenderTargetBuffers;
  std::vector<size_t> mRenderTargetStrides{};
  // HACK end
};

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
