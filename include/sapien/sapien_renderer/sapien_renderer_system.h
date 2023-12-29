#pragma once

#include "../component.h"
#include "../system.h"
#include "cubemap.h"
#include "sapien/array.h"
#include "sapien/math/vec3.h"
#include <set>
#include <svulkan2/core/context.h>
#include <svulkan2/scene/scene.h>

typedef struct CUstream_st *cudaStream_t;
typedef struct CUexternalSemaphore_st *cudaExternalSemaphore_t;

namespace sapien {
namespace sapien_renderer {
class SapienRenderBodyComponent;
class SapienRenderCameraComponent;
class SapienRenderLightComponent;
class PointCloudComponent;
class CudaDeformableMeshComponent;
class SapienRenderCubemap;

class SapienRenderEngine {
public:
  static std::shared_ptr<SapienRenderEngine>
  Get(bool offscreenOnly = false, uint32_t maxNumMaterials = 512, uint32_t maxNumTextures = 128,
      uint32_t defaultMipLevels = 1, std::string const &device = "",
      bool doNotLoadTexture = false);

  SapienRenderEngine(bool offscreenOnly, uint32_t maxNumMaterials, uint32_t maxNumTextures,
                     uint32_t defaultMipLevels, std::string const &device, bool doNotLoadTexture);

  std::shared_ptr<svulkan2::core::Context> getContext() const { return mContext; }
  std::shared_ptr<svulkan2::resource::SVResourceManager> getResourceManager() const {
    return mResourceManager;
  }

  std::shared_ptr<svulkan2::resource::SVMesh> getSphereMesh();
  std::shared_ptr<svulkan2::resource::SVMesh> getPlaneMesh();
  std::shared_ptr<svulkan2::resource::SVMesh> getBoxMesh();

  std::string getSummary();

  /**
   *  Args:
   *      sceneTransformRefs: uint64 pointers pointing to per-scene transform buffers
   *      sceneIndices: scene index for each shape
   *      transformIndices: per-scene transform index for each shape
   *      localTransforms: local pose for each shape
   *      localScales: local scales for each shape
   *      parnetIndices: index fo each shape's parent in the parentTranasforms array
   *      parentTransforms: transform array for parent objects
   *  */
  void gpuTransferPosesToRenderScenes(CudaArrayHandle sceneTransformRefs,
                                      CudaArrayHandle sceneIndices,
                                      CudaArrayHandle transformIndices,
                                      CudaArrayHandle localTransforms, CudaArrayHandle localScales,
                                      CudaArrayHandle parentIndices,
                                      CudaArrayHandle parentTransforms);
  void gpuTransferPosesToRenderCameras();
  void gpuNotifyPosesUpdated();

  ~SapienRenderEngine();

private:
  std::shared_ptr<svulkan2::core::Context> mContext;
  std::shared_ptr<svulkan2::resource::SVResourceManager> mResourceManager;

  std::shared_ptr<svulkan2::resource::SVMesh> mSphereMesh;
  std::shared_ptr<svulkan2::resource::SVMesh> mPlaneMesh;
  std::shared_ptr<svulkan2::resource::SVMesh> mBoxMesh;

#ifdef SAPIEN_CUDA
  vk::UniqueSemaphore mSem{};
  cudaExternalSemaphore_t mCudaSem{};
  uint64_t mSemValue{0};
  void *mCudaStream{nullptr};
#endif
};

class SapienRendererSystem : public System {
public:
  SapienRendererSystem();
  std::shared_ptr<SapienRenderEngine> getEngine();
  std::shared_ptr<svulkan2::scene::Scene> getScene() { return mScene; }

  Vec3 getAmbientLight() const;
  void setAmbientLight(Vec3 l);
  void setCubemap(std::shared_ptr<SapienRenderCubemap> cubemap);
  std::shared_ptr<SapienRenderCubemap> getCubemap() const;

  void registerComponent(std::shared_ptr<SapienRenderBodyComponent> c);
  void registerComponent(std::shared_ptr<SapienRenderCameraComponent> c);
  void registerComponent(std::shared_ptr<SapienRenderLightComponent> c);
  void registerComponent(std::shared_ptr<PointCloudComponent> c);
  void registerComponent(std::shared_ptr<CudaDeformableMeshComponent> c);

  void unregisterComponent(std::shared_ptr<SapienRenderBodyComponent> c);
  void unregisterComponent(std::shared_ptr<SapienRenderCameraComponent> c);
  void unregisterComponent(std::shared_ptr<SapienRenderLightComponent> c);
  void unregisterComponent(std::shared_ptr<PointCloudComponent> c);
  void unregisterComponent(std::shared_ptr<CudaDeformableMeshComponent> c);

  std::vector<std::shared_ptr<SapienRenderBodyComponent>> getRenderBodyComponents() const {
    return std::vector<std::shared_ptr<SapienRenderBodyComponent>>{mRenderBodyComponents.begin(),
                                                                   mRenderBodyComponents.end()};
  }
  std::vector<std::shared_ptr<SapienRenderCameraComponent>> getCameraComponents() const {
    return std::vector<std::shared_ptr<SapienRenderCameraComponent>>{
        mRenderCameraComponents.begin(), mRenderCameraComponents.end()};
  }
  std::vector<std::shared_ptr<SapienRenderLightComponent>> getLightComponents() const {
    return std::vector<std::shared_ptr<SapienRenderLightComponent>>{mRenderLightComponents.begin(),
                                                                    mRenderLightComponents.end()};
  }

  void step() override;
  std::string getName() const override { return "render"; }

  CudaArrayHandle getTransformCudaArray();
  void disableAutoUpload();
  bool isAutoUploadEnabled();
  void notifyCudaTransformUpdated(cudaStream_t cudaStream);

  ~SapienRendererSystem();

  template <class Archive> void save(Archive &ar) const { ar(getAmbientLight(), getCubemap()); }
  template <class Archive> void load(Archive &ar) {
    Vec3 ambientLight;
    std::shared_ptr<SapienRenderCubemap> cubemap;
    ar(ambientLight, cubemap);
    setAmbientLight(ambientLight);
    setCubemap(cubemap);
  }

  uint64_t nextRenderId() { return mNextRenderId++; };

private:
  uint64_t mNextRenderId{1};

  std::shared_ptr<SapienRenderEngine> mEngine;
  std::shared_ptr<svulkan2::scene::Scene> mScene;

  std::set<std::shared_ptr<SapienRenderBodyComponent>, comp_cmp> mRenderBodyComponents;
  std::set<std::shared_ptr<SapienRenderCameraComponent>, comp_cmp> mRenderCameraComponents;
  std::set<std::shared_ptr<SapienRenderLightComponent>, comp_cmp> mRenderLightComponents;
  std::set<std::shared_ptr<PointCloudComponent>, comp_cmp> mPointCloudComponents;
  std::set<std::shared_ptr<CudaDeformableMeshComponent>, comp_cmp> mCudaDeformableMeshComponents;

  std::shared_ptr<SapienRenderCubemap> mCubemap;

  bool mAutoUploadEnabled{true};
};

} // namespace sapien_renderer
} // namespace sapien

