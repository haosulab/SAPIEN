#pragma once

#include "../component.h"
#include "../device.h"
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
  static std::shared_ptr<SapienRenderEngine> Get(std::shared_ptr<Device> device = nullptr);

  SapienRenderEngine(std::shared_ptr<Device> device);

  std::shared_ptr<svulkan2::core::Context> getContext() const { return mContext; }
  std::shared_ptr<svulkan2::resource::SVResourceManager> getResourceManager() const {
    return mResourceManager;
  }

  std::shared_ptr<svulkan2::resource::SVMesh> getSphereMesh();
  std::shared_ptr<svulkan2::resource::SVMesh> getPlaneMesh();
  std::shared_ptr<svulkan2::resource::SVMesh> getBoxMesh();

  std::string getSummary();

  std::shared_ptr<Device> getDevice() const { return mDevice; }

  ~SapienRenderEngine();

private:
  std::shared_ptr<Device> mDevice;
  std::shared_ptr<svulkan2::core::Context> mContext;
  std::shared_ptr<svulkan2::resource::SVResourceManager> mResourceManager;

  std::shared_ptr<svulkan2::resource::SVMesh> mSphereMesh;
  std::shared_ptr<svulkan2::resource::SVMesh> mPlaneMesh;
  std::shared_ptr<svulkan2::resource::SVMesh> mBoxMesh;
};

class SapienRendererSystem : public System {
public:
  SapienRendererSystem(std::shared_ptr<Device> device);
  std::shared_ptr<Device> getDevice() const;

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

  ~SapienRendererSystem();

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
};

} // namespace sapien_renderer
} // namespace sapien
