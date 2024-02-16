#include "sapien/sapien_renderer/sapien_renderer_system.h"
#include "sapien/sapien_renderer/camera_component.h"
#include "sapien/sapien_renderer/cubemap.h"
#include "sapien/sapien_renderer/deformable_mesh_component.h"
#include "sapien/sapien_renderer/light_component.h"
#include "sapien/sapien_renderer/point_cloud_component.h"
#include "sapien/sapien_renderer/render_body_component.h"
#include "sapien/sapien_renderer/sapien_renderer_default.h"
#include <svulkan2/core/context.h>
#include <svulkan2/core/physical_device.h>
#include <svulkan2/renderer/renderer.h>
#include <svulkan2/renderer/rt_renderer.h>
#include <svulkan2/scene/scene.h>

#if SAPIEN_CUDA
#include "sapien/utils/cuda.h"
#endif

namespace sapien {
namespace sapien_renderer {

std::shared_ptr<SapienRenderEngine> SapienRenderEngine::Get() {
  static std::weak_ptr<SapienRenderEngine> gEngine;
  std::shared_ptr<SapienRenderEngine> engine;
  if ((engine = gEngine.lock())) {
    return engine;
  }
  gEngine = engine = std::make_shared<SapienRenderEngine>();
  return engine;
}

std::string SapienRenderEngine::getSummary() {
  std::stringstream ss;
  auto info = svulkan2::core::Instance::Get(VK_MAKE_VERSION(0, 0, 1), VK_MAKE_VERSION(0, 0, 1),
                                            VK_API_VERSION_1_2)
                  ->summarizePhysicalDevices();
  for (auto const &entry : info) {
    ss << "GPU: " << entry.name << "\n";
    ss << "  Supported: " << entry.supported << "\n";
    ss << "  Present:   " << entry.present << "\n";
    ss << "  cudaId:    " << entry.cudaId << "\n";
    ss << "  rayTrace:  " << entry.rayTracing << "\n";
    ss << "  cudaMode   " << entry.cudaComputeMode << "\n";
  }
  return ss.str();
}

SapienRenderEngine::SapienRenderEngine() {
  auto &d = SapienRendererDefault::Get();
  auto device = d.getDevice();
  mContext = svulkan2::core::Context::Create(d.getMaxNumMaterials(), d.getMaxNumTextures(),
                                             d.getDefaultMipMaps(), d.getDoNotLoadTexture(),
                                             device ? device->getAlias() : "");
  mResourceManager = mContext->createResourceManager();
}

std::shared_ptr<svulkan2::resource::SVMesh> SapienRenderEngine::getSphereMesh() {
  if (!mSphereMesh) {
    mSphereMesh = svulkan2::resource::SVMesh::CreateUVSphere(32, 16);
  }
  return mSphereMesh;
}

std::shared_ptr<svulkan2::resource::SVMesh> SapienRenderEngine::getPlaneMesh() {
  if (!mPlaneMesh) {
    mPlaneMesh = svulkan2::resource::SVMesh::CreateYZPlane();
  }
  return mPlaneMesh;
}

std::shared_ptr<svulkan2::resource::SVMesh> SapienRenderEngine::getBoxMesh() {
  if (!mBoxMesh) {
    mBoxMesh = svulkan2::resource::SVMesh::CreateCube();
  }
  return mBoxMesh;
}

SapienRenderEngine::~SapienRenderEngine() {}

SapienRendererSystem::SapienRendererSystem() {
  mEngine = SapienRenderEngine::Get();
  mScene = std::make_shared<svulkan2::scene::Scene>();
}

Vec3 SapienRendererSystem::getAmbientLight() const {
  auto l = mScene->getAmbientLight();
  return {l.r, l.g, l.b};
}

void SapienRendererSystem::setAmbientLight(Vec3 l) {
  mScene->setAmbientLight({l.x, l.y, l.z, 1.f});
}

void SapienRendererSystem::setCubemap(std::shared_ptr<SapienRenderCubemap> cubemap) {
  mScene->setEnvironmentMap(cubemap ? cubemap->getCubemap() : nullptr);
  mCubemap = cubemap;
}
std::shared_ptr<SapienRenderCubemap> SapienRendererSystem::getCubemap() const { return mCubemap; }

void SapienRendererSystem::registerComponent(std::shared_ptr<SapienRenderBodyComponent> c) {
  mRenderBodyComponents.insert(c);
}

void SapienRendererSystem::registerComponent(std::shared_ptr<SapienRenderCameraComponent> c) {
  mRenderCameraComponents.insert(c);
}

void SapienRendererSystem::registerComponent(std::shared_ptr<SapienRenderLightComponent> c) {
  mRenderLightComponents.insert(c);
}

void SapienRendererSystem::registerComponent(std::shared_ptr<PointCloudComponent> c) {
  mPointCloudComponents.insert(c);
}

void SapienRendererSystem::registerComponent(std::shared_ptr<CudaDeformableMeshComponent> c) {
  mCudaDeformableMeshComponents.insert(c);
}

void SapienRendererSystem::unregisterComponent(std::shared_ptr<SapienRenderBodyComponent> c) {
  mRenderBodyComponents.erase(c);
}

void SapienRendererSystem::unregisterComponent(std::shared_ptr<SapienRenderCameraComponent> c) {
  mRenderCameraComponents.erase(c);
}

void SapienRendererSystem::unregisterComponent(std::shared_ptr<SapienRenderLightComponent> c) {
  mRenderLightComponents.erase(c);
}

void SapienRendererSystem::unregisterComponent(std::shared_ptr<PointCloudComponent> c) {
  mPointCloudComponents.erase(c);
}

void SapienRendererSystem::unregisterComponent(std::shared_ptr<CudaDeformableMeshComponent> c) {
  mCudaDeformableMeshComponents.erase(c);
}

void SapienRendererSystem::step() {
  for (auto c : mRenderBodyComponents) {
    c->internalUpdate();
  }
  for (auto c : mRenderCameraComponents) {
    c->internalUpdate();
  }
  for (auto c : mRenderLightComponents) {
    c->internalUpdate();
  }
  for (auto c : mPointCloudComponents) {
    c->internalUpdate();
  }
  for (auto c : mCudaDeformableMeshComponents) {
    c->internalUpdate();
  }
  mScene->updateModelMatrices();
}

CudaArrayHandle SapienRendererSystem::getTransformCudaArray() {
  mScene->prepareObjectTransformBuffer();
  auto buffer = mScene->getObjectTransformBuffer();
  return CudaArrayHandle{.shape = {static_cast<int>(buffer->getSize() / 64), 4, 4},
                         .strides = {64, 16, 4},
                         .type = "f4",
                         .cudaId = buffer->getCudaDeviceId(),
                         .ptr = buffer->getCudaPtr()};
}

SapienRendererSystem::~SapienRendererSystem() {}

} // namespace sapien_renderer
} // namespace sapien
