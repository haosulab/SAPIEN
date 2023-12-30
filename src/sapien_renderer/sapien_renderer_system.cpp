#include "sapien/sapien_renderer/sapien_renderer_system.h"
#include "sapien/sapien_renderer/camera_component.h"
#include "sapien/sapien_renderer/cubemap.h"
#include "sapien/sapien_renderer/deformable_mesh_component.h"
#include "sapien/sapien_renderer/light_component.h"
#include "sapien/sapien_renderer/point_cloud_component.h"
#include "sapien/sapien_renderer/render_body_component.h"
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

std::shared_ptr<SapienRenderEngine>
SapienRenderEngine::Get(bool offscreenOnly, uint32_t maxNumMaterials, uint32_t maxNumTextures,
                        uint32_t defaultMipLevels, std::string const &device,
                        bool doNotLoadTexture) {
  static std::weak_ptr<SapienRenderEngine> gEngine;
  std::shared_ptr<SapienRenderEngine> engine;
  if ((engine = gEngine.lock())) {
    return engine;
  }
  gEngine = engine = std::make_shared<SapienRenderEngine>(
      offscreenOnly, maxNumMaterials, maxNumTextures, defaultMipLevels, device, doNotLoadTexture);
  return engine;
}

std::string SapienRenderEngine::getSummary() {
  std::stringstream ss;
  auto info = mContext->getPhysicalDevice2()->summarizeDeviceInfo();
  for (auto const &entry : info) {
    ss << "GPU: " << entry.name << "\n";
    ss << "  Present:   " << entry.present << "\n";
    ss << "  Supported: " << entry.supported << "\n";
    ss << "  cudaId:    " << entry.cudaId << "\n";
    ss << "  pciBus:    " << entry.pciBus << "\n";
    ss << "  rayTrace:  " << entry.rayTracing << "\n";
    ss << "  cudaMode   " << entry.cudaComputeMode << "\n";
  }
  return ss.str();
}

SapienRenderEngine::SapienRenderEngine(bool offscreenOnly, uint32_t maxNumMaterials,
                                       uint32_t maxNumTextures, uint32_t defaultMipLevels,
                                       std::string const &device, bool doNotLoadTexture) {
  mContext = svulkan2::core::Context::Create(!offscreenOnly, maxNumMaterials, maxNumTextures,
                                             defaultMipLevels, doNotLoadTexture, device);
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

// void SapienRenderEngine::gpuTransferPosesToRenderScenes(
//     CudaArrayHandle sceneTransformRefs, CudaArrayHandle sceneIndices,
//     CudaArrayHandle transformIndices, CudaArrayHandle localTransforms, CudaArrayHandle
//     localScales, CudaArrayHandle parentIndices, CudaArrayHandle parentTransforms) {
//   sceneTransformRefs.checkCongiguous();
//   sceneIndices.checkCongiguous();
//   transformIndices.checkCongiguous();
//   localTransforms.checkCongiguous();
//   localScales.checkCongiguous();
//   parentIndices.checkCongiguous();
//   parentTransforms.checkCongiguous();

//   sceneTransformRefs.checkShape({-1});
//   sceneTransformRefs.checkStride({8});

//   sceneIndices.checkShape({-1});
//   transformIndices.checkShape({-1});
//   localTransforms.checkShape({-1, 7});
//   localScales.checkShape({-1, 3});
//   parentIndices.checkShape({-1});

//   if (!(sceneIndices.shape[0] == transformIndices.shape[0] &&
//         sceneIndices.shape[0] == localTransforms.shape[0] &&
//         sceneIndices.shape[0] == localScales.shape[0] &&
//         sceneIndices.shape[0] == parentIndices.shape[0])) {
//     throw std::runtime_error(
//         "scene indices, transform indices, local transforms, local scales, and parent "
//         "indices must have the same length.");
//   }

//   transform_sapien_to_render(
//       (float **)sceneTransformRefs.ptr, (int *)sceneIndices.ptr, (int *)transformIndices.ptr,
//       (float *)localTransforms.ptr, (float *)localScales.ptr, (int *)parentIndices.ptr,
//       (float *)parentTransforms.ptr, parentTransforms.shape.at(parentTransforms.shape.size() -
//       1), sceneIndices.shape[0], (cudaStream_t)mCudaStream);
// }

// // TODO: SAPIEN_CUDA stuff
// void SapienRenderEngine::gpuNotifyPosesUpdated() {
//   if (!mSem) {
//     vk::SemaphoreTypeCreateInfo timelineCreateInfo(vk::SemaphoreType::eTimeline, 0);
//     vk::SemaphoreCreateInfo createInfo{};
//     vk::ExportSemaphoreCreateInfo exportCreateInfo(
//         vk::ExternalSemaphoreHandleTypeFlagBits::eOpaqueFd);
//     createInfo.setPNext(&exportCreateInfo);
//     exportCreateInfo.setPNext(&timelineCreateInfo);
//     mSem = getContext()->getDevice().createSemaphoreUnique(createInfo);

//     int fd = getContext()->getDevice().getSemaphoreFdKHR(
//         {mSem.get(), vk::ExternalSemaphoreHandleTypeFlagBits::eOpaqueFd});
//     cudaExternalSemaphoreHandleDesc desc = {};
//     desc.flags = 0;
//     desc.handle.fd = fd;
//     desc.type = cudaExternalSemaphoreHandleTypeTimelineSemaphoreFd;
//     checkCudaErrors(cudaImportExternalSemaphore(&mCudaSem, &desc));
//   }

//   cudaExternalSemaphoreSignalParams sigParams{};
//   sigParams.flags = 0;
//   sigParams.params.fence.value = ++mSemValue;
//   cudaSignalExternalSemaphoresAsync(&mCudaSem, &sigParams, 1, (cudaStream_t)mCudaStream);

//   vk::PipelineStageFlags stage = vk::PipelineStageFlagBits::eAllCommands;
//   getContext()->getQueue().submit({}, mSem.get(), stage, mSemValue, {}, {}, {});
// }

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
