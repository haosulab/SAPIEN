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

#ifdef SAPIEN_CUDA
#include "sapien/utils/cuda.h"
#include <cuda_runtime.h>
#endif

namespace sapien {
namespace sapien_renderer {

std::shared_ptr<SapienRenderEngine> SapienRenderEngine::Get(std::shared_ptr<Device> device) {
  static std::weak_ptr<SapienRenderEngine> gEngine;
  std::shared_ptr<SapienRenderEngine> engine;
  if ((engine = gEngine.lock())) {
    if (device && engine->mDevice != device) {
      throw std::runtime_error("failed to create renderer on device \"" + device->getAlias() +
                               "\": current SAPIEN version only supports single-GPU rendering.");
    }
    return engine;
  }
  gEngine = engine = std::make_shared<SapienRenderEngine>(device);
  return engine;
}

std::string SapienRenderEngine::getSummary() {
  try {
    std::shared_ptr<svulkan2::core::Context> context;
    try {
      context = svulkan2::core::Context::Get();
    } catch (std::runtime_error &) {
      context = svulkan2::core::Context::Create();
    }

    auto info = context->getInstance2()->summarizePhysicalDevices();
    std::stringstream ss;

    for (auto const &entry : info) {
      ss << "GPU: " << entry.name << "\n";
      ss << "  Supported: " << entry.supported << "\n";
      ss << "  Present:   " << entry.present << "\n";
      ss << "  cudaId:    " << entry.cudaId << "\n";
      ss << "  rayTrace:  " << entry.rayTracing << "\n";
      ss << "  cudaMode   " << entry.cudaComputeMode << "\n";
    }
    return ss.str();
  } catch (std::runtime_error &) {
    return "Failed to initialize Vulkan";
  }
}

SapienRenderEngine::SapienRenderEngine(std::shared_ptr<Device> device) {
  if (!device) {
    device = findBestRenderDevice();
  }
  if (!device) {
    throw std::runtime_error("failed to find a rendering device");
  }

  mDevice = device;
  auto &d = SapienRendererDefault::Get();
  mContext = svulkan2::core::Context::Create(d.getMaxNumMaterials(), d.getMaxNumTextures(),
                                             d.getDefaultMipMaps(), d.getDoNotLoadTexture(),
                                             device->getAlias(), d.getVREnabled());
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

SapienRendererSystem::SapienRendererSystem(std::shared_ptr<Device> device) {
  mEngine = SapienRenderEngine::Get(device);
  mScene = std::make_shared<svulkan2::scene::Scene>();
}

std::shared_ptr<Device> SapienRendererSystem::getDevice() const { return mEngine->getDevice(); }

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
  int offset = mScene->getGpuTransformBufferSize();

  auto buffer = mScene->getObjectTransformBuffer();
#ifdef SAPIEN_CUDA
  return CudaArrayHandle{.shape = {static_cast<int>(buffer->getSize() / offset), 4, 4},
                         .strides = {offset, 16, 4},
                         .type = "f4",
                         .cudaId = buffer->getCudaDeviceId(),
                         .ptr = buffer->getCudaPtr()};
#else
  return CudaArrayHandle{.shape = {static_cast<int>(buffer->getSize() / offset), 4, 4},
                         .strides = {offset, 16, 4},
                         .type = "f4"};
#endif
}

SapienRendererSystem::~SapienRendererSystem() {}

} // namespace sapien_renderer
} // namespace sapien
