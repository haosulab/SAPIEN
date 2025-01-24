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
#include "sapien/sapien_renderer/deformable_mesh_component.h"
#include "sapien/entity.h"
#include "sapien/sapien_renderer/material.h"
#include "sapien/scene.h"

#ifdef SAPIEN_CUDA
#include <cuda_runtime.h>
#endif

namespace sapien {
namespace sapien_renderer {

CudaDeformableMeshComponent::CudaDeformableMeshComponent(uint32_t maxVertexCount,
                                                         uint32_t maxTriangleCount) {
  mEngine = SapienRenderEngine::Get();
  mMesh = std::make_shared<svulkan2::resource::SVMeshDeformable>(maxVertexCount, maxTriangleCount);

#ifdef SAPIEN_CUDA
  vk::SemaphoreTypeCreateInfo timelineCreateInfo(vk::SemaphoreType::eTimeline, 0);
  vk::SemaphoreCreateInfo createInfo{};
  vk::ExportSemaphoreCreateInfo exportCreateInfo(
      vk::ExternalSemaphoreHandleTypeFlagBits::eOpaqueFd);
  createInfo.setPNext(&exportCreateInfo);
  exportCreateInfo.setPNext(&timelineCreateInfo);
  auto device = mEngine->getContext()->getDevice();
  mSem = device.createSemaphoreUnique(createInfo);

  int fd =
      device.getSemaphoreFdKHR({mSem.get(), vk::ExternalSemaphoreHandleTypeFlagBits::eOpaqueFd});
  cudaExternalSemaphoreHandleDesc desc = {};
  desc.flags = 0;
  desc.handle.fd = fd;
  desc.type = cudaExternalSemaphoreHandleTypeTimelineSemaphoreFd;
  auto err = cudaImportExternalSemaphore(&mCudaSem, &desc);
  if (err != cudaSuccess) {
    throw std::runtime_error("Failed to export semaphore to CUDA: " +
                             std::string(cudaGetErrorName(err)));
  }
#endif

  mMaterial = std::make_shared<SapienRenderMaterial>();
}

void CudaDeformableMeshComponent::setMaterial(std::shared_ptr<SapienRenderMaterial> material) {
  mMaterial = material;
}
std::shared_ptr<SapienRenderMaterial> CudaDeformableMeshComponent::getMaterial() const {
  return mMaterial;
}

uint32_t CudaDeformableMeshComponent::getVertexCount() const { return mMesh->getVertexCount(); }
void CudaDeformableMeshComponent::setVertexCount(uint32_t count) { mMesh->setVertexCount(count); }

uint32_t CudaDeformableMeshComponent::getTriangleCount() const {
  return mMesh->getTriangleCount();
}
void CudaDeformableMeshComponent::setTriangleCount(uint32_t count) {
  mMesh->setTriangleCount(count);
}

void CudaDeformableMeshComponent::setTriangles(
    Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor> const &triangles) {
  setTriangleCount(triangles.rows());
  mMesh->getIndexBuffer().upload(triangles.data(), triangles.size() * sizeof(uint32_t));
}

uint32_t CudaDeformableMeshComponent::getMaxVertexCount() const {
  return mMesh->getMaxVertexCount();
}

uint32_t CudaDeformableMeshComponent::getMaxTriangleCount() const {
  return mMesh->getMaxTriangleCount();
}

CudaArrayHandle CudaDeformableMeshComponent::getVertexBufferCudaArray() const {
#ifdef SAPIEN_CUDA
  int count = mMesh->getMaxVertexCount();
  int channels = mMesh->getVertexSize() / 4;
  int itemsize = 4;

  assert(static_cast<int>(mMesh->getVertexBuffer().getSize()) == count * channels * itemsize);

  return CudaArrayHandle{.shape = {count, channels},
                         .strides = {channels * itemsize, itemsize},
                         .type = "f4",
                         .cudaId = mMesh->getVertexBuffer().getCudaDeviceId(),
                         .ptr = mMesh->getVertexBuffer().getCudaPtr()};
#else
  throw std::runtime_error("sapien is not copmiled with CUDA support");
#endif
}

CudaArrayHandle CudaDeformableMeshComponent::getIndexBufferCudaArray() const {
#ifdef SAPIEN_CUDA
  int count = mMesh->getMaxTriangleCount();
  int channels = 3;
  int itemsize = 4;

  assert(static_cast<int>(mMesh->getIndexBuffer().getSize()) == count * channels * itemsize);

  return CudaArrayHandle{.shape = {count, channels},
                         .strides = {channels * itemsize, itemsize},
                         .type = "u4",
                         .cudaId = mMesh->getIndexBuffer().getCudaDeviceId(),
                         .ptr = mMesh->getIndexBuffer().getCudaPtr()};
#else
  throw std::runtime_error("sapien is not copmiled with CUDA support");
#endif
}

void CudaDeformableMeshComponent::onAddToScene(Scene &scene) {
  auto system = scene.getSapienRendererSystem();
  auto s = system->getScene();

  mObject = &s->addDeformableObject(svulkan2::resource::SVModel::FromData(
      {svulkan2::resource::SVShape::Create(mMesh, mMaterial->getMaterial())}));
  mObject->setCustomDataInt("shadeFlat", true);

  system->registerComponent(
      std::static_pointer_cast<CudaDeformableMeshComponent>(shared_from_this()));
}

void CudaDeformableMeshComponent::onRemoveFromScene(Scene &scene) {
  auto system = scene.getSapienRendererSystem();
  auto s = system->getScene();

  s->removeNode(*mObject);

  system->unregisterComponent(
      std::static_pointer_cast<CudaDeformableMeshComponent>(shared_from_this()));
}

void CudaDeformableMeshComponent::notifyVertexUpdated(uintptr_t stream) {
#ifdef SAPIEN_CUDA
  cudaExternalSemaphoreSignalParams sigParams{};
  sigParams.flags = 0;
  sigParams.params.fence.value = ++mSemValue;
  cudaSignalExternalSemaphoresAsync(&mCudaSem, &sigParams, 1,
                                    reinterpret_cast<cudaStream_t>(stream));
  vk::PipelineStageFlags flags = vk::PipelineStageFlagBits::eVertexInput;
  mEngine->getContext()->getQueue().submit({}, mSem.get(), flags, mSemValue, {}, {}, {});
#endif
}

void CudaDeformableMeshComponent::internalUpdate() {}

} // namespace sapien_renderer
} // namespace sapien
