#include "sapien/component/sapien_renderer/deformable_mesh_component.h"
#include "sapien/component/interface.h"
#include "sapien/component/sapien_renderer/material.h"
#include "sapien/entity.h"
#include "sapien/scene.h"

#include <cuda_runtime.h>

namespace sapien {
namespace component {

CudaDeformableMeshComponent::CudaDeformableMeshComponent(uint32_t maxVertexCount,
                                                         uint32_t maxTriangleCount) {
  mEngine = SapienRenderEngine::Get();
  mMesh = std::make_shared<svulkan2::resource::SVMeshDeformable>(maxVertexCount, maxTriangleCount);

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

  // TODO: set material
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

uint32_t CudaDeformableMeshComponent::getMaxVertexCount() const {
  return mMesh->getMaxVertexCount();
}

uint32_t CudaDeformableMeshComponent::getMaxTriangleCount() const {
  return mMesh->getMaxTriangleCount();
}

CudaArrayHandle CudaDeformableMeshComponent::getVertexBufferCudaArray() const {
  int count = mMesh->getMaxVertexCount();
  int channels = mMesh->getVertexSize() / 4;
  int itemsize = 4;

  assert(static_cast<int>(mMesh->getVertexBuffer().getSize()) == count * channels * itemsize);

  return CudaArrayHandle{.shape = {count, channels},
                         .strides = {channels * itemsize, itemsize},
                         .type = "f4",
                         .cudaId = mMesh->getVertexBuffer().getCudaDeviceId(),
                         .ptr = mMesh->getVertexBuffer().getCudaPtr()};
}

CudaArrayHandle CudaDeformableMeshComponent::getIndexBufferCudaArray() const {
  int count = mMesh->getMaxTriangleCount();
  int channels = 3;
  int itemsize = 4;

  assert(static_cast<int>(mMesh->getIndexBuffer().getSize()) == count * channels * itemsize);

  return CudaArrayHandle{.shape = {count, channels},
                         .strides = {channels * itemsize, itemsize},
                         .type = "u4",
                         .cudaId = mMesh->getIndexBuffer().getCudaDeviceId(),
                         .ptr = mMesh->getIndexBuffer().getCudaPtr()};
}

void CudaDeformableMeshComponent::onAddToScene(Scene &scene) {
  auto system = scene.getSapienRendererSystem();
  auto s = system->getScene();

  mObject = &s->addObject(svulkan2::resource::SVModel::FromData(
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

void CudaDeformableMeshComponent::internalUpdate() {
  if (!mVertexProvider) {
    return;
  }

  void *source = reinterpret_cast<void *>(mVertexProvider->getCudaPointer());
  cudaStream_t stream = reinterpret_cast<cudaStream_t>(mVertexProvider->getCudaStream());
  if (!source) {
    return;
  }
  cudaMemcpy2DAsync(mMesh->getVertexBuffer().getCudaPtr(), mMesh->getVertexSize(), source, 12, 12,
                    getVertexCount(), cudaMemcpyDeviceToDevice, stream);

  // cudaDeviceSynchronize();

  // semaphore

  cudaExternalSemaphoreSignalParams sigParams{};
  sigParams.flags = 0;
  sigParams.params.fence.value = ++mSemValue;
  cudaSignalExternalSemaphoresAsync(&mCudaSem, &sigParams, 1, stream);
  vk::PipelineStageFlags flags = vk::PipelineStageFlagBits::eVertexInput;
  mEngine->getContext()->getQueue().submit({}, mSem.get(), flags, mSemValue, {}, {}, {});
}

void CudaDeformableMeshComponent::setDataSource(std::shared_ptr<CudaDataSource> vertexProvider) {
  mVertexProvider = vertexProvider;
}

} // namespace component
} // namespace sapien