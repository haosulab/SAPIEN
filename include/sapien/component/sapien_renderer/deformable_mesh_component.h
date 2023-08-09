#pragma once
#include "../component.h"
#include "../interface.h"
#include "sapien/array.h"
#include "sapien/component/sapien_renderer/sapien_renderer_system.h"
#include "sapien/math/pose.h"
#include "sapien/serialize.h"
#include <svulkan2/resource/model.h>

#include <cuda_runtime.h>

namespace sapien {
namespace component {

class CudaDataSource;
class SapienRenderMaterial;

class CudaDeformableMeshComponent : public Component {
public:
  CudaDeformableMeshComponent(uint32_t maxVertexCount, uint32_t maxTriangleCount);

  void setDataSource(std::shared_ptr<CudaDataSource> vertexProvider);

  uint32_t getVertexCount() const;
  void setVertexCount(uint32_t count);

  uint32_t getTriangleCount() const;
  void setTriangleCount(uint32_t count);

  uint32_t getMaxVertexCount() const;
  uint32_t getMaxTriangleCount() const;

  void onAddToScene(Scene &scene) override;
  void onRemoveFromScene(Scene &scene) override;

  CudaArrayHandle getVertexBufferCudaArray() const;
  CudaArrayHandle getIndexBufferCudaArray() const;

  void setMaterial(std::shared_ptr<SapienRenderMaterial> material);
  std::shared_ptr<SapienRenderMaterial> getMaterial() const;

  void internalUpdate();

private:
  std::shared_ptr<SapienRenderEngine> mEngine;
  std::shared_ptr<svulkan2::resource::SVMeshDeformable> mMesh;
  std::shared_ptr<SapienRenderMaterial> mMaterial;
  std::shared_ptr<CudaDataSource> mVertexProvider;

  svulkan2::scene::Object *mObject{};

  vk::UniqueSemaphore mSem{};
  cudaExternalSemaphore_t mCudaSem{};
  uint64_t mSemValue{0};
};

} // namespace component
} // namespace sapien
