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
#pragma once
#include "../component.h"
#include "sapien/array.h"
#include "sapien/math/pose.h"
#include "sapien_renderer_system.h"
#include <Eigen/Dense>
#include <svulkan2/resource/model.h>

namespace sapien {

namespace sapien_renderer {

class SapienRenderMaterial;

class CudaDeformableMeshComponent : public Component {
public:
  CudaDeformableMeshComponent(uint32_t maxVertexCount, uint32_t maxTriangleCount);

  void setTriangles(Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor> const &triangles);

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

  void notifyVertexUpdated(uintptr_t stream);

private:
  std::shared_ptr<SapienRenderEngine> mEngine;
  std::shared_ptr<svulkan2::resource::SVMeshDeformable> mMesh;
  std::shared_ptr<SapienRenderMaterial> mMaterial;

  svulkan2::scene::Object *mObject{};

#ifdef SAPIEN_CUDA
  vk::UniqueSemaphore mSem{};
  cudaExternalSemaphore_t mCudaSem{};
  uint64_t mSemValue{0};
#endif
};

} // namespace sapien_renderer
} // namespace sapien
