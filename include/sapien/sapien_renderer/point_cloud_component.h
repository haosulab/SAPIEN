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

class PointCloudComponent : public Component {
public:
  PointCloudComponent(uint32_t capacity = 0);

  std::shared_ptr<PointCloudComponent>
  setVertices(Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &vertices);
  Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> getVertices();

  std::shared_ptr<PointCloudComponent> setAttribute(
      std::string const &name,
      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> const &attribute);

  inline std::shared_ptr<svulkan2::resource::SVPointSet> const &getPointSet() const {
    return mPointSet;
  }

  void onAddToScene(Scene &scene) override;
  void onRemoveFromScene(Scene &scene) override;

  void internalUpdate();

  CudaArrayHandle getCudaArray() const;

  /** temporary, used to update AABB for ray tracing BLAS */
  std::optional<CudaArrayHandle> getCudaAABBArray() const;

  PointCloudComponent(PointCloudComponent const &) = delete;
  PointCloudComponent &operator=(PointCloudComponent const &) = delete;
  PointCloudComponent(PointCloudComponent const &&) = delete;
  PointCloudComponent &operator=(PointCloudComponent const &&) = delete;

private:
  svulkan2::scene::Transform getTransform() const;

  std::shared_ptr<SapienRenderEngine> mEngine;
  Pose mLocalPose{};
  std::shared_ptr<svulkan2::resource::SVPointSet> mPointSet;
  svulkan2::scene::PointObject *mObject{};
  Vec3 mScale{1.f};
};

} // namespace sapien_renderer
} // namespace sapien
