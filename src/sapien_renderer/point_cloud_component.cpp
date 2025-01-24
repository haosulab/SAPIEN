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
#include "sapien/sapien_renderer/point_cloud_component.h"
#include "sapien/entity.h"
#include "sapien/scene.h"

namespace sapien {
namespace sapien_renderer {

PointCloudComponent::PointCloudComponent(uint32_t capacity) {
  mEngine = SapienRenderEngine::Get();
  mPointSet = std::make_shared<svulkan2::resource::SVPointSet>(capacity);
}

std::shared_ptr<PointCloudComponent> PointCloudComponent::setVertices(
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &vertices) {
  setAttribute("position", vertices);
  return std::static_pointer_cast<PointCloudComponent>(shared_from_this());
}

Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> PointCloudComponent::getVertices() {
  auto pos = mPointSet->getVertexAttribute("position");
  return Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>>(pos.data(),
                                                                              pos.size() / 3, 3);
}

std::shared_ptr<PointCloudComponent> PointCloudComponent::setAttribute(
    std::string const &name,
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> const &attribute) {
  std::vector<float> data(attribute.data(), attribute.data() + attribute.size());
  mPointSet->setVertexAttribute(name, data);
  return std::static_pointer_cast<PointCloudComponent>(shared_from_this());
}

void PointCloudComponent::onAddToScene(Scene &scene) {
  auto system = scene.getSapienRendererSystem();
  auto s = system->getScene();
  mObject = &s->addPointObject(mPointSet, getTransform());
  system->registerComponent(std::static_pointer_cast<PointCloudComponent>(shared_from_this()));
}

void PointCloudComponent::onRemoveFromScene(Scene &scene) {
  auto system = scene.getSapienRendererSystem();
  auto s = system->getScene();
  s->removeNode(*mObject);
  mObject = nullptr;
  system->unregisterComponent(std::static_pointer_cast<PointCloudComponent>(shared_from_this()));
}

// called by system to sync pose
void PointCloudComponent::internalUpdate() {
  auto pose = getEntity()->getPose();
  mObject->setPosition({pose.p.x, pose.p.y, pose.p.z});
  mObject->setRotation({pose.q.w, pose.q.x, pose.q.y, pose.q.z});
}

CudaArrayHandle PointCloudComponent::getCudaArray() const {
#ifdef SAPIEN_CUDA
  int count = mPointSet->getVertexCount();
  int channels = mPointSet->getVertexSize() / 4;
  int itemsize = 4;

  return CudaArrayHandle{.shape = {count, channels},
                         .strides = {channels * itemsize, itemsize},
                         .type = "f4",
                         .cudaId = mPointSet->getVertexBuffer().getCudaDeviceId(),
                         .ptr = mPointSet->getVertexBuffer().getCudaPtr()};
#else
  throw std::runtime_error("sapien is not copmiled with CUDA support");
#endif
}

std::optional<CudaArrayHandle> PointCloudComponent::getCudaAABBArray() const {
#ifdef SAPIEN_CUDA
  auto buffer = mPointSet->getAabbBuffer();
  if (!buffer) {
    return {};
  }

  int count = mPointSet->getVertexCount();

  return CudaArrayHandle{.shape = {count, 2, 3},
                         .strides = {6 * sizeof(float), 3 * sizeof(float), sizeof(float)},
                         .type = "f4",
                         .cudaId = buffer->getCudaDeviceId(),
                         .ptr = buffer->getCudaPtr()};
#else
  throw std::runtime_error("sapien is not copmiled with CUDA support");
#endif
}

svulkan2::scene::Transform PointCloudComponent::getTransform() const {
  auto pose = getPose();
  return {
      .position = {pose.p.x, pose.p.y, pose.p.z},
      .rotation = {pose.q.w, pose.q.x, pose.q.y, pose.q.z},
      .scale = {1.f, 1.f, 1.f},
  };
}

} // namespace sapien_renderer
} // namespace sapien
