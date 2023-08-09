#include "sapien/component/sapien_renderer/point_cloud_component.h"
#include "sapien/entity.h"
#include "sapien/scene.h"

namespace sapien {
namespace component {

PointCloudComponent::PointCloudComponent(uint32_t capacity) {
  mEngine = SapienRenderEngine::Get();
  mPointSet = std::make_shared<svulkan2::resource::SVPointSet>(capacity);
}

void PointCloudComponent::setVertices(
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &vertices) {
  setAttribute("position", vertices);
}

void PointCloudComponent::setAttribute(
    std::string const &name,
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> const &vertices) {
  std::vector<float> data(vertices.data(), vertices.data() + vertices.size());
  mPointSet->setVertexAttribute(name, data);
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
  int count = mPointSet->getVertexCount();
  int channels = mPointSet->getVertexSize() / 4;
  int itemsize = 4;

  return CudaArrayHandle{.shape = {count, channels},
                   .strides = {channels * itemsize, itemsize},
                   .type = "f4",
                   .cudaId = mPointSet->getVertexBuffer().getCudaDeviceId(),
                   .ptr = mPointSet->getVertexBuffer().getCudaPtr()};
}

// void *PointCloudComponent::getCudaPtr() { return mPointSet->getVertexBuffer().getCudaPtr(); }
// int PointCloudComponent::getCudaDevice() { return
// mPointSet->getVertexBuffer().getCudaDeviceId(); } uint32_t PointCloudComponent::getVertexCount()
// { return mPointSet->getVertexCount(); } uint32_t PointCloudComponent::getVertexChannels() {
// return mPointSet->getVertexSize() / 4; } std::string PointCloudComponent::getVertexTypestr() {
// return "f4"; }

svulkan2::scene::Transform PointCloudComponent::getTransform() const {
  auto pose = getPose();
  return {
      .position = {pose.p.x, pose.p.y, pose.p.z},
      .rotation = {pose.q.w, pose.q.x, pose.q.y, pose.q.z},
      .scale = {1.f, 1.f, 1.f},
  };
}

} // namespace component
} // namespace sapien
