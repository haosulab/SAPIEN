#include "sapien/renderer/svulkan2_pointbody.h"
#include "sapien/renderer/dlpack.hpp"
#include "sapien/renderer/svulkan2_scene.h"

namespace sapien {
namespace Renderer {

SVulkan2PointBody::SVulkan2PointBody(SVulkan2Scene *scene, svulkan2::scene::PointObject *object)
    : mParentScene(scene), mObject(object) {}

void SVulkan2PointBody::setVisibility(float visibility) {
  mObject->setTransparency(1 - visibility);
}
void SVulkan2PointBody::setRenderMode(uint32_t mode) { mObject->setShadingMode(mode); }
void SVulkan2PointBody::destroyVisualObject() { mParentScene->getScene()->removeNode(*mObject); }

void SVulkan2PointBody::setRenderedVertexCount(uint32_t count) { mObject->setVertexCount(count); }
uint32_t SVulkan2PointBody::getRenderedVertexCount() { return mObject->getVertexCount(); }

void SVulkan2PointBody::setAttribute(
    std::string_view name,
    Eigen::Ref<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> value) {
  mObject->getPointSet()->setVertexAttribute(
      std::string(name), std::vector<float>{value.data(), value.data() + value.size()});
}

DLManagedTensor *SVulkan2PointBody::getDLVertices() {
  auto &buffer = mObject->getPointSet()->getVertexBuffer();
  void *ptr = buffer.getCudaPtr();
  int id = buffer.getCudaDeviceId();

  int64_t vertexCount = mObject->getPointSet()->getVertexCount();
  int64_t vertexSize = mObject->getPointSet()->getVertexSize();

  return dl_wrapper<svulkan2::resource::SVPointSet>(mObject->getPointSet(), ptr, id,
                                                    {vertexCount, vertexSize / 4},
                                                    {DLDataTypeCode::kDLFloat, 32, 1});
}

} // namespace Renderer
} // namespace sapien
