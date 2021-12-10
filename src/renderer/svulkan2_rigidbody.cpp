#include "svulkan2_rigidbody.h"
#include "svulkan2_renderer.h"
#include "svulkan2_shape.h"
#include <iostream>

namespace sapien {
namespace Renderer {

SVulkan2Rigidbody::SVulkan2Rigidbody(SVulkan2Scene *scene,
                                     std::vector<svulkan2::scene::Object *> const &objects,
                                     physx::PxGeometryType::Enum type, physx::PxVec3 scale)
    : mParentScene(scene), mObjects(objects), mType(type), mScale(scale) {}

void SVulkan2Rigidbody::setUniqueId(uint32_t uniqueId) {
  mUniqueId = uniqueId;
  for (auto obj : mObjects) {
    auto seg = obj->getSegmentation();
    seg[0] = uniqueId;
    obj->setSegmentation(seg);
  }
}

uint32_t SVulkan2Rigidbody::getUniqueId() const { return mUniqueId; }

void SVulkan2Rigidbody::setSegmentationId(uint32_t segmentationId) {
  mSegmentationId = segmentationId;
  for (auto obj : mObjects) {
    auto seg = obj->getSegmentation();
    seg[1] = segmentationId;
    obj->setSegmentation(seg);
  }
}

uint32_t SVulkan2Rigidbody::getSegmentationId() const { return mSegmentationId; }

void SVulkan2Rigidbody::setSegmentationCustomData(const std::vector<float> &customData) {
  if (customData.size() != 16) {
    throw std::runtime_error("object custom data must contain 16 float numbers");
  }
  glm::mat4 mat(customData[0], customData[1], customData[2], customData[3], customData[4],
                customData[5], customData[6], customData[7], customData[8], customData[9],
                customData[10], customData[11], customData[12], customData[13], customData[14],
                customData[15]);
  for (auto obj : mObjects) {
    obj->setCustomDataFloat44("custom", mat);
  }
}

void SVulkan2Rigidbody::setInitialPose(const physx::PxTransform &transform) {
  mInitialPose = transform;
  update({{0, 0, 0}, physx::PxIdentity});
}

void SVulkan2Rigidbody::update(const physx::PxTransform &transform) {
  auto pose = transform * mInitialPose;
  for (auto obj : mObjects) {
    obj->setPosition({pose.p.x, pose.p.y, pose.p.z});
    obj->setRotation({pose.q.w, pose.q.x, pose.q.y, pose.q.z});
  }
}

void SVulkan2Rigidbody::destroy() { mParentScene->removeRigidbody(this); }
void SVulkan2Rigidbody::destroyVisualObjects() {
  for (auto obj : mObjects) {
    mParentScene->getScene()->removeNode(*obj);
  }
}

void SVulkan2Rigidbody::setVisibility(float visibility) {
  for (auto obj : mObjects) {
    obj->setTransparency(1 - visibility);
  }
}

void SVulkan2Rigidbody::setVisible(bool visible) { setVisibility(visible ? 1.f : 0.f); }

void SVulkan2Rigidbody::setRenderMode(uint32_t mode) {
  if (mode == 0) {
    setVisibility(1.f);
    return;
  }
  if (mode == 1) {
    setVisibility(0.f);
    return;
  }
  if (mode == 2) {
    setVisibility(0.5f);
  }
}

void SVulkan2Rigidbody::setShadeFlat(bool shadeFlat) {
  for (auto obj : mObjects) {
    obj->setShadeFlat(shadeFlat);
  }
}
bool SVulkan2Rigidbody::getShadeFlat() { return mObjects.at(0)->getShadeFlat(); }

physx::PxVec3 SVulkan2Rigidbody::getScale() const { return mScale; }

std::vector<std::shared_ptr<IPxrRenderShape>> SVulkan2Rigidbody::getRenderShapes() {
  auto objects = getVisualObjects();
  std::vector<std::shared_ptr<IPxrRenderShape>> result;

  for (auto obj : objects) {
    for (auto &shape : obj->getModel()->getShapes()) {
      result.push_back(std::make_shared<SVulkan2RenderShape>(shape, this));
    }
  }

  return result;
}

} // namespace Renderer
} // namespace sapien
