#ifdef _USE_VULKAN
#include "sapien_vulkan_renderer.h"
#include <iostream>

namespace sapien {
namespace Renderer {

SapienVulkanRigidbody::SapienVulkanRigidbody(
    SapienVulkanScene *scene, std::vector<svulkan::Object *> const &objects)
    : mParentScene(scene), mObjects(objects) {}

void SapienVulkanRigidbody::setUniqueId(uint32_t uniqueId) {
  mUniqueId = uniqueId;
  for (auto obj : mObjects) {
    obj->setObjectId(uniqueId);
  }
}

uint32_t SapienVulkanRigidbody::getUniqueId() const { return mUniqueId; }

void SapienVulkanRigidbody::setSegmentationId(uint32_t segmentationId) {
  mSegmentationId = segmentationId;
  for (auto obj : mObjects) {
    obj->setSegmentId(segmentationId);
  }
}

uint32_t SapienVulkanRigidbody::getSegmentationId() const { return mSegmentationId; }

void SapienVulkanRigidbody::setSegmentationCustomData(const std::vector<float> &customData) {
  std::cerr << "Custom data is not supported yet" << std::endl;
}

void SapienVulkanRigidbody::setInitialPose(const physx::PxTransform &transform) {
  mInitialPose = transform;
  update({{0, 0, 0}, physx::PxIdentity});
}

void SapienVulkanRigidbody::update(const physx::PxTransform &transform) {
  auto pose = transform * mInitialPose;
  for (auto obj : mObjects) {
    obj->mTransform.position = {pose.p.x, pose.p.y, pose.p.z};
    obj->mTransform.rotation = {pose.q.w, pose.q.x, pose.q.y, pose.q.z};
  }
}

void SapienVulkanRigidbody::destroy() { mParentScene->removeRigidbody(this); }
void SapienVulkanRigidbody::destroyVisualObjects() {
  for (auto obj : mObjects) {
    mParentScene->getScene()->removeObject(obj);
  }
}

void SapienVulkanRigidbody::setVisibility(float visibility) {
  for (auto obj : mObjects) {
    obj->mVisibility = visibility;
  }
}

void SapienVulkanRigidbody::setVisible(bool visible) {
  for (auto obj : mObjects) {
    obj->mVisibility = visible ? 1.f : 0.f;
  }
}

void SapienVulkanRigidbody::setRenderMode(uint32_t mode) {
  if (mode == 0) {
    for (auto obj : mObjects) {
      obj->mVisibility = 1.f;
    }
    return;
  }
  if (mode == 1) {
    for (auto obj : mObjects) {
      obj->mVisibility = 0.f;
    }
    return;
  }
  if (mode == 2) {
    for (auto obj : mObjects) {
      obj->mVisibility = 0.5f;
    }
  }
}

}
} // namespace sapien
#endif
