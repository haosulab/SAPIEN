#ifdef _USE_VULKAN
#include "sapien_vulkan_renderer.h"
#include <iostream>

namespace sapien {
namespace Renderer {

SapienVulkanRigidbody::SapienVulkanRigidbody(SapienVulkanScene *scene,
                                             std::vector<svulkan::Object *> const &objects)
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
  for (auto obj : mObjects) {
    obj->setUserData(customData);
  }
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

std::vector<std::unique_ptr<RenderShape>> SapienVulkanRigidbody::getRenderShapes() const {
  auto objects = getVisualObjects();
  auto context = mParentScene->getParentRenderer()->mContext.get();
  std::vector<std::unique_ptr<RenderShape>> result;
  for (auto obj : objects) {
    result.emplace_back(std::make_unique<RenderShape>());
    RenderShape *shape = result.back().get();
    shape->type = "mesh";
    shape->pose = getInitialPose();
    shape->scale = {
        obj->mTransform.scale.x,
        obj->mTransform.scale.y,
        obj->mTransform.scale.z,
    };

    auto vertices = obj->getVulkanObject()->mMesh->downloadVertices(
        context->getPhysicalDevice(), context->getDevice(), context->getCommandPool(),
        context->getGraphicsQueue());
    auto indices = obj->getVulkanObject()->mMesh->downloadIndices(
        context->getPhysicalDevice(), context->getDevice(), context->getCommandPool(),
        context->getGraphicsQueue());
    auto mesh = std::make_unique<RenderMeshGeometry>();
    for (auto &v : vertices) {
      mesh->vertices.push_back(v.position[0]);
      mesh->vertices.push_back(v.position[1]);
      mesh->vertices.push_back(v.position[2]);
      mesh->normals.push_back(v.normal[0]);
      mesh->normals.push_back(v.normal[1]);
      mesh->normals.push_back(v.normal[2]);
      mesh->uvs.push_back(v.uv[0]);
      mesh->uvs.push_back(v.uv[1]);
      mesh->tangents.push_back(v.tangent[0]);
      mesh->tangents.push_back(v.tangent[1]);
      mesh->tangents.push_back(v.tangent[2]);
      mesh->bitangents.push_back(v.bitangent[0]);
      mesh->bitangents.push_back(v.bitangent[1]);
      mesh->bitangents.push_back(v.bitangent[2]);
    }
    mesh->indices = indices;

    shape->geometry = std::move(mesh);
  }
  return result;
}

} // namespace Renderer
} // namespace sapien
#endif
