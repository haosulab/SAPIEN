#include "svulkan2_renderer.h"
#include <iostream>

namespace sapien {
namespace Renderer {

SVulkan2Rigidbody::SVulkan2Rigidbody(SVulkan2Scene *scene,
                                     std::vector<svulkan2::scene::Object *> const &objects)
    : mParentScene(scene), mObjects(objects) {}

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

std::vector<std::unique_ptr<RenderShape>> SVulkan2Rigidbody::getRenderShapes() const {
  auto objects = getVisualObjects();
  std::vector<std::unique_ptr<RenderShape>> result;
  for (auto obj : objects) {
    for (auto &shape : obj->getModel()->getShapes()) {
      result.emplace_back(std::make_unique<RenderShape>());
      RenderShape *renderShape = result.back().get();
      renderShape->type = "mesh";
      renderShape->pose = getInitialPose();
      auto &T = obj->getTransform();
      renderShape->scale = {T.scale.x, T.scale.y, T.scale.z};
      renderShape->objId = obj->getSegmentation()[0];

      auto position = shape->mesh->getVertexAttribute("position");
      auto index = shape->mesh->getIndices();
      auto mesh = std::make_unique<RenderMeshGeometry>();
      mesh->vertices = position;

      if (shape->mesh->hasVertexAttribute("normal")) {
        mesh->normals = shape->mesh->getVertexAttribute("normal");
      }
      if (shape->mesh->hasVertexAttribute("tangent")) {
        mesh->tangents = shape->mesh->getVertexAttribute("tangent");
      }
      if (shape->mesh->hasVertexAttribute("bitangent")) {
        mesh->bitangents = shape->mesh->getVertexAttribute("bitangent");
      }
      if (shape->mesh->hasVertexAttribute("uv")) {
        mesh->bitangents = shape->mesh->getVertexAttribute("uv");
      }
      mesh->indices = index;
      renderShape->geometry = std::move(mesh);
    }
  }
  return result;
}

} // namespace Renderer
} // namespace sapien
