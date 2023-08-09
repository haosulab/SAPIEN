#include "sapien/component/sapien_renderer/render_body_component.h"
#include "sapien/component/sapien_renderer/render_shape.h"
#include "sapien/component/sapien_renderer/sapien_renderer_system.h"
#include "sapien/entity.h"
#include "sapien/scene.h"

namespace sapien {
namespace component {

uint64_t SapienRenderBodyComponent::gNextVisualId = 1ul;

SapienRenderBodyComponent::SapienRenderBodyComponent() : mVisualId(gNextVisualId++) {}

std::shared_ptr<SapienRenderBodyComponent>
SapienRenderBodyComponent::attachRenderShape(std::shared_ptr<RenderShape> shape) {
  if (getEntity()) {
    throw std::runtime_error("failed to attach visual shape: adding shape to render body that is "
                             "already part of an entity is not implemented.");
  }
  mRenderShapes.push_back(shape);
  shape->internalSetParent(this);

  return std::static_pointer_cast<SapienRenderBodyComponent>(shared_from_this());
}

void SapienRenderBodyComponent::onAddToScene(Scene &scene) {
  auto system = scene.getSapienRendererSystem();
  auto s = system->getScene();
  mNode = &s->addNode();
  for (auto &shape : mRenderShapes) {
    auto &obj = s->addObject(*mNode, shape->getModel());
    obj.setTransform(shape->getLocalTransform());
    obj.setSegmentation({mVisualId, getEntity()->getId(), 0, 0});
    obj.setTransparency(1.f - mVisibility);
  }

  // register
  system->registerComponent(
      std::static_pointer_cast<SapienRenderBodyComponent>(shared_from_this()));
}

void SapienRenderBodyComponent::onRemoveFromScene(Scene &scene) {
  auto system = scene.getSapienRendererSystem();
  auto s = system->getScene();
  s->removeNode(*mNode);
  mNode = nullptr;

  system->unregisterComponent(
      std::static_pointer_cast<SapienRenderBodyComponent>(shared_from_this()));
}

void SapienRenderBodyComponent::setVisibility(float v) {
  mVisibility = v;
  if (mNode) {
    for (auto c : mNode->getChildren()) {
      if (auto o = dynamic_cast<svulkan2::scene::Object *>(c)) {
        o->setTransparency(1.f - v);
      }
    }
  }
}
float SapienRenderBodyComponent::getVisibility() const { return mVisibility; }

void SapienRenderBodyComponent::setShadingMode(int mode) {
  mShadingMode = mode;
  if (mNode) {
    for (auto c : mNode->getChildren()) {
      if (auto o = dynamic_cast<svulkan2::scene::Object *>(c)) {
        o->setShadingMode(mode);
      }
    }
  }
}
int SapienRenderBodyComponent::getShadingMode() const { return mShadingMode; }

void SapienRenderBodyComponent::setProperty(std::string const &name, int value) {
  if (!mNode) {
    throw std::runtime_error("object property in only available when added to scene");
  }
  for (auto &n : mNode->getChildren()) {
    if (auto o = dynamic_cast<svulkan2::scene::Object *>(n)) {
      o->setCustomDataInt(name, value);
    }
  }
}
void SapienRenderBodyComponent::setProperty(std::string const &name, float value) {
  if (!mNode) {
    throw std::runtime_error("object property in only available when added to scene");
  }
  for (auto &n : mNode->getChildren()) {
    if (auto o = dynamic_cast<svulkan2::scene::Object *>(n)) {
      o->setCustomDataFloat(name, value);
    }
  }
}
void SapienRenderBodyComponent::setProperty(std::string const &name, Vec3 const &value) {
  if (!mNode) {
    throw std::runtime_error("object property in only available when added to scene");
  }
  for (auto &n : mNode->getChildren()) {
    if (auto o = dynamic_cast<svulkan2::scene::Object *>(n)) {
      o->setCustomDataFloat3(name, {value.x, value.y, value.z});
    }
  }
}

void SapienRenderBodyComponent::setTexture(std::string const &name,
                                           std::shared_ptr<SapienRenderTexture> texture) {
  if (!mNode) {
    throw std::runtime_error("object texture in only available when added to scene");
  }
  for (auto &n : mNode->getChildren()) {
    if (auto o = dynamic_cast<svulkan2::scene::Object *>(n)) {
      o->setCustomTexture(name, texture->getTexture());
    }
  }
}
void SapienRenderBodyComponent::setTextureArray(
    std::string const &name, std::vector<std::shared_ptr<SapienRenderTexture>> textures) {
  if (!mNode) {
    throw std::runtime_error("object texture in only available when added to scene");
  }

  std::vector<std::shared_ptr<svulkan2::resource::SVTexture>> ts;
  for (auto &t : textures) {
    ts.push_back(t->getTexture());
  }

  for (auto &n : mNode->getChildren()) {
    if (auto o = dynamic_cast<svulkan2::scene::Object *>(n)) {
      o->setCustomTextureArray(name, ts);
    }
  }
}

AABB SapienRenderBodyComponent::getGlobalAABBFast() {
  AABB aabb;
  for (auto &s : getRenderShapes()) {
    aabb = aabb + s->getGlobalAABBFast();
  }
  return aabb;
}

AABB SapienRenderBodyComponent::computeGlobalAABBTight() {
  AABB aabb;
  for (auto &s : getRenderShapes()) {
    aabb = aabb + s->computeGlobalAABBTight();
  }
  return aabb;
}

void SapienRenderBodyComponent::internalUpdate() {
  auto pose = getEntity()->getPose();
  mNode->setPosition({pose.p.x, pose.p.y, pose.p.z});
  mNode->setRotation({pose.q.w, pose.q.x, pose.q.y, pose.q.z});
}

SapienRenderBodyComponent::~SapienRenderBodyComponent() {}

} // namespace component
} // namespace sapien
