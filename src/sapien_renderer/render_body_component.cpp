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
#include "sapien/sapien_renderer/render_body_component.h"
#include "sapien/entity.h"
#include "sapien/profiler.h"
#include "sapien/sapien_renderer/render_shape.h"
#include "sapien/sapien_renderer/sapien_renderer_system.h"
#include "sapien/scene.h"

namespace sapien {
namespace sapien_renderer {

SapienRenderBodyComponent::SapienRenderBodyComponent() {}

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

AABB SapienRenderBodyComponent::getGlobalAABBFast() const {
  auto shapes = getRenderShapes();
  if (shapes.size() == 0) {
    throw std::runtime_error("afiled to get bounding box: no collision shapes attached");
  }
  AABB aabb = shapes.at(0)->getGlobalAABBFast();
  for (uint32_t i = 1; i < shapes.size(); ++i) {
    aabb = aabb + shapes[i]->getGlobalAABBFast();
  }
  return aabb;
}
AABB SapienRenderBodyComponent::computeGlobalAABBTight() const {
  auto shapes = getRenderShapes();
  if (shapes.size() == 0) {
    throw std::runtime_error("afiled to get bounding box: no collision shapes attached");
  }
  AABB aabb = shapes.at(0)->computeGlobalAABBTight();
  for (uint32_t i = 1; i < shapes.size(); ++i) {
    aabb = aabb + shapes[i]->computeGlobalAABBTight();
  }
  return aabb;
}

void SapienRenderBodyComponent::onAddToScene(Scene &scene) {
  auto system = scene.getSapienRendererSystem();
  auto s = system->getScene();
  mNode = &s->addNode();
  for (auto &shape : mRenderShapes) {
    auto &obj = s->addObject(*mNode, shape->getModel());
    obj.setTransform(shape->getLocalTransform());
    obj.setFrontFace(shape->getFrontFace());

    uint64_t id = mRenderIdDisabled ? 0 : system->nextRenderId();
    shape->internalSetRenderId(id);
    shape->internalSetRenderObject(&obj);

    obj.setSegmentation({id, getEntity()->getPerSceneId(), scene.getId(), 0});
    obj.setTransparency(1.f - mVisibility);
  }

  // register
  system->registerComponent(
      std::static_pointer_cast<SapienRenderBodyComponent>(shared_from_this()));
}

void SapienRenderBodyComponent::onRemoveFromScene(Scene &scene) {
  auto system = scene.getSapienRendererSystem();
  auto s = system->getScene();
  for (auto &s : mRenderShapes) {
    s->internalSetRenderId(0);
    s->internalSetRenderObject(nullptr);
  }
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

void SapienRenderBodyComponent::internalUpdate() {
  auto pose = getEntity()->getPose();
  mNode->setPosition({pose.p.x, pose.p.y, pose.p.z});
  mNode->setRotation({pose.q.w, pose.q.x, pose.q.y, pose.q.z});
}

SapienRenderBodyComponent::~SapienRenderBodyComponent() {
  for (auto s : mRenderShapes) {
    s->internalSetParent(nullptr);
  }
}

void SapienRenderBodyComponent::disableRenderId() {
  if (getScene()) {
    throw std::runtime_error("failed to disable render id: component already added to scene");
  }
  mRenderIdDisabled = true;
}
void SapienRenderBodyComponent::enableRenderId() {
  if (getScene()) {
    throw std::runtime_error("failed to disable render id: component already added to scene");
  }
  mRenderIdDisabled = false;
}

std::shared_ptr<SapienRenderBodyComponent> SapienRenderBodyComponent::clone() const {
  SAPIEN_PROFILE_FUNCTION;

  auto body = std::make_shared<SapienRenderBodyComponent>();
  for (auto s : getRenderShapes()) {
    body->attachRenderShape(s->clone());
  }
  body->setVisibility(getVisibility());
  body->setShadingMode(getShadingMode());
  body->mRenderIdDisabled = mRenderIdDisabled;
  // TODO copy properties
  return body;
}

} // namespace sapien_renderer
} // namespace sapien
