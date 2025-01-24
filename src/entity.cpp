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
#include "sapien/entity.h"
#include "./logger.h"
#include "sapien/component.h"
#include "sapien/scene.h"

namespace sapien {

static uint64_t gNextEntityId = 1ul;
static uint64_t gEntityCount = 0;

Entity::Entity() : mId(gNextEntityId++) {
  gEntityCount++;
  logger::info("Created Entity {}, total {}", mId, gEntityCount);
}

std::shared_ptr<Scene> Entity::getScene() { return mScene ? mScene->shared_from_this() : nullptr; }

void Entity::setPose(Pose const &pose) {
  mPose = pose;

  // TODO: defer the sync?
  for (auto &c : mComponents) {
    c->onSetPose(mPose);
  }
}

std::shared_ptr<Entity> Entity::addComponent(std::shared_ptr<Component> component) {
  if (component->getEntity()) {
    throw std::runtime_error("failed to add component: component already added.");
  }

  component->internalSetEntity(shared_from_this());
  mComponents.push_back(component);
  component->onSetPose(mPose);

  if (mScene && component->getEnabled()) {
    component->onAddToScene(*mScene);
  }
  return shared_from_this();
}

void Entity::internalSwapInComponent(uint32_t index, std::shared_ptr<Component> component) {
  if (component->getEntity()) {
    throw std::runtime_error("failed to add component: component already added.");
  }
  component->internalSetEntity(shared_from_this());
  mComponents[index] = component;
  if (mScene) {
    component->onAddToScene(*mScene);
  }
}

void Entity::removeComponent(std::shared_ptr<Component> component) {
  if (component->getEntity().get() != this) {
    throw std::runtime_error("failed to add component: component already added.");
  }

  if (mScene && component->getEnabled()) {
    component->onRemoveFromScene(*mScene);
  }

  std::erase(mComponents, component);
  component->internalSetEntity(nullptr);
}

void Entity::onAddToScene(Scene &scene) {
  for (auto c : mComponents) {
    if (c->getEnabled()) {
      c->onAddToScene(scene);
    }
  }
}
void Entity::onRemoveFromScene(Scene &scene) {
  for (auto it = mComponents.rbegin(); it != mComponents.rend(); ++it) {
    auto &c = *it;
    if (c->getEnabled()) {
      c->onRemoveFromScene(scene);
    }
  }
}

Pose Entity::getPose() const { return mPose; }
void Entity::internalSyncPose(Pose const &pose) { mPose = pose; }

/** same as Scene::addEntity */
std::shared_ptr<Entity> Entity::addToScene(Scene &scene) {
  scene.addEntity(shared_from_this());
  return shared_from_this();
}
void Entity::removeFromScene() {
  if (!mScene) {
    throw std::runtime_error(
        "failed to remove entity from scene: the entity is not added to any scene.");
  }
  mScene->removeEntity(shared_from_this());
}

Entity::~Entity() {
  gEntityCount--;
  logger::info("Deleting Entity {}, total {}", mId, gEntityCount);
}

} // namespace sapien
