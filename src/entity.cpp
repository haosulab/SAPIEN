#include "sapien/entity.h"
#include "sapien/component/component.h"
#include "sapien/scene.h"

namespace sapien {

static uint64_t gNextEntityId = 1ul;

Entity::Entity() : mId(gNextEntityId++) {}

std::shared_ptr<Scene> Entity::getScene() { return mScene ? mScene->shared_from_this() : nullptr; }

void Entity::setPose(Pose const &pose) {
  mPose = pose;

  // TODO: defer the sync?
  for (auto &c : mComponents) {
    c->onSetPose(mPose);
  }
}

std::shared_ptr<Entity> Entity::addComponent(std::shared_ptr<component::Component> component) {
  if (component->getEntity()) {
    throw std::runtime_error("failed to add component: component already added.");
  }

  component->internalSetEntity(shared_from_this());
  mComponents.push_back(component);
  component->onSetPose(mPose);

  if (mScene) {
    component->onAddToScene(*mScene);
  }
  return shared_from_this();
}

void Entity::internalSwapInComponent(uint32_t index,
                                     std::shared_ptr<component::Component> component) {
  if (component->getEntity()) {
    throw std::runtime_error("failed to add component: component already added.");
  }
  component->internalSetEntity(shared_from_this());
  mComponents[index] = component;
  if (mScene) {
    component->onAddToScene(*mScene);
  }
}

void Entity::removeComponent(std::shared_ptr<component::Component> component) {
  if (component->getEntity().get() != this) {
    throw std::runtime_error("failed to add component: component already added.");
  }

  if (mScene) {
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

} // namespace sapien
