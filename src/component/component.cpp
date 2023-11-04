#include "sapien/component/component.h"
#include "sapien/entity.h"
#include "sapien/scene.h"

namespace sapien::component {

static uint64_t gNextComponentId = 1ul;
Component::Component() : mId(gNextComponentId++) {}

std::shared_ptr<Scene> Component::getScene() {
  if (auto e = getEntity()) {
    return e->getScene();
  }
  return nullptr;
}

Pose Component::getPose() const {
  if (mEntity.expired()) {
    throw std::runtime_error("failed to get pose: component is not attached to an entity");
  }
  return getEntity()->getPose();
}

void Component::setPose(Pose const &pose) {
  if (mEntity.expired()) {
    throw std::runtime_error("failed to set pose: component is not attached to an entity");
  }
  getEntity()->setPose(pose);
}

void Component::enable() { setEnabled(true); }
void Component::disable() { setEnabled(false); }

void Component::setEnabled(bool enabled) {
  if (enabled == mEnabled) {
    return;
  }
  mEnabled = enabled;

  auto entity = getEntity();

  if (enabled && entity && entity->getScene()) {
    onAddToScene(*entity->getScene());
  }
  if (!enabled && entity && entity->getScene()) {
    onRemoveFromScene(*entity->getScene());
  }
}

} // namespace sapien::component
