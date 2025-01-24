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
#include "sapien/component.h"
#include "sapien/entity.h"
#include "sapien/scene.h"

namespace sapien {

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

} // namespace sapien
