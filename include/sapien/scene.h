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
#pragma once
#include "entity.h"
#include "system.h"
#include <typeindex>
#include <unordered_map>

namespace sapien {

namespace physx {
class PhysxSystem;
} // namespace physx

namespace sapien_renderer {
class SapienRendererSystem;
}

class Scene : public std::enable_shared_from_this<Scene> {
  static uint64_t gNextSceneId;

public:
  Scene(std::vector<std::shared_ptr<System>> const &systems = {});

  void addEntity(std::shared_ptr<Entity> entity);
  void removeEntity(std::shared_ptr<Entity> entity);

  void addSystem(std::shared_ptr<System> system);
  std::shared_ptr<System> getSystem(std::string const &name) const;

  template <class T> std::shared_ptr<T> getSystemWithType(std::string const &name) const {
    if (auto s = std::dynamic_pointer_cast<T>(getSystem(name))) {
      return s;
    }
    throw std::runtime_error("failed to get system: type mismatch");
  }

  std::vector<std::shared_ptr<Entity>> getEntities() const { return mEntities; };

  // for convenience
  std::shared_ptr<physx::PhysxSystem> getPhysxSystem() const;
  void step();

  std::shared_ptr<sapien_renderer::SapienRendererSystem> getSapienRendererSystem() const;
  void updateRender();

  std::string packEntityPoses();
  void unpackEntityPoses(std::string const &data);

  uint64_t getId() const { return mId; };

  void clear();

  ~Scene();
  Scene(Scene const &) = delete;
  Scene &operator=(Scene const &) = delete;
  Scene(Scene const &&) = delete;
  Scene &operator=(Scene const &&) = delete;

private:
  uint64_t mNextEntityId{1};

  uint64_t mId{};
  std::unordered_map<std::string, std::shared_ptr<System>> mSystems;
  std::vector<std::shared_ptr<Entity>> mEntities;
};

} // namespace sapien
