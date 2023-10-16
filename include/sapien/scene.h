#pragma once
#include "entity.h"
#include "sapien/component/system.h"
#include <cereal/types/unordered_map.hpp>
#include <typeindex>
#include <unordered_map>

namespace sapien {

namespace component {
class PhysxSystem;
class SapienRendererSystem;
} // namespace component

class Scene : public std::enable_shared_from_this<Scene> {
public:
  Scene(std::vector<std::shared_ptr<component::System>> const &systems = {});

  void addEntity(std::shared_ptr<Entity> entity);
  void removeEntity(std::shared_ptr<Entity> entity);

  void addSystem(std::shared_ptr<component::System> system);
  std::shared_ptr<component::System> getSystem(std::string const &name) const;

  std::vector<std::shared_ptr<Entity>> getEntities() const { return mEntities; };

  // for convenience
  std::shared_ptr<component::PhysxSystem> getPhysxSystem() const;
  void step();

  std::shared_ptr<component::SapienRendererSystem> getSapienRendererSystem() const;
  void updateRender();

  std::string packEntityPoses();
  void unpackEntityPoses(std::string const &data);

  template <class Archive> void save(Archive &ar) const { ar(mSystems, mEntities); }
  template <class Archive> void load(Archive &ar) {
    std::unordered_map<std::string, std::shared_ptr<component::System>> systems;
    std::vector<std::shared_ptr<Entity>> entities;
    ar(systems, entities);

    for (auto &[n, s] : systems) {
      addSystem(s);
    }
    for (auto &e : entities) {
      addEntity(e);
    }
  }

  ~Scene();
  Scene(Scene const &) = delete;
  Scene &operator=(Scene const &) = delete;
  Scene(Scene const &&) = delete;
  Scene &operator=(Scene const &&) = delete;

private:
  std::unordered_map<std::string, std::shared_ptr<component::System>> mSystems;
  std::vector<std::shared_ptr<Entity>> mEntities;
};

} // namespace sapien
