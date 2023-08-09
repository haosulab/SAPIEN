#include "sapien/scene.h"
#include "sapien/component/physx/physx_system.h"
#include "sapien/component/sapien_renderer/sapien_renderer.h"
#include "sapien/entity.h"

namespace sapien {

Scene::Scene(std::vector<std::shared_ptr<component::System>> const &systems) {
  for (auto s : systems) {
    addSystem(s);
  }
}

void Scene::addSystem(std::shared_ptr<component::System> system) {
  auto name = system->getName();
  if (mSystems.contains(name)) {
    throw std::runtime_error("faedil to add system: a system with name [" + name +
                             "] is already added to scene");
  }
  mSystems[name] = system;
}

std::shared_ptr<component::System> Scene::getSystem(std::string const &name) const {
  try {
    return mSystems.at(name);
  } catch (std::out_of_range &e) {
    throw std::runtime_error("failed to get system: no system with anem [" + name +
                             "] is added to scene");
  }
}

std::shared_ptr<component::PhysxSystem> Scene::getPhysxSystem() const {
  return std::dynamic_pointer_cast<component::PhysxSystem>(getSystem("physx"));
}

std::shared_ptr<component::SapienRendererSystem> Scene::getSapienRendererSystem() const {
  return std::dynamic_pointer_cast<component::SapienRendererSystem>(getSystem("render"));
}

void Scene::step() { getPhysxSystem()->step(); }
void Scene::updateRender() { getSapienRendererSystem()->step(); }

void Scene::addEntity(std::shared_ptr<Entity> entity) {
  if (entity->getScene()) {
    throw std::runtime_error("failed to add entity to scene: entity is already added to a scene.");
  }

  mEntities.push_back(entity);
  entity->internalSetScene(this);
  entity->onAddToScene(*this);
}

void Scene::removeEntity(std::shared_ptr<Entity> entity) {
  auto count = std::erase_if(mEntities, [=](auto &e) { return e == entity; });
  if (count == 0) {
    throw std::runtime_error("failed to remove entity: not added");
  }
  entity->onRemoveFromScene(*this);
  entity->internalSetScene(nullptr);
}

// void Scene::addModule(std::shared_ptr<Module> mod) {
//   if (mod->getScene()) {
//     throw std::runtime_error("failed to add entity to scene: entity is already added to a
//     scene.");
//   }
//   mModules.push_back(mod);
//   for (auto e : mod->getEntities()) {
//     addEntity(e);
//   }
// }

// void Scene::removeModule(std::shared_ptr<Module> mod) {
//   auto count = std::erase_if(mModules, [=](auto &m) { return m == mod; });
//   if (count == 0) {
//     throw std::runtime_error("failed to remove module: not added");
//   }
//   for (auto e : mod->getEntities()) {
//     removeEntity(e);
//   }
//   mod->internalSetScene(nullptr);
// }

Scene::~Scene() {}

} // namespace sapien
