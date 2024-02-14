#include "sapien/scene.h"
#include "./logger.h"
#include "sapien/entity.h"
#include "sapien/physx/physx_system.h"
#include "sapien/sapien_renderer/sapien_renderer.h"

namespace sapien {

uint64_t Scene::gNextSceneId = 1;

static uint64_t gSceneCount = 0;

Scene::Scene(std::vector<std::shared_ptr<System>> const &systems) : mId(gNextSceneId++) {
  for (auto s : systems) {
    addSystem(s);
  }
  gSceneCount++;
  logger::info("Created Scene {}, total {}", mId, gSceneCount);
}

void Scene::addSystem(std::shared_ptr<System> system) {
  auto name = system->getName();
  if (mSystems.contains(name)) {
    throw std::runtime_error("faedil to add system: a system with name [" + name +
                             "] is already added to scene");
  }
  mSystems[name] = system;
}

std::shared_ptr<System> Scene::getSystem(std::string const &name) const {
  try {
    return mSystems.at(name);
  } catch (std::out_of_range &e) {
    throw std::runtime_error("failed to get system: no system with anem [" + name +
                             "] is added to scene");
  }
}

std::shared_ptr<physx::PhysxSystem> Scene::getPhysxSystem() const {
  return std::dynamic_pointer_cast<physx::PhysxSystem>(getSystem("physx"));
}

std::shared_ptr<sapien_renderer::SapienRendererSystem> Scene::getSapienRendererSystem() const {
  return std::dynamic_pointer_cast<sapien_renderer::SapienRendererSystem>(getSystem("render"));
}

void Scene::step() { getPhysxSystem()->step(); }
void Scene::updateRender() { getSapienRendererSystem()->step(); }

void Scene::addEntity(std::shared_ptr<Entity> entity) {
  if (!entity) {
    throw std::runtime_error("failed to add entity to scene: entity is null");
  }

  if (entity->getScene()) {
    throw std::runtime_error("failed to add entity to scene: entity is already added to a scene.");
  }

  mEntities.push_back(entity);
  entity->internalSetScene(this);
  entity->internalSetPerSceneId(mNextEntityId++);
  entity->onAddToScene(*this);
}

void Scene::removeEntity(std::shared_ptr<Entity> entity) {
  auto count = std::erase_if(mEntities, [=](auto &e) { return e == entity; });
  if (count == 0) {
    throw std::runtime_error("failed to remove entity: not added");
  }
  entity->onRemoveFromScene(*this);
  entity->internalSetPerSceneId(0);
  entity->internalSetScene(nullptr);
}

std::string Scene::packEntityPoses() {
  std::ostringstream ss;
  for (auto e : mEntities) {
    Pose pose = e->getPose();
    ss.write(reinterpret_cast<char *>(&pose), sizeof(Pose));
  }
  return ss.str();
}

void Scene::unpackEntityPoses(std::string const &data) {
  std::istringstream ss(data);
  for (auto e : mEntities) {
    Pose pose;
    ss.read(reinterpret_cast<char *>(&pose), sizeof(Pose));
    e->internalSyncPose(pose);
  }
  // TODO: check nothing is left
}

void Scene::clear() {
  for (auto &entity : mEntities) {
    entity->onRemoveFromScene(*this);
    entity->internalSetPerSceneId(0);
    entity->internalSetScene(nullptr);
  }
  mEntities.clear();
}

Scene::~Scene() {
  gSceneCount--;
  logger::info("Deleting Scene {}, total {}", mId, gSceneCount);
  for (auto &entity : mEntities) {
    entity->onRemoveFromScene(*this);
    entity->internalSetPerSceneId(0);
    entity->internalSetScene(nullptr);
  }
  mEntities.clear();
}

} // namespace sapien
