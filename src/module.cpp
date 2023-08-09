// #include "sapien/module.h"
// #include "sapien/scene.h"

// namespace sapien {

// std::shared_ptr<Scene> Module::getScene() const { return mScene->shared_from_this(); }
// void Module::internalSetScene(Scene *scene) { mScene = scene; }

// void Module::addEntity(std::shared_ptr<Entity> entity) {
//   if (entity->getScene() != getScene()) {
//     throw std::runtime_error(
//         "entity can only be added to a module when they are in the same scene");
//   }
//   mEntities.push_back(entity);
// }
// void Module::removeEntity(std::shared_ptr<Entity> entity) {
//   if (std::erase(mEntities, entity) == 0) {
//     throw std::runtime_error("entity does not belong to this module");
//   }
// }

// } // namespace sapien
