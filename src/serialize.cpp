#include "sapien/serialize.h"
#include "sapien/component/physx/physx.h"
#include "sapien/component/sapien_renderer/sapien_renderer.h"
#include "sapien/scene.h"
#include <sstream>

namespace sapien {

// std::string serializeScene(std::shared_ptr<Scene> scene) {
//   std::stringstream os;
//   cereal::BinaryOutputArchive archive(os);
//   archive(scene);
//   auto string = os.str();
//   return {string.begin(), string.end()};
// }

// std::string serializeEntity(std::shared_ptr<Entity> entity) {
//   std::stringstream os;
//   cereal::BinaryOutputArchive archive(os);
//   archive(entity);
//   auto string = os.str();
//   return {string.begin(), string.end()};
// }

// std::string serializeEntityGroup(std::vector<std::shared_ptr<Entity>> const &entities) {
//   std::stringstream os;
//   cereal::BinaryOutputArchive archive(os);
//   archive(entities);
//   auto string = os.str();
//   return {string.begin(), string.end()};
// }

// std::string
// serializeArticulationEntityGroup(std::shared_ptr<component::PhysxArticulation> articulation) {
//   std::vector<std::shared_ptr<Entity>> entities;
//   auto links = articulation->getLinks();
//   for (auto l : links) {
//     if (!l->getEntity()) {
//       throw std::runtime_error(
//           "failed to serialize articulation entity group: some link is not attached to an entity");
//     }
//     entities.push_back(l->getEntity());
//   }
//   return serializeEntityGroup(entities);
// }

// std::string serializeComponent(std::shared_ptr<component::Component> component) {
//   std::stringstream os;
//   cereal::BinaryOutputArchive archive(os);
//   archive(component);
//   auto string = os.str();
//   return {string.begin(), string.end()};
// }

// std::shared_ptr<Scene> unserializeScene(std::string const &data) {
//   std::stringstream is(std::string(data.begin(), data.end()));
//   cereal::BinaryInputArchive archive(is);
//   std::shared_ptr<Scene> scene;
//   archive(scene);
//   return scene;
// }

// std::shared_ptr<Entity> unserializeEntity(std::string const &data) {
//   std::stringstream is(std::string(data.begin(), data.end()));
//   cereal::BinaryInputArchive archive(is);
//   std::shared_ptr<Entity> entity;
//   archive(entity);
//   return entity;
// }

// std::vector<std::shared_ptr<Entity>> unserializeEntityGroup(std::string const &data) {
//   std::stringstream is(std::string(data.begin(), data.end()));
//   cereal::BinaryInputArchive archive(is);
//   std::vector<std::shared_ptr<Entity>> entities;
//   archive(entities);
//   return entities;
// }

// std::shared_ptr<component::Component> unserializeComponent(std::string const &data) {
//   std::stringstream is(std::string(data.begin(), data.end()));
//   cereal::BinaryInputArchive archive(is);
//   std::shared_ptr<component::Component> component;
//   archive(component);
//   return component;
// }

} // namespace sapien
