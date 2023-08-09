#include "./serialization.h"
#include "sapien/component/component.h"
#include "sapien/component/physx/physx.h"
#include "sapien/component/sapien_renderer/sapien_renderer.h"
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/smart_holder.h>
#include <pybind11/stl.h>

namespace py = pybind11;

using namespace sapien;

std::string serializeScene(std::shared_ptr<Scene> const &scene) {
  std::stringstream os;
  cereal::BinaryOutputArchive archive(os);
  archive(scene);
  auto string = os.str();
  return {string.begin(), string.end()};
}

std::string serializeEntity(std::shared_ptr<Entity> const &entity) {
  std::stringstream os;
  cereal::BinaryOutputArchive archive(os);
  archive(entity);
  auto string = os.str();
  return {string.begin(), string.end()};
}

std::string serializeEntityGroup(std::vector<std::shared_ptr<Entity>> const &entities) {
  std::stringstream os;
  cereal::BinaryOutputArchive archive(os);
  archive(entities);
  auto string = os.str();
  return {string.begin(), string.end()};
}

std::string serializeArticulationEntityGroup(
    std::shared_ptr<component::PhysxArticulation> const &articulation) {
  std::vector<std::shared_ptr<Entity>> entities;
  auto links = articulation->getLinks();
  for (auto l : links) {
    if (!l->getEntity()) {
      throw std::runtime_error(
          "failed to serialize articulation entity group: some link is not attached to an entity");
    }
    entities.push_back(l->getEntity());
  }
  return serializeEntityGroup(entities);
}

std::string serializeComponent(std::shared_ptr<component::Component> const &component) {
  std::stringstream os;
  cereal::BinaryOutputArchive archive(os);
  archive(component);
  auto string = os.str();
  return {string.begin(), string.end()};
}

std::shared_ptr<Scene> unserializeScene(std::string const &data) {
  std::stringstream is(std::string(data.begin(), data.end()));
  cereal::BinaryInputArchive archive(is);
  std::shared_ptr<Scene> scene;
  archive(scene);
  return scene;
}

std::shared_ptr<Entity> unserializeEntity(std::string const &data) {
  std::stringstream is(std::string(data.begin(), data.end()));
  cereal::BinaryInputArchive archive(is);
  std::shared_ptr<Entity> entity;
  archive(entity);
  return entity;
}

std::vector<std::shared_ptr<Entity>> unserializeEntityGroup(std::string const &data) {
  std::stringstream is(std::string(data.begin(), data.end()));
  cereal::BinaryInputArchive archive(is);
  std::vector<std::shared_ptr<Entity>> entities;
  archive(entities);
  return entities;
}

std::shared_ptr<component::Component> unserializeComponent(std::string const &data) {
  std::stringstream is(std::string(data.begin(), data.end()));
  cereal::BinaryInputArchive archive(is);
  std::shared_ptr<component::Component> component;
  archive(component);
  return component;
}

template <typename T>
std::function<py::bytes(T const &)> byteWrapper(std::function<std::string(T const &)> func) {
  return [=](T const &data) { return py::bytes(func(data)); };
}

template <typename T>
std::function<T(py::bytes)> byteUnwrapper(std::function<T(std::string)> func) {
  return [=](py::bytes data) { return (func(std::string(data))); };
}

void init_serialization(py::module &m) {
  m.def_submodule("_serialization")
      .def("_serialize_scene", byteWrapper<std::shared_ptr<Scene>>(serializeScene))
      .def("_serialize_entity", byteWrapper<std::shared_ptr<Entity>>(serializeEntity))
      .def("_serialize_entity_group",
           byteWrapper<std::vector<std::shared_ptr<Entity>>>(serializeEntityGroup))
      .def("_serialize_articulation_entity_group",
           byteWrapper<std::shared_ptr<component::PhysxArticulation>>(
               serializeArticulationEntityGroup))
      .def("_serialize_component",
           byteWrapper<std::shared_ptr<component::Component>>(serializeComponent))
      .def("_unserialize_scene", byteUnwrapper<std::shared_ptr<Scene>>(unserializeScene))
      .def("_unserialize_entity", byteUnwrapper<std::shared_ptr<Entity>>(unserializeEntity))
      .def("_unserialize_component",
           byteUnwrapper<std::shared_ptr<component::Component>>(unserializeComponent))
      .def("_unserialize_entity_group",
           byteUnwrapper<std::vector<std::shared_ptr<Entity>>>(unserializeEntityGroup));
}
