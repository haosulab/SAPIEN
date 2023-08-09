#pragma once
#include <Eigen/Eigen>
#include <cereal/archives/binary.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/types/vector.hpp>

namespace cereal {
template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows,
          int _MaxCols>
void save(Archive &ar,
          Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> const &m) {
  int32_t rows = m.rows();
  int32_t cols = m.cols();
  ar(rows);
  ar(cols);
  ar(binary_data(m.data(), rows * cols * sizeof(_Scalar)));
}

template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows,
          int _MaxCols>
void load(Archive &ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &m) {
  int32_t rows;
  int32_t cols;
  ar(rows);
  ar(cols);
  m.resize(rows, cols);
  ar(binary_data(m.data(), static_cast<std::size_t>(rows * cols * sizeof(_Scalar))));
}
} // namespace cereal

namespace sapien {
class Scene;
class Entity;

namespace component {
class Component;
class PhysxArticulation;
} // namespace component

// std::string serializeScene(std::shared_ptr<Scene> scene);
// std::shared_ptr<Scene> unserializeScene(std::string const &data);

// std::string serializeEntity(std::shared_ptr<Entity> entity);
// std::shared_ptr<Entity> unserializeEntity(std::string const &data);

// std::string serializeEntityGroup(std::vector<std::shared_ptr<Entity>> const &entities);
// std::vector<std::shared_ptr<Entity>> unserializeEntityGroup(std::string const &data);

// std::string
// serializeArticulationEntityGroup(std::shared_ptr<component::PhysxArticulation> articulation);

// std::string serializeComponent(std::shared_ptr<component::Component> component);
// std::shared_ptr<component::Component> unserializeComponent(std::string const &data);

} // namespace sapien

