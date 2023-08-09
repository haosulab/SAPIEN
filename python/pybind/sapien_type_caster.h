#pragma once
#include "sapien/math/math.h"
#include <pybind11/numpy.h>
#include <pybind11/smart_holder.h>

namespace py = pybind11;
using namespace sapien;

namespace pybind11::detail {
template <> struct type_caster<Vec3> {
  PYBIND11_TYPE_CASTER(Vec3, _("numpy.ndarray[numpy.float32, _Shape[3]]"));

  bool load(py::handle src, bool convert) {
    if (!convert && !py::array_t<float>::check_(src))
      return false;
    auto buf = py::array_t<float, py::array::c_style | py::array::forcecast>::ensure(src);
    if (!buf)
      return false;
    if (buf.ndim() != 1 || buf.shape(0) != 3) {
      return false;
    }
    value = Vec3(buf.at(0), buf.at(1), buf.at(2));
    return true;
  }

  static py::handle cast(Vec3 const &src, py::return_value_policy policy, py::handle parent) {
    return py::array_t<float>(3, &src.x).release();
  }
};

template <> struct type_caster<Quat> {
  PYBIND11_TYPE_CASTER(Quat, _("numpy.ndarray[numpy.float32, _Shape[4]]"));

  bool load(py::handle src, bool convert) {
    if (!convert && !py::array_t<float>::check_(src))
      return false;
    auto buf = py::array_t<float, py::array::c_style | py::array::forcecast>::ensure(src);
    if (!buf)
      return false;
    if (buf.ndim() != 1 || buf.shape(0) != 4) {
      return false;
    }
    value = Quat(buf.at(0), buf.at(1), buf.at(2), buf.at(3));
    return true;
  }

  static py::handle cast(Quat const &src, py::return_value_policy policy, py::handle parent) {
    return py::array_t<float>(4, &src.w).release();
  }
};
} // namespace pybind11::detail
