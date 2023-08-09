#pragma once
#include "sapien/array.h"
#include <pybind11/numpy.h>
#include <pybind11/smart_holder.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace sapien;

namespace pybind11::detail {
template <> struct type_caster<CpuArrayHandle> {
  PYBIND11_TYPE_CASTER(CpuArrayHandle, _("numpy.ndarray"));

  bool load(py::handle src, bool convert) { return false; }

  static py::handle cast(CpuArrayHandle const &src, py::return_value_policy policy,
                         py::handle parent) {

    std::vector<ssize_t> shape;
    std::vector<ssize_t> strides;
    for (auto s : src.shape) {
      shape.push_back(s);
    }
    for (auto s : src.strides) {
      strides.push_back(s);
    }

    // this makes a copy
    auto array = py::array(py::dtype(src.type), shape, strides, src.ptr);

    // this makes array in-place
    // assert(static_cast<const void *>(array.request().ptr) == static_cast<const void
    // *>(src.ptr));

    return array.release();
  }
};

template <> struct type_caster<CpuArray> {
  PYBIND11_TYPE_CASTER(CpuArray, _("numpy.ndarray"));

  bool load(py::handle src, bool convert) { return false; }

  static py::handle cast(CpuArray const &src, py::return_value_policy policy, py::handle parent) {

    std::vector<ssize_t> shape;
    std::vector<ssize_t> strides;
    for (auto s : src.shape) {
      shape.push_back(s);
    }
    for (auto s : src.strides) {
      strides.push_back(s);
    }

    auto array = py::array(py::dtype(src.type), shape, strides, src.data.data());
    return array.release();
  }
};

} // namespace pybind11::detail
