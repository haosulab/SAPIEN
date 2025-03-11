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
#include "sapien/array.h"
#ifdef SAPIEN_CUDA
#include <dlpack/dlpack.h>
#endif
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace sapien;

namespace sapien {

#ifdef SAPIEN_CUDA
inline py::capsule DLPackToCapsule(DLManagedTensor *tensor) {
  return py::capsule(tensor, "dltensor", [](PyObject *data) {
    auto tensor = static_cast<DLManagedTensor *>(PyCapsule_GetPointer(data, "dltensor"));
    if (tensor) {
      tensor->deleter(tensor);
    } else {
      PyErr_Clear();
    }
  });
}
#endif

} // namespace sapien

namespace pybind11::detail {

template <> struct type_caster<CpuArrayHandle> {
  PYBIND11_TYPE_CASTER(CpuArrayHandle, _("numpy.ndarray"));

  bool load(py::handle src, bool convert) { return false; }

  static py::handle cast(CpuArrayHandle const &src, py::return_value_policy policy,
                         py::handle parent) {

    std::vector<py::ssize_t> shape;
    std::vector<py::ssize_t> strides;
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

    std::vector<py::ssize_t> shape;
    for (auto s : src.shape) {
      shape.push_back(s);
    }

    auto array = py::array(py::dtype(src.type), shape, src.data.data());
    return array.release();
  }
};

} // namespace pybind11::detail
