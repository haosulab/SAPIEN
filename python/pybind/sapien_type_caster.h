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
#include "sapien/math/bounding_box.h"
#include "sapien/math/math.h"
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace sapien;

namespace pybind11::detail {
template <> struct type_caster<Vec3> {
  PYBIND11_TYPE_CASTER(Vec3, _("numpy.ndarray[typing.Literal[3], numpy.dtype[numpy.float32]]"));

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
  PYBIND11_TYPE_CASTER(Quat, _("numpy.ndarray[typing.Literal[4], numpy.dtype[numpy.float32]]"));

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

template <> struct type_caster<AABB> {
  PYBIND11_TYPE_CASTER(
      AABB,
      _("numpy.ndarray[tuple[typing.Literal[2], typing.Literal[3]], numpy.dtype[numpy.float32]]"));

  bool load(py::handle src, bool convert) { return false; }

  static py::handle cast(AABB const &src, py::return_value_policy policy, py::handle parent) {
    std::vector<float> arr{src.lower.x, src.lower.y, src.lower.z,
                           src.upper.x, src.upper.y, src.upper.z};
    return py::array_t<float>({2, 3}, arr.data()).release();
  }
};

} // namespace pybind11::detail
