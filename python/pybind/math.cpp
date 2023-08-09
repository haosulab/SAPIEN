#include "sapien/math/math.h"
#include "sapien/scene.h"
#include "sapien_type_caster.h"
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/smart_holder.h>
#include <pybind11/stl.h>

namespace py = pybind11;

using namespace sapien;

void init_math(py::module &sapien) {
  auto m = sapien.def_submodule("math");
  m.def("shortest_rotation", &ShortestRotation, py::arg("source"), py::arg("target"));
}
