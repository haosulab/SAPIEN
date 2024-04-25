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

  py::class_<MassProperties>(m, "MassProperties")
      .def_readwrite("mass", &MassProperties::mass)
      .def_readwrite("cm", &MassProperties::cm)
      .def_property_readonly("cm_inertia", &MassProperties::getCMInertia)
      .def_property_readonly("origin_inertia", &MassProperties::getOriginInertia)

      .def("scale_mass", &MassProperties::scaleMass, py::arg("scale"))
      .def("scale_size", &MassProperties::scaleSize, py::arg("scale"))
      .def("transform", &MassProperties::transform, py::arg("pose"))
      .def("decompose", &MassProperties::decompose);

  m.def("compute_mesh_mass_properties", &MassProperties::FromMesh);

  m.def("shortest_rotation", &ShortestRotation, py::arg("source"), py::arg("target"));
  m.attr("pose_gl_to_ros") = POSE_GL_TO_ROS;
  m.attr("pose_ros_to_gl") = POSE_ROS_TO_GL;
}
