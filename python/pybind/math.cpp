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
#include "sapien/math/math.h"
#include "sapien/scene.h"
#include "sapien_type_caster.h"
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

using namespace sapien;

void init_math(py::module &sapien) {
  auto m = sapien.def_submodule("math");

  py::classh<MassProperties>(m, "MassProperties")
      .def(py::init(&MassProperties::FromMassInertia), py::arg("mass"), py::arg("cm"),
           py::arg("inertia"),
           "construct inertia from mass, center of mass, inertia at center of mass")
      .def(py::init(&MassProperties::FromCMLocalPose), py::arg("mass"),
           py::arg("cmass_local_pose"), py::arg("inertia"),
           "construct inertia from mass, cmass_local_pose, and principal inertia")
      .def_readwrite("mass", &MassProperties::mass)
      .def_readwrite("cm", &MassProperties::cm)
      .def_property("cm_inertia", &MassProperties::getCMInertia, &MassProperties::setCMInertia)
      .def_property("origin_inertia", &MassProperties::getOriginInertia,
                    &MassProperties::setOriginInertia)
      .def("scale_mass", &MassProperties::scaleMass, py::arg("scale"),
           "compute new mass properties as if  the object density is scaled uniformly")
      .def("scale_size", &MassProperties::scaleSize, py::arg("scale"),
           "compute new mass properties as if the object volume is scaled around the origin "
           "while keeping density the same")
      .def("transform", &MassProperties::transform, py::arg("pose"),
           "compute new mass properties as if the origin of the current object is moved to given "
           "pose")
      .def("decompose", &MassProperties::decompose,
           "decompose mass properties into mass, cmass_local_pose, and principal inertia")
      .def(py::self + py::self, py::arg("other"));

  m.def("compute_mesh_mass_properties", &MassProperties::FromMesh, py::arg("vertices"),
        py::arg("triangles"))
      .def("compute_sphere_mass_properties", &MassProperties::FromSphere, py::arg("radius"))
      .def("compute_box_mass_properties", &MassProperties::FromBox, py::arg("half_size"))
      .def("compute_capsule_mass_properties", &MassProperties::FromCapsule, py::arg("radius"),
           py::arg("half_length"))
      .def("compute_cylinder_mass_properties", &MassProperties::FromCylinder, py::arg("radius"),
           py::arg("half_length"));

  m.def("shortest_rotation", &ShortestRotation, py::arg("source"), py::arg("target"));
  m.attr("pose_gl_to_ros") = POSE_GL_TO_ROS;
  m.attr("pose_ros_to_gl") = POSE_ROS_TO_GL;
}
