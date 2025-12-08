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
#include "sapien/physx/physx.h"
#include "./array.hpp"
#include "generator.hpp"
#include "sapien_type_caster.h"
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace sapien;
using namespace sapien::physx;

namespace pybind11::detail {

template <> struct type_caster<::physx::PxArticulationJointType::Enum> {
  PYBIND11_TYPE_CASTER(
      ::physx::PxArticulationJointType::Enum,
      _("typing.Literal['fixed', 'revolute', 'revolute_unwrapped', 'prismatic', 'free']"));

  bool load(py::handle src, bool convert) {
    std::string name = py::cast<std::string>(src);
    if (name == "fixed") {
      value = ::physx::PxArticulationJointType::eFIX;
      return true;
    } else if (name == "revolute" || name == "hinge" || name == "continuous") {
      value = ::physx::PxArticulationJointType::eREVOLUTE;
      return true;
    } else if (name == "revolute_unwrapped") {
      value = ::physx::PxArticulationJointType::eREVOLUTE_UNWRAPPED;
      return true;
    } else if (name == "prismatic" || name == "slider") {
      value = ::physx::PxArticulationJointType::ePRISMATIC;
      return true;
    } else if (name == "spherical" || name == "ball") {
      value = ::physx::PxArticulationJointType::eSPHERICAL;
      return true;
    } else if (name == "undefined" || name == "free") {
      value = ::physx::PxArticulationJointType::eUNDEFINED;
      return true;
    }
    return false;
  }

  static py::handle cast(::physx::PxArticulationJointType::Enum const &src,
                         py::return_value_policy policy, py::handle parent) {
    switch (src) {
    case ::physx::PxArticulationJointType::eFIX:
      return py::str("fixed").release();
    case ::physx::PxArticulationJointType::ePRISMATIC:
      return py::str("prismatic").release();
    case ::physx::PxArticulationJointType::eREVOLUTE:
      return py::str("revolute").release();
    case ::physx::PxArticulationJointType::eREVOLUTE_UNWRAPPED:
      return py::str("revolute_unwrapped").release();
    case ::physx::PxArticulationJointType::eSPHERICAL:
      return py::str("spherical").release();
    case ::physx::PxArticulationJointType::eUNDEFINED:
      return py::str("undefined").release();
    }
    throw std::runtime_error("invalid joint type");
  }
};

template <> struct type_caster<::physx::PxArticulationDriveType::Enum> {
  PYBIND11_TYPE_CASTER(::physx::PxArticulationDriveType::Enum,
                       _("typing.Literal['force', 'acceleration']"));

  bool load(py::handle src, bool convert) {
    std::string name = py::cast<std::string>(src);
    if (name == "force") {
      value = ::physx::PxArticulationDriveType::eFORCE;
      return true;
    } else if (name == "acceleration" || name == "acc") {
      value = ::physx::PxArticulationDriveType::eACCELERATION;
      return true;
    } else if (name == "target") {
      value = ::physx::PxArticulationDriveType::eTARGET;
      return true;
    } else if (name == "velocity") {
      value = ::physx::PxArticulationDriveType::eVELOCITY;
      return true;
    } else if (name == "none") {
      value = ::physx::PxArticulationDriveType::eNONE;
      return true;
    }
    return false;
  }

  static py::handle cast(::physx::PxArticulationDriveType::Enum const &src,
                         py::return_value_policy policy, py::handle parent) {
    switch (src) {
    case ::physx::PxArticulationDriveType::eFORCE:
      return py::str("force").release();
    case ::physx::PxArticulationDriveType::eACCELERATION:
      return py::str("acceleration").release();
    case ::physx::PxArticulationDriveType::eTARGET:
      return py::str("target").release();
    case ::physx::PxArticulationDriveType::eVELOCITY:
      return py::str("velocity").release();
    case ::physx::PxArticulationDriveType::eNONE:
      return py::str("none").release();
    }
    throw std::runtime_error("invalid drive type");
  }
};

template <> struct type_caster<PhysxDriveComponent::DriveMode> {
  PYBIND11_TYPE_CASTER(PhysxDriveComponent::DriveMode,
                       _("typing.Literal['force', 'acceleration']"));

  bool load(py::handle src, bool convert) {
    std::string name = py::cast<std::string>(src);
    if (name == "force") {
      value = PhysxDriveComponent::DriveMode::eFORCE;
      return true;
    } else if (name == "acceleration" || name == "acc") {
      value = PhysxDriveComponent::DriveMode::eACCELERATION;
      return true;
    }
    return false;
  }

  static py::handle cast(PhysxDriveComponent::DriveMode src, py::return_value_policy policy,
                         py::handle parent) {
    switch (src) {
    case PhysxDriveComponent::DriveMode::eFORCE:
      return py::str("force").release();
    case PhysxDriveComponent::DriveMode::eACCELERATION:
      return py::str("acceleration").release();
    }
    throw std::runtime_error("invalid drive type");
  }
};

template <> struct type_caster<::physx::PxForceMode::Enum> {
  PYBIND11_TYPE_CASTER(::physx::PxForceMode::Enum,
                       _("typing.Literal['force', 'acceleration', 'velocity_change', 'impulse']"));

  bool load(py::handle src, bool convert) {
    std::string name = py::cast<std::string>(src);
    if (name == "force" || name == "f") {
      value = ::physx::PxForceMode::eFORCE;
      return true;
    } else if (name == "acceleration" || name == "acc") {
      value = ::physx::PxForceMode::eACCELERATION;
      return true;
    } else if (name == "velocity_change") {
      value = ::physx::PxForceMode::eVELOCITY_CHANGE;
      return true;
    } else if (name == "impulse") {
      value = ::physx::PxForceMode::eIMPULSE;
      return true;
    }
    return false;
  }

  static py::handle cast(::physx::PxForceMode::Enum const &src, py::return_value_policy policy,
                         py::handle parent) {
    switch (src) {
    case ::physx::PxForceMode::eFORCE:
      return py::str("force").release();
    case ::physx::PxForceMode::eACCELERATION:
      return py::str("acceleration").release();
    case ::physx::PxForceMode::eVELOCITY_CHANGE:
      return py::str("velocity_change").release();
    case ::physx::PxForceMode::eIMPULSE:
      return py::str("impulse").release();
    }
    throw std::runtime_error("invalid drive type");
  }
};

} // namespace pybind11::detail

Generator<int> init_physx(py::module &sapien) {
  auto m = sapien.def_submodule("physx");

  auto PyPhysxSceneConfig = py::classh<PhysxSceneConfig>(m, "PhysxSceneConfig");
  PyPhysxSceneConfig.def(py::init<>())
      .def_readwrite("gravity", &PhysxSceneConfig::gravity)
      .def_readwrite("bounce_threshold", &PhysxSceneConfig::bounceThreshold)
      .def_readwrite("enable_pcm", &PhysxSceneConfig::enablePCM)
      .def_readwrite("enable_tgs", &PhysxSceneConfig::enableTGS)
      .def_readwrite("enable_ccd", &PhysxSceneConfig::enableCCD)
      .def_readwrite("enable_enhanced_determinism", &PhysxSceneConfig::enableEnhancedDeterminism)
      .def_readwrite("enable_friction_every_iteration",
                     &PhysxSceneConfig::enableFrictionEveryIteration)
      .def("__repr__", [](PhysxSceneConfig &) { return "PhysxSceneConfig()"; })
      .def(py::pickle(
          [](PhysxSceneConfig &config) {
            return py::make_tuple(config.gravity, config.bounceThreshold, config.enablePCM,
                                  config.enableTGS, config.enableCCD,
                                  config.enableEnhancedDeterminism,
                                  config.enableFrictionEveryIteration);
          },
          [](py::tuple t) {
            if (t.size() != 7) {
              throw std::runtime_error("Invalid state!");
            }
            PhysxSceneConfig config;
            config.gravity = t[0].cast<decltype(config.gravity)>();
            config.bounceThreshold = t[1].cast<decltype(config.bounceThreshold)>();
            config.enablePCM = t[2].cast<decltype(config.enablePCM)>();
            config.enableTGS = t[3].cast<decltype(config.enableTGS)>();
            config.enableCCD = t[4].cast<decltype(config.enableCCD)>();
            config.enableEnhancedDeterminism =
                t[5].cast<decltype(config.enableEnhancedDeterminism)>();
            config.enableFrictionEveryIteration =
                t[6].cast<decltype(config.enableFrictionEveryIteration)>();
            return config;
          }));

  auto PyPhysxShapeConfig = py::classh<PhysxShapeConfig>(m, "PhysxShapeConfig");
  PyPhysxShapeConfig.def(py::init<>())
      .def_readwrite("contact_offset", &PhysxShapeConfig::contactOffset)
      .def_readwrite("rest_offset", &PhysxShapeConfig::restOffset)
      .def("__repr__", [](PhysxShapeConfig &) { return "PhysxShapeConfig()"; })
      .def(py::pickle(
          [](PhysxShapeConfig &config) {
            return py::make_tuple(config.contactOffset, config.restOffset);
          },
          [](py::tuple t) {
            if (t.size() != 2) {
              throw std::runtime_error("Invalid state!");
            }
            PhysxShapeConfig config;
            config.contactOffset = t[0].cast<decltype(config.contactOffset)>();
            config.restOffset = t[1].cast<decltype(config.restOffset)>();
            return config;
          }));

  auto PyPhysxBodyConfig = py::classh<PhysxBodyConfig>(m, "PhysxBodyConfig");
  PyPhysxBodyConfig.def(py::init<>())
      .def_readwrite("sleep_threshold", &PhysxBodyConfig::sleepThreshold)
      .def_readwrite("solver_position_iterations", &PhysxBodyConfig::solverPositionIterations)
      .def_readwrite("solver_velocity_iterations", &PhysxBodyConfig::solverVelocityIterations)
      .def("__repr__", [](PhysxBodyConfig &) { return "PhysxBodyConfig()"; })
      .def(py::pickle(
          [](PhysxBodyConfig &config) {
            return py::make_tuple(config.solverPositionIterations, config.solverVelocityIterations,
                                  config.sleepThreshold);
          },
          [](py::tuple t) {
            if (t.size() != 3) {
              throw std::runtime_error("Invalid state!");
            }
            PhysxBodyConfig config;
            config.solverPositionIterations =
                t[0].cast<decltype(config.solverPositionIterations)>();
            config.solverVelocityIterations =
                t[1].cast<decltype(config.solverVelocityIterations)>();
            config.sleepThreshold = t[2].cast<decltype(config.sleepThreshold)>();
            return config;
          }));

  auto PyPhysxSDFConfig = py::classh<PhysxSDFShapeConfig>(m, "PhysxSDFConfig");
  PyPhysxSDFConfig.def(py::init<>())
      .def_readwrite("spacing", &PhysxSDFShapeConfig::spacing)
      .def_readwrite("subgridSize", &PhysxSDFShapeConfig::subgridSize)
      .def_readwrite("num_threads_for_construction",
                     &PhysxSDFShapeConfig::numThreadsForConstruction)
      .def("__repr__", [](PhysxSDFShapeConfig &) { return "PhysxSDFConfig()"; })
      .def(py::pickle(
          [](PhysxSDFShapeConfig &config) {
            return py::make_tuple(config.spacing, config.subgridSize,
                                  config.numThreadsForConstruction);
          },
          [](py::tuple t) {
            if (t.size() != 3) {
              throw std::runtime_error("Invalid state!");
            }
            PhysxSDFShapeConfig config;
            config.spacing = t[0].cast<decltype(config.spacing)>();
            config.subgridSize = t[1].cast<decltype(config.subgridSize)>();
            config.numThreadsForConstruction =
                t[2].cast<decltype(config.numThreadsForConstruction)>();
            return config;
          }));

  auto PyPhysxEngine = py::classh<PhysxEngine>(m, "PhysxEngine");
  auto PyPhysxContactPoint = py::classh<ContactPoint>(m, "PhysxContactPoint");
  auto PyPhysxContact = py::classh<Contact>(m, "PhysxContact");

  auto PyPhysxRayHit = py::classh<PhysxHitInfo>(m, "PhysxRayHit");

  auto PyPhysxSystem = py::classh<PhysxSystem, System>(m, "PhysxSystem");
  auto PyPhysxSystemCpu = py::classh<PhysxSystemCpu, PhysxSystem>(m, "PhysxCpuSystem");

  auto PyPhysxSystemGpu = py::classh<PhysxSystemGpu, PhysxSystem>(m, "PhysxGpuSystem");

#ifdef SAPIEN_CUDA
  auto PyPhysxGpuContactPairImpulseQuery =
      py::classh<PhysxGpuContactPairImpulseQuery>(m, "PhysxGpuContactPairImpulseQuery");
  auto PyPhysxGpuContactBodyImpulseQuery =
      py::classh<PhysxGpuContactBodyImpulseQuery>(m, "PhysxGpuContactBodyImpulseQuery");
#endif

  auto PyPhysxBaseComponent = py::classh<PhysxBaseComponent, Component>(m, "PhysxBaseComponent");
  auto PyPhysxRigidBaseComponent =
      py::classh<PhysxRigidBaseComponent, PhysxBaseComponent>(m, "PhysxRigidBaseComponent");
  auto PyPhysxRigidStaticComponent =
      py::classh<PhysxRigidStaticComponent, PhysxRigidBaseComponent>(m,
                                                                     "PhysxRigidStaticComponent");
  auto PyPhysxRigidBodyComponent =
      py::classh<PhysxRigidBodyComponent, PhysxRigidBaseComponent>(m, "PhysxRigidBodyComponent");
  auto PyPhysxRigidDynamicComponent =
      py::classh<PhysxRigidDynamicComponent, PhysxRigidBodyComponent>(
          m, "PhysxRigidDynamicComponent");
  auto PyPhysxArticulationLinkComponent =
      py::classh<PhysxArticulationLinkComponent, PhysxRigidBodyComponent>(
          m, "PhysxArticulationLinkComponent");
  auto PyPhysxJointComponent =
      py::classh<PhysxJointComponent, PhysxBaseComponent>(m, "PhysxJointComponent");
  auto PyPhysxDriveComponent =
      py::classh<PhysxDriveComponent, PhysxJointComponent>(m, "PhysxDriveComponent");
  auto PyPhysxGearComponent =
      py::classh<PhysxGearComponent, PhysxJointComponent>(m, "PhysxGearComponent");
  auto PyPhysxDistanceJointComponent =
      py::classh<PhysxDistanceJointComponent, PhysxJointComponent>(m,
                                                                   "PhysxDistanceJointComponent");

  auto PyPhysxMaterial = py::classh<PhysxMaterial>(m, "PhysxMaterial");

  auto PyPhysxCollisionShape = py::classh<PhysxCollisionShape>(m, "PhysxCollisionShape");
  auto PyPhysxCollisionShapePlane =
      py::classh<PhysxCollisionShapePlane, PhysxCollisionShape>(m, "PhysxCollisionShapePlane");
  auto PyPhysxCollisionShapeBox =
      py::classh<PhysxCollisionShapeBox, PhysxCollisionShape>(m, "PhysxCollisionShapeBox");
  auto PyPhysxCollisionShapeSphere =
      py::classh<PhysxCollisionShapeSphere, PhysxCollisionShape>(m, "PhysxCollisionShapeSphere");
  auto PyPhysxCollisionShapeCapsule =
      py::classh<PhysxCollisionShapeCapsule, PhysxCollisionShape>(m, "PhysxCollisionShapeCapsule");
  auto PyPhysxCollisionShapeCylinder =
      py::classh<PhysxCollisionShapeCylinder, PhysxCollisionShape>(m,
                                                                   "PhysxCollisionShapeCylinder");
  auto PyPhysxCollisionShapeConvexMesh =
      py::classh<PhysxCollisionShapeConvexMesh, PhysxCollisionShape>(
          m, "PhysxCollisionShapeConvexMesh");
  auto PyPhysxCollisionShapeTriangleMesh =
      py::classh<PhysxCollisionShapeTriangleMesh, PhysxCollisionShape>(
          m, "PhysxCollisionShapeTriangleMesh");

  auto PyPhysxArticulation = py::classh<PhysxArticulation>(m, "PhysxArticulation");
  auto PyPhysxArticulationJoint = py::classh<PhysxArticulationJoint>(m, "PhysxArticulationJoint");

  co_yield 0;

  PyPhysxEngine.def(py::init(&PhysxEngine::Get), py::arg("tolerance_length"),
                    py::arg("tolerance_speed"));

  PyPhysxContactPoint.def_readonly("position", &ContactPoint::position)
      .def_readonly("normal", &ContactPoint::normal)
      .def_readonly("impulse", &ContactPoint::impulse)
      .def_readonly("separation", &ContactPoint::separation);

  PyPhysxContact.def_readonly("bodies", &Contact::components, py::return_value_policy::reference)
      .def_readonly("shapes", &Contact::shapes, py::return_value_policy::reference)
      .def_readonly("points", &Contact::points)
      .def("__repr__", [](Contact const &c) {
        return "Contact(entity0=" + c.components[0]->getEntity()->getName() +
               ", entity1=" + c.components[1]->getEntity()->getName() + ")";
      });

  PyPhysxRayHit.def_readonly("position", &PhysxHitInfo::position)
      .def_readonly("normal", &PhysxHitInfo::normal)
      .def_readonly("distance", &PhysxHitInfo::distance)
      .def_readonly("shape", &PhysxHitInfo::shape, py::return_value_policy::reference)
      .def_readonly("component", &PhysxHitInfo::component, py::return_value_policy::reference)
      .def("__repr__", [](PhysxHitInfo const &hit) {
        std::ostringstream s;
        s << "PhysxRayHit(position=[";
        s << hit.position.x << ", " << hit.position.y << ", " << hit.position.z << "], normal=[";
        s << hit.normal.x << ", " << hit.normal.y << ", " << hit.normal.z << "], distance=";
        s << hit.distance << ")";
        return s.str();
      });

  PyPhysxSystem
      .def(py::init([]() -> std::shared_ptr<PhysxSystem> {
        throw std::runtime_error(
            "PhysxSystem has been removed, use PhysxCpuSystem or PhysxGpuSystem instead");
      }))

      .def_property_readonly("config", &PhysxSystem::getSceneConfig)
      .def("get_config", &PhysxSystem::getSceneConfig)
      .def_property("timestep", &PhysxSystem::getTimestep, &PhysxSystem::setTimestep)
      .def("get_timestep", &PhysxSystem::getTimestep)
      .def("set_timestep", &PhysxSystem::setTimestep, py::arg("timestep"))

      .def_property("scene_collision_id", &PhysxSystem::getSceneCollisionId,
                    &PhysxSystem::setSceneCollisionId)
      .def("set_scene_collision_id", &PhysxSystem::setSceneCollisionId, py::arg("id"))
      .def("get_scene_collision_id", &PhysxSystem::getSceneCollisionId)

      .def_property_readonly("rigid_dynamic_components", &PhysxSystem::getRigidDynamicComponents)
      .def("get_rigid_dynamic_components", &PhysxSystem::getRigidDynamicComponents)
      .def_property_readonly("rigid_static_components", &PhysxSystem::getRigidStaticComponents)
      .def("get_rigid_static_components", &PhysxSystem::getRigidStaticComponents)
      .def_property_readonly("articulation_link_components",
                             &PhysxSystem::getArticulationLinkComponents)
      .def("get_articulation_link_components", &PhysxSystem::getArticulationLinkComponents);

  PyPhysxSystemCpu.def(py::init<>())
      .def("get_contacts", &PhysxSystemCpu::getContacts, py::return_value_policy::reference)
      .def("raycast", &PhysxSystemCpu::raycast, py::arg("position"), py::arg("direction"),
           py::arg("distance"),
           R"doc(Casts a ray and returns the closest hit. Returns None if no hit)doc")
      .def("pack", [](PhysxSystemCpu &s) { return py::bytes(s.packState()); })
      .def(
          "unpack", [](PhysxSystemCpu &s, py::bytes data) { s.unpackState(data); },
          py::arg("data"));

#ifdef SAPIEN_CUDA
  PyPhysxSystemGpu
      .def(py::init([](std::string const &device) {
             return std::make_shared<PhysxSystemGpu>(findDevice(device));
           }),
           py::arg("device") = "cuda")
      .def(py::init([](std::shared_ptr<Device> device) {
             return std::make_shared<PhysxSystemGpu>(device);
           }),
           py::arg("device"))
      .def_property_readonly("device", &PhysxSystemGpu::getDevice)
      .def("get_scene_offset", &PhysxSystemGpu::getSceneOffset, py::arg("scene"))
      .def("set_scene_offset", &PhysxSystemGpu::setSceneOffset, py::arg("scene"),
           py::arg("offset"), R"doc(
In GPU mode, all SAPIEN scenes share the same PhysX scene. One should call this
function to apply an offset to avoid bodies in different scenes interfere with
each other. This function must be called before any PhysX body is added to scene.

Example: After calling `set_scene_offset([2, 1, 0])`, an SAPIEN object with
position `[1, 1, 1]` will be at position `[1, 1, 1] + [2, 1, 0] = [3, 2, 1]` in
PhysX scene.
)doc")

      .def("gpu_init", &PhysxSystemGpu::gpuInit, R"doc(
   "Warm start" the GPU simulation by stepping the system once. This function
   must be called each time when actors are added or removed from the scene. One
   may call `gpu_apply_*` functions to initialize the system after calling this
   function.
)doc")
      .def("gpu_set_cuda_stream", &PhysxSystemGpu::gpuSetCudaStream, py::arg("stream"), R"doc(
PhysX GPU APIs will be synchronized with the provided stream and SAPIEN's CUDA
kernels will be launched to the provided stream.

Args:
    stream: integer representation of a cuda stream pointer
)doc")

      .def_property_readonly("cuda_rigid_body_data", &PhysxSystemGpu::gpuGetRigidBodyCudaHandle)
      .def_property_readonly("cuda_rigid_dynamic_data",
                             &PhysxSystemGpu::gpuGetRigidDynamicCudaHandle)
      .def_property_readonly("cuda_articulation_link_data",
                             &PhysxSystemGpu::gpuGetArticulationLinkCudaHandle)

      .def_property_readonly("cuda_rigid_body_force",
                             &PhysxSystemGpu::gpuGetRigidBodyForceCudaHandle)
      .def_property_readonly("cuda_rigid_dynamic_force",
                             &PhysxSystemGpu::gpuGetRigidDynamicForceCudaHandle)
      .def_property_readonly("cuda_rigid_body_torque",
                             &PhysxSystemGpu::gpuGetRigidBodyTorqueCudaHandle)
      .def_property_readonly("cuda_rigid_dynamic_torque",
                             &PhysxSystemGpu::gpuGetRigidDynamicTorqueCudaHandle)

      .def_property_readonly("cuda_articulation_qpos",
                             &PhysxSystemGpu::gpuGetArticulationQposCudaHandle)
      .def_property_readonly("cuda_articulation_qvel",
                             &PhysxSystemGpu::gpuGetArticulationQvelCudaHandle)
      .def_property_readonly("cuda_articulation_qacc",
                             &PhysxSystemGpu::gpuGetArticulationQaccCudaHandle)
      .def_property_readonly("cuda_articulation_qf",
                             &PhysxSystemGpu::gpuGetArticulationQfCudaHandle)
      .def_property_readonly("cuda_articulation_target_qpos",
                             &PhysxSystemGpu::gpuGetArticulationQTargetPosCudaHandle)
      .def_property_readonly("cuda_articulation_target_qvel",
                             &PhysxSystemGpu::gpuGetArticulationQTargetVelCudaHandle)
      .def_property_readonly("cuda_articulation_link_incoming_joint_forces",
                             &PhysxSystemGpu::gpuGetArticulationLinkIncomingJointForceHandle)

      .def("gpu_fetch_rigid_dynamic_data", &PhysxSystemGpu::gpuFetchRigidDynamicData)
      .def("gpu_fetch_articulation_link_pose", &PhysxSystemGpu::gpuFetchArticulationLinkPose)
      .def("gpu_fetch_articulation_link_velocity", &PhysxSystemGpu::gpuFetchArticulationLinkVel)
      .def("gpu_fetch_articulation_qpos", &PhysxSystemGpu::gpuFetchArticulationQpos)
      .def("gpu_fetch_articulation_qvel", &PhysxSystemGpu::gpuFetchArticulationQvel)
      .def("gpu_fetch_articulation_qacc", &PhysxSystemGpu::gpuFetchArticulationQacc)
      .def("gpu_fetch_articulation_target_qpos", &PhysxSystemGpu::gpuFetchArticulationQTargetPos)
      .def("gpu_fetch_articulation_target_qvel", &PhysxSystemGpu::gpuFetchArticulationQTargetVel)
      .def("gpu_fetch_articulation_link_incoming_joint_forces",
           &PhysxSystemGpu::gpuFetchArticulationLinkIncomingJointForce)

      .def("gpu_create_contact_pair_impulse_query",
           &PhysxSystemGpu::gpuCreateContactPairImpulseQuery, py::arg("body_pairs"))
      .def("gpu_query_contact_pair_impulses", &PhysxSystemGpu::gpuQueryContactPairImpulses,
           py::arg("query"))

      .def("gpu_create_contact_body_impulse_query",
           &PhysxSystemGpu::gpuCreateContactBodyImpulseQuery, py::arg("bodies"))
      .def("gpu_query_contact_body_impulses", &PhysxSystemGpu::gpuQueryContactBodyImpulses,
           py::arg("query"),
           R"doc(Query net contact forces for specific bodies of the last simulation step.
Usage:
    query = system.gpu_create_contact_body_force_query(bodies)  # create force query in advance

    # after simulation step
    system.gpu_query_contact_body_forces(query)
    # query.cuda_buffer is now filled with net contact forces for each body
)doc")

      .def("gpu_update_articulation_kinematics", &PhysxSystemGpu::gpuUpdateArticulationKinematics)

      // TODO apply force torque
      .def("gpu_apply_rigid_dynamic_data",
           py::overload_cast<>(&PhysxSystemGpu::gpuApplyRigidDynamicData))
      .def("gpu_apply_rigid_dynamic_force",
           py::overload_cast<>(&PhysxSystemGpu::gpuApplyRigidDynamicForce))
      .def("gpu_apply_rigid_dynamic_torque",
           py::overload_cast<>(&PhysxSystemGpu::gpuApplyRigidDynamicTorque))

      .def("gpu_apply_articulation_root_pose",
           py::overload_cast<>(&PhysxSystemGpu::gpuApplyArticulationRootPose))
      .def("gpu_apply_articulation_root_velocity",
           py::overload_cast<>(&PhysxSystemGpu::gpuApplyArticulationRootVel))
      .def("gpu_apply_articulation_qpos",
           py::overload_cast<>(&PhysxSystemGpu::gpuApplyArticulationQpos))
      .def("gpu_apply_articulation_qvel",
           py::overload_cast<>(&PhysxSystemGpu::gpuApplyArticulationQvel))
      .def("gpu_apply_articulation_qf",
           py::overload_cast<>(&PhysxSystemGpu::gpuApplyArticulationQf))
      .def("gpu_apply_articulation_target_position",
           py::overload_cast<>(&PhysxSystemGpu::gpuApplyArticulationQTargetPos))
      .def("gpu_apply_articulation_target_velocity",
           py::overload_cast<>(&PhysxSystemGpu::gpuApplyArticulationQTargetVel))

      .def("gpu_apply_rigid_dynamic_data",
           py::overload_cast<CudaArrayHandle const &>(&PhysxSystemGpu::gpuApplyRigidDynamicData),
           py::arg("index_buffer"))
      .def("gpu_apply_articulation_root_pose",
           py::overload_cast<CudaArrayHandle const &>(
               &PhysxSystemGpu::gpuApplyArticulationRootPose),
           py::arg("index_buffer"))
      .def(
          "gpu_apply_articulation_root_velocity",
          py::overload_cast<CudaArrayHandle const &>(&PhysxSystemGpu::gpuApplyArticulationRootVel),
          py::arg("index_buffer"))
      .def("gpu_apply_articulation_qpos",
           py::overload_cast<CudaArrayHandle const &>(&PhysxSystemGpu::gpuApplyArticulationQpos),
           py::arg("index_buffer"))
      .def("gpu_apply_articulation_qvel",
           py::overload_cast<CudaArrayHandle const &>(&PhysxSystemGpu::gpuApplyArticulationQvel),
           py::arg("index_buffer"))
      .def("gpu_apply_articulation_qf",
           py::overload_cast<CudaArrayHandle const &>(&PhysxSystemGpu::gpuApplyArticulationQf),
           py::arg("index_buffer"))
      .def("gpu_apply_articulation_target_position",
           py::overload_cast<CudaArrayHandle const &>(
               &PhysxSystemGpu::gpuApplyArticulationQTargetPos),
           py::arg("index_buffer"))
      .def("gpu_apply_articulation_target_velocity",
           py::overload_cast<CudaArrayHandle const &>(
               &PhysxSystemGpu::gpuApplyArticulationQTargetVel),
           py::arg("index_buffer"))

      .def("sync_poses_gpu_to_cpu", &PhysxSystemGpu::syncPosesGpuToCpu,
           "Warning: this function is super slow and for debug only. Download all poses from the "
           "GPU and copy to SAPIEN entities.")

      .def("step_start", &PhysxSystemGpu::stepStart)
      .def("step_finish", &PhysxSystemGpu::stepFinish);

  PyPhysxGpuContactPairImpulseQuery.def_property_readonly(
      "cuda_impulses", [](PhysxGpuContactPairImpulseQuery const &q) { return q.buffer.handle(); });

  PyPhysxGpuContactBodyImpulseQuery.def_property_readonly(
      "cuda_impulses", [](PhysxGpuContactBodyImpulseQuery const &q) { return q.buffer.handle(); });

#else
  PyPhysxSystemGpu.def(py::init([](std::string const &device) {
                         return std::make_shared<PhysxSystemGpu>(findDevice(device));
                       }),
                       py::arg("device") = "cuda");
#endif
  PyPhysxMaterial
      .def(py::init<float, float, float>(), py::arg("static_friction"),
           py::arg("dynamic_friction"), py::arg("restitution"))

      .def_property("dynamic_friction", &PhysxMaterial::getDynamicFriction,
                    &PhysxMaterial::setDynamicFriction)
      .def("get_dynamic_friction", &PhysxMaterial::getDynamicFriction)
      .def("set_dynamic_friction", &PhysxMaterial::setDynamicFriction, py::arg("friction"))

      .def_property("static_friction", &PhysxMaterial::getStaticFriction,
                    &PhysxMaterial::setStaticFriction)
      .def("get_static_friction", &PhysxMaterial::getStaticFriction)
      .def("set_static_friction", &PhysxMaterial::setStaticFriction, py::arg("friction"))

      .def_property("restitution", &PhysxMaterial::getRestitution, &PhysxMaterial::setRestitution)
      .def("get_restitution", &PhysxMaterial::getRestitution)
      .def("set_restitution", &PhysxMaterial::setRestitution, py::arg("restitution"));

  PyPhysxCollisionShape
      .def_property("local_pose", &PhysxCollisionShape::getLocalPose,
                    &PhysxCollisionShape::setLocalPose)
      .def("get_local_pose", &PhysxCollisionShape::getLocalPose)
      .def("set_local_pose", &PhysxCollisionShape::setLocalPose, py::arg("pose"))

      .def_property("rest_offset", &PhysxCollisionShape::getRestOffset,
                    &PhysxCollisionShape::setRestOffset)
      .def("get_rest_offset", &PhysxCollisionShape::getRestOffset)
      .def("set_rest_offset", &PhysxCollisionShape::setRestOffset, py::arg("offset"))
      .def_property("contact_offset", &PhysxCollisionShape::getContactOffset,
                    &PhysxCollisionShape::setContactOffset)
      .def("get_contact_offset", &PhysxCollisionShape::getContactOffset)
      .def("set_contact_offset", &PhysxCollisionShape::setContactOffset, py::arg("offset"))
      .def_property("patch_radius", &PhysxCollisionShape::getTorsionalPatchRadius,
                    &PhysxCollisionShape::setTorsionalPatchRadius)
      .def("get_patch_radius", &PhysxCollisionShape::getTorsionalPatchRadius)
      .def("set_patch_radius", &PhysxCollisionShape::setTorsionalPatchRadius, py::arg("radius"))
      .def_property("min_patch_radius", &PhysxCollisionShape::getMinTorsionalPatchRadius,
                    &PhysxCollisionShape::setMinTorsionalPatchRadius)
      .def("get_min_patch_radius", &PhysxCollisionShape::getMinTorsionalPatchRadius)
      .def("set_min_patch_radius", &PhysxCollisionShape::setMinTorsionalPatchRadius,
           py::arg("radius"))

      .def_property("physical_material", &PhysxCollisionShape::getPhysicalMaterial,
                    &PhysxCollisionShape::setPhysicalMaterial)
      .def("get_physical_material", &PhysxCollisionShape::getPhysicalMaterial)
      .def("set_physical_material", &PhysxCollisionShape::setPhysicalMaterial, py::arg("material"))

      .def_property_readonly("collision_groups", &PhysxCollisionShape::getCollisionGroups)
      .def("get_collision_groups", &PhysxCollisionShape::getCollisionGroups)
      .def("set_collision_groups", &PhysxCollisionShape::setCollisionGroups, py::arg("groups"),
           R"doc(
collision groups determine the collision behavior of objects. Let A.gx denote the collision group x of collision shape A. Collision shape A and B will collide iff the following condition holds:

((A.g0 & B.g1) or (A.g1 & B.g0)) and (not ((A.g2 & B.g2) and ((A.g3 & 0xffff) == (B.g3 & 0xffff))))

Here is some explanation: g2 is the "ignore group" and g3 is the "id group". Only the lower 16 bits of the id group is used since the upper 16 bits are reserved for other purposes in the future. When 2 collision shapes have the same ID (g3), then if any of their g2 bits match, their collisions are always ignored.

If after testing g2 and g3, the objects may collide, g0 and g1 come into play. g0 is the "contact type group" and g1 is the "contact affinity group". Collision shapes collide only when a bit in the contact type of the first shape matches a bit in the contact affinity of the second shape.)doc")

      .def_property("density", &PhysxCollisionShape::getDensity, &PhysxCollisionShape::setDensity)
      .def("get_density", &PhysxCollisionShape::getDensity)
      .def("set_density", &PhysxCollisionShape::setDensity, py::arg("density"))

      .def_property("contact_offset", &PhysxCollisionShape::getContactOffset,
                    &PhysxCollisionShape::setContactOffset)
      .def("get_contact_offset", &PhysxCollisionShape::getContactOffset)
      .def("set_contact_offset", &PhysxCollisionShape::setContactOffset, py::arg("offset"))

      .def_property("rest_offset", &PhysxCollisionShape::getRestOffset,
                    &PhysxCollisionShape::setRestOffset)
      .def("get_rest_offset", &PhysxCollisionShape::getRestOffset)
      .def("set_rest_offset", &PhysxCollisionShape::setRestOffset, py::arg("offset"));

  PyPhysxCollisionShapePlane.def(py::init<std::shared_ptr<PhysxMaterial>>(), py::arg("material"));
  PyPhysxCollisionShapeBox
      .def(py::init<Vec3, std::shared_ptr<PhysxMaterial>>(), py::arg("half_size"),
           py::arg("material"))
      .def_property_readonly("half_size", &PhysxCollisionShapeBox::getHalfLengths)
      .def("get_half_size", &PhysxCollisionShapeBox::getHalfLengths);

  PyPhysxCollisionShapeSphere
      .def(py::init<float, std::shared_ptr<PhysxMaterial>>(), py::arg("radius"),
           py::arg("material"))
      .def_property_readonly("radius", &PhysxCollisionShapeSphere::getRadius)
      .def("get_radius", &PhysxCollisionShapeSphere::getRadius);
  PyPhysxCollisionShapeCapsule
      .def(py::init<float, float, std::shared_ptr<PhysxMaterial>>(), py::arg("radius"),
           py::arg("half_length"), py::arg("material"))
      .def_property_readonly("radius", &PhysxCollisionShapeCapsule::getRadius)
      .def("get_radius", &PhysxCollisionShapeCapsule::getRadius)
      .def_property_readonly("half_length", &PhysxCollisionShapeCapsule::getHalfLength)
      .def("get_half_length", &PhysxCollisionShapeCapsule::getHalfLength);
  PyPhysxCollisionShapeCylinder
      .def(py::init<float, float, std::shared_ptr<PhysxMaterial>>(), py::arg("radius"),
           py::arg("half_length"), py::arg("material"))
      .def_property_readonly("radius", &PhysxCollisionShapeCylinder::getRadius)
      .def("get_radius", &PhysxCollisionShapeCylinder::getRadius)
      .def_property_readonly("half_length", &PhysxCollisionShapeCylinder::getHalfLength)
      .def("get_half_length", &PhysxCollisionShapeCylinder::getHalfLength);

  // TODO: support load from vertices
  PyPhysxCollisionShapeConvexMesh
      .def(py::init<std::string const &, Vec3, std::shared_ptr<PhysxMaterial>>(),
           py::arg("filename"), py::arg("scale"), py::arg("material"))
      .def(py::init<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &, Vec3 const &,
                    std::shared_ptr<PhysxMaterial>>(),
           py::arg("vertices"), py::arg("scale"), py::arg("material"))
      .def_static("load_multiple", &PhysxCollisionShapeConvexMesh::LoadMultiple,
                  py::arg("filename"), py::arg("scale"), py::arg("material"))

      .def_property_readonly("scale", &PhysxCollisionShapeConvexMesh::getScale)
      .def("get_scale", &PhysxCollisionShapeConvexMesh::getScale)
      .def_property_readonly("vertices", &PhysxCollisionShapeConvexMesh::getVertices)
      .def("get_vertices", &PhysxCollisionShapeConvexMesh::getVertices)
      .def_property_readonly("triangles", &PhysxCollisionShapeConvexMesh::getTriangles)
      .def("get_triangles", &PhysxCollisionShapeConvexMesh::getTriangles);

  PyPhysxCollisionShapeTriangleMesh
      .def(py::init<std::string const &, Vec3, std::shared_ptr<PhysxMaterial>, bool>(),
           py::arg("filename"), py::arg("scale"), py::arg("material"), py::arg("sdf") = false)
      .def(py::init<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &,
                    Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor> const &,
                    Vec3 const &, std::shared_ptr<PhysxMaterial>, bool>(),
           py::arg("vertices"), py::arg("triangles"), py::arg("scale"), py::arg("material"),
           py::arg("sdf") = false)
      .def_property_readonly("scale", &PhysxCollisionShapeTriangleMesh::getScale)
      .def("get_scale", &PhysxCollisionShapeTriangleMesh::getScale)
      .def_property_readonly("vertices", &PhysxCollisionShapeTriangleMesh::getVertices)
      .def("get_vertices", &PhysxCollisionShapeTriangleMesh::getVertices)
      .def_property_readonly("triangles", &PhysxCollisionShapeTriangleMesh::getTriangles)
      .def("get_triangles", &PhysxCollisionShapeTriangleMesh::getTriangles);

  PyPhysxRigidBaseComponent
      .def("attach", &PhysxRigidBaseComponent::attachCollision, py::arg("collision_shape"))
      .def_property_readonly("collision_shapes", &PhysxRigidBaseComponent::getCollisionShapes)
      .def("get_collision_shapes", &PhysxRigidBaseComponent::getCollisionShapes)

      .def("compute_global_aabb_tight", &PhysxRigidBaseComponent::computeGlobalAABBTight)
      .def("get_global_aabb_fast", &PhysxRigidBaseComponent::getGlobalAABBFast)
      .def_property_readonly("_physx_pointer", [](PhysxRigidBaseComponent const &c) {
        return reinterpret_cast<uintptr_t>(c.getPxActor());
      });

  PyPhysxRigidStaticComponent.def(py::init<>());

  PyPhysxRigidBodyComponent
      .def_property_readonly("auto_compute_mass", &PhysxRigidBodyComponent::getAutoComputeMass)
      .def("get_auto_compute_mass", &PhysxRigidBodyComponent::getAutoComputeMass)

      .def_property("linear_damping", &PhysxRigidBodyComponent::getLinearDamping,
                    &PhysxRigidBodyComponent::setLinearDamping)
      .def("get_linear_damping", &PhysxRigidBodyComponent::getLinearDamping)
      .def("set_linear_damping", &PhysxRigidBodyComponent::setLinearDamping, py::arg("damping"))
      .def_property("angular_damping", &PhysxRigidBodyComponent::getAngularDamping,
                    &PhysxRigidBodyComponent::setAngularDamping)
      .def("get_angular_damping", &PhysxRigidBodyComponent::getAngularDamping)
      .def("set_angular_damping", &PhysxRigidBodyComponent::setAngularDamping, py::arg("damping"))

      .def_property("max_linear_velocity", &PhysxRigidBodyComponent::getMaxLinearVelocity,
                    &PhysxRigidBodyComponent::setMaxLinearVelocity)
      .def("get_max_linear_velocity", &PhysxRigidBodyComponent::getMaxLinearVelocity)
      .def("set_max_linear_velocity", &PhysxRigidBodyComponent::setMaxLinearVelocity,
           py::arg("velocity"))
      .def_property("max_angular_velocity", &PhysxRigidBodyComponent::getMaxAngularVelocity,
                    &PhysxRigidBodyComponent::setMaxAngularVelocity)
      .def("get_max_angular_velocity", &PhysxRigidBodyComponent::getMaxAngularVelocity)
      .def("set_max_angular_velocity", &PhysxRigidBodyComponent::setMaxAngularVelocity,
           py::arg("velocity"))

      .def_property("max_depenetration_velocity",
                    &PhysxRigidBodyComponent::getMaxDepenetrationVelocity,
                    &PhysxRigidBodyComponent::setMaxDepenetrationVelocity)
      .def("get_max_depenetration_velocity", &PhysxRigidBodyComponent::getMaxDepenetrationVelocity)
      .def("set_max_depenetration_velocity", &PhysxRigidBodyComponent::setMaxDepenetrationVelocity,
           py::arg("velocity"))

      .def_property("max_contact_impulse", &PhysxRigidBodyComponent::getMaxContactImpulse,
                    &PhysxRigidBodyComponent::setMaxContactImpulse)
      .def("get_max_contact_impulse", &PhysxRigidBodyComponent::getMaxContactImpulse)
      .def("set_max_contact_impulse", &PhysxRigidBodyComponent::setMaxContactImpulse,
           py::arg("impulse"))

      .def_property("mass", &PhysxRigidBodyComponent::getMass, &PhysxRigidBodyComponent::setMass)
      .def("get_mass", &PhysxRigidBodyComponent::getMass)
      .def("set_mass", &PhysxRigidBodyComponent::setMass, py::arg("mass"))

      .def_property("inertia", &PhysxRigidBodyComponent::getInertia,
                    &PhysxRigidBodyComponent::setInertia)
      .def("get_inertia", &PhysxRigidBodyComponent::getInertia)
      .def("set_inertia", &PhysxRigidBodyComponent::setInertia, py::arg("inertia"))

      .def_property("cmass_local_pose", &PhysxRigidBodyComponent::getCMassLocalPose,
                    &PhysxRigidBodyComponent::setCMassLocalPose)
      .def("get_cmass_local_pose", &PhysxRigidBodyComponent::getCMassLocalPose)
      .def("set_cmass_local_pose", &PhysxRigidBodyComponent::setCMassLocalPose, py::arg("pose"))

      .def_property_readonly("linear_velocity", &PhysxRigidBodyComponent::getLinearVelocity)
      .def("get_linear_velocity", &PhysxRigidBodyComponent::getLinearVelocity)

      .def_property_readonly("angular_velocity", &PhysxRigidBodyComponent::getAngularVelocity)
      .def("get_angular_velocity", &PhysxRigidBodyComponent::getAngularVelocity)

      .def_property("disable_gravity", &PhysxRigidBodyComponent::getDisableGravity,
                    &PhysxRigidBodyComponent::setDisableGravity)
      .def("get_disable_gravity", &PhysxRigidBodyComponent::getDisableGravity)
      .def("set_disable_gravity", &PhysxRigidBodyComponent::setDisableGravity, py::arg("disable"))

      .def("add_force_torque", &PhysxRigidBodyComponent::addForceTorque, py::arg("force"),
           py::arg("torque"), py::arg("mode") = ::physx::PxForceMode::eFORCE)
      .def("add_force_at_point", &PhysxRigidBodyComponent::addForceAtPoint, py::arg("force"),
           py::arg("point"), py::arg("mode") = ::physx::PxForceMode::eFORCE);

  PyPhysxRigidDynamicComponent
      .def(py::init<>())

      .def_property("linear_velocity", &PhysxRigidDynamicComponent::getLinearVelocity,
                    &PhysxRigidDynamicComponent::setLinearVelocity)
      .def("get_linear_velocity", &PhysxRigidDynamicComponent::getLinearVelocity)
      .def("set_linear_velocity", &PhysxRigidDynamicComponent::setLinearVelocity,
           py::arg("velocity"))

      .def_property("angular_velocity", &PhysxRigidDynamicComponent::getAngularVelocity,
                    &PhysxRigidDynamicComponent::setAngularVelocity)
      .def("get_angular_velocity", &PhysxRigidDynamicComponent::getAngularVelocity)
      .def("set_angular_velocity", &PhysxRigidDynamicComponent::setAngularVelocity,
           py::arg("velocity"))

      .def_property_readonly("locked_motion_axes",
                             &PhysxRigidDynamicComponent::getLockedMotionAxes)
      .def("get_locked_motion_axes", &PhysxRigidDynamicComponent::getLockedMotionAxes)
      .def("set_locked_motion_axes", &PhysxRigidDynamicComponent::setLockedMotionAxes,
           py::arg("axes"),
           R"doc(
set some motion axes of the dynamic rigid body to be locked
Args:
    axes: list of 6 true/false values indicating whether which  of the 6 DOFs of the body is locked.
          The order is linear X, Y, Z followed by angular X, Y, Z.

Example:
    set_locked_motion_axes([True, False, False, False, True, False]) allows the object to move along the X axis and rotate about the Y axis
)doc")

      .def_property_readonly("is_sleeping", &PhysxRigidDynamicComponent::isSleeping)
      .def("wake_up", &PhysxRigidDynamicComponent::wakeUp)
      .def("put_to_sleep", &PhysxRigidDynamicComponent::putToSleep)

      .def_property("solver_position_iterations",
                    &PhysxRigidDynamicComponent::getSolverPositionIterations,
                    &PhysxRigidDynamicComponent::setSolverPositionIterations)
      .def("get_solver_position_iterations",
           &PhysxRigidDynamicComponent::getSolverPositionIterations)
      .def("set_solver_position_iterations",
           &PhysxRigidDynamicComponent::setSolverPositionIterations, py::arg("count"))

      .def_property("solver_velocity_iterations",
                    &PhysxRigidDynamicComponent::getSolverVelocityIterations,
                    &PhysxRigidDynamicComponent::setSolverVelocityIterations)
      .def("get_solver_velocity_iterations",
           &PhysxRigidDynamicComponent::getSolverVelocityIterations)
      .def("set_solver_velocity_iterations",
           &PhysxRigidDynamicComponent::setSolverVelocityIterations, py::arg("count"))

      .def_property("sleep_threshold", &PhysxRigidDynamicComponent::getSleepThreshold,
                    &PhysxRigidDynamicComponent::setSleepThreshold)
      .def("get_sleep_threshold", &PhysxRigidDynamicComponent::getSleepThreshold)
      .def("set_sleep_threshold", &PhysxRigidDynamicComponent::setSleepThreshold,
           py::arg("threshold"))

      .def_property("kinematic", &PhysxRigidDynamicComponent::isKinematic,
                    &PhysxRigidDynamicComponent::setKinematic)
      .def("get_kinematic", &PhysxRigidDynamicComponent::isKinematic)
      .def("set_kinematic", &PhysxRigidDynamicComponent::setKinematic, py::arg("kinematic"))

      .def_property("kinematic_target", &PhysxRigidDynamicComponent::getKinematicTarget,
                    &PhysxRigidDynamicComponent::setKinematicTarget)
      .def("get_kinematic_target", &PhysxRigidDynamicComponent::getKinematicTarget)
      .def("set_kinematic_target", &PhysxRigidDynamicComponent::setKinematicTarget,
           py::arg("target"))

      .def_property_readonly("gpu_index", &PhysxRigidDynamicComponent::getGpuIndex)
      .def("get_gpu_index", &PhysxRigidDynamicComponent::getGpuIndex)
      .def_property_readonly("gpu_pose_index", &PhysxRigidDynamicComponent::getGpuPoseIndex)
      .def("get_gpu_pose_index", &PhysxRigidDynamicComponent::getGpuPoseIndex);

  PyPhysxArticulationLinkComponent
      .def(py::init(&PhysxArticulationLinkComponent::Create), py::arg("parent") = nullptr)

      .def_property_readonly("is_root", &PhysxArticulationLinkComponent::isRoot)

      .def_property_readonly("parent", &PhysxArticulationLinkComponent::getParent)
      .def("get_parent", &PhysxArticulationLinkComponent::getParent)
      .def("set_parent", &PhysxArticulationLinkComponent::setParent, py::arg("parent"))

      .def_property_readonly("children", &PhysxArticulationLinkComponent::getChildren)
      .def("get_children", &PhysxArticulationLinkComponent::getChildren)

      .def_property_readonly("articulation", &PhysxArticulationLinkComponent::getArticulation)
      .def("get_articulation", &PhysxArticulationLinkComponent::getArticulation)

      .def_property_readonly("joint", &PhysxArticulationLinkComponent::getJoint)
      .def("get_joint", &PhysxArticulationLinkComponent::getJoint)

      .def_property_readonly("sleeping", &PhysxArticulationLinkComponent::isSleeping)
      .def("wake_up", &PhysxArticulationLinkComponent::wakeUp)
      .def("put_to_sleep", &PhysxArticulationLinkComponent::putToSleep)

      .def_property_readonly("index", &PhysxArticulationLinkComponent::getIndex)
      .def("get_index", &PhysxArticulationLinkComponent::getIndex)

      .def_property_readonly("gpu_pose_index", &PhysxArticulationLinkComponent::getGpuPoseIndex)
      .def("get_gpu_pose_index", &PhysxArticulationLinkComponent::getGpuPoseIndex);

  PyPhysxArticulationJoint
      .def_property("name", &PhysxArticulationJoint::getName, &PhysxArticulationJoint::setName)
      .def("get_name", &PhysxArticulationJoint::getName)
      .def("set_name", &PhysxArticulationJoint::setName, py::arg("name"))

      .def_property_readonly("dof", &PhysxArticulationJoint::getDof)
      .def("get_dof", &PhysxArticulationJoint::getDof)

      .def_property("type", &PhysxArticulationJoint::getType, &PhysxArticulationJoint::setType)
      .def("get_type", &PhysxArticulationJoint::getType)
      .def("set_type", &PhysxArticulationJoint::setType, py::arg("type"))

      .def_property("pose_in_parent", &PhysxArticulationJoint::getAnchorPoseInParent,
                    &PhysxArticulationJoint::setAnchorPoseInParent)
      .def("get_pose_in_parent", &PhysxArticulationJoint::getAnchorPoseInParent)
      .def("set_pose_in_parent", &PhysxArticulationJoint::setAnchorPoseInParent, py::arg("pose"))

      .def_property("pose_in_child", &PhysxArticulationJoint::getAnchorPoseInChild,
                    &PhysxArticulationJoint::setAnchorPoseInChild)
      .def("get_pose_in_child", &PhysxArticulationJoint::getAnchorPoseInChild)
      .def("set_pose_in_child", &PhysxArticulationJoint::setAnchorPoseInChild, py::arg("pose"))

      .def_property("friction", &PhysxArticulationJoint::getFriction,
                    &PhysxArticulationJoint::setFriction)
      .def("get_friction", &PhysxArticulationJoint::getFriction)
      .def("set_friction", &PhysxArticulationJoint::setFriction, py::arg("friction"))

      .def_property("max_velocity", &PhysxArticulationJoint::getMaxVelocity,
                    &PhysxArticulationJoint::setMaxVelocity)
      .def("get_max_velocity", &PhysxArticulationJoint::getMaxVelocity)
      .def("set_max_velocity", &PhysxArticulationJoint::setMaxVelocity, py::arg("velocity"))

      .def_property("limit", &PhysxArticulationJoint::getLimit, &PhysxArticulationJoint::setLimit)
      .def_property("limits", &PhysxArticulationJoint::getLimit, &PhysxArticulationJoint::setLimit)
      .def("get_limit", &PhysxArticulationJoint::getLimit, R"doc(same as get_limits)doc")
      .def("get_limits", &PhysxArticulationJoint::getLimit)
      .def("set_limit", &PhysxArticulationJoint::setLimit, py::arg("limit"),
           R"doc(same as set_limits)doc")
      .def("set_limits", &PhysxArticulationJoint::setLimit, py::arg("limit"))

      .def_property("armature", &PhysxArticulationJoint::getArmature,
                    &PhysxArticulationJoint::setArmature)
      .def("get_armature", &PhysxArticulationJoint::getArmature)
      .def("set_armature", &PhysxArticulationJoint::setArmature, py::arg("armature"))

      .def("set_drive_properties", &PhysxArticulationJoint::setDriveProperties,
           py::arg("stiffness"), py::arg("damping"), py::arg("force_limit") = PX_MAX_F32,
           py::arg("mode") = ::physx::PxForceMode::eFORCE)
      .def("set_drive_property", &PhysxArticulationJoint::setDriveProperties, py::arg("stiffness"),
           py::arg("damping"), py::arg("force_limit") = PX_MAX_F32,
           py::arg("mode") = ::physx::PxForceMode::eFORCE, R"doc(same as set_drive_properties)doc")

      .def_property("drive_target", &PhysxArticulationJoint::getDriveTargetPosition,
                    py::overload_cast<Eigen::VectorXf const &>(
                        &PhysxArticulationJoint::setDriveTargetPosition))
      .def("set_drive_target",
           py::overload_cast<float>(&PhysxArticulationJoint::setDriveTargetPosition),
           py::arg("target"))
      .def("set_drive_target",
           py::overload_cast<Eigen::VectorXf const &>(
               &PhysxArticulationJoint::setDriveTargetPosition),
           py::arg("target"))
      .def("get_drive_target", &PhysxArticulationJoint::getDriveTargetPosition)

      .def_property("drive_velocity_target", &PhysxArticulationJoint::getDriveTargetVelocity,
                    py::overload_cast<Eigen::VectorXf const &>(
                        &PhysxArticulationJoint::setDriveTargetVelocity))
      .def("set_drive_velocity_target",
           py::overload_cast<float>(&PhysxArticulationJoint::setDriveTargetVelocity),
           py::arg("velocity"))
      .def("set_drive_velocity_target",
           py::overload_cast<Eigen::VectorXf const &>(
               &PhysxArticulationJoint::setDriveTargetVelocity),
           py::arg("velocity"))
      .def("get_drive_velocity_target", &PhysxArticulationJoint::getDriveTargetVelocity)

      .def_property_readonly("stiffness", &PhysxArticulationJoint::getDriveStiffness)
      .def("get_stiffness", &PhysxArticulationJoint::getDriveStiffness)

      .def_property_readonly("damping", &PhysxArticulationJoint::getDriveDamping)
      .def("get_damping", &PhysxArticulationJoint::getDriveDamping)

      .def_property_readonly("force_limit", &PhysxArticulationJoint::getDriveForceLimit)
      .def("get_force_limit", &PhysxArticulationJoint::getDriveForceLimit)

      .def_property_readonly("drive_mode", &PhysxArticulationJoint::getDriveType)
      .def("get_drive_mode", &PhysxArticulationJoint::getDriveType)

      .def_property_readonly("parent_link", &PhysxArticulationJoint::getParentLink)
      .def("get_parent_link", &PhysxArticulationJoint::getParentLink)

      .def_property_readonly("child_link", &PhysxArticulationJoint::getChildLink)
      .def("get_child_link", &PhysxArticulationJoint::getChildLink)

      .def_property_readonly("global_pose", &PhysxArticulationJoint::getGlobalPose)
      .def("get_global_pose", &PhysxArticulationJoint::getGlobalPose);

  PyPhysxArticulation
      .def_property("name", &PhysxArticulation::getName, &PhysxArticulation::setName)
      .def("get_name", &PhysxArticulation::getName)
      .def("set_name", &PhysxArticulation::setName, py::arg("name"))

      .def_property_readonly("dof", &PhysxArticulation::getDof)
      .def("get_dof", &PhysxArticulation::getDof)

      .def_property_readonly("root", &PhysxArticulation::getRoot)
      .def("get_root", &PhysxArticulation::getRoot)

      .def_property_readonly("links", &PhysxArticulation::getLinks)
      .def("get_links", &PhysxArticulation::getLinks)

      .def_property_readonly("joints", &PhysxArticulation::getJoints)
      .def("get_joints", &PhysxArticulation::getJoints)

      .def_property_readonly("active_joints", &PhysxArticulation::getActiveJoints)
      .def("get_active_joints", &PhysxArticulation::getActiveJoints)

      .def_property("qpos", &PhysxArticulation::getQpos, &PhysxArticulation::setQpos)
      .def("get_qpos", &PhysxArticulation::getQpos)
      .def("set_qpos", &PhysxArticulation::setQpos, py::arg("qpos"))

      .def_property("qvel", &PhysxArticulation::getQvel, &PhysxArticulation::setQvel)
      .def("get_qvel", &PhysxArticulation::getQvel)
      .def("set_qvel", &PhysxArticulation::setQvel, py::arg("qvel"))

      .def_property("qacc", &PhysxArticulation::getQacc, &PhysxArticulation::setQacc)
      .def("get_qacc", &PhysxArticulation::getQacc)
      .def("set_qacc", &PhysxArticulation::setQacc, py::arg("qacc"))

      .def_property("qf", &PhysxArticulation::getQf, &PhysxArticulation::setQf)
      .def("get_qf", &PhysxArticulation::getQf)
      .def("set_qf", &PhysxArticulation::setQf, py::arg("qf"))

      .def_property_readonly("qlimit", &PhysxArticulation::getQLimit)
      .def_property_readonly("qlimits", &PhysxArticulation::getQLimit)
      .def("get_qlimit", &PhysxArticulation::getQLimit, R"doc(same as get_qlimit)doc")
      .def("get_qlimits", &PhysxArticulation::getQLimit)

      .def_property_readonly("link_incoming_joint_forces",
                             &PhysxArticulation::getLinkIncomingJointForces)
      .def("get_link_incoming_joint_forces", &PhysxArticulation::getLinkIncomingJointForces)

      .def_property("root_pose", &PhysxArticulation::getRootPose, &PhysxArticulation::setRootPose)
      .def("get_root_pose", &PhysxArticulation::getRootPose)
      .def("set_root_pose", &PhysxArticulation::setRootPose, py::arg("pose"))
      .def_property("root_linear_velocity", &PhysxArticulation::getRootLinearVelocity,
                    &PhysxArticulation::setRootLinearVelocity)
      .def("get_root_linear_velocity", &PhysxArticulation::getRootLinearVelocity)
      .def("set_root_linear_velocity", &PhysxArticulation::setRootLinearVelocity,
           py::arg("velocity"))
      .def_property("root_angular_velocity", &PhysxArticulation::getRootAngularVelocity,
                    &PhysxArticulation::setRootAngularVelocity)
      .def("get_root_angular_velocity", &PhysxArticulation::getRootAngularVelocity)
      .def("set_root_angular_velocity", &PhysxArticulation::setRootAngularVelocity,
           py::arg("velocity"))

      .def_property("pose", &PhysxArticulation::getRootPose, &PhysxArticulation::setRootPose)
      .def("set_pose", &PhysxArticulation::setRootPose, py::arg("pose"))
      .def("get_pose", &PhysxArticulation::getRootPose)
      .def("compute_passive_force", &PhysxArticulation::computePassiveForce,
           py::arg("gravity") = true, py::arg("coriolis_and_centrifugal") = true)

      .def_property("solver_position_iterations", &PhysxArticulation::getSolverPositionIterations,
                    &PhysxArticulation::setSolverPositionIterations)
      .def("get_solver_position_iterations", &PhysxArticulation::getSolverPositionIterations)
      .def("set_solver_position_iterations", &PhysxArticulation::setSolverPositionIterations,
           py::arg("count"))

      .def_property("solver_velocity_iterations", &PhysxArticulation::getSolverVelocityIterations,
                    &PhysxArticulation::setSolverVelocityIterations)
      .def("get_solver_velocity_iterations", &PhysxArticulation::getSolverVelocityIterations)
      .def("set_solver_velocity_iterations", &PhysxArticulation::setSolverVelocityIterations,
           py::arg("count"))

      .def_property("sleep_threshold", &PhysxArticulation::getSleepThreshold,
                    &PhysxArticulation::setSleepThreshold)
      .def("get_sleep_threshold", &PhysxArticulation::getSleepThreshold)
      .def("set_sleep_threshold", &PhysxArticulation::setSleepThreshold, py::arg("threshold"))

      .def("create_fixed_tendon", &PhysxArticulation::createFixedTendon, py::arg("link_chain"),
           py::arg("coefficients"), py::arg("recip_coefficients"), py::arg("rest_length") = 0,
           py::arg("offset") = 0, py::arg("stiffness") = 0, py::arg("damping") = 0,
           py::arg("low") = -PX_MAX_F32, py::arg("high") = PX_MAX_F32,
           py::arg("limit_stiffness") = 0)

      .def(
          "find_link_by_name",
          [](PhysxArticulation &a, std::string const &name) {
            for (auto &l : a.getLinks()) {
              if (l->getName() == name) {
                return l;
              }
            }
            return std::shared_ptr<PhysxArticulationLinkComponent>{};
          },
          py::arg("name"))
      .def(
          "find_joint_by_name",
          [](PhysxArticulation &a, std::string const &name) {
            for (auto &j : a.getJoints()) {
              if (j->getName() == name) {
                return j;
              }
            }
            return std::shared_ptr<PhysxArticulationJoint>{};
          },
          py::arg("name"))

      .def("clone_links",
           [](PhysxArticulation &a) {
             return PhysxArticulationLinkComponent::cloneArticulation(a.getRoot());
           })

      .def_property_readonly("gpu_index", &PhysxArticulation::getGpuIndex)
      .def("get_gpu_index", &PhysxArticulation::getGpuIndex);

  PyPhysxJointComponent
      .def_property("parent", &PhysxJointComponent::getParent, &PhysxJointComponent::setParent)
      .def("get_parent", &PhysxJointComponent::getParent)
      .def("set_parent", &PhysxJointComponent::setParent, py::arg("parent"))

      .def_property("pose_in_parent", &PhysxJointComponent::getParentAnchorPose,
                    &PhysxJointComponent::setParentAnchorPose)
      .def("get_pose_in_parent", &PhysxJointComponent::getParentAnchorPose)
      .def("set_pose_in_parent", &PhysxJointComponent::setParentAnchorPose, py::arg("pose"))

      .def_property("pose_in_child", &PhysxJointComponent::getChildAnchorPose,
                    &PhysxJointComponent::setChildAnchorPose)
      .def("get_pose_in_child", &PhysxJointComponent::getChildAnchorPose)
      .def("set_pose_in_child", &PhysxJointComponent::setChildAnchorPose, py::arg("pose"))

      .def_property_readonly("relative_pose", &PhysxJointComponent::getRelativePose)
      .def("get_relative_pose", &PhysxJointComponent::getRelativePose)
      .def("set_inv_mass_scales", &PhysxJointComponent::setInvMassScales, py::arg("scale0"),
           py::arg("scale1"))
      .def("set_inv_inertia_scales", &PhysxJointComponent::setInvInertiaScales, py::arg("scale0"),
           py::arg("scale1"));

  PyPhysxDriveComponent.def(py::init(&PhysxDriveComponent::Create), py::arg("body"))
      .def_property("drive_target", &PhysxDriveComponent::getDriveTarget,
                    &PhysxDriveComponent::setDriveTarget)
      .def("get_drive_target", &PhysxDriveComponent::getDriveTarget)
      .def("set_drive_target", &PhysxDriveComponent::setDriveTarget, py::arg("target"))

      .def("set_drive_velocity_target", &PhysxDriveComponent::setDriveTargetVelocity,
           py::arg("linear"), py::arg("angular"))
      .def("get_drive_velocity_target", &PhysxDriveComponent::getDriveTargetVelocity)

      .def("set_limit_x", &PhysxDriveComponent::setXLimit, py::arg("low"), py::arg("high"),
           py::arg("stiffness") = 0.f, py::arg("damping") = 0.f)
      .def("set_limit_y", &PhysxDriveComponent::setYLimit, py::arg("low"), py::arg("high"),
           py::arg("stiffness") = 0.f, py::arg("damping") = 0.f)
      .def("set_limit_z", &PhysxDriveComponent::setZLimit, py::arg("low"), py::arg("high"),
           py::arg("stiffness") = 0.f, py::arg("damping") = 0.f)
      .def("set_limit_cone", &PhysxDriveComponent::setYZConeLimit, py::arg("angle_y"),
           py::arg("angle_z"), py::arg("stiffness") = 0.f, py::arg("damping") = 0.f)
      .def("set_limit_pyramid", &PhysxDriveComponent::setYZPyramidLimit, py::arg("low_y"),
           py::arg("high_y"), py::arg("low_z"), py::arg("high_z"), py::arg("stiffness") = 0.f,
           py::arg("damping") = 0.f)
      .def("set_limit_twist", &PhysxDriveComponent::setXTwistLimit, py::arg("low"),
           py::arg("high"), py::arg("stiffness") = 0.f, py::arg("damping") = 0.f)

      .def("get_limit_x", &PhysxDriveComponent::getXLimit)
      .def("get_limit_y", &PhysxDriveComponent::getYLimit)
      .def("get_limit_z", &PhysxDriveComponent::getZLimit)
      .def("get_limit_cone", &PhysxDriveComponent::getYZConeLimit)
      .def("get_limit_pyramid", &PhysxDriveComponent::getZPyramidLimit)
      .def("get_limit_twist", &PhysxDriveComponent::getXTwistLimit)

      .def("set_drive_property_x", &PhysxDriveComponent::setXDriveProperties, py::arg("stiffness"),
           py::arg("damping"), py::arg("force_limit") = PX_MAX_F32,
           py::arg("mode") = PhysxDriveComponent::DriveMode::eFORCE)
      .def("set_drive_property_y", &PhysxDriveComponent::setYDriveProperties, py::arg("stiffness"),
           py::arg("damping"), py::arg("force_limit") = PX_MAX_F32,
           py::arg("mode") = PhysxDriveComponent::DriveMode::eFORCE)
      .def("set_drive_property_z", &PhysxDriveComponent::setZDriveProperties, py::arg("stiffness"),
           py::arg("damping"), py::arg("force_limit") = PX_MAX_F32,
           py::arg("mode") = PhysxDriveComponent::DriveMode::eFORCE)
      .def("set_drive_property_twist", &PhysxDriveComponent::setXTwistDriveProperties,
           py::arg("stiffness"), py::arg("damping"), py::arg("force_limit") = PX_MAX_F32,
           py::arg("mode") = PhysxDriveComponent::DriveMode::eFORCE)
      .def("set_drive_property_swing", &PhysxDriveComponent::setYZSwingDriveProperties,
           py::arg("stiffness"), py::arg("damping"), py::arg("force_limit") = PX_MAX_F32,
           py::arg("mode") = PhysxDriveComponent::DriveMode::eFORCE)
      .def("set_drive_property_slerp", &PhysxDriveComponent::setSlerpDriveProperties,
           py::arg("stiffness"), py::arg("damping"), py::arg("force_limit") = PX_MAX_F32,
           py::arg("mode") = PhysxDriveComponent::DriveMode::eFORCE)

      .def("get_drive_property_x", &PhysxDriveComponent::getXDriveProperties)
      .def("get_drive_property_y", &PhysxDriveComponent::getYDriveProperties)
      .def("get_drive_property_z", &PhysxDriveComponent::getZDriveProperties)
      .def("get_drive_property_twist", &PhysxDriveComponent::getXTwistDriveProperties)
      .def("get_drive_property_swing", &PhysxDriveComponent::getYZSwingDriveProperties)
      .def("get_drive_property_slerp", &PhysxDriveComponent::getSlerpDriveProperties);

  PyPhysxGearComponent.def(py::init(&PhysxGearComponent::Create), py::arg("body"))
      .def_property("gear_ratio", &PhysxGearComponent::getGearRatio,
                    &PhysxGearComponent::setGearRatio)
      .def("get_gear_ratio", &PhysxGearComponent::getGearRatio)
      .def("set_gear_ratio", &PhysxGearComponent::setGearRatio, py::arg("ratio"))

      .def("enable_hinges", &PhysxGearComponent::enableHinges)
      .def_property_readonly("is_hinges_enabled", &PhysxGearComponent::getHingesEnabled);

  PyPhysxDistanceJointComponent
      .def(py::init(&PhysxDistanceJointComponent::Create), py::arg("body"))
      .def("set_limit", &PhysxDistanceJointComponent::setLimit, py::arg("low"), py::arg("high"),
           py::arg("stiffness") = 0.f, py::arg("damping") = 0.f)

      .def_property_readonly("distance", &PhysxDistanceJointComponent::getDistance)
      .def("get_distance", &PhysxDistanceJointComponent::getDistance);

  ////////// global //////////

  m.def("clear_cache", MeshManager::Clear)
      .def("set_default_material", &PhysxDefault::SetDefaultMaterial, py::arg("static_friction"),
           py::arg("dynamic_friction"), py::arg("restitution"))
      .def("get_default_material", &PhysxDefault::GetDefaultMaterial)
      .def("_enable_gpu", &PhysxDefault::EnableGPU)
      .def("is_gpu_enabled", &PhysxDefault::GetGPUEnabled)
      .def("set_gpu_memory_config", &PhysxDefault::setGpuMemoryConfig,
           py::arg("temp_buffer_capacity") = 16 * 1024 * 1024,
           py::arg("max_rigid_contact_count") = 1024 * 512,
           py::arg("max_rigid_patch_count") = 1024 * 80,
           py::arg("heap_capacity") = 64 * 1024 * 1024,
           py::arg("found_lost_pairs_capacity") = 256 * 1024,
           py::arg("found_lost_aggregate_pairs_capacity") = 1024,
           py::arg("total_aggregate_pairs_capacity") = 1024,
           py::arg("collision_stack_size") = 64 * 64 * 1024)

      .def("set_scene_config",
           py::overload_cast<Vec3, float, bool, bool, bool, bool, bool, uint32_t>(
               &PhysxDefault::setSceneConfig),
           py::arg("gravity") = Vec3{0, 0, -9.81}, py::arg("bounce_threshold") = 2.f,
           py::arg("enable_pcm") = true, py::arg("enable_tgs") = true,
           py::arg("enable_ccd") = false, py::arg("enable_enhanced_determinism") = false,
           py::arg("enable_friction_every_iteration") = true, py::arg("cpu_workers") = 0)
      .def("set_scene_config",
           py::overload_cast<PhysxSceneConfig const &>(&PhysxDefault::setSceneConfig),
           py::arg("config"))
      .def("get_scene_config", &PhysxDefault::getSceneConfig)

      .def("set_shape_config", py::overload_cast<float, float>(&PhysxDefault::setShapeConfig),
           py::arg("contact_offset") = 0.01f, py::arg("rest_offset") = 0.f)
      .def("set_shape_config",
           py::overload_cast<PhysxShapeConfig const &>(&PhysxDefault::setShapeConfig),
           py::arg("config"))
      .def("get_shape_config", &PhysxDefault::getShapeConfig)

      .def("set_body_config",
           py::overload_cast<uint32_t, uint32_t, float>(&PhysxDefault::setBodyConfig),
           py::arg("solver_position_iterations") = 10, py::arg("solver_velocity_iterations") = 1,
           py::arg("sleep_threshold") = 0.005f)
      .def("set_body_config",
           py::overload_cast<PhysxBodyConfig const &>(&PhysxDefault::setBodyConfig),
           py::arg("config"))
      .def("get_body_config", &PhysxDefault::getBodyConfig)

      .def("set_sdf_config",
           py::overload_cast<float, uint32_t, uint32_t>(&PhysxDefault::setSDFShapeConfig),
           py::arg("spacing") = 0.01f, py::arg("subgrid_size") = 6,
           py::arg("num_threads_for_construction") = 4)
      .def("set_sdf_config",
           py::overload_cast<PhysxSDFShapeConfig const &>(&PhysxDefault::setSDFShapeConfig),
           py::arg("config"))
      .def("get_sdf_config", &PhysxDefault::getSDFShapeConfig)

      .def("version", []() { return PhysxDefault::getPhysxVersion(); });

  ////////// end global //////////

  auto atexit = py::module_::import("atexit");
  atexit.attr("register")(py::cpp_function([]() { MeshManager::Clear(); }));
}
