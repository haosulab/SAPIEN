#include "sapien/component/physx/physx.h"
#include "generator.hpp"
#include "sapien_type_caster.h"
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/smart_holder.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace sapien;
using namespace sapien::component;

namespace pybind11::detail {

template <> struct type_caster<physx::PxArticulationJointType::Enum> {
  PYBIND11_TYPE_CASTER(
      physx::PxArticulationJointType::Enum,
      _("typing.Literal['fixed', 'revolute', 'revolute_unwrapped', 'prismatic', 'free']"));

  bool load(py::handle src, bool convert) {
    std::string name = py::cast<std::string>(src);
    if (name == "fixed") {
      value = physx::PxArticulationJointType::eFIX;
      return true;
    } else if (name == "revolute" || name == "hinge" || name == "continuous") {
      value = physx::PxArticulationJointType::eREVOLUTE;
      return true;
    } else if (name == "revolute_unwrapped") {
      value = physx::PxArticulationJointType::eREVOLUTE_UNWRAPPED;
      return true;
    } else if (name == "prismatic" || name == "slider") {
      value = physx::PxArticulationJointType::ePRISMATIC;
      return true;
    } else if (name == "spherical" || name == "ball") {
      value = physx::PxArticulationJointType::eSPHERICAL;
      return true;
    } else if (name == "undefined" || name == "free") {
      value = physx::PxArticulationJointType::eUNDEFINED;
      return true;
    }
    return false;
  }

  static py::handle cast(physx::PxArticulationJointType::Enum const &src,
                         py::return_value_policy policy, py::handle parent) {
    switch (src) {
    case physx::PxArticulationJointType::eFIX:
      return py::str("fixed").release();
    case physx::PxArticulationJointType::ePRISMATIC:
      return py::str("prismatic").release();
    case physx::PxArticulationJointType::eREVOLUTE:
      return py::str("revolute").release();
    case physx::PxArticulationJointType::eREVOLUTE_UNWRAPPED:
      return py::str("revolute_unwrapped").release();
    case physx::PxArticulationJointType::eSPHERICAL:
      return py::str("spherical").release();
    case physx::PxArticulationJointType::eUNDEFINED:
      return py::str("undefined").release();
    }
    throw std::runtime_error("invalid joint type");
  }
};

template <> struct type_caster<physx::PxArticulationDriveType::Enum> {
  PYBIND11_TYPE_CASTER(physx::PxArticulationDriveType::Enum,
                       _("typing.Literal['force', 'acceleration']"));

  bool load(py::handle src, bool convert) {
    std::string name = py::cast<std::string>(src);
    if (name == "force") {
      value = physx::PxArticulationDriveType::eFORCE;
      return true;
    } else if (name == "acceleration" || name == "acc") {
      value = physx::PxArticulationDriveType::eACCELERATION;
      return true;
    } else if (name == "target") {
      value = physx::PxArticulationDriveType::eTARGET;
      return true;
    } else if (name == "velocity") {
      value = physx::PxArticulationDriveType::eVELOCITY;
      return true;
    } else if (name == "none") {
      value = physx::PxArticulationDriveType::eNONE;
      return true;
    }
    return false;
  }

  static py::handle cast(physx::PxArticulationDriveType::Enum const &src,
                         py::return_value_policy policy, py::handle parent) {
    switch (src) {
    case physx::PxArticulationDriveType::eFORCE:
      return py::str("force").release();
    case physx::PxArticulationDriveType::eACCELERATION:
      return py::str("acceleration").release();
    case physx::PxArticulationDriveType::eTARGET:
      return py::str("target").release();
    case physx::PxArticulationDriveType::eVELOCITY:
      return py::str("velocity").release();
    case physx::PxArticulationDriveType::eNONE:
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

template <> struct type_caster<physx::PxForceMode::Enum> {
  PYBIND11_TYPE_CASTER(physx::PxForceMode::Enum,
                       _("typing.Literal['force', 'acceleration', 'velocity_change', 'impulse']"));

  bool load(py::handle src, bool convert) {
    std::string name = py::cast<std::string>(src);
    if (name == "force" || name == "f") {
      value = physx::PxForceMode::eFORCE;
      return true;
    } else if (name == "acceleration" || name == "acc") {
      value = physx::PxForceMode::eACCELERATION;
      return true;
    } else if (name == "velocity_change") {
      value = physx::PxForceMode::eVELOCITY_CHANGE;
      return true;
    } else if (name == "impulse") {
      value = physx::PxForceMode::eIMPULSE;
      return true;
    }
    return false;
  }

  static py::handle cast(physx::PxForceMode::Enum const &src, py::return_value_policy policy,
                         py::handle parent) {
    switch (src) {
    case physx::PxForceMode::eFORCE:
      return py::str("force").release();
    case physx::PxForceMode::eACCELERATION:
      return py::str("acceleration").release();
    case physx::PxForceMode::eVELOCITY_CHANGE:
      return py::str("velocity_change").release();
    case physx::PxForceMode::eIMPULSE:
      return py::str("impulse").release();
    }
    throw std::runtime_error("invalid drive type");
  }
};

} // namespace pybind11::detail

Generator<int> init_physx(py::module &sapien) {
  auto m = sapien.def_submodule("physx");

  m.def("_unload", []() { MeshManager::Clear(); });

  auto PyPhysxSceneConfig = py::class_<PhysxSceneConfig>(m, "PhysxSceneConfig");
  PyPhysxSceneConfig.def(py::init<>())
      .def_readwrite("gravity", &PhysxSceneConfig::gravity)
      .def_readwrite("default_static_friction", &PhysxSceneConfig::static_friction)
      .def_readwrite("default_dynamic_friction", &PhysxSceneConfig::dynamic_friction)
      .def_readwrite("default_restitution", &PhysxSceneConfig::restitution)
      .def_readwrite("bounce_threshold", &PhysxSceneConfig::bounceThreshold)
      .def_readwrite("sleep_threshold", &PhysxSceneConfig::sleepThreshold)
      .def_readwrite("contact_offset", &PhysxSceneConfig::contactOffset)
      .def_readwrite("solver_iterations", &PhysxSceneConfig::solverIterations)
      .def_readwrite("solver_velocity_iterations", &PhysxSceneConfig::solverVelocityIterations)
      .def_readwrite("enable_pcm", &PhysxSceneConfig::enablePCM)
      .def_readwrite("enable_tgs", &PhysxSceneConfig::enableTGS)
      .def_readwrite("enable_ccd", &PhysxSceneConfig::enableCCD)
      .def_readwrite("enable_enhanced_determinism", &PhysxSceneConfig::enableEnhancedDeterminism)
      .def_readwrite("enable_friction_every_iteration",
                     &PhysxSceneConfig::enableFrictionEveryIteration)
      .def("__repr__", [](PhysxSceneConfig &) { return "SceneConfig()"; })
      .def(py::pickle(
          [](PhysxSceneConfig &config) {
            return py::make_tuple(
                config.gravity, config.static_friction, config.dynamic_friction,
                config.restitution, config.bounceThreshold, config.sleepThreshold,
                config.contactOffset, config.solverIterations, config.solverVelocityIterations,
                config.enablePCM, config.enableTGS, config.enableCCD,
                config.enableEnhancedDeterminism, config.enableFrictionEveryIteration);
          },
          [](py::tuple t) {
            if (t.size() != 14) {
              throw std::runtime_error("Invalid state!");
            }
            PhysxSceneConfig config;
            config.gravity = t[0].cast<decltype(config.gravity)>();
            config.static_friction = t[1].cast<decltype(config.static_friction)>();
            config.dynamic_friction = t[2].cast<decltype(config.dynamic_friction)>();
            config.restitution = t[3].cast<decltype(config.restitution)>();
            config.bounceThreshold = t[4].cast<decltype(config.bounceThreshold)>();
            config.sleepThreshold = t[5].cast<decltype(config.sleepThreshold)>();
            config.contactOffset = t[6].cast<decltype(config.contactOffset)>();
            config.solverIterations = t[7].cast<decltype(config.solverIterations)>();
            config.solverVelocityIterations =
                t[8].cast<decltype(config.solverVelocityIterations)>();
            config.enablePCM = t[9].cast<decltype(config.enablePCM)>();
            config.enableTGS = t[10].cast<decltype(config.enableTGS)>();
            config.enableCCD = t[11].cast<decltype(config.enableCCD)>();
            config.enableEnhancedDeterminism =
                t[12].cast<decltype(config.enableEnhancedDeterminism)>();
            config.enableFrictionEveryIteration =
                t[13].cast<decltype(config.enableFrictionEveryIteration)>();
            return config;
          }));

  auto PyPhysxEngine = py::class_<PhysxEngine>(m, "PhysxEngine");
  auto PyPhysxContactPoint = py::class_<ContactPoint>(m, "PhysxContactPoint");
  auto PyPhysxContact = py::class_<Contact>(m, "PhysxContact");

  auto PyPhysxSystem = py::class_<PhysxSystem, System>(m, "PhysxSystem");

  auto PyPhysxBaseComponent = py::class_<PhysxBaseComponent, Component>(m, "PhysxBaseComponent");
  auto PyPhysxRigidBaseComponent =
      py::class_<PhysxRigidBaseComponent, PhysxBaseComponent>(m, "PhysxRigidBaseComponent");
  auto PyPhysxRigidStaticComponent =
      py::class_<PhysxRigidStaticComponent, PhysxRigidBaseComponent>(m,
                                                                     "PhysxRigidStaticComponent");
  auto PyPhysxRigidBodyComponent =
      py::class_<PhysxRigidBodyComponent, PhysxRigidBaseComponent>(m, "PhysxRigidBodyComponent");
  auto PyPhysxRigidDynamicComponent =
      py::class_<PhysxRigidDynamicComponent, PhysxRigidBodyComponent>(
          m, "PhysxRigidDynamicComponent");
  auto PyPhysxArticulationLinkComponent =
      py::class_<PhysxArticulationLinkComponent, PhysxRigidBodyComponent>(
          m, "PhysxArticulationLinkComponent");
  auto PyPhysxJointComponent =
      py::class_<PhysxJointComponent, PhysxBaseComponent>(m, "PhysxJointComponent");
  auto PyPhysxDriveComponent =
      py::class_<PhysxDriveComponent, PhysxJointComponent>(m, "PhysxDriveComponent");
  auto PyPhysxGearComponent =
      py::class_<PhysxGearComponent, PhysxJointComponent>(m, "PhysxGearComponent");
  auto PyPhysxDistanceJointComponent =
      py::class_<PhysxDistanceJointComponent, PhysxJointComponent>(m,
                                                                   "PhysxDistanceJointComponent");

  auto PyPhysxMaterial = py::class_<PhysxMaterial>(m, "PhysxMaterial");

  auto PyPhysxCollisionShape = py::class_<PhysxCollisionShape>(m, "PhysxCollisionShape");
  auto PyPhysxCollisionShapePlane =
      py::class_<PhysxCollisionShapePlane, PhysxCollisionShape>(m, "PhysxCollisionShapePlane");
  auto PyPhysxCollisionShapeBox =
      py::class_<PhysxCollisionShapeBox, PhysxCollisionShape>(m, "PhysxCollisionShapeBox");
  auto PyPhysxCollisionShapeSphere =
      py::class_<PhysxCollisionShapeSphere, PhysxCollisionShape>(m, "PhysxCollisionShapeSphere");
  auto PyPhysxCollisionShapeCapsule =
      py::class_<PhysxCollisionShapeCapsule, PhysxCollisionShape>(m, "PhysxCollisionShapeCapsule");
  auto PyPhysxCollisionShapeConvexMesh =
      py::class_<PhysxCollisionShapeConvexMesh, PhysxCollisionShape>(
          m, "PhysxCollisionShapeConvexMesh");
  auto PyPhysxCollisionShapeTriangleMesh =
      py::class_<PhysxCollisionShapeTriangleMesh, PhysxCollisionShape>(
          m, "PhysxCollisionShapeTriangleMesh");

  auto PyPhysxArticulation = py::class_<PhysxArticulation>(m, "PhysxArticulation");
  auto PyPhysxArticulationJoint = py::class_<PhysxArticulationJoint>(m, "PhysxArticulationJoint");

  co_yield 0;

  PyPhysxEngine.def(py::init(&PhysxEngine::Get), py::arg("tolerance_length"),
                    py::arg("tolerance_speed"));

  PyPhysxContactPoint.def_readonly("position", &ContactPoint::position)
      .def_readonly("normal", &ContactPoint::normal)
      .def_readonly("impulse", &ContactPoint::impulse)
      .def_readonly("separation", &ContactPoint::separation);

  PyPhysxContact
      .def_readonly("components", &Contact::components, py::return_value_policy::reference)
      .def_readonly("shapes", &Contact::shapes, py::return_value_policy::reference)
      .def_readonly("points", &Contact::points)
      .def("__repr__", [](Contact const &c) {
        return "Contact(entity0=" + c.components[0]->getEntity()->getName() +
               ", entity1=" + c.components[1]->getEntity()->getName() + ")";
      });

  PyPhysxSystem.def(py::init<PhysxSceneConfig const &>(), py::arg("config") = PhysxSceneConfig())
      .def_property_readonly("config", &PhysxSystem::getSceneConfig)
      .def("get_config", &PhysxSystem::getSceneConfig)

      .def_property("timestep", &PhysxSystem::getTimestep, &PhysxSystem::setTimestep)
      .def("get_timestep", &PhysxSystem::getTimestep)
      .def("set_timestep", &PhysxSystem::setTimestep)

      .def_property_readonly("rigid_dynamic_components", &PhysxSystem::getRigidDynamicComponents)
      .def("get_rigid_dynamic_components", &PhysxSystem::getRigidDynamicComponents)

      .def_property_readonly("rigid_static_components", &PhysxSystem::getRigidStaticComponents)
      .def("get_rigid_static_components", &PhysxSystem::getRigidStaticComponents)

      .def_property_readonly("articulation_link_components",
                             &PhysxSystem::getArticulationLinkComponents)
      .def("get_articulation_link_components", &PhysxSystem::getArticulationLinkComponents)

      .def("get_contacts", &PhysxSystem::getContacts, py::return_value_policy::reference)
      .def("pack", [](PhysxSystem &s) { return py::bytes(s.packState()); })
      .def(
          "unpack", [](PhysxSystem &s, py::bytes data) { s.unpackState(data); }, py::arg("data"));

  PyPhysxMaterial
      .def(py::init<float, float, float>(), py::arg("static_friction"),
           py::arg("dynamic_friction"), py::arg("restitution"))

      .def_property("dynamic_friction", &PhysxMaterial::getDynamicFriction,
                    &PhysxMaterial::setDynamicFriction)
      .def("get_dynamic_friction", &PhysxMaterial::getDynamicFriction)
      .def("set_dynamic_friction", &PhysxMaterial::setDynamicFriction)

      .def_property("static_friction", &PhysxMaterial::getStaticFriction,
                    &PhysxMaterial::setStaticFriction)
      .def("get_static_friction", &PhysxMaterial::getStaticFriction)
      .def("set_static_friction", &PhysxMaterial::setStaticFriction)

      .def_property("restitution", &PhysxMaterial::getRestitution, &PhysxMaterial::setRestitution)
      .def("get_restitution", &PhysxMaterial::getRestitution)
      .def("set_restitution", &PhysxMaterial::setRestitution);

  PyPhysxCollisionShape
      .def_property("local_pose", &PhysxCollisionShape::getLocalPose,
                    &PhysxCollisionShape::setLocalPose)
      .def("get_local_pose", &PhysxCollisionShape::getLocalPose)
      .def("set_local_pose", &PhysxCollisionShape::setLocalPose)

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
      .def("set_collision_groups", &PhysxCollisionShape::setCollisionGroups, py::arg("groups"))

      .def_property("density", &PhysxCollisionShape::getDensity, &PhysxCollisionShape::setDensity)
      .def("get_density", &PhysxCollisionShape::getDensity)
      .def("set_density", &PhysxCollisionShape::setDensity);

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

  // TODO: support load from vertices
  PyPhysxCollisionShapeConvexMesh
      .def(py::init<std::string const &, Vec3, std::shared_ptr<PhysxMaterial>>(),
           py::arg("filename"), py::arg("scale"), py::arg("material"))
      .def_static("load_multiple", &PhysxCollisionShapeConvexMesh::LoadMultiple,
                  py::arg("filename"), py::arg("scale"), py::arg("material"))

      .def_property_readonly("scale", &PhysxCollisionShapeConvexMesh::getScale)
      .def("get_scale", &PhysxCollisionShapeConvexMesh::getScale)
      .def_property_readonly("vertices", &PhysxCollisionShapeConvexMesh::getVertices)
      .def("get_vertices", &PhysxCollisionShapeConvexMesh::getVertices)
      .def_property_readonly("triangles", &PhysxCollisionShapeConvexMesh::getTriangles)
      .def("get_triangles", &PhysxCollisionShapeConvexMesh::getTriangles);

  PyPhysxCollisionShapeTriangleMesh
      .def(py::init<std::string const &, Vec3, std::shared_ptr<PhysxMaterial>>(),
           py::arg("filename"), py::arg("scale"), py::arg("material"))
      .def_property_readonly("scale", &PhysxCollisionShapeTriangleMesh::getScale)
      .def("get_scale", &PhysxCollisionShapeTriangleMesh::getScale)
      .def_property_readonly("vertices", &PhysxCollisionShapeTriangleMesh::getVertices)
      .def("get_vertices", &PhysxCollisionShapeTriangleMesh::getVertices)
      .def_property_readonly("triangles", &PhysxCollisionShapeTriangleMesh::getTriangles)
      .def("get_triangles", &PhysxCollisionShapeTriangleMesh::getTriangles);

  PyPhysxRigidBaseComponent
      .def("attach", &PhysxRigidBaseComponent::attachCollision, py::arg("collision_shape"))
      .def_property_readonly("collision_shapes", &PhysxRigidBaseComponent::getCollisionShapes)
      .def("get_collision_shapes", &PhysxRigidBaseComponent::getCollisionShapes);

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

      .def_property("mass", &PhysxRigidBodyComponent::getMass, &PhysxRigidBodyComponent::setMass)
      .def("get_mass", &PhysxRigidBodyComponent::getMass)
      .def("set_mass", &PhysxRigidBodyComponent::setMass)

      .def_property("inertia", &PhysxRigidBodyComponent::getInertia,
                    &PhysxRigidBodyComponent::setInertia)
      .def("get_inertia", &PhysxRigidBodyComponent::getInertia)
      .def("set_inertia", &PhysxRigidBodyComponent::setInertia)

      .def_property("cmass_local_pose", &PhysxRigidBodyComponent::getCMassLocalPose,
                    &PhysxRigidBodyComponent::setCMassLocalPose)
      .def("get_cmass_local_pose", &PhysxRigidBodyComponent::getCMassLocalPose)
      .def("set_cmass_local_pose", &PhysxRigidBodyComponent::setCMassLocalPose)

      .def_property_readonly("linear_velocity", &PhysxRigidBodyComponent::getLinearVelocity)
      .def("get_linear_velocity", &PhysxRigidBodyComponent::getLinearVelocity)

      .def_property_readonly("angular_velocity", &PhysxRigidBodyComponent::getAngularVelocity)
      .def("get_angular_velocity", &PhysxRigidBodyComponent::getAngularVelocity)

      .def_property("disable_gravity", &PhysxRigidBodyComponent::getDisableGravity,
                    &PhysxRigidBodyComponent::setDisableGravity)
      .def("get_disable_gravity", &PhysxRigidBodyComponent::getDisableGravity)
      .def("set_disable_gravity", &PhysxRigidBodyComponent::setDisableGravity)

      .def("add_force_torque", &PhysxRigidBodyComponent::addForceTorque, py::arg("force"),
           py::arg("torque"), py::arg("mode") = physx::PxForceMode::eFORCE)
      .def("add_force_at_point", &PhysxRigidBodyComponent::addForceAtPoint, py::arg("force"),
           py::arg("point"), py::arg("mode") = physx::PxForceMode::eFORCE);

  PyPhysxRigidDynamicComponent
      .def(py::init<>())

      .def_property("linear_velocity", &PhysxRigidDynamicComponent::getLinearVelocity,
                    &PhysxRigidDynamicComponent::setLinearVelocity)
      .def("get_linear_velocity", &PhysxRigidDynamicComponent::getLinearVelocity)
      .def("set_linear_velocity", &PhysxRigidDynamicComponent::setLinearVelocity)

      .def_property("angular_velocity", &PhysxRigidDynamicComponent::getAngularVelocity,
                    &PhysxRigidDynamicComponent::setAngularVelocity)
      .def("get_angular_velocity", &PhysxRigidDynamicComponent::getAngularVelocity)
      .def("set_angular_velocity", &PhysxRigidDynamicComponent::setAngularVelocity)

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

      .def_property("kinematic", &PhysxRigidDynamicComponent::isKinematic,
                    &PhysxRigidDynamicComponent::setKinematic)
      .def("get_kinematic", &PhysxRigidDynamicComponent::isKinematic)
      .def("set_kinematic", &PhysxRigidDynamicComponent::setKinematic)

      .def_property("kinematic_target", &PhysxRigidDynamicComponent::getKinematicTarget,
                    &PhysxRigidDynamicComponent::setKinematicTarget)
      .def("get_kinematic_target", &PhysxRigidDynamicComponent::getKinematicTarget)
      .def("set_kinematic_target", &PhysxRigidDynamicComponent::setKinematicTarget);

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
      .def("get_index", &PhysxArticulationLinkComponent::getIndex);

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

      .def_property("limit", &PhysxArticulationJoint::getLimit, &PhysxArticulationJoint::setLimit)
      .def("get_limit", &PhysxArticulationJoint::getLimit)
      .def("set_limit", &PhysxArticulationJoint::setLimit, py::arg("limit"))

      .def_property("armature", &PhysxArticulationJoint::getArmature,
                    &PhysxArticulationJoint::setArmature)
      .def("get_armature", &PhysxArticulationJoint::getArmature)
      .def("set_armature", &PhysxArticulationJoint::setArmature, py::arg("armature"))

      .def("set_drive_properties", &PhysxArticulationJoint::setDriveProperties,
           py::arg("stiffness"), py::arg("damping"), py::arg("force_limit") = PX_MAX_F32,
           py::arg("mode") = physx::PxForceMode::eFORCE)

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
      .def("set_name", &PhysxArticulation::setName)

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
      .def("get_qlimit", &PhysxArticulation::getQLimit)

      .def_property("root_pose", &PhysxArticulation::getRootPose, &PhysxArticulation::setRootPose)
      .def("get_root_pose", &PhysxArticulation::getRootPose)
      .def("set_root_pose", &PhysxArticulation::setRootPose, py::arg("pose"))
      .def_property("root_velocity", &PhysxArticulation::getRootLinearVelocity,
                    &PhysxArticulation::setRootLinearVelocity)
      .def("get_root_velocity", &PhysxArticulation::getRootLinearVelocity)
      .def("set_root_velocity", &PhysxArticulation::setRootLinearVelocity, py::arg("velocity"))
      .def_property("root_angular_velocity", &PhysxArticulation::getRootAngularVelocity,
                    &PhysxArticulation::setRootAngularVelocity)
      .def("get_root_angular_velocity", &PhysxArticulation::getRootAngularVelocity)
      .def("set_root_angular_velocity", &PhysxArticulation::setRootAngularVelocity,
           py::arg("velocity"))

      .def_property_readonly("pose", &PhysxArticulation::getRootPose)
      .def("set_pose", &PhysxArticulation::setRootPose)
      .def("get_pose", &PhysxArticulation::getRootPose)
      .def("compute_passive_force", &PhysxArticulation::computePassiveForce,
           py::arg("gravity") = true, py::arg("coriolis_and_centrifugal") = true)

      .def("find_link_by_name",
           [](PhysxArticulation &a, std::string const &name) {
             for (auto &l : a.getLinks()) {
               if (l->getName() == name) {
                 return l;
               }
             }
             return std::shared_ptr<PhysxArticulationLinkComponent>{};
           })
      .def("find_joint_by_name", [](PhysxArticulation &a, std::string const &name) {
        for (auto &j : a.getJoints()) {
          if (j->getName() == name) {
            return j;
          }
        }
        return std::shared_ptr<PhysxArticulationJoint>{};
      });

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
      .def("get_relative_pose", &PhysxJointComponent::getRelativePose);

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

      .def_property("contact_distance", &PhysxDistanceJointComponent::getContactDistance,
                    &PhysxDistanceJointComponent::setContactDistance)
      .def("get_contact_distance", &PhysxDistanceJointComponent::getContactDistance)
      .def("set_contact_distance", &PhysxDistanceJointComponent::setContactDistance,
           py::arg("distance"))

      .def_property_readonly("distance", &PhysxDistanceJointComponent::getDistance)
      .def("get_distance", &PhysxDistanceJointComponent::getDistance);

  ////////// global //////////

  m.def("set_default_material", &PhysxDefault::setDefaultMaterial, py::arg("static_friction"),
        py::arg("dynamic_friction"), py::arg("restitution"))
      .def("get_default_material", &PhysxDefault::getDefaultMaterial);

  ////////// end global //////////
}
