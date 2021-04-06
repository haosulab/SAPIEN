#pragma once
// #include "utils/torch_tensor.hpp"
#include "utils/dlpack_tensor.hpp"

#include "renderer/optifuser_controller.h"
#include "renderer/optifuser_renderer.h"

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "actor_builder.h"
#include "renderer/render_interface.h"
#include "sapien_actor.h"
#include "sapien_actor_base.h"
#include "sapien_contact.h"
#include "sapien_drive.h"
#include "sapien_material.h"
#include "sapien_scene.h"
#include "simulation.h"

#include "articulation/articulation_builder.h"
#include "articulation/sapien_articulation.h"
#include "articulation/sapien_articulation_base.h"
#include "articulation/sapien_joint.h"
#include "articulation/sapien_kinematic_articulation.h"
#include "articulation/sapien_kinematic_joint.h"
#include "articulation/sapien_link.h"
#include "articulation/urdf_loader.h"
#include "event_system/event_system.h"

#include "renderer/svulkan2_renderer.h"
#include "renderer/svulkan2_window.h"

#ifdef _USE_PINOCCHIO
#include "articulation/pinocchio_model.h"
#endif

#include "utils/pose.hpp"

using namespace sapien;
namespace py = pybind11;

URDF::URDFConfig parseURDFConfig(py::dict &dict) {
  URDF::URDFConfig config;
  if (dict.contains("material")) {
    config.material = dict["material"].cast<std::shared_ptr<SPhysicalMaterial>>();
  }
  if (dict.contains("density")) {
    config.density = dict["density"].cast<float>();
  }
  if (dict.contains("link")) {

    auto linkDict = dict["link"].cast<py::dict>();
    for (auto kv : linkDict) {
      auto name = kv.first.cast<std::string>();
      auto dict2 = kv.second.cast<py::dict>();
      if (dict2.contains("material")) {
        config.link[name].material = dict2["material"].cast<std::shared_ptr<SPhysicalMaterial>>();
      } else {
        config.link[name].material = config.material;
      }
      if (dict2.contains("density")) {
        config.link[name].density = dict2["density"].cast<float>();
      } else {
        config.link[name].density = config.density;
      }
      if (dict2.contains("patch_radius")) {
        config.link[name].patchRadius = dict2["patch_radius"].cast<float>();
      }
      if (dict2.contains("min_patch_radius")) {
        config.link[name].minPatchRadius = dict2["min_patch_radius"].cast<float>();
      }
      if (dict2.contains("shape")) {
        auto shapeDict = dict2["shape"].cast<py::dict>();
        for (auto kv2 : shapeDict) {
          auto idx = kv2.first.cast<int>();
          auto dict3 = kv2.second.cast<py::dict>();

          if (dict3.contains("material")) {
            config.link[name].shape[idx].material =
                dict3["material"].cast<std::shared_ptr<SPhysicalMaterial>>();
          } else {
            config.link[name].shape[idx].material = config.link[name].material;
          }
          if (dict3.contains("density")) {
            config.link[name].shape[idx].density = dict3["density"].cast<float>();
          } else {
            config.link[name].shape[idx].density = config.link[name].density;
          }
          if (dict3.contains("patch_radius")) {
            config.link[name].shape[idx].patchRadius = dict3["patch_radius"].cast<float>();
          } else {
            config.link[name].shape[idx].patchRadius = config.link[name].patchRadius;
          }
          if (dict3.contains("min_patch_radius")) {
            config.link[name].shape[idx].minPatchRadius = dict3["min_patch_radius"].cast<float>();
          } else {
            config.link[name].shape[idx].minPatchRadius = config.link[name].minPatchRadius;
          }
        }
      }
    }
  }
  return config;
}

PxVec3 array2vec3(const py::array_t<PxReal> &arr) { return {arr.at(0), arr.at(1), arr.at(2)}; }

template <typename T> py::array_t<T> make_array(std::vector<T> const &values) {
  return py::array_t(values.size(), values.data());
}

py::array_t<PxReal> vec32array(PxVec3 const &vec) {
  std::vector<PxReal> v = {vec.x, vec.y, vec.z};
  return make_array(v);
}

py::array_t<PxReal> mat42array(glm::mat4 const &mat) {
  float arr[] = {mat[0][0], mat[1][0], mat[2][0], mat[3][0], mat[0][1], mat[1][1],
                 mat[2][1], mat[3][1], mat[0][2], mat[1][2], mat[2][2], mat[3][2],
                 mat[0][3], mat[1][3], mat[2][3], mat[3][3]};
  return py::array_t<PxReal>({4, 4}, arr);
}

void buildSapien(py::module &m) {
  m.doc() = "SAPIEN core module";

  // collision geometry and shape
  auto PyGeometry = py::class_<SGeometry>(m, "CollisionGeometry");
  auto PyBoxGeometry = py::class_<SBoxGeometry, SGeometry>(m, "BoxGeometry");
  auto PySphereGeometry = py::class_<SSphereGeometry, SGeometry>(m, "SphereGeometry");
  auto PyCapsuleGeometry = py::class_<SCapsuleGeometry, SGeometry>(m, "CapsuleGeometry");
  auto PyPlaneGeometry = py::class_<SPlaneGeometry, SGeometry>(m, "PlaneGeometry");
  auto PyConvexMeshGeometry = py::class_<SConvexMeshGeometry, SGeometry>(m, "ConvexMeshGeometry");
  // auto PyShape = py::class_<SShape>(m, "CollisionShape");
  auto PyCollisionShape = py::class_<SCollisionShape>(m, "CollisionShape");

  auto PyURDFLoader = py::class_<URDF::URDFLoader>(m, "URDFLoader");
  auto PyPhysicalMaterial =
      py::class_<SPhysicalMaterial, std::shared_ptr<SPhysicalMaterial>>(m, "PhysicalMaterial");
  auto PyPose = py::class_<PxTransform>(m, "Pose");
  auto PyRenderMaterial =
      py::class_<Renderer::IPxrMaterial, std::shared_ptr<Renderer::IPxrMaterial>>(
          m, "RenderMaterial");
  auto PyOptifuserMaterial =
      py::class_<Renderer::PxrMaterial, Renderer::IPxrMaterial,
                 std::shared_ptr<Renderer::PxrMaterial>>(m, "OptifuserMaterial");
  auto PyRenderer = py::class_<Renderer::IPxrRenderer, std::shared_ptr<Renderer::IPxrRenderer>>(
      m, "IPxrRenderer");
  auto PyRenderScene = py::class_<Renderer::IPxrScene>(m, "RenderScene");
  auto PyRenderBody = py::class_<Renderer::IPxrRigidbody>(m, "RenderBody");
  auto PyISensor = py::class_<Renderer::ISensor>(m, "ISensor");
  auto PyICamera = py::class_<Renderer::ICamera, Renderer::ISensor>(m, "ICamera");

  auto PyOptifuserRenderer =
      py::class_<Renderer::OptifuserRenderer, Renderer::IPxrRenderer,
                 std::shared_ptr<Renderer::OptifuserRenderer>>(m, "OptifuserRenderer");
  auto PyOptifuserConfig = py::class_<Renderer::OptifuserConfig>(m, "OptifuserConfig");
  auto PyInput = py::class_<Optifuser::Input>(m, "Input");
  auto PyOptifuserController = py::class_<Renderer::OptifuserController>(m, "OptifuserController");
  auto PyCameraSpec = py::class_<Optifuser::CameraSpec>(m, "CameraSpec");
  auto PyOptifuserCamera =
      py::class_<Renderer::OptifuserCamera, Renderer::ICamera>(m, "OptifuserCamera");

  auto PyEngine = py::class_<Simulation, std::shared_ptr<Simulation>>(m, "Engine");
  auto PySceneConfig = py::class_<SceneConfig>(m, "SceneConfig");
  auto PyScene = py::class_<SScene>(m, "Scene");
  auto PyDrive = py::class_<SDrive>(m, "Drive");
  auto PyActorBase = py::class_<SActorBase>(m, "ActorBase");
  auto PyActorDynamicBase = py::class_<SActorDynamicBase, SActorBase>(m, "ActorDynamicBase");
  auto PyActorStatic = py::class_<SActorStatic, SActorBase>(m, "ActorStatic");
  auto PyActor = py::class_<SActor, SActorDynamicBase>(m, "Actor");
  auto PyLinkBase = py::class_<SLinkBase, SActorDynamicBase>(m, "LinkBase");
  auto PyLink = py::class_<SLink, SLinkBase>(m, "Link");
  auto PyKinematicLink = py::class_<SKLink, SLinkBase>(m, "KinematicLink");
  auto PyJointBase = py::class_<SJointBase>(m, "JointBase");
  auto PyJoint = py::class_<SJoint, SJointBase>(m, "Joint");
  py::class_<SKJoint, SJointBase>(m, "KinematicJoint");
  py::class_<SKJointFixed, SKJoint>(m, "KinematicJointFixed");
  py::class_<SKJointSingleDof, SKJoint>(m, "KinematicJointSingleDof");
  py::class_<SKJointPrismatic, SKJointSingleDof>(m, "KinematicJointPrismatic");
  py::class_<SKJointRevolute, SKJointSingleDof>(m, "KinematicJointRevolute");
  auto PyArticulationBase = py::class_<SArticulationBase>(m, "ArticulationBase");
  auto PyArticulationDrivable =
      py::class_<SArticulationDrivable, SArticulationBase>(m, "ArticulationDrivable");
  auto PyArticulation = py::class_<SArticulation, SArticulationDrivable>(m, "Articulation");
  py::class_<SKArticulation, SArticulationDrivable>(m, "KinematicArticulation");
  auto PyContact = py::class_<SContact>(m, "Contact");
  auto PyTrigger = py::class_<STrigger>(m, "Trigger");
  auto PyContactPoint = py::class_<SContactPoint>(m, "ContactPoint");

  auto PyActorBuilder = py::class_<ActorBuilder>(m, "ActorBuilder");
  auto PyShapeRecord = py::class_<ActorBuilder::ShapeRecord>(m, "ShapeRecord");
  auto PyVisualRecord = py::class_<ActorBuilder::VisualRecord>(m, "VisualRecord");
  auto PyLinkBuilder = py::class_<LinkBuilder, ActorBuilder>(m, "LinkBuilder");
  auto PyJointRecord = py::class_<LinkBuilder::JointRecord>(m, "JointRecord");
  auto PyArticulationBuilder = py::class_<ArticulationBuilder>(m, "ArticulationBuilder");

  auto PyRenderShape = py::class_<Renderer::RenderShape>(m, "RenderShape");
  auto PyRenderMeshGeometry = py::class_<Renderer::RenderMeshGeometry>(m, "RenderGeometry");

  auto PySubscription = py::class_<Subscription>(m, "Subscription");

#ifdef _USE_PINOCCHIO
  auto PyPinocchioModel = py::class_<PinocchioModel>(m, "PinocchioModel");
#endif

  auto PyVulkanRenderer =
      py::class_<Renderer::SVulkan2Renderer, Renderer::IPxrRenderer,
                 std::shared_ptr<Renderer::SVulkan2Renderer>>(m, "VulkanRenderer");
  auto PyVulkanMaterial =
      py::class_<Renderer::SVulkan2Material, Renderer::IPxrMaterial,
                 std::shared_ptr<Renderer::SVulkan2Material>>(m, "VulkanMaterial");
  auto PyVulkanCamera = py::class_<Renderer::SVulkan2Camera, Renderer::ICamera>(m, "VulkanCamera");
  auto PyVulkanWindow = py::class_<Renderer::SVulkan2Window>(m, "VulkanWindow");
  auto PyVulkanScene = py::class_<Renderer::SVulkan2Scene, Renderer::IPxrScene>(m, "VulkanScene");

  //======== Internal ========//

  PyPhysicalMaterial.def("get_static_friction", &SPhysicalMaterial::getStaticFriction)
      .def("get_dynamic_friction", &SPhysicalMaterial::getDynamicFriction)
      .def("get_restitution", &SPhysicalMaterial::getRestitution)
      .def("set_static_friction", &SPhysicalMaterial::setStaticFriction, py::arg("coef"))
      .def("set_dynamic_friction", &SPhysicalMaterial::setDynamicFriction, py::arg("coef"))
      .def("set_restitution", &SPhysicalMaterial::setRestitution, py::arg("coef"));

  PyPose
      .def(py::init([](py::array_t<PxReal> p, py::array_t<PxReal> q) {
             return new PxTransform({p.at(0), p.at(1), p.at(2)},
                                    {q.at(1), q.at(2), q.at(3), q.at(0)});
           }),
           py::return_value_policy::automatic, py::arg("p") = make_array<float>({0, 0, 0}),
           py::arg("q") = make_array<float>({1, 0, 0, 0}))
      .def_static("from_transformation_matrix", &utils::fromTransFormationMatrix,
                  py::return_value_policy::automatic, py::arg("mat44"))
      .def("to_transformation_matrix", &utils::toTransformationMatrix)
      .def_property_readonly(
          "p", [](PxTransform &t) { return Eigen::Matrix<PxReal, 3, 1>(t.p.x, t.p.y, t.p.z); })
      .def_property_readonly(
          "q",
          [](PxTransform &t) { return Eigen::Matrix<PxReal, 4, 1>(t.q.w, t.q.x, t.q.y, t.q.z); })
      .def("inv", &PxTransform::getInverse)
      .def("__repr__", &utils::poseRepresentation)
      .def("transform", [](PxTransform &t, PxTransform &src) { return t.transform(src); })
      .def(
          "set_p", [](PxTransform &t, const py::array_t<PxReal> &arr) { t.p = array2vec3(arr); },
          py::arg("p"))
      .def(
          "set_q",
          [](PxTransform &t, const py::array_t<PxReal> &arr) {
            t.q = {arr.at(1), arr.at(2), arr.at(3), arr.at(0)}; // NOTE: wxyz to xyzw
          },
          py::arg("q"))
      .def("set_rotation",
           [](PxTransform &t, const py::array_t<PxReal> &rotation) {
             assert(rotation.size() == 9 && rotation.shape()[0] == 3);
             auto um = rotation.unchecked<2>();
             float w = 0.5 * std::sqrt(1.0 + um(0, 0) + um(1, 1) + um(2, 2));
             float over_w = 0.25 / w;
             float x = um(2, 1) - um(1, 2) * over_w;
             float y = um(0, 2) - um(2, 0) * over_w;
             float z = um(1, 0) - um(0, 1) * over_w;
             t.q = {x, y, z, w};
           })
      .def(py::self * py::self);

  //======== Geometry ========//

  PyBoxGeometry.def_property_readonly("half_lengths",
                                      [](SBoxGeometry &g) { return vec32array(g.halfLengths); });
  PySphereGeometry.def_readonly("radius", &SSphereGeometry::radius);
  PyCapsuleGeometry.def_readonly("radius", &SCapsuleGeometry::radius)
      .def_readonly("half_length", &SCapsuleGeometry::halfLength);
  PyConvexMeshGeometry
      .def_property_readonly("scale", [](SConvexMeshGeometry &g) { return vec32array(g.scale); })
      .def_property_readonly(
          "rotation",
          [](SConvexMeshGeometry &g) {
            return make_array<PxReal>({g.rotation.w, g.rotation.x, g.rotation.y, g.rotation.z});
          })
      .def_property_readonly("vertices",
                             [](SConvexMeshGeometry &g) {
                               int nRows = g.vertices.size() / 3;
                               int nCols = 3;
                               return py::array_t<PxReal>({nRows, nCols},
                                                          {sizeof(PxReal) * nCols, sizeof(PxReal)},
                                                          g.vertices.data());
                             })
      .def_property_readonly(
          "indices", [](SConvexMeshGeometry &g) { return make_array<uint32_t>(g.indices); });

  PyCollisionShape
      .def_property_readonly("actor", &SCollisionShape::getActor,
                             py::return_value_policy::reference)

      .def("get_collision_groups", &SCollisionShape::getCollisionGroups)
      .def("set_collision_groups", &SCollisionShape::setCollisionGroups, py::arg("group0"),
           py::arg("group1"), py::arg("group2"), py::arg("group3"))
      .def_property("rest_offset", &SCollisionShape::getRestOffset,
                    &SCollisionShape::setRestOffset)
      .def_property("contact_offset", &SCollisionShape::getContactOffset,
                    &SCollisionShape::setContactOffset)
      .def_property("patch_radius", &SCollisionShape::getTorsionalPatchRadius,
                    &SCollisionShape::setTorsionalPatchRadius)
      .def_property("min_patch_radius", &SCollisionShape::getMinTorsionalPatchRadius,
                    &SCollisionShape::setMinTorsionalPatchRadius)
      .def_property("is_trigger", &SCollisionShape::isTrigger, &SCollisionShape::setIsTrigger)
      .def("get_local_pose", &SCollisionShape::getLocalPose)
      .def("set_local_pose", &SCollisionShape::setLocalPose, py::arg("pose"))
      .def("set_physical_material", &SCollisionShape::setPhysicalMaterial, py::arg("material"))
      .def("get_physical_material", &SCollisionShape::getPhysicalMaterial)

      .def_property_readonly("type", &SCollisionShape::getType)
      .def_property_readonly("geometry", &SCollisionShape::getGeometry);

  // PyShape.def_readonly("type", &SShape::type)
  //     .def_readonly("pose", &SShape::pose)
  //     .def_property_readonly(
  //         "box_geometry"
  //         [](SShape &s) {
  //           if (s.type == "box") {
  //             return static_cast<SBoxGeometry *>(s.geometry.get());
  //           }
  //           return static_cast<SBoxGeometry *>(nullptr);
  //         },
  //         py::return_value_policy::reference)
  //     .def_property_readonly(
  //         "sphere_geometry",
  //         [](SShape &s) {
  //           if (s.type == "sphere") {
  //             return static_cast<SSphereGeometry *>(s.geometry.get());
  //           }
  //           return static_cast<SSphereGeometry *>(nullptr);
  //         },
  //         py::return_value_policy::reference)
  //     .def_property_readonly(
  //         "capsule_geometry",
  //         [](SShape &s) {
  //           if (s.type == "capsule") {
  //             return static_cast<SCapsuleGeometry *>(s.geometry.get());
  //           }
  //           return static_cast<SCapsuleGeometry *>(nullptr);
  //         },
  //         py::return_value_policy::reference)
  //     .def_property_readonly(
  //         "plane_geometry",
  //         [](SShape &s) {
  //           if (s.type == "plane") {
  //             return static_cast<SPlaneGeometry *>(s.geometry.get());
  //           }
  //           return static_cast<SPlaneGeometry *>(nullptr);
  //         },
  //         py::return_value_policy::reference)
  //     .def_property_readonly(
  //         "convex_mesh_geometry",
  //         [](SShape &s) {
  //           if (s.type == "convex_mesh") {
  //             return static_cast<SConvexMeshGeometry *>(s.geometry.get());
  //           }
  //           return static_cast<SConvexMeshGeometry *>(nullptr);
  //         },
  //         py::return_value_policy::reference);

  //======== Render Interface ========//
  PyRenderMaterial
      .def(
          "set_base_color",
          [](Renderer::IPxrMaterial &mat, py::array_t<float> color) {
            mat.setBaseColor({color.at(0), color.at(1), color.at(2), color.at(3)});
          },
          py::arg("rgba"))
      .def("set_specular", &Renderer::IPxrMaterial::setSpecular, py::arg("specular"))
      .def("set_metallic", &Renderer::IPxrMaterial::setMetallic, py::arg("metallic"))
      .def("set_roughness", &Renderer::IPxrMaterial::setRoughness, py::arg("roughness"));

  //     // TODO: implement those together with UV
  //     // .def_readwrite("color_texture", &Renderer::PxrMaterial::color_texture)
  //     // .def_readwrite("specular_texture", &Renderer::PxrMaterial::specular_texture)
  //     // .def_readwrite("normal_texture", &Renderer::PxrMaterial::normal_texture)
  //     ;

  PyISensor.def("set_initial_pose", &Renderer::ISensor::setInitialPose, py::arg("pose"))
      .def("get_pose", &Renderer::ISensor::getPose)
      .def("set_pose", &Renderer::ISensor::setPose, py::arg("pose"));

  PyICamera.def("get_name", &Renderer::ICamera::getName)
      .def("get_width", &Renderer::ICamera::getWidth)
      .def("get_height", &Renderer::ICamera::getHeight)
      .def("get_fovy", &Renderer::ICamera::getFovy)
      .def("get_near", &Renderer::ICamera::getNear)
      .def("get_far", &Renderer::ICamera::getFar)
      .def("take_picture", &Renderer::ICamera::takePicture)
      .def("get_color_rgba",
           [](Renderer::ICamera &cam) {
             return py::array_t<float>(
                 {static_cast<int>(cam.getHeight()), static_cast<int>(cam.getWidth()), 4},
                 cam.getColorRGBA().data());
           })
      .def("get_albedo_rgba",
           [](Renderer::ICamera &cam) {
             return py::array_t<float>(
                 {static_cast<int>(cam.getHeight()), static_cast<int>(cam.getWidth()), 4},
                 cam.getAlbedoRGBA().data());
           })
      .def("get_normal_rgba",
           [](Renderer::ICamera &cam) {
             return py::array_t<float>(
                 {static_cast<int>(cam.getHeight()), static_cast<int>(cam.getWidth()), 4},
                 cam.getNormalRGBA().data());
           })
      .def("get_depth",
           [](Renderer::ICamera &cam) {
             return py::array_t<float>(
                 {static_cast<int>(cam.getHeight()), static_cast<int>(cam.getWidth())},
                 cam.getDepth().data());
           })
      .def("get_segmentation",
           [](Renderer::ICamera &cam) {
             return py::array_t<int>(
                 {static_cast<int>(cam.getHeight()), static_cast<int>(cam.getWidth())},
                 cam.getSegmentation().data());
           })
      .def("get_obj_segmentation", [](Renderer::ICamera &cam) {
        return py::array_t<int>(
            {static_cast<int>(cam.getHeight()), static_cast<int>(cam.getWidth())},
            cam.getObjSegmentation().data());
      });

  PyOptifuserConfig.def(py::init<>())
      .def_readwrite("use_shadow", &Renderer::OptifuserConfig::useShadow)
      .def_readwrite("use_ao", &Renderer::OptifuserConfig::useAo)
      .def_readwrite("shadow_map_size", &Renderer::OptifuserConfig::shadowMapSize)
      .def_readwrite("shadow_frustum_size", &Renderer::OptifuserConfig::shadowFrustumSize);

  PyOptifuserRenderer
      .def("enable_global_axes", &Renderer::OptifuserRenderer::enableGlobalAxes,
           py::arg("enable") = true)
      .def_static("set_default_shader_config", Renderer::OptifuserRenderer::setDefaultShaderConfig,
                  py::arg("glsl_dir"), py::arg("glsl_version"))
#ifdef _USE_OPTIX
      .def_static("set_optix_config", Renderer::OptifuserRenderer::setOptixConfig,
                  py::arg("ptx_dir"))
#endif
      .def(py::init<std::string const &, std::string const &, Renderer::OptifuserConfig const &>(),
           py::arg("glsl_dir") = "", py::arg("glsl_version") = "",
           py::arg("config") = Renderer::OptifuserConfig())
      .def("set_log_level", &Renderer::OptifuserRenderer::setLogLevel, py::arg("level"));

  PyInput.def("get_key_state", &Optifuser::Input::getKeyState)
      .def("get_key_down", &Optifuser::Input::getKeyDown)
      .def("get_key_mods", &Optifuser::Input::getKeyMods);

  PyOptifuserController.def(py::init<Renderer::OptifuserRenderer *>(), py::arg("renderer"))
      .def("show_window", &Renderer::OptifuserController::showWindow)
      .def("hide_window", &Renderer::OptifuserController::hideWindow)
      .def("set_current_scene", &Renderer::OptifuserController::setCurrentScene, py::arg("scene"))
      .def("render", &Renderer::OptifuserController::render)
      .def("set_camera_position", &Renderer::OptifuserController::setCameraPosition, py::arg("x"),
           py::arg("y"), py::arg("z"))
      .def("set_camera_rotation", &Renderer::OptifuserController::setCameraRotation,
           py::arg("yaw"), py::arg("pitch"))
      .def("get_camera_pose", &Renderer::OptifuserController::getCameraPose)
      .def("focus", &Renderer::OptifuserController::focus, py::arg("actor"))
      .def_property_readonly("should_quit", &Renderer::OptifuserController::shouldQuit)
      .def_property_readonly(
          "input", [](Renderer::OptifuserController &) { return Optifuser::getInput(); },
          py::return_value_policy::reference)
      .def("get_selected_actor", &Renderer::OptifuserController::getSelectedActor,
           py::return_value_policy::reference);

  PyCameraSpec.def_readwrite("name", &Optifuser::CameraSpec::name)
      .def(
          "set_position",
          [](Optifuser::CameraSpec &c, const py::array_t<PxReal> &arr) {
            c.position = {arr.at(0), arr.at(1), arr.at(2)};
          },
          py::arg("position"))
      .def(
          "set_rotation",
          [](Optifuser::CameraSpec &c, const py::array_t<PxReal> &arr) {
            c.setRotation({arr.at(0), arr.at(1), arr.at(2), arr.at(3)});
          },
          py::arg("rotation"))
      .def_property_readonly(
          "position",
          [](Optifuser::CameraSpec &c) { return py::array_t<PxReal>(3, (float *)(&c.position)); })
      .def_property_readonly("rotation",
                             [](Optifuser::CameraSpec &c) {
                               auto rot = c.getRotation();
                               return make_array<float>({rot.w, rot.x, rot.y, rot.z});
                             })
      .def_readwrite("near", &Optifuser::CameraSpec::near)
      .def_readwrite("far", &Optifuser::CameraSpec::far)
      .def_readwrite("aspect", &Optifuser::CameraSpec::aspect)
      .def(
          "lookAt",
          [](Optifuser::CameraSpec &c, const py::array_t<PxReal> &dir,
             const py::array_t<PxReal> &up) {
            c.lookAt({dir.at(0), dir.at(1), dir.at(2)}, {up.at(0), up.at(1), up.at(2)});
          },
          py::arg("direction"), py::arg("up"))
      .def("get_model_matrix",
           [](Optifuser::CameraSpec &c) { return mat42array(c.getModelMat()); })
      .def("get_projection_matrix",
           [](Optifuser::CameraSpec &c) { return mat42array(c.getProjectionMat()); });

  PyOptifuserCamera
      .def("get_camera_matrix",
           [](Renderer::OptifuserCamera &c) { return mat42array(c.getCameraMatrix()); })
      .def("get_model_matrix",
           [](Renderer::OptifuserCamera &c) { return mat42array(c.mCameraSpec->getModelMat()); })
      .def("get_projection_matrix",
           [](Renderer::OptifuserCamera &c) {
             return mat42array(c.mCameraSpec->getProjectionMat());
           })
      .def("set_mode_orthographic", &Renderer::OptifuserCamera::changeModeToOrthographic,
           py::arg("scaling") = 1.f)
      .def("set_mode_perspective", &Renderer::OptifuserCamera::changeModeToPerspective,
           py::arg("fovy") = glm::radians(35.f))
      .def("get_custom_rgba",
           [](Renderer::OptifuserCamera &c) {
             return py::array_t<float>(
                 {static_cast<int>(c.getHeight()), static_cast<int>(c.getWidth()), 4},
                 c.getCustomRGBA().data());
           })
#ifdef _USE_OPTIX
      .def(
          "take_raytraced_picture",
          [](Renderer::OptifuserCamera &cam, uint32_t samplesPerPixel, uint32_t reflectionCount,
             bool denoise) {
            return py::array_t<PxReal>(
                {static_cast<int>(cam.getHeight()), static_cast<int>(cam.getWidth()), 4},
                cam.takeRaytracedPicture(samplesPerPixel, reflectionCount, denoise).data());
          },
          py::arg("samples_per_pixel") = 128, py::arg("reflection_count") = 4,
          py::arg("use_denoiser") = true)
#endif
      ;

  //======== Simulation ========//
  PyEngine
      .def(py::init([](uint32_t nthread, PxReal toleranceLength, PxReal toleranceSpeed) {
             return Simulation::getInstance(nthread, toleranceLength, toleranceSpeed);
           }),
           py::arg("thread_count") = 0, py::arg("tolerance_length") = 0.1f,
           py::arg("tolerance_speed") = 0.2f)
      .def("create_scene", &Simulation::createScene, py::arg("config") = SceneConfig())
      .def("get_renderer", &Simulation::getRenderer, py::return_value_policy::reference)
      .def("set_renderer", &Simulation::setRenderer, py::arg("renderer"))
      .def("set_log_level", &Simulation::setLogLevel, py::arg("level"))
      .def("create_physical_material", &Simulation::createPhysicalMaterial,
           py::arg("static_friction"), py::arg("dynamic_friction"), py::arg("restitution"));

  PySceneConfig.def(py::init<>())
      .def_readwrite("gravity", &SceneConfig::gravity)
      .def_readwrite("default_static_friction", &SceneConfig::static_friction)
      .def_readwrite("default_dynamic_friction", &SceneConfig::dynamic_friction)
      .def_readwrite("default_restitution", &SceneConfig::restitution)
      .def_readwrite("bounce_threshold", &SceneConfig::bounceThreshold)
      .def_readwrite("sleep_threshold", &SceneConfig::sleepThreshold)
      .def_readwrite("contact_offset", &SceneConfig::contactOffset)
      .def_readwrite("solver_iterations", &SceneConfig::solverIterations)
      .def_readwrite("solver_velocity_iterations", &SceneConfig::solverVelocityIterations)
      .def_readwrite("enable_pcm", &SceneConfig::enablePCM)
      .def_readwrite("enable_tgs", &SceneConfig::enableTGS)
      .def_readwrite("enable_ccd", &SceneConfig::enableCCD)
      .def_readwrite("enable_enhanced_determinism", &SceneConfig::enableEnhancedDeterminism)
      .def_readwrite("enable_friction_every_iteration", &SceneConfig::enableFrictionEveryIteration)
      .def_readwrite("enable_adaptive_force", &SceneConfig::enableAdaptiveForce);

  PyScene.def_property_readonly("name", &SScene::getName)
      .def("set_timestep", &SScene::setTimestep, py::arg("second"))
      .def("get_timestep", &SScene::getTimestep)
      .def_property("timestep", &SScene::getTimestep, &SScene::setTimestep)
      .def_property("default_physical_material", &SScene::getDefaultMaterial,
                    &SScene::setDefaultMaterial)
      .def("create_actor_builder", &SScene::createActorBuilder)
      .def("create_articulation_builder", &SScene::createArticulationBuilder)
      .def("create_urdf_loader", &SScene::createURDFLoader)
      .def("create_physical_material", &SScene::createPhysicalMaterial, py::arg("static_friction"),
           py::arg("dynamic_friction"), py::arg("restitution"))
      .def("remove_actor", &SScene::removeActor, py::arg("actor"))
      .def("remove_articulation", &SScene::removeArticulation, py::arg("articulation"))
      .def("remove_kinematic_articulation", &SScene::removeKinematicArticulation,
           py::arg("kinematic_articulation"))
      .def("find_actor_by_id", &SScene::findActorById, py::arg("id"),
           py::return_value_policy::reference)
      .def("find_articulation_link_by_link_id", &SScene::findArticulationLinkById, py::arg("id"),
           py::return_value_policy::reference)
      .def("add_mounted_camera", &SScene::addMountedCamera, py::arg("name"), py::arg("actor"),
           py::arg("pose"), py::arg("width"), py::arg("height"), py::arg("fovx"), py::arg("fovy"),
           py::arg("near"), py::arg("far"), py::return_value_policy::reference)
      .def("get_mounted_cameras", &SScene::getMountedCameras, py::return_value_policy::reference)
      .def("get_mounted_actors", &SScene::getMountedActors, py::return_value_policy::reference)
      .def("remove_mounted_camera", &SScene::removeMountedCamera, py::arg("camera"))
      .def("find_mounted_camera", &SScene::findMountedCamera, py::arg("name"),
           py::arg("actor") = nullptr, py::return_value_policy::reference)

      .def("step", &SScene::step)
      .def("step_async", &SScene::stepAsync)
      .def("step_wait", &SScene::stepWait)
      .def("update_render", &SScene::updateRender)
      .def("add_ground", &SScene::addGround, py::arg("altitude"), py::arg("render") = true,
           py::arg("material") = nullptr, py::arg("render_material") = nullptr,
           py::return_value_policy::reference)
      .def("get_contacts", &SScene::getContacts, py::return_value_policy::reference)
      .def("get_all_actors", &SScene::getAllActors, py::return_value_policy::reference)
      .def("get_all_articulations", &SScene::getAllArticulations,
           py::return_value_policy::reference)

      .def(
          "set_shadow_light",
          [](SScene &s, py::array_t<PxReal> const &direction, py::array_t<PxReal> const &color) {
            s.setShadowLight(array2vec3(direction), array2vec3(color));
          },
          py::arg("direction"), py::arg("color"))
      .def(
          "set_ambient_light",
          [](SScene &s, py::array_t<PxReal> const &color) {
            s.setAmbientLight(array2vec3(color));
          },
          py::arg("clor"))
      .def(
          "add_point_light",
          [](SScene &s, py::array_t<PxReal> const &position, py::array_t<PxReal> const &color) {
            s.addPointLight(array2vec3(position), array2vec3(color));
          },
          py::arg("position"), py::arg("color"))
      .def(
          "add_directional_light",
          [](SScene &s, py::array_t<PxReal> const &direction, py::array_t<PxReal> const &color) {
            s.addDirectionalLight(array2vec3(direction), array2vec3(color));
          },
          py::arg("direction"), py::arg("color"))

      // drive, constrains, and joints
      .def("create_drive", &SScene::createDrive, py::arg("actor1"), py::arg("pose1"),
           py::arg("actor2"), py::arg("pose2"), py::return_value_policy::reference)
      .def_property_readonly("render_id_to_visual_name", &SScene::findRenderId2VisualName)

      // renderer
      .def("get_render_scene", &SScene::getRendererScene, py::return_value_policy::reference)
      .def("generate_unique_render_id", &SScene::generateUniqueRenderId)
      .def("pack",
           [](SScene &scene) {
             auto data = scene.packScene();
             std::map<std::string, std::map<physx_id_t, std::vector<PxReal>>> output;
             output["actor"] = data.mActorData;
             output["articulation"] = data.mArticulationData;
             output["articulation_drive"] = data.mArticulationDriveData;
             return output;
           })
      .def(
          "unpack",
          [](SScene &scene,
             std::map<std::string, std::map<physx_id_t, std::vector<PxReal>>> const &input) {
            SceneData data;
            auto t1 = input.find("actor");
            auto t2 = input.find("articulation");
            auto t3 = input.find("articulation_drive");
            if (t1 == input.end()) {
              throw std::invalid_argument("unpack missing key: actor");
            }
            if (t2 == input.end()) {
              throw std::invalid_argument("unpack missing key: articulation");
            }
            if (t3 == input.end()) {
              throw std::invalid_argument("unpack missing key: articulation_drive");
            }
            data.mActorData = t1->second;
            data.mArticulationData = t2->second;
            data.mArticulationDriveData = t3->second;
            scene.unpackScene(data);
          },
          py::arg("data"));

  //======= Drive =======//
  PyDrive
      .def("set_properties", &SDrive::setProperties, py::arg("stiffness"), py::arg("damping"),
           py::arg("force_limit") = PX_MAX_F32, py::arg("is_acceleration") = true)
      .def("set_x_properties", &SDrive::setXProperties, py::arg("stiffness"), py::arg("damping"),
           py::arg("force_limit") = PX_MAX_F32, py::arg("is_acceleration") = true)
      .def("set_y_properties", &SDrive::setYProperties, py::arg("stiffness"), py::arg("damping"),
           py::arg("force_limit") = PX_MAX_F32, py::arg("is_acceleration") = true)
      .def("set_z_properties", &SDrive::setZProperties, py::arg("stiffness"), py::arg("damping"),
           py::arg("force_limit") = PX_MAX_F32, py::arg("is_acceleration") = true)
      .def("lock_motion", &SDrive::lockMotion, py::arg("tx"), py::arg("ty"), py::arg("tz"),
           py::arg("rx"), py::arg("ry"), py::arg("rz"))
      .def("set_target", &SDrive::setTarget, py::arg("pose"))
      .def(
          "set_target_velocity",
          [](SDrive &d, py::array_t<PxReal> const &linear, py::array_t<PxReal> const &angular) {
            d.setTargetVelocity(array2vec3(linear), array2vec3(angular));
          },
          py::arg("linear"), py::arg("angular"))
      .def("destroy", &SDrive::destroy);

  //======== Actor ========//
  PyActorBase.def_property("name", &SActorBase::getName, &SActorBase::setName)
      .def("__repr__",
           [](SActorBase &actor) {
             std::ostringstream oss;
             oss << "Actor [" << actor.getName() << "] with id number [" << actor.getId() << "]\n";
             return oss.str();
           })
      .def("get_name", &SActorBase::getName)
      .def("set_name", &SActorBase::setName, py::arg("name"))
      .def_property_readonly("type",
                             [](SActorBase &actor) {
                               switch (actor.getType()) {
                               case EActorType::STATIC:
                                 return "static";
                               case EActorType::KINEMATIC:
                                 return "kinematic";
                               case EActorType::DYNAMIC:
                                 return "dynamic";
                               case EActorType::ARTICULATION_LINK:
                                 return "link";
                               case EActorType::KINEMATIC_ARTICULATION_LINK:
                                 return "kinematic_link";
                               }
                               throw std::runtime_error("actor has invalid type");
                             })

      .def_property_readonly("id", &SActorBase::getId)
      .def("get_id", &SActorBase::getId)
      .def("get_scene", &SActorBase::getScene, py::return_value_policy::reference)
      .def_property_readonly("pose", &SActorBase::getPose)
      .def("get_pose", &SActorBase::getPose)
      .def_property_readonly("col1", &SActorBase::getCollisionGroup1)
      .def_property_readonly("col2", &SActorBase::getCollisionGroup2)
      .def_property_readonly("col3", &SActorBase::getCollisionGroup3)
      .def("get_collision_shapes", &SActorBase::getCollisionShapes,
           py::return_value_policy::reference)
      .def("get_visual_bodies", &SActorBase::getRenderBodies, py::return_value_policy::reference)
      .def("get_collision_visual_bodies", &SActorBase::getCollisionBodies,
           py::return_value_policy::reference)
      .def("render_collision", &SActorBase::renderCollisionBodies, py::arg("render") = true)
      .def("hide_visual", &SActorBase::hideVisual)
      .def("unhide_visual", &SActorBase::unhideVisual)
      .def("is_hiding_visual", &SActorBase::isHidingVisual)
      .def("on_step", &SActorBase::onStep, py::arg("func"))
      .def("on_contact", &SActorBase::onContact, py::arg("func"))
      .def("on_trigger", &SActorBase::onTrigger, py::arg("func"));

  PyActorDynamicBase
      .def_property_readonly("velocity",
                             [](SActorDynamicBase &a) { return vec32array(a.getVelocity()); })
      .def("get_velocity", [](SActorDynamicBase &a) { return vec32array(a.getVelocity()); })
      .def_property_readonly(
          "angular_velocity",
          [](SActorDynamicBase &a) { return vec32array(a.getAngularVelocity()); })
      .def("get_angular_velocity",
           [](SActorDynamicBase &a) { return vec32array(a.getAngularVelocity()); })
      .def_property_readonly("mass", &SActorDynamicBase::getMass)
      .def("get_mass", &SActorDynamicBase::getMass)
      .def_property_readonly("inertia",
                             [](SActorDynamicBase &a) { return vec32array(a.getInertia()); })
      .def("get_inertia", [](SActorDynamicBase &a) { return vec32array(a.getInertia()); })
      .def_property_readonly("cmass_local_pose", &SActorDynamicBase::getCMassLocalPose)
      .def("get_cmass_local_pose", &SActorDynamicBase::getCMassLocalPose)
      .def(
          "add_force_at_point",
          [](SActorDynamicBase &a, py::array_t<PxReal> const &force,
             py::array_t<PxReal> const &point) {
            a.addForceAtPoint(array2vec3(force), array2vec3(point));
          },
          py::arg("force"), py::arg("point"))

      .def(
          "add_force_torque",
          [](SActorDynamicBase &a, py::array_t<PxReal> const &force,
             py::array_t<PxReal> const &torque) {
            a.addForceTorque(array2vec3(force), array2vec3(torque));
          },
          py::arg("force"), py::arg("torque"))
      .def("set_damping", &SActorDynamicBase::setDamping, py::arg("linear"), py::arg("angular"));

  PyActorStatic.def("set_pose", &SActorStatic::setPose, py::arg("pose"))
      .def("pack", &SActorStatic::packData)
      .def("unpack", [](SActorStatic &a, const py::array_t<PxReal> &arr) {
        a.unpackData(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
      });

  PyActor.def("set_pose", &SActor::setPose, py::arg("pose"))
      .def("set_velocity", [](SActor &a, py::array_t<PxReal> v) { a.setVelocity(array2vec3(v)); })
      .def("set_angular_velocity",
           [](SActor &a, py::array_t<PxReal> v) { a.setAngularVelocity(array2vec3(v)); })
      .def("lock_motion", &SActor::lockMotion, py::arg("x") = true, py::arg("y") = true,
           py::arg("z") = true, py::arg("rx") = true, py::arg("ry") = true, py::arg("rz") = true)
      .def("pack", &SActor::packData)
      .def("unpack",
           [](SActor &a, const py::array_t<PxReal> &arr) {
             a.unpackData(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
           })
      .def("set_solver_iterations", &SActor::setSolverIterations, py::arg("position"),
           py::arg("velocity") = 1);

  PyLinkBase.def("get_index", &SLinkBase::getIndex)
      .def("get_articulation", &SLinkBase::getArticulation, py::return_value_policy::reference);

  PyLink.def("get_articulation", &SLink::getArticulation, py::return_value_policy::reference);
  PyKinematicLink.def("get_articulation", &SKLink::getArticulation,
                      py::return_value_policy::reference);

  //======== End Actor ========//

  //======== Joint ========//
  PyJointBase.def_property("name", &SJointBase::getName, &SJointBase::setName)
      .def_property_readonly("type",
                             [](SJointBase &joint) {
                               switch (joint.getType()) {
                               case physx::PxArticulationJointType::eFIX:
                                 return "fixed";
                               case physx::PxArticulationJointType::eREVOLUTE:
                                 return "revolute";
                               case physx::PxArticulationJointType::ePRISMATIC:
                                 return "prismatic";
                               case physx::PxArticulationJointType::eUNDEFINED:
                                 return "unknown";
                               default:
                                 return "unknown";
                               }
                             })
      .def("__repr__ ",
           [](SJointBase &joint) {
             std::ostringstream oss;
             oss << "Joint [" << joint.getName() << "] with parent link ["
                 << joint.getParentLink()->getName() << "] and child link ["
                 << joint.getChildLink()->getName() << "]\n";
             return oss.str();
           })
      .def("get_name", &SJointBase::getName)
      .def("set_name", &SJointBase::setName, py::arg("name"))
      .def("get_parent_link", &SJointBase::getParentLink, py::return_value_policy::reference)
      .def("get_child_link", &SJointBase::getChildLink, py::return_value_policy::reference)
      .def("get_dof", &SJointBase::getDof)
      .def("get_pose_in_parent_frame", &SJointBase::getParentPose)
      .def("get_pose_in_child_frame", &SJointBase::getChildPose)
      .def("get_limits",
           [](SJointBase &j) {
             auto limits = j.getLimits();
             return py::array_t<PxReal>({(int)limits.size(), 2},
                                        {sizeof(std::array<PxReal, 2>), sizeof(PxReal)},
                                        reinterpret_cast<PxReal *>(limits.data()));
           })
      .def(
          "set_limits",
          [](SJointBase &j, py::array_t<PxReal> limits) {
            std::vector<std::array<PxReal, 2>> l;
            if (limits.ndim() == 2) {
              if (limits.shape(1) != 2) {
                throw std::runtime_error("Joint limit should have shape [dof, 2]");
              }
              for (uint32_t i = 0; i < limits.size() / 2; ++i) {
                l.push_back({limits.at(i, 0), limits.at(i, 1)});
              }
            }
            j.setLimits(l);
          },
          py::arg("limits"))
      .def_property_readonly("articulation", &SJointBase::getArticulation,
                             py::return_value_policy::reference);

  PyJoint.def("set_friction", &SJoint::setFriction, py::arg("friction"))
      .def_property_readonly("friction", &SJoint::getFriction)
      .def(
          "set_drive_property",
          [](SJoint &joint, float stiffness, float damping, float forceLimit, std::string mode) {
            if (mode != "force" && mode != "acceleration") {
              throw std::runtime_error("drive mode must be either force or acceleration");
            }
            joint.setDriveProperty(stiffness, damping, forceLimit, mode == "force" ? false : true);
          },
          py::arg("stiffness"), py::arg("damping"), py::arg("force_limit") = PX_MAX_F32,
          py::arg("mode") = "force")
      .def_property_readonly("stiffness", &SJoint::getDriveStiffness)
      .def_property_readonly("damping", &SJoint::getDriveDamping)
      .def_property_readonly("force_limit", &SJoint::getDriveForceLimit)
      .def_property_readonly(
          "drive_mode",
          [](SJoint &j) { return j.getDriveIsAcceleration() ? "acceleration" : "force"; })
      .def(
          "set_drive_velocity_target", [](SJoint &j, PxReal v) { j.setDriveVelocityTarget(v); },
          py::arg("velocity"))
      .def("get_drive_velocity_target", [](SJoint &j) { return j.getDriveVelocityTarget(); })
      .def(
          "set_drive_target", [](SJoint &j, PxReal p) { j.setDriveTarget(p); }, py::arg("target"))
      .def("get_drive_target", [](SJoint &j) { return j.getDriveTarget(); })
      // TODO wrapper for array-valued targets
      .def("get_global_pose", &SJoint::getGlobalPose);

  //======== End Joint ========//

  PyArticulationBase.def_property("name", &SArticulationBase::getName, &SArticulationBase::setName)
      .def("get_name", &SArticulationBase::getName)
      .def("set_name", &SArticulationBase::setName, py::arg("name"))
      .def("get_links", &SArticulationBase::getBaseLinks, py::return_value_policy::reference)
      .def("get_joints", &SArticulationBase::getBaseJoints, py::return_value_policy::reference)
      .def_property_readonly("type",
                             [](SArticulationBase &art) {
                               switch (art.getType()) {
                               case EArticulationType::DYNAMIC:
                                 return "dynamic";
                               case EArticulationType::KINEMATIC:
                                 return "kinematic";
                               }
                               throw std::runtime_error("invalid articulation type");
                             })
      .def_property_readonly("dof", &SArticulationBase::dof)
      .def("get_qpos",
           [](SArticulationBase &a) {
             auto qpos = a.getQpos();
             return py::array_t<PxReal>(qpos.size(), qpos.data());
           })
      .def(
          "set_qpos",
          [](SArticulationBase &a, const py::array_t<PxReal> &arr) {
            a.setQpos(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
          },
          py::arg("qpos"))

      .def("get_qvel",
           [](SArticulationBase &a) {
             auto qvel = a.getQvel();
             return py::array_t<PxReal>(qvel.size(), qvel.data());
           })
      .def(
          "set_qvel",
          [](SArticulationBase &a, const py::array_t<PxReal> &arr) {
            a.setQvel(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
          },
          py::arg("qvel"))
      .def("get_qacc",
           [](SArticulationBase &a) {
             auto qacc = a.getQacc();
             return py::array_t<PxReal>(qacc.size(), qacc.data());
           })
      .def(
          "set_qacc",
          [](SArticulationBase &a, const py::array_t<PxReal> &arr) {
            a.setQacc(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
          },
          py::arg("qacc"))
      .def("get_qf",
           [](SArticulationBase &a) {
             auto qf = a.getQf();
             return py::array_t<PxReal>(qf.size(), qf.data());
           })
      .def(
          "set_qf",
          [](SArticulationBase &a, const py::array_t<PxReal> &arr) {
            a.setQf(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
          },
          py::arg("qf"))

      .def("get_qlimits",
           [](SArticulationBase &a) {
             std::vector<std::array<PxReal, 2>> limits = a.getQlimits();
             return py::array_t<PxReal>({(int)limits.size(), 2},
                                        {sizeof(std::array<PxReal, 2>), sizeof(PxReal)},
                                        reinterpret_cast<PxReal *>(limits.data()));
           })
      .def(
          "set_qlimits",
          [](SArticulationBase &a, py::array_t<PxReal> limits) {
            std::vector<std::array<PxReal, 2>> l;
            if (limits.ndim() == 2) {
              if (limits.shape(1) != 2) {
                throw std::runtime_error("Joint limits should have shape [dof, 2]");
              }
              for (uint32_t i = 0; i < limits.size() / 2; ++i) {
                l.push_back({limits.at(i, 0), limits.at(i, 1)});
              }
            }
            a.setQlimits(l);
          },
          py::arg("qlimits"))

      .def_property_readonly("pose", &SArticulationBase::getRootPose, "same as get_root_pose()")
      .def("get_root_pose", &SArticulationBase::getRootPose)
      .def("get_pose", &SArticulationBase::getRootPose, "same as get_root_pose")

      .def("set_root_pose", &SArticulationBase::setRootPose, py::arg("pose"))
      .def("set_pose", &SArticulationBase::setRootPose, py::arg("pose"), "same as set_root_pose")
#ifdef _USE_PINOCCHIO
      .def("create_pinocchio_model", &SArticulationBase::createPinocchioModel)
#endif
      .def("export_urdf", &SArticulationBase::exportURDF, py::arg("cache_dir") = std::string());

  PyArticulationDrivable
      .def("get_drive_target",
           [](SArticulationDrivable &a) {
             auto target = a.getDriveTarget();
             return py::array_t<PxReal>(target.size(), target.data());
           })
      .def(
          "set_drive_target",
          [](SArticulationDrivable &a, const py::array_t<PxReal> &arr) {
            a.setDriveTarget(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
          },
          py::arg("drive_target"));

  PyArticulation //.def("get_links", &SArticulation::getSLinks, py::return_value_policy::reference)
                 //.def("get_joints", &SArticulation::getSJoints,
                 // py::return_value_policy::reference)
      .def("get_active_joints", &SArticulation::getActiveJoints,
           py::return_value_policy::reference)
      .def(
          "set_root_velocity",
          [](SArticulation &a, py::array_t<PxReal> v) { a.setRootVelocity(array2vec3(v)); },
          py::arg("vel"))
      .def(
          "set_root_angular_velocity",
          [](SArticulation &a, py::array_t<PxReal> v) { a.setRootAngularVelocity(array2vec3(v)); },
          py::arg("vel"))
      .def(
          "compute_passive_force",
          [](SArticulation &a, bool gravity, bool coriolisAndCentrifugal, bool external) {
            auto force = a.computePassiveForce(gravity, coriolisAndCentrifugal, external);
            return py::array_t<PxReal>(force.size(), force.data());
          },
          py::arg("gravity") = true, py::arg("coriolis_and_centrifugal") = true,
          py::arg("external") = true)
      .def("compute_inverse_dynamics",
           [](SArticulation &a, const py::array_t<PxReal> &arr) {
             assert(arr.size() == a.dof());
             std::vector<PxReal> qacc(arr.data(), arr.data() + a.dof());
             auto qf = a.computeInverseDynamics(qacc);
             return py::array_t<PxReal>(qf.size(), qf.data());
           })
      .def("compute_forward_dynamics",
           [](SArticulation &a, const py::array_t<PxReal> &arr) {
             assert(arr.size() == a.dof());
             std::vector<PxReal> qf(arr.data(), arr.data() + a.dof());
             auto qacc = a.computeForwardDynamics(qf);
             return py::array_t<PxReal>(qacc.size(), qacc.data());
           })
      .def("compute_manipulator_inertia_matrix", &SArticulation::computeManipulatorInertiaMatrix)
      .def("compute_spatial_twist_jacobian", &SArticulation::computeSpatialTwistJacobianMatrix)
      .def("compute_world_cartesian_jacobian", &SArticulation::computeWorldCartesianJacobianMatrix)
      .def("compute_transformation_matrix",
           py::overload_cast<uint32_t, uint32_t>(&SArticulation::computeRelativeTransformation),
           py::arg("source_link_id"), py::arg("target_link_id"))
      .def("compute_adjoint_matrix",
           py::overload_cast<uint32_t, uint32_t>(&SArticulation::computeAdjointMatrix),
           py::arg("source_link_ik"), py::arg("target_link_id"))
      .def("compute_twist_diff_ik", &SArticulation::computeTwistDiffIK, py::arg("spatial_twist"),
           py::arg("commanded_link_id"), py::arg("active_joint_ids") = std::vector<uint32_t>())
      .def("compute_cartesian_diff_ik", &SArticulation::computeCartesianVelocityDiffIK,
           py::arg("world_velocity"), py::arg("commanded_link_id"),
           py::arg("active_joint_ids") = std::vector<uint32_t>())
      .def("pack", &SArticulation::packData)
      .def("unpack", [](SArticulation &a, const py::array_t<PxReal> &arr) {
        a.unpackData(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
      });

  //======== End Articulation ========//

  PyContact
      .def_property_readonly(
          "actor1", [](SContact &contact) { return contact.actors[0]; },
          py::return_value_policy::reference)
      .def_property_readonly(
          "actor2", [](SContact &contact) { return contact.actors[1]; },
          py::return_value_policy::reference)
      .def_readonly("starts", &SContact::starts)
      .def_readonly("persists", &SContact::persists)
      .def_readonly("ends", &SContact::ends)
      .def_readonly("points", &SContact::points)
      .def("__repr__", [](SContact const &c) {
        std::ostringstream oss;
        oss << "Contact(actor0=" << c.actors[0]->getName() << ", actor1=" << c.actors[1]->getName()
            << ")";
        return oss.str();
      });

  PyTrigger
      .def_property_readonly(
          "actor_trigger", [](STrigger &trigger) { return trigger.triggerActor; },
          py::return_value_policy::reference)
      .def_property_readonly(
          "actor_other", [](STrigger &trigger) { return trigger.otherActor; },
          py::return_value_policy::reference)
      .def_readonly("starts", &STrigger::starts)
      .def_readonly("ends", &STrigger::ends)
      .def("__repr__", [](STrigger const &trigger) {
        std::ostringstream oss;
        oss << "Trigger(actor_trigger=" << trigger.triggerActor->getName()
            << ", actor_other=" << trigger.otherActor->getName() << ")";
        return oss.str();
      });

  PyContactPoint
      .def_property_readonly(
          "position",
          [](SContactPoint &point) {
            return make_array<PxReal>({point.position.x, point.position.y, point.position.z});
          })
      .def_property_readonly(
          "normal",
          [](SContactPoint &point) {
            return make_array<PxReal>({point.normal.x, point.normal.y, point.normal.z});
          })
      .def_property_readonly(
          "impulse",
          [](SContactPoint &point) {
            return make_array<PxReal>({point.impulse.x, point.impulse.y, point.impulse.z});
          })
      .def_readonly("separation", &SContactPoint::separation);

  //======== Builders ========

  PyActorBuilder
      .def(
          "add_convex_shape_from_file",
          [](ActorBuilder &a, std::string const &filename, PxTransform const &pose,
             py::array_t<PxReal> const &scale, std::shared_ptr<SPhysicalMaterial> material,
             PxReal density, PxReal patchRadius, PxReal minPatchRadius, bool isTrigger) {
            a.addConvexShapeFromFile(filename, pose, array2vec3(scale), material, density,
                                     patchRadius, minPatchRadius, isTrigger);
          },
          py::arg("filename"), py::arg("pose") = PxTransform(PxIdentity),
          py::arg("scale") = make_array<PxReal>({1, 1, 1}), py::arg("material") = nullptr,
          py::arg("density") = 1000, py::arg("patch_radius") = 0.f,
          py::arg("min_patch_radius") = 0.f, py::arg("is_trigger") = false)
      .def(
          "add_multiple_convex_shapes_from_file",
          [](ActorBuilder &a, std::string const &filename, PxTransform const &pose,
             py::array_t<PxReal> const &scale, std::shared_ptr<SPhysicalMaterial> material,
             PxReal density, PxReal patchRadius, PxReal minPatchRadius, bool isTrigger) {
            a.addMultipleConvexShapesFromFile(filename, pose, array2vec3(scale), material, density,
                                              patchRadius, minPatchRadius, isTrigger);
          },
          py::arg("filename"), py::arg("pose") = PxTransform(PxIdentity),
          py::arg("scale") = make_array<PxReal>({1, 1, 1}), py::arg("material") = nullptr,
          py::arg("density") = 1000, py::arg("patch_radius") = 0.f,
          py::arg("min_patch_radius") = 0.f, py::arg("is_trigger") = false)
      .def(
          "add_box_shape",
          [](ActorBuilder &a, PxTransform const &pose, py::array_t<PxReal> const &halfSize,
             std::shared_ptr<SPhysicalMaterial> material, PxReal density, PxReal patchRadius,
             PxReal minPatchRadius, bool isTrigger) {
            a.addBoxShape(pose, array2vec3(halfSize), material, density, patchRadius,
                          minPatchRadius, isTrigger);
          },
          py::arg("pose") = PxTransform(PxIdentity),
          py::arg("half_size") = make_array<PxReal>({1, 1, 1}), py::arg("material") = nullptr,
          py::arg("density") = 1000, py::arg("patch_radius") = 0.f,
          py::arg("min_patch_radius") = 0.f, py::arg("is_trigger") = false)
      .def("add_capsule_shape", &ActorBuilder::addCapsuleShape,
           py::arg("pose") = PxTransform(PxIdentity), py::arg("radius") = 1,
           py::arg("half_length") = 1, py::arg("material") = nullptr, py::arg("density") = 1000,
           py::arg("patch_radius") = 0.f, py::arg("min_patch_radius") = 0.f,
           py::arg("is_trigger") = false)
      .def("add_sphere_shape", &ActorBuilder::addSphereShape,
           py::arg("pose") = PxTransform(PxIdentity), py::arg("radius") = 1,
           py::arg("material") = nullptr, py::arg("density") = 1000, py::arg("patch_radius") = 0.f,
           py::arg("min_patch_radius") = 0.f, py::arg("is_trigger") = false)
      .def(
          "add_box_visual",
          [](ActorBuilder &a, PxTransform const &pose, py::array_t<PxReal> const &halfSize,
             py::array_t<PxReal> color, std::string const &name) {
            a.addBoxVisual(pose, array2vec3(halfSize), array2vec3(color), name);
          },
          py::arg("pose") = PxTransform(PxIdentity),
          py::arg("half_size") = make_array<PxReal>({1, 1, 1}),
          py::arg("color") = make_array<PxReal>({1, 1, 1}), py::arg("name") = "")
      .def(
          "add_box_visual_complex",
          [](ActorBuilder &a, PxTransform const &pose, py::array_t<PxReal> const &halfSize,
             std::shared_ptr<Renderer::IPxrMaterial> &mat, std::string const &name) {
            a.addBoxVisualWithMaterial(pose, array2vec3(halfSize), mat, name);
          },
          py::arg("pose") = PxTransform(PxIdentity),
          py::arg("half_size") = make_array<PxReal>({1, 1, 1}), py::arg("material") = nullptr,
          py::arg("name") = "")
      .def(
          "add_capsule_visual",
          [](ActorBuilder &a, PxTransform const &pose, PxReal radius, PxReal halfLength,
             py::array_t<PxReal> color, std::string const &name) {
            a.addCapsuleVisual(pose, radius, halfLength, array2vec3(color), name);
          },
          py::arg("pose") = PxTransform(PxIdentity), py::arg("radius") = 1,
          py::arg("half_length") = 1, py::arg("color") = make_array<PxReal>({1, 1, 1}),
          py::arg("name") = "")
      .def(
          "add_capsule_visual_complex",
          [](ActorBuilder &a, PxTransform const &pose, PxReal radius, PxReal halfLength,
             std::shared_ptr<Renderer::IPxrMaterial> &mat, std::string const &name) {
            a.addCapsuleVisualWithMaterial(pose, radius, halfLength, mat, name);
          },
          py::arg("pose") = PxTransform(PxIdentity), py::arg("radius") = 1,
          py::arg("half_length") = 1, py::arg("material") = nullptr, py::arg("name") = "")
      .def(
          "add_sphere_visual",
          [](ActorBuilder &a, PxTransform const &pose, PxReal radius, py::array_t<PxReal> color,
             std::string const &name) {
            a.addSphereVisual(pose, radius, array2vec3(color), name);
          },
          py::arg("pose") = PxTransform(PxIdentity), py::arg("radius") = 1,
          py::arg("color") = make_array<PxReal>({1, 1, 1}), py::arg("name") = "")
      .def(
          "add_sphere_visual_complex",
          [](ActorBuilder &a, PxTransform const &pose, PxReal radius,
             std::shared_ptr<Renderer::IPxrMaterial> &mat,
             std::string const &name) { a.addSphereVisualWithMaterial(pose, radius, mat, name); },
          py::arg("pose") = PxTransform(PxIdentity), py::arg("radius") = 1,
          py::arg("material") = nullptr, py::arg("name") = "")
      .def(
          "add_visual_from_file",
          [](ActorBuilder &a, std::string const &filename, PxTransform const &pose,
             py::array_t<PxReal> scale, std::string const &name) {
            a.addVisualFromFile(filename, pose, array2vec3(scale), name);
          },
          py::arg("filename"), py::arg("pose") = PxTransform(PxIdentity),
          py::arg("scale") = make_array<PxReal>({1, 1, 1}), py::arg("name") = "")
      .def("remove_all_shapes", &ActorBuilder::removeAllShapes)
      .def("remove_all_visuals", &ActorBuilder::removeAllVisuals)
      .def("remove_shape_at", &ActorBuilder::removeShapeAt, py::arg("index"))
      .def("remove_visual_at", &ActorBuilder::removeVisualAt, py::arg("index"))
      .def("get_shapes", &ActorBuilder::getShapes, py::return_value_policy::reference)
      .def("get_visuals", &ActorBuilder::getVisuals, py::return_value_policy::reference)
      .def("set_collision_group", &ActorBuilder::setCollisionGroup)
      .def("add_collision_group", &ActorBuilder::addCollisionGroup)
      .def("reset_collision_group", &ActorBuilder::resetCollisionGroup)
      .def("set_mass_and_inertia",
           [](ActorBuilder &a, PxReal mass, PxTransform const &cMassPose,
              py::array_t<PxReal> inertia) {
             a.setMassAndInertia(mass, cMassPose, array2vec3(inertia));
           })
      .def("set_scene", &ActorBuilder::setScene)
      .def("build", &ActorBuilder::build, py::arg("is_kinematic") = false, py::arg("name") = "",
           py::return_value_policy::reference)
      .def("build_static", &ActorBuilder::buildStatic, py::return_value_policy::reference,
           py::arg("name") = "");

  PyShapeRecord.def_readonly("filename", &ActorBuilder::ShapeRecord::filename)
      .def_property_readonly("density",
                             [](ActorBuilder::ShapeRecord const &r) {
                               switch (r.type) {
                               case sapien::ActorBuilder::ShapeRecord::SingleMesh:
                                 return "Mesh";
                               case sapien::ActorBuilder::ShapeRecord::MultipleMeshes:
                                 return "Meshes";
                               case sapien::ActorBuilder::ShapeRecord::Box:
                                 return "Box";
                               case sapien::ActorBuilder::ShapeRecord::Capsule:
                                 return "Capsule";
                               case sapien::ActorBuilder::ShapeRecord::Sphere:
                                 return "Sphere";
                               }
                               return "";
                             })
      .def_property_readonly(
          "scale", [](ActorBuilder::ShapeRecord const &r) { return vec32array(r.scale); })
      .def_readonly("radius", &ActorBuilder::ShapeRecord::radius)
      .def_readonly("radius", &ActorBuilder::ShapeRecord::length)
      .def_readonly("pose", &ActorBuilder::ShapeRecord::pose)
      .def_readonly("density", &ActorBuilder::ShapeRecord::density)
      .def_readonly("material", &ActorBuilder::ShapeRecord::material,
                    py::return_value_policy::reference);

  PyVisualRecord.def_readonly("filename", &ActorBuilder::VisualRecord::filename)
      .def_property_readonly("density",
                             [](ActorBuilder::VisualRecord const &r) {
                               switch (r.type) {
                               case sapien::ActorBuilder::VisualRecord::Mesh:
                                 return "Mesh";
                               case sapien::ActorBuilder::VisualRecord::Box:
                                 return "Box";
                               case sapien::ActorBuilder::VisualRecord::Capsule:
                                 return "Capsule";
                               case sapien::ActorBuilder::VisualRecord::Sphere:
                                 return "Sphere";
                               }
                               return "";
                             })
      .def_property_readonly(
          "scale", [](ActorBuilder::VisualRecord const &r) { return vec32array(r.scale); })
      .def_readonly("radius", &ActorBuilder::VisualRecord::radius)
      .def_readonly("length", &ActorBuilder::VisualRecord::length)
      .def_readonly("pose", &ActorBuilder::VisualRecord::pose)
      .def_readonly("material", &ActorBuilder::VisualRecord::material,
                    py::return_value_policy::reference);

  PyLinkBuilder.def("get_index", &LinkBuilder::getIndex)
      .def("get_parent", &LinkBuilder::getParent)
      .def("set_parent", &LinkBuilder::setParent)
      .def("get_name", &LinkBuilder::getName)
      .def("set_name", &LinkBuilder::setName)
      .def("set_joint_name", &LinkBuilder::setJointName)
      .def(
          "set_joint_properties",
          [](LinkBuilder &b, std::string const &jointType, py::array_t<PxReal> const &limits,
             PxTransform const &parentPose, PxTransform const &childPose, PxReal friction,
             PxReal damping) {
            std::vector<std::array<PxReal, 2>> l;
            if (limits.ndim() == 2) {
              if (limits.shape(1) != 2) {
                throw std::runtime_error("Joint limit should have shape [dof, 2]");
              }
              for (uint32_t i = 0; i < limits.size() / 2; ++i) {
                l.push_back({limits.at(i, 0), limits.at(i, 1)});
              }
            } else if (limits.ndim() != 1) {
              throw std::runtime_error("Joint limit must be 2D array");
            }

            PxArticulationJointType::Enum t;
            if (jointType == "revolute" || jointType == "hinge") {
              t = PxArticulationJointType::Enum::eREVOLUTE;
            } else if (jointType == "prismatic" || jointType == "slider") {
              t = PxArticulationJointType::Enum::ePRISMATIC;
            } else if (jointType == "fixed") {
              t = PxArticulationJointType::Enum::eFIX;
            } else {
              throw std::runtime_error("Unsupported joint type: " + jointType +
                                       "; supported types are: revolute, prismatic, fixed.");
            }

            b.setJointProperties(t, l, parentPose, childPose, friction, damping);
          },
          py::arg("joint_type"), py::arg("limits"),
          py::arg("parent_pose") = PxTransform(PxIdentity),
          py::arg("child_pose") = PxTransform(PxIdentity), py::arg("friction") = 0,
          py::arg("damping") = 0)

      .def("get_joints", &LinkBuilder::getJoint, py::return_value_policy::reference);

  PyJointRecord
      .def_property_readonly("joint_type",
                             [](LinkBuilder::JointRecord const &r) {
                               switch (r.jointType) {
                               case physx::PxArticulationJointType::eFIX:
                                 return "fixed";
                               case physx::PxArticulationJointType::eREVOLUTE:
                                 return "revolute";
                               case physx::PxArticulationJointType::ePRISMATIC:
                                 return "prismatic";
                               case physx::PxArticulationJointType::eUNDEFINED:
                                 return "unknown";
                               default:
                                 return "unknown";
                               }
                             })
      .def_readonly("parent_pose", &LinkBuilder::JointRecord::parentPose)
      .def_readonly("child_pose", &LinkBuilder::JointRecord::childPose)
      .def_readonly("friction", &LinkBuilder::JointRecord::friction)
      .def_readonly("damping", &LinkBuilder::JointRecord::damping)
      .def_readonly("name", &LinkBuilder::JointRecord::name)
      .def_property_readonly("limits", [](LinkBuilder::JointRecord const &j) {
        auto limits = j.limits;
        return py::array_t<PxReal>({(int)limits.size(), 2},
                                   {sizeof(std::array<PxReal, 2>), sizeof(PxReal)},
                                   reinterpret_cast<PxReal *>(limits.data()));
      });

  PyArticulationBuilder.def("set_scene", &ArticulationBuilder::setScene, py::arg("scene"))
      .def("get_scene", &ArticulationBuilder::getScene)
      .def(
          "create_link_builder",
          [](ArticulationBuilder &b, LinkBuilder *parent) { return b.createLinkBuilder(parent); },
          py::arg("parent") = nullptr, py::return_value_policy::reference)
      .def("build", &ArticulationBuilder::build, py::arg("fix_base") = false,
           py::return_value_policy::reference)
      .def("build_kinematic", &ArticulationBuilder::buildKinematic,
           py::return_value_policy::reference)
      .def("get_link_builders", &ArticulationBuilder::getLinkBuilders,
           py::return_value_policy::reference);

  PyURDFLoader.def(py::init<SScene *>(), py::arg("scene"))
      .def_readwrite("fix_root_link", &URDF::URDFLoader::fixRootLink)
      .def_readwrite("load_multiple_shapes_from_file", &URDF::URDFLoader::multipleMeshesInOneFile)
      .def_readwrite("collision_is_visual", &URDF::URDFLoader::collisionIsVisual)
      .def_readwrite("scale", &URDF::URDFLoader::scale)
      .def_property(
          "default_density",
          [](URDF::URDFLoader &) {
            throw std::runtime_error("default_density is moved to URDF config");
          },
          [](URDF::URDFLoader &) {
            throw std::runtime_error("default_density is moved to URDF config");
          })
      .def(
          "load",
          [](URDF::URDFLoader &loader, std::string const &filename, py::dict &dict) {
            auto config = parseURDFConfig(dict);
            return loader.load(filename, config);
          },
          py::return_value_policy::reference, py::arg("filename"), py::arg("config") = py::dict())
      .def(
          "load_kinematic",
          [](URDF::URDFLoader &loader, std::string const &filename, py::dict &dict) {
            auto config = parseURDFConfig(dict);
            return loader.loadKinematic(filename, config);
          },
          py::return_value_policy::reference, py::arg("filename"), py::arg("config") = py::dict())
      .def(
          "load_from_string",
          [](URDF::URDFLoader &loader, std::string const &urdf, std::string const &srdf,
             py::dict &dict) {
            auto config = parseURDFConfig(dict);
            return loader.loadFromXML(urdf, srdf, config);
          },
          py::return_value_policy::reference, py::arg("urdf_string"), py::arg("srdf_string"),
          py::arg("config") = py::dict())
      .def(
          "load_file_as_articulation_builder",
          [](URDF::URDFLoader &loader, std::string const &filename, py::dict &dict) {
            auto config = parseURDFConfig(dict);
            return loader.loadFileAsArticulationBuilder(filename, config);
          },
          py::return_value_policy::reference, py::arg("filename"), py::arg("config") = py::dict());

  PySubscription.def("unsubscribe", &Subscription::unsubscribe);

#ifdef _USE_PINOCCHIO
  PyPinocchioModel
      .def("compute_forward_kinematics", &PinocchioModel::computeForwardKinematics,
           py::arg("qpos"))
      .def("get_link_pose", &PinocchioModel::getLinkPose, py::arg("link_index"))
      .def("compute_inverse_kinematics", &PinocchioModel::computeInverseKinematics,
           py::arg("link_index"), py::arg("pose"), py::arg("initial_qpos") = Eigen::VectorXd{},
           py::arg("eps") = 1e-4, py::arg("max_iterations") = 1000, py::arg("dt") = 0.1,
           py::arg("damp") = 1e-6)
      .def("compute_forward_dynamics", &PinocchioModel::computeForwardDynamics, py::arg("qpos"),
           py::arg("qvel"), py::arg("qf"))
      .def("compute_inverse_dynamics", &PinocchioModel::computeForwardDynamics, py::arg("qpos"),
           py::arg("qvel"), py::arg("qacc"))
      .def("compute_generalized_mass_matrix", &PinocchioModel::computeGeneralizedMassMatrix,
           py::arg("qpos"))
      .def("compute_coriolis_matrix", &PinocchioModel::computeCoriolisMatrix, py::arg("qpos"),
           py::arg("qvel"))

      .def("compute_full_jacobian", &PinocchioModel::computeFullJacobian, py::arg("qpos"))
      .def("get_link_jacobian", &PinocchioModel::getLinkJacobian, py::arg("link_index"),
           py::arg("local") = false)
      .def("compute_single_link_local_jacobian", &PinocchioModel::computeSingleLinkLocalJacobian,
           py::arg("qpos"), py::arg("link_index"));
#endif

  PyVulkanRenderer
      .def_static("set_log_level", &Renderer::SVulkan2Renderer::setLogLevel, py::arg("level"))
      .def(py::init<bool, uint32_t, uint32_t, uint32_t>(), py::arg("offscreen_only") = false,
           py::arg("max_num_materials") = 5000, py::arg("max_num_textures") = 5000,
           py::arg("default_mipmap_levels") = 1)
      .def_static("set_viewer_shader_dir", &Renderer::setDefaultViewerShaderDirectory,
                  py::arg("shader_dir"))
      .def_static("set_camera_shader_dir", &Renderer::setDefaultCameraShaderDirectory,
                  py::arg("shader_dir"))
      .def(
          "create_window",
          [](std::shared_ptr<Renderer::SVulkan2Renderer> renderer, int width, int height,
             std::string const &shaderDir) {
            return new Renderer::SVulkan2Window(renderer, width, height, shaderDir);
          },
          py::arg("width") = 800, py::arg("height") = 600, py::arg("shader_dir") = "")
      .def_property_readonly(
          "_internal_context",
          [](Renderer::SVulkan2Renderer &renderer) { return renderer.mContext.get(); },
          py::return_value_policy::reference);

  PyVulkanCamera
      .def(
          "get_float_texture",
          [](Renderer::SVulkan2Camera &cam, std::string const &name) {
            auto [image, sizes] = cam.getFloatTexture(name);
            if (sizes[2] == 1) {
              return py::array_t<float>({static_cast<int>(sizes[0]), static_cast<int>(sizes[1])},
                                        image.data());
            } else {
              return py::array_t<float>({static_cast<int>(sizes[0]), static_cast<int>(sizes[1]),
                                         static_cast<int>(sizes[2])},
                                        image.data());
            }
          },
          py::arg("texture_name"))
      .def(
          "get_uint32_texture",
          [](Renderer::SVulkan2Camera &cam, std::string const &name) {
            auto [image, sizes] = cam.getUint32Texture(name);
            if (sizes[2] == 1) {
              return py::array_t<uint32_t>(
                  {static_cast<int>(sizes[0]), static_cast<int>(sizes[1])}, image.data());
            } else {
              return py::array_t<uint32_t>({static_cast<int>(sizes[0]), static_cast<int>(sizes[1]),
                                            static_cast<int>(sizes[2])},
                                           image.data());
            }
          },
          py::arg("texture_name"))
#ifdef SAPIEN_DLPACK_INTEROP
      .def(
          "get_dl_tensor",
          [](Renderer::SVulkan2Camera &cam, std::string const &name) {
            auto [buffer, sizes, format] = cam.getCudaBuffer(name);
            std::vector<long> dim;
            for (auto s : sizes) {
              dim.push_back(s);
            }
            DLManagedTensor *tensor = DLTensorFromCudaBuffer(std::move(buffer), dim, format);
            auto capsule_destructor = [](PyObject *data) {
              DLManagedTensor *tensor = (DLManagedTensor *)PyCapsule_GetPointer(data, "dltensor");
              if (tensor) {
                tensor->deleter(const_cast<DLManagedTensor *>(tensor));
              } else {
                PyErr_Clear();
              }
            };
            return py::capsule(tensor, "dltensor", capsule_destructor);
          },
          py::arg("texture_name"))
#endif
      .def("get_camera_matrix",
           [](Renderer::SVulkan2Camera &c) { return mat42array(c.getCameraMatrix()); })
      .def("get_model_matrix",
           [](Renderer::SVulkan2Camera &c) { return mat42array(c.getModelMatrix()); })
      .def("get_projection_matrix",
           [](Renderer::SVulkan2Camera &c) { return mat42array(c.getProjectionMatrix()); })

      .def("set_orthographic", &Renderer::SVulkan2Camera::setOrthographicParameters,
           py::arg("near"), py::arg("far"), py::arg("aspect"), py::arg("scale"))
      .def("set_perspective", &Renderer::SVulkan2Camera::setPerspectiveParameters, py::arg("near"),
           py::arg("far"), py::arg("fovy"), py::arg("aspect"))
      .def("set_full_perspective", &Renderer::SVulkan2Camera::setFullPerspectiveParameters,
           py::arg("near"), py::arg("far"), py::arg("fx"), py::arg("fy"), py::arg("cx"),
           py::arg("cy"), py::arg("width"), py::arg("height"), py::arg("skew"))
      .def_property_readonly("mode", &Renderer::SVulkan2Camera::getMode);

  PyVulkanWindow.def("show", &Renderer::SVulkan2Window::show)
      .def("hide", &Renderer::SVulkan2Window::hide)
      .def("set_camera_parameters", &Renderer::SVulkan2Window::setCameraParameters,
           py::arg("near"), py::arg("far"), py::arg("fovy"))
      .def(
          "set_camera_position",
          [](Renderer::SVulkan2Window &window, py::array_t<float> position) {
            window.setCameraPosition({position.at(0), position.at(1), position.at(2)});
          },
          py::arg("position"))
      .def(
          "set_camera_rotation",
          [](Renderer::SVulkan2Window &window, py::array_t<float> quat) {
            window.setCameraRotation({quat.at(0), quat.at(1), quat.at(2), quat.at(3)});
          },
          py::arg("quat"))
      .def("get_camera_position",
           [](Renderer::SVulkan2Window &window) {
             auto pos = window.getCameraPosition();
             return make_array<float>({pos.x, pos.y, pos.z});
           })
      .def("get_camera_rotation",
           [](Renderer::SVulkan2Window &window) {
             auto quat = window.getCameraRotation();
             return make_array<float>({quat.w, quat.x, quat.y, quat.z});
           })
      .def(
          "set_scene",
          [](Renderer::SVulkan2Window &window, SScene *scene) {
            if (!scene) {
              window.setScene(nullptr);
            }
            auto rs = dynamic_cast<Renderer::SVulkan2Scene *>(scene->getRendererScene());
            window.setScene(rs);
          },
          py::arg("scene"))
      .def_property_readonly("target_names", &Renderer::SVulkan2Window::getDisplayTargetNames)
      .def("get_target_size", &Renderer::SVulkan2Window::getRenderTargetSize, py::arg("name"))
      .def("render", &Renderer::SVulkan2Window::render, py::arg("target_name"),
           py::arg("ui_windows") = std::vector<std::shared_ptr<svulkan2::ui::Window>>())
      .def("resize", &Renderer::SVulkan2Window::resize, py::arg("width"), py::arg("height"))
      .def_property_readonly("fps", &Renderer::SVulkan2Window::getFPS)
      .def_property_readonly("size", &Renderer::SVulkan2Window::getWindowSize)

      // Download images from window
      .def(
          "download_float_target",
          [](Renderer::SVulkan2Window &window, std::string const &name) {
            auto [image, sizes] = window.downloadFloatTarget(name);
            if (sizes[2] == 1) {
              return py::array_t<float>({static_cast<int>(sizes[0]), static_cast<int>(sizes[1])},
                                        image.data());
            } else {
              return py::array_t<float>({static_cast<int>(sizes[0]), static_cast<int>(sizes[1]),
                                         static_cast<int>(sizes[2])},
                                        image.data());
            }
          },
          py::arg("name"))
      .def(
          "download_uint32_target",
          [](Renderer::SVulkan2Window &window, std::string const &name) {
            auto [image, sizes] = window.downloadUint32Target(name);
            if (sizes[2] == 1) {
              return py::array_t<uint32_t>(
                  {static_cast<int>(sizes[0]), static_cast<int>(sizes[1])}, image.data());
            } else {
              return py::array_t<uint32_t>({static_cast<int>(sizes[0]), static_cast<int>(sizes[1]),
                                            static_cast<int>(sizes[2])},
                                           image.data());
            }
          },
          py::arg("name"))
      .def(
          "download_uint8_target",
          [](Renderer::SVulkan2Window &window, std::string const &name) {
            auto [image, sizes] = window.downloadUint8Target(name);
            if (sizes[2] == 1) {
              return py::array_t<uint8_t>({static_cast<int>(sizes[0]), static_cast<int>(sizes[1])},
                                          image.data());
            } else {
              return py::array_t<uint8_t>({static_cast<int>(sizes[0]), static_cast<int>(sizes[1]),
                                           static_cast<int>(sizes[2])},
                                          image.data());
            }
          },
          py::arg("name"))
      .def(
          "download_float_target_pixel",
          [](Renderer::SVulkan2Window &window, std::string const &name, uint32_t x, uint32_t y) {
            auto v = window.downloadFloatTargetPixel(name, x, y);
            return py::array_t<float>(static_cast<int>(v.size()), v.data());
          },
          py::arg("name"), py::arg("x"), py::arg("y"))
      .def(
          "download_uint32_target_pixel",
          [](Renderer::SVulkan2Window &window, std::string const &name, uint32_t x, uint32_t y) {
            auto v = window.downloadUint32TargetPixel(name, x, y);
            return py::array_t<uint32_t>(static_cast<int>(v.size()), v.data());
          },
          py::arg("name"), py::arg("x"), py::arg("y"))
      .def(
          "download_uint8_target_pixel",
          [](Renderer::SVulkan2Window &window, std::string const &name, uint32_t x, uint32_t y) {
            auto v = window.downloadUint8TargetPixel(name, x, y);
            return py::array_t<uint8_t>(static_cast<int>(v.size()), v.data());
          },
          py::arg("name"), py::arg("x"), py::arg("y"))

      .def_property_readonly("shift", &Renderer::SVulkan2Window::isShiftDown)
      .def_property_readonly("alt", &Renderer::SVulkan2Window::isAltDown)
      .def_property_readonly("ctrl", &Renderer::SVulkan2Window::isCtrlDown)
      .def_property_readonly("super", &Renderer::SVulkan2Window::isSuperDown)

      .def("key_down", &Renderer::SVulkan2Window::isKeyDown, py::arg("key"))
      .def("key_press", &Renderer::SVulkan2Window::isKeyPressed, py::arg("key"))
      .def("mouse_down", &Renderer::SVulkan2Window::isMouseKeyDown, py::arg("key"))
      .def("mouse_click", &Renderer::SVulkan2Window::isMouseKeyClicked, py::arg("key"))
      .def_property_readonly("mouse_position", &Renderer::SVulkan2Window::getMousePosition)
      .def_property_readonly("mouse_delta", &Renderer::SVulkan2Window::getMouseDelta)
      .def_property_readonly("mouse_wheel_delta", &Renderer::SVulkan2Window::getMouseWheelDelta);

  PyVulkanScene
      .def(
          "add_shadow_point_light",
          [](Renderer::SVulkan2Scene &scene, py::array_t<float> const &position,
             py::array_t<float> const &color, float near, float far) {
            scene.addPointLight({position.at(0), position.at(1), position.at(2)},
                                {color.at(0), color.at(1), color.at(2)}, true, near, far);
          },
          py::arg("position"), py::arg("color"), py::arg("near") = 0.1, py::arg("far") = 10)
      .def(
          "add_shadow_directional_light",
          [](Renderer::SVulkan2Scene &scene, py::array_t<float> const &direction,
             py::array_t<float> const &color, py::array_t<float> const &position, float scale,
             float near, float far) {
            scene.addDirectionalLight({direction.at(0), direction.at(1), direction.at(2)},
                                      {color.at(0), color.at(1), color.at(2)}, true,
                                      {position.at(0), position.at(1), position.at(2)}, scale,
                                      near, far);
          },
          py::arg("direction"), py::arg("color"),
          py::arg("position") = make_array<float>({0.f, 0.f, 0.f}), py::arg("scale") = 10.f,
          py::arg("near") = -10.f, py::arg("far") = 10.f)
      .def(
          "add_shadow_spot_light",
          [](Renderer::SVulkan2Scene &scene, py::array_t<float> const &position,
             py::array_t<float> const &direction, float fov, py::array_t<float> const &color,
             float near, float far) {
            scene.addSpotLight({position.at(0), position.at(1), position.at(2)},
                               {direction.at(0), direction.at(1), direction.at(2)}, fov,
                               {color.at(0), color.at(1), color.at(2)}, true, near, far);
          },
          py::arg("position"), py::arg("direction"), py::arg("fov"), py::arg("color"),
          py::arg("near") = 0.1f, py::arg("far") = 10.f)
      .def_property_readonly(
          "_internal_scene", [](Renderer::SVulkan2Scene &scene) { return scene.getScene(); },
          py::return_value_policy::reference);

  PyRenderer.def("create_material", &Renderer::IPxrRenderer::createMaterial);

  PyRenderScene
      .def(
          "add_mesh_from_file",
          [](Renderer::IPxrScene &scene, std::string const &meshFile, py::array_t<float> scale) {
            return scene.addRigidbody(meshFile, array2vec3(scale));
          },
          py::arg("mesh_file"), py::arg("scale"), py::return_value_policy::reference)
      .def(
          "add_primitive_mesh",
          [](Renderer::IPxrScene &scene, std::string const &type, py::array_t<float> scale,
             std::shared_ptr<Renderer::IPxrMaterial> &material) {
            physx::PxGeometryType::Enum ptype;
            if (type == "box") {
              ptype = physx::PxGeometryType::eBOX;
            } else if (type == "sphere") {
              ptype = physx::PxGeometryType::eSPHERE;
            } else if (type == "capsule") {
              ptype = physx::PxGeometryType::eCAPSULE;
            } else if (type == "plane") {
              ptype = physx::PxGeometryType::ePLANE;
            } else {
              throw std::invalid_argument("Unknown type " + type);
            }
            return scene.addRigidbody(ptype, array2vec3(scale), material);
          },
          py::arg("type"), py::arg("scale") = make_array<float>({1.f, 1.f, 1.f}),
          py::arg("material") = nullptr, py::return_value_policy::reference)
      // TODO: add mesh from vertices
      .def("remove_mesh", &Renderer::IPxrScene::removeRigidbody, py::arg("mesh"));

  PyRenderBody.def("get_name", &Renderer::IPxrRigidbody::getName)
      .def("set_name", &Renderer::IPxrRigidbody::setName, py::arg("name"))
      .def("set_unique_id", &Renderer::IPxrRigidbody::setUniqueId, py::arg("id"))
      .def("get_unique_id", &Renderer::IPxrRigidbody::getUniqueId)
      .def("get_segmentation_id", &Renderer::IPxrRigidbody::getSegmentationId)
      .def("set_custom_data", &Renderer::IPxrRigidbody::setSegmentationCustomData,
           py::arg("custom_data"))
      .def("get_render_shapes", &Renderer::IPxrRigidbody::getRenderShapes)
      .def("set_pose", &Renderer::IPxrRigidbody::update, py::arg("pose"))
      .def("set_visibility", &Renderer::IPxrRigidbody::setVisibility, py::arg("visibility"));

  PyRenderShape.def_readonly("type", &Renderer::RenderShape::type)
      .def_readonly("pose", &Renderer::RenderShape::pose)
      .def_readonly("obj_id", &Renderer::RenderShape::objId)
      .def_property_readonly("scale",
                             [](Renderer::RenderShape &shape) { return vec32array(shape.scale); })
      .def_property_readonly(
          "mesh", [](Renderer::RenderShape &shape) { return shape.geometry.get(); },
          py::return_value_policy::reference)
      .def_readonly("material", &Renderer::RenderShape::material);

  PyRenderMeshGeometry
      .def_property_readonly("vertices",
                             [](Renderer::RenderMeshGeometry &geom) {
                               return py::array_t<float>(
                                   {static_cast<int>(geom.vertices.size() / 3), 3},
                                   {sizeof(float) * 3, sizeof(float)}, geom.vertices.data());
                             })
      .def_property_readonly("normals",
                             [](Renderer::RenderMeshGeometry &geom) {
                               return py::array_t<float>(
                                   {static_cast<int>(geom.normals.size() / 3), 3},
                                   {sizeof(float) * 3, sizeof(float)}, geom.normals.data());
                             })
      .def_property_readonly("uvs",
                             [](Renderer::RenderMeshGeometry &geom) {
                               return py::array_t<float>(
                                   {static_cast<int>(geom.uvs.size() / 2), 2},
                                   {sizeof(float) * 2, sizeof(float)}, geom.uvs.data());
                             })
      .def_property_readonly("tangents",
                             [](Renderer::RenderMeshGeometry &geom) {
                               return py::array_t<float>(
                                   {static_cast<int>(geom.tangents.size() / 3), 3},
                                   {sizeof(float) * 3, sizeof(float)}, geom.tangents.data());
                             })
      .def_property_readonly("bitangents",
                             [](Renderer::RenderMeshGeometry &geom) {
                               return py::array_t<float>(
                                   {static_cast<int>(geom.bitangents.size() / 3), 3},
                                   {sizeof(float) * 3, sizeof(float)}, geom.bitangents.data());
                             })
      .def_property_readonly(
          "indices", [](Renderer::RenderMeshGeometry &geom) { return make_array(geom.indices); });
}
