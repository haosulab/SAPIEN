#pragma once
#include <pybind11/eigen.h>
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

#ifdef _USE_PINOCCHIO
#include "articulation/pinocchio_model.h"
#endif

#include "renderer/optifuser_controller.h"
#include "renderer/optifuser_renderer.h"

#include "utils/pose.hpp"

#ifdef _USE_VULKAN
#include "renderer/sapien_vulkan_renderer.h"
#ifdef ON_SCREEN
#include "renderer/sapien_vulkan_controller.h"
#endif
#endif

using namespace sapien;
namespace py = pybind11;

URDF::URDFConfig parseURDFConfig(py::dict &dict) {
  URDF::URDFConfig config;
  if (dict.contains("material")) {
    std::cout << "found material" << std::endl;
    config.material = dict["material"].cast<PxMaterial *>();
  }
  if (dict.contains("density")) {
    std::cout << "found density" << std::endl;
    config.density = dict["density"].cast<float>();
  }
  if (dict.contains("link")) {
    std::cout << "found link" << std::endl;

    auto linkDict = dict["link"].cast<py::dict>();
    for (auto kv : linkDict) {
      auto name = kv.first.cast<std::string>();
      auto dict2 = kv.second.cast<py::dict>();
      if (dict2.contains("material")) {
        std::cout << "found link material" << std::endl;
        config.link[name].material = dict2["material"].cast<PxMaterial *>();
      } else {
        config.link[name].material = config.material;
      }
      if (dict2.contains("density")) {
        std::cout << "found link density" << std::endl;
        config.link[name].density = dict2["density"].cast<float>();
      } else {
        config.link[name].density = config.density;
      }
      if (dict2.contains("patch_radius")) {
        std::cout << "found link patch" << std::endl;
        config.link[name].patchRadius = dict2["patch_radius"].cast<float>();
      }
      if (dict2.contains("min_patch_radius")) {
        config.link[name].minPatchRadius = dict2["min_patch_radius"].cast<float>();
      }
      if (dict2.contains("shape")) {
        std::cout << "found shape" << std::endl;
        auto shapeDict = dict2["shape"].cast<py::dict>();
        for (auto kv2 : shapeDict) {
          auto idx = kv2.first.cast<int>();
          auto dict3 = kv2.second.cast<py::dict>();

          if (dict3.contains("material")) {
            std::cout << "found shape material" << std::endl;
            config.link[name].shape[idx].material = dict3["material"].cast<PxMaterial *>();
          } else {
            config.link[name].shape[idx].material = config.link[name].material;
          }
          if (dict3.contains("density")) {
            std::cout << "found shape density" << std::endl;
            config.link[name].shape[idx].density = dict3["density"].cast<float>();
          } else {
            config.link[name].shape[idx].density = config.link[name].density;
          }
          if (dict3.contains("patch_radius")) {
            std::cout << "found shape patch" << std::endl;
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
  auto PyShape = py::class_<SShape>(m, "CollisionShape");

  // enums
  auto PySolverType = py::enum_<PxSolverType::Enum>(m, "SolverType");
  auto PyActorType = py::enum_<EActorType>(m, "ActorType");
  auto PyArticulationJointType =
      py::enum_<PxArticulationJointType::Enum>(m, "ArticulationJointType");
  auto PyArticulationType = py::enum_<EArticulationType>(m, "ArticulationType");

  auto PyURDFLoader = py::class_<URDF::URDFLoader>(m, "URDFLoader");
  auto PyPxMaterial =
      py::class_<PxMaterial, std::unique_ptr<PxMaterial, py::nodelete>>(m, "PxMaterial");
  auto PyPose = py::class_<PxTransform>(m, "Pose");
  auto PyRenderMaterial = py::class_<Renderer::PxrMaterial>(m, "PxrMaterial");
  py::class_<Renderer::IPxrRenderer>(m, "IPxrRenderer");
  py::class_<Renderer::IPxrScene>(m, "IPxrScene");
  auto PyISensor = py::class_<Renderer::ISensor>(m, "ISensor");
  auto PyICamera = py::class_<Renderer::ICamera, Renderer::ISensor>(m, "ICamera");

  auto PyOptifuserRenderer =
      py::class_<Renderer::OptifuserRenderer, Renderer::IPxrRenderer>(m, "OptifuserRenderer");
  auto PyOptifuserConfig = py::class_<Renderer::OptifuserConfig>(m, "OptifuserConfig");
  auto PyInput = py::class_<Optifuser::Input>(m, "Input");
  auto PyOptifuserController = py::class_<Renderer::OptifuserController>(m, "OptifuserController");
  auto PyCameraSpec = py::class_<Optifuser::CameraSpec>(m, "CameraSpec");
  auto PyOptifuserCamera =
      py::class_<Renderer::OptifuserCamera, Renderer::ICamera>(m, "OptifuserCamera");

  auto PyEngine = py::class_<Simulation>(m, "Engine");
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
  auto PyContactPoint = py::class_<SContactPoint>(m, "ContactPoint");

  auto PyActorBuilder = py::class_<ActorBuilder>(m, "ActorBuilder");
  auto PyShapeRecord = py::class_<ActorBuilder::ShapeRecord>(m, "ShapeRecord");
  auto PyVisualRecord = py::class_<ActorBuilder::VisualRecord>(m, "VisualRecord");
  auto PyLinkBuilder = py::class_<LinkBuilder, ActorBuilder>(m, "LinkBuilder");
  auto PyJointRecord = py::class_<LinkBuilder::JointRecord>(m, "JointRecord");
  auto PyArticulationBuilder = py::class_<ArticulationBuilder>(m, "ArticulationBuilder");

#ifdef _USE_VULKAN
  auto PySapienVulkanRenderer =
      py::class_<Renderer::SapienVulkanRenderer, Renderer::IPxrRenderer>(m, "VulkanRenderer");
  auto PySapienVulkanCamera =
      py::class_<Renderer::SapienVulkanCamera, Renderer::ICamera>(m, "VulkanCamera");
#ifdef ON_SCREEN
  auto PySapienVulkanController =
      py::class_<Renderer::SapienVulkanController>(m, "VulkanController");
#endif
#endif

#ifdef _USE_PINOCCHIO
  auto PyPinocchioModel = py::class_<PinocchioModel>(m, "PinocchioModel");
#endif

  //======== Internal ========//
  PySolverType.value("PGS", PxSolverType::ePGS).value("TGS", PxSolverType::eTGS).export_values();

  PyArticulationJointType.value("PRISMATIC", PxArticulationJointType::ePRISMATIC)
      .value("REVOLUTE", PxArticulationJointType::eREVOLUTE)
      .value("SPHERICAL", PxArticulationJointType::eSPHERICAL)
      .value("FIX", PxArticulationJointType::eFIX)
      .value("UNDEFINED", PxArticulationJointType::eUNDEFINED)
      .export_values();

  PyPxMaterial.def("get_static_friction", &PxMaterial::getStaticFriction)
      .def("get_dynamic_friction", &PxMaterial::getDynamicFriction)
      .def("get_restitution", &PxMaterial::getRestitution)
      .def("set_static_friction", &PxMaterial::setStaticFriction, py::arg("coef"))
      .def("set_dynamic_friction", &PxMaterial::setDynamicFriction, py::arg("coef"))
      .def("set_restitution", &PxMaterial::setRestitution, py::arg("coef"));

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
      .def("set_p", [](PxTransform &t, const py::array_t<PxReal> &arr) { t.p = array2vec3(arr); },
           py::arg("p"))
      .def("set_q",
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

  PyShape.def_readonly("type", &SShape::type)
      .def_readonly("pose", &SShape::pose)
      .def_property_readonly("box_geometry",
                             [](SShape &s) {
                               if (s.type == "box") {
                                 return static_cast<SBoxGeometry *>(s.geometry.get());
                               }
                               return static_cast<SBoxGeometry *>(nullptr);
                             },
                             py::return_value_policy::reference)
      .def_property_readonly("sphere_geometry",
                             [](SShape &s) {
                               if (s.type == "sphere") {
                                 return static_cast<SSphereGeometry *>(s.geometry.get());
                               }
                               return static_cast<SSphereGeometry *>(nullptr);
                             },
                             py::return_value_policy::reference)
      .def_property_readonly("capsule_geometry",
                             [](SShape &s) {
                               if (s.type == "capsule") {
                                 return static_cast<SCapsuleGeometry *>(s.geometry.get());
                               }
                               return static_cast<SCapsuleGeometry *>(nullptr);
                             },
                             py::return_value_policy::reference)
      .def_property_readonly("plane_geometry",
                             [](SShape &s) {
                               if (s.type == "plane") {
                                 return static_cast<SPlaneGeometry *>(s.geometry.get());
                               }
                               return static_cast<SPlaneGeometry *>(nullptr);
                             },
                             py::return_value_policy::reference)
      .def_property_readonly("convex_mesh_geometry",
                             [](SShape &s) {
                               if (s.type == "convex_mesh") {
                                 return static_cast<SConvexMeshGeometry *>(s.geometry.get());
                               }
                               return static_cast<SConvexMeshGeometry *>(nullptr);
                             },
                             py::return_value_policy::reference);

  //======== Render Interface ========//
  PyRenderMaterial.def(py::init<>())
      .def("set_base_color",
           [](Renderer::PxrMaterial &mat, py::array_t<float> color) {
             mat.base_color = {color.at(0), color.at(1), color.at(2), color.at(3)};
           },
           py::arg("rgba"))
      .def_property_readonly("base_color",
                             [](Renderer::PxrMaterial &mat) {
                               return make_array<float>({mat.base_color[0], mat.base_color[1],
                                                         mat.base_color[2], mat.base_color[3]});
                             })
      .def_readwrite("specular", &Renderer::PxrMaterial::specular)
      .def_readwrite("roughness", &Renderer::PxrMaterial::roughness)
      .def_readwrite("metallic", &Renderer::PxrMaterial::metallic)
      .def("__repr__", [](Renderer::PxrMaterial &m) { return "PxrMaterial()"; })

      // TODO: implement those together with UV
      // .def_readwrite("color_texture", &Renderer::PxrMaterial::color_texture)
      // .def_readwrite("specular_texture", &Renderer::PxrMaterial::specular_texture)
      // .def_readwrite("normal_texture", &Renderer::PxrMaterial::normal_texture)
      ;

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
      .def_property_readonly("input",
                             [](Renderer::OptifuserController &) { return Optifuser::getInput(); },
                             py::return_value_policy::reference)
      .def("get_selected_actor", &Renderer::OptifuserController::getSelectedActor,
           py::return_value_policy::reference);

  PyCameraSpec.def_readwrite("name", &Optifuser::CameraSpec::name)
      .def("set_position",
           [](Optifuser::CameraSpec &c, const py::array_t<PxReal> &arr) {
             c.position = {arr.at(0), arr.at(1), arr.at(2)};
           },
           py::arg("position"))
      .def("set_rotation",
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
      .def("lookAt",
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
      .def("take_raytraced_picture",
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
      .def(py::init<uint32_t, PxReal, PxReal>(), py::arg("n_thread") = 0,
           py::arg("tolerance_length") = 0.1f, py::arg("tolerance_speed") = 0.2f)
      .def("set_renderer", &Simulation::setRenderer, py::arg("renderer"))
      .def("get_renderer", &Simulation::getRenderer, py::return_value_policy::reference)
      .def("create_physical_material", &Simulation::createPhysicalMaterial,
           py::arg("static_friction"), py::arg("dynamic_friction"), py::arg("restitution"),
           py::return_value_policy::reference)
      .def("set_log_level", &Simulation::setLogLevel, py::arg("level"))
      .def("create_scene", &Simulation::createScene, py::arg("config") = SceneConfig());

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
      .def("create_actor_builder", &SScene::createActorBuilder)
      .def("create_articulation_builder", &SScene::createArticulationBuilder)
      .def("create_urdf_loader", &SScene::createURDFLoader)
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
           py::arg("material") = nullptr, py::arg("render_material") = Renderer::PxrMaterial())
      .def("get_contacts", &SScene::getContacts, py::return_value_policy::reference)
      .def("get_all_actors", &SScene::getAllActors, py::return_value_policy::reference)
      .def("get_all_articulations", &SScene::getAllArticulations,
           py::return_value_policy::reference)

      .def("set_shadow_light",
           [](SScene &s, py::array_t<PxReal> const &direction, py::array_t<PxReal> const &color) {
             s.setShadowLight(array2vec3(direction), array2vec3(color));
           },
           py::arg("direction"), py::arg("color"))
      .def("set_ambient_light",
           [](SScene &s, py::array_t<PxReal> const &color) {
             s.setAmbientLight(array2vec3(color));
           },
           py::arg("clor"))
      .def("add_point_light",
           [](SScene &s, py::array_t<PxReal> const &position, py::array_t<PxReal> const &color) {
             s.addPointLight(array2vec3(position), array2vec3(color));
           },
           py::arg("position"), py::arg("color"))
      .def("add_directional_light",
           [](SScene &s, py::array_t<PxReal> const &direction, py::array_t<PxReal> const &color) {
             s.addDirectionalLight(array2vec3(direction), array2vec3(color));
           },
           py::arg("direction"), py::arg("color"))
      .def("get_renderer_scene", &SScene::getRendererScene)

      // drive, constrains, and joints
      .def("create_drive", &SScene::createDrive, py::arg("actor1"), py::arg("pose1"),
           py::arg("actor2"), py::arg("pose2"), py::return_value_policy::reference)
      .def_readonly("render_id_to_visual_name", &SScene::mRenderId2VisualName);

  //======= Drive =======//
  PyDrive
      .def("set_properties", &SDrive::setProperties, py::arg("stiffness"), py::arg("damping"),
           py::arg("force_limit") = PX_MAX_F32, py::arg("is_acceleration") = true)
      .def("set_target", &SDrive::setTarget, py::arg("pose"))
      .def("set_target_velocity",
           [](SDrive &d, py::array_t<PxReal> const &linear, py::array_t<PxReal> const &angular) {
             d.setTargetVelocity(array2vec3(linear), array2vec3(angular));
           },
           py::arg("linear"), py::arg("angular"))
      .def("destroy", &SDrive::destroy);

  //======== Actor ========//

  PyActorType.value("STATIC", EActorType::STATIC)
      .value("KINEMATIC", EActorType::KINEMATIC)
      .value("DYNAMIC", EActorType::DYNAMIC)
      .value("LINK", EActorType::ARTICULATION_LINK)
      .value("KINEMATIC_LINK", EActorType::KINEMATIC_ARTICULATION_LINK)
      .export_values();

  PyActorBase.def_property("name", &SActorBase::getName, &SActorBase::setName)
      .def("__repr__",
           [](SActorBase &actor) {
             std::ostringstream oss;
             oss << "Actor [" << actor.getName() << "] with id number [" << actor.getId() << "]\n";
             return oss.str();
           })
      .def("get_name", &SActorBase::getName)
      .def("set_name", &SActorBase::setName, py::arg("name"))
      .def_property_readonly("type", &SActorBase::getType)
      .def_property_readonly("id", &SActorBase::getId)
      .def("get_id", &SActorBase::getId)
      .def("get_scene", &SActorBase::getScene, py::return_value_policy::reference)
      .def_property_readonly("pose", &SActorBase::getPose)
      .def("get_pose", &SActorBase::getPose)
      .def_property_readonly("col1", &SActorBase::getCollisionGroup1)
      .def_property_readonly("col2", &SActorBase::getCollisionGroup2)
      .def_property_readonly("col3", &SActorBase::getCollisionGroup3)
      .def("get_collision_shapes", &SActorBase::getCollisionShapes)
      .def("hide_visual", &SActorBase::hideVisual)
      .def("unhide_visual", &SActorBase::unhideVisual)
      .def("is_hiding_visual", &SActorBase::isHidingVisual);

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
      .def("add_force_at_point",
           [](SActorDynamicBase &a, py::array_t<PxReal> const &force,
              py::array_t<PxReal> const &point) {
             a.addForceAtPoint(array2vec3(force), array2vec3(point));
           },
           py::arg("force"), py::arg("point"))

      .def("add_force_torque",
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
      .def_property_readonly("type", &SJointBase::getType)
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
      .def("set_limits",
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
           py::arg("limits"));

  PyJoint.def("set_friction", &SJoint::setFriction, py::arg("friction"))
      .def_property_readonly("friction", &SJoint::getFriction)
      .def("set_drive_property", &SJoint::setDriveProperty, py::arg("stiffness"),
           py::arg("damping"), py::arg("force_limit") = PX_MAX_F32)
      .def_property_readonly("stiffness", &SJoint::getDriveStiffness)
      .def_property_readonly("damping", &SJoint::getDriveDamping)
      .def_property_readonly("force_limit", &SJoint::getDriveForceLimit)
      .def("set_drive_velocity_target", [](SJoint &j, PxReal v) { j.setDriveVelocityTarget(v); },
           py::arg("velocity"))
      .def("set_drive_target", [](SJoint &j, PxReal p) { j.setDriveTarget(p); }, py::arg("target"))
      // TODO wrapper for array-valued targets
      .def("get_global_pose", &SJoint::getGlobalPose);

  //======== End Joint ========//

  //======== Articulation ========//
  PyArticulationType.value("DYNAMIC", EArticulationType::DYNAMIC)
      .value("KINEMATIC", EArticulationType::KINEMATIC)
      .export_values();

  PyArticulationBase.def_property("name", &SArticulationBase::getName, &SArticulationBase::setName)
      .def("get_name", &SArticulationBase::getName)
      .def("set_name", &SArticulationBase::setName, py::arg("name"))
      .def("get_base_links", &SArticulationBase::getBaseLinks, py::return_value_policy::reference)
      .def("get_base_joints", &SArticulationBase::getBaseJoints,
           py::return_value_policy::reference)
      .def_property_readonly("type", &SArticulationBase::getType)
      .def_property_readonly("dof", &SArticulationBase::dof)
      .def("get_qpos",
           [](SArticulationBase &a) {
             auto qpos = a.getQpos();
             return py::array_t<PxReal>(qpos.size(), qpos.data());
           })
      .def("set_qpos",
           [](SArticulationBase &a, const py::array_t<PxReal> &arr) {
             a.setQpos(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
           },
           py::arg("qpos"))

      .def("get_qvel",
           [](SArticulationBase &a) {
             auto qvel = a.getQvel();
             return py::array_t<PxReal>(qvel.size(), qvel.data());
           })
      .def("set_qvel",
           [](SArticulationBase &a, const py::array_t<PxReal> &arr) {
             a.setQvel(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
           },
           py::arg("qvel"))
      .def("get_qacc",
           [](SArticulationBase &a) {
             auto qacc = a.getQacc();
             return py::array_t<PxReal>(qacc.size(), qacc.data());
           })
      .def("set_qacc",
           [](SArticulationBase &a, const py::array_t<PxReal> &arr) {
             a.setQacc(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
           },
           py::arg("qacc"))
      .def("get_qf",
           [](SArticulationBase &a) {
             auto qf = a.getQf();
             return py::array_t<PxReal>(qf.size(), qf.data());
           })
      .def("set_qf",
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
      .def("set_qlimits",
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
      ;

  PyArticulationDrivable
      .def("get_drive_target",
           [](SArticulationDrivable &a) {
             auto target = a.getDriveTarget();
             return py::array_t<PxReal>(target.size(), target.data());
           })
      .def("set_drive_target",
           [](SArticulationDrivable &a, const py::array_t<PxReal> &arr) {
             a.setDriveTarget(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
           },
           py::arg("drive_target"));

  PyArticulation.def("get_links", &SArticulation::getSLinks, py::return_value_policy::reference)
      .def("get_joints", &SArticulation::getSJoints, py::return_value_policy::reference)
      .def("get_active_joints", &SArticulation::getActiveJoints,
           py::return_value_policy::reference)
      .def("set_root_velocity",
           [](SArticulation &a, py::array_t<PxReal> v) { a.setRootVelocity(array2vec3(v)); },
           py::arg("vel"))
      .def(
          "set_root_angular_velocity",
          [](SArticulation &a, py::array_t<PxReal> v) { a.setRootAngularVelocity(array2vec3(v)); },
          py::arg("vel"))
      .def("compute_passive_force",
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
      .def_property_readonly("actor1", [](SContact &contact) { return contact.actors[0]; },
                             py::return_value_policy::reference)
      .def_property_readonly("actor2", [](SContact &contact) { return contact.actors[1]; },
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
      .def("add_convex_shape_from_file",
           [](ActorBuilder &a, std::string const &filename, PxTransform const &pose,
              py::array_t<PxReal> const &scale, PxMaterial *material, PxReal density,
              PxReal patchRadius, PxReal minPatchRadius) {
             a.addConvexShapeFromFile(filename, pose, array2vec3(scale), material, density,
                                      patchRadius, minPatchRadius);
           },
           py::arg("filename"), py::arg("pose") = PxTransform(PxIdentity),
           py::arg("scale") = make_array<PxReal>({1, 1, 1}), py::arg("material") = nullptr,
           py::arg("density") = 1000, py::arg("patch_radius") = 0.f,
           py::arg("min_patch_radius") = 0.f)
      .def("add_multiple_convex_shapes_from_file",
           [](ActorBuilder &a, std::string const &filename, PxTransform const &pose,
              py::array_t<PxReal> const &scale, PxMaterial *material, PxReal density,
              PxReal patchRadius, PxReal minPatchRadius) {
             a.addMultipleConvexShapesFromFile(filename, pose, array2vec3(scale), material,
                                               density, patchRadius, minPatchRadius);
           },
           py::arg("filename"), py::arg("pose") = PxTransform(PxIdentity),
           py::arg("scale") = make_array<PxReal>({1, 1, 1}), py::arg("material") = nullptr,
           py::arg("density") = 1000, py::arg("patch_radius") = 0.f,
           py::arg("min_patch_radius") = 0.f)
      .def("add_box_shape",
           [](ActorBuilder &a, PxTransform const &pose, py::array_t<PxReal> const &size,
              PxMaterial *material, PxReal density, PxReal patchRadius, PxReal minPatchRadius) {
             a.addBoxShape(pose, array2vec3(size), material, density, patchRadius, minPatchRadius);
           },
           py::arg("pose") = PxTransform(PxIdentity),
           py::arg("size") = make_array<PxReal>({1, 1, 1}), py::arg("material") = nullptr,
           py::arg("density") = 1000, py::arg("patch_radius") = 0.f,
           py::arg("min_patch_radius") = 0.f)
      .def("add_capsule_shape", &ActorBuilder::addCapsuleShape,
           py::arg("pose") = PxTransform(PxIdentity), py::arg("radius") = 1,
           py::arg("half_length") = 1, py::arg("material") = nullptr, py::arg("density") = 1000,
           py::arg("patch_radius") = 0.f, py::arg("min_patch_radius") = 0.f)
      .def("add_sphere_shape", &ActorBuilder::addSphereShape,
           py::arg("pose") = PxTransform(PxIdentity), py::arg("radius") = 1,
           py::arg("material") = nullptr, py::arg("density") = 1000, py::arg("patch_radius") = 0.f,
           py::arg("min_patch_radius") = 0.f)

      .def("add_box_visual",
           [](ActorBuilder &a, PxTransform const &pose, py::array_t<PxReal> const &size,
              py::array_t<PxReal> color, std::string const &name) {
             a.addBoxVisual(pose, array2vec3(size), array2vec3(color), name);
           },
           py::arg("pose") = PxTransform(PxIdentity),
           py::arg("size") = make_array<PxReal>({1, 1, 1}),
           py::arg("color") = make_array<PxReal>({1, 1, 1}), py::arg("name") = "")
      .def("add_box_visual_complex",
           [](ActorBuilder &a, PxTransform const &pose, py::array_t<PxReal> const &size,
              const Renderer::PxrMaterial &mat, std::string const &name) {
             a.addBoxVisualWithMaterial(pose, array2vec3(size), mat, name);
           },
           py::arg("pose") = PxTransform(PxIdentity),
           py::arg("size") = make_array<PxReal>({1, 1, 1}),
           py::arg("material") = Renderer::PxrMaterial(), py::arg("name") = "")
      .def("add_capsule_visual",
           [](ActorBuilder &a, PxTransform const &pose, PxReal radius, PxReal halfLength,
              py::array_t<PxReal> color, std::string const &name) {
             a.addCapsuleVisual(pose, radius, halfLength, array2vec3(color), name);
           },
           py::arg("pose") = PxTransform(PxIdentity), py::arg("radius") = 1,
           py::arg("half_length") = 1, py::arg("color") = make_array<PxReal>({1, 1, 1}),
           py::arg("name") = "")
      .def("add_capsule_visual_complex",
           [](ActorBuilder &a, PxTransform const &pose, PxReal radius, PxReal halfLength,
              const Renderer::PxrMaterial &mat, std::string const &name) {
             a.addCapsuleVisualWithMaterial(pose, radius, halfLength, mat, name);
           },
           py::arg("pose") = PxTransform(PxIdentity), py::arg("radius") = 1,
           py::arg("half_length") = 1, py::arg("material") = Renderer::PxrMaterial(),
           py::arg("name") = "")
      .def("add_sphere_visual",
           [](ActorBuilder &a, PxTransform const &pose, PxReal radius, py::array_t<PxReal> color,
              std::string const &name) {
             a.addSphereVisual(pose, radius, array2vec3(color), name);
           },
           py::arg("pose") = PxTransform(PxIdentity), py::arg("radius") = 1,
           py::arg("color") = make_array<PxReal>({1, 1, 1}), py::arg("name") = "")
      .def("add_sphere_visual_complex",
           [](ActorBuilder &a, PxTransform const &pose, PxReal radius,
              const Renderer::PxrMaterial &mat,
              std::string const &name) { a.addSphereVisualWithMaterial(pose, radius, mat, name); },
           py::arg("pose") = PxTransform(PxIdentity), py::arg("radius") = 1,
           py::arg("material") = Renderer::PxrMaterial(), py::arg("name") = "")
      .def("add_visual_from_file",
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
      .def("set_parent", &LinkBuilder::setParent)
      .def("set_name", &LinkBuilder::setName)
      .def("set_joint_name", &LinkBuilder::setJointName)
      .def("set_joint_properties",
           [](LinkBuilder &b, PxArticulationJointType::Enum jointType,
              py::array_t<PxReal> const &limits, PxTransform const &parentPose,
              PxTransform const &childPose, PxReal friction, PxReal damping) {
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
             b.setJointProperties(jointType, l, parentPose, childPose, friction, damping);
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
                                 return "undefined";
                               default:
                                 return "undefined";
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
      .def("create_link_builder",
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
      .def_readwrite("collision_is_visual", &URDF::URDFLoader::collisionIsVisual)
      .def_readwrite("scale", &URDF::URDFLoader::scale)
      .def_property("default_density",
                    [](URDF::URDFLoader &) {
                      throw std::runtime_error("default_density is moved to URDF config");
                    },
                    [](URDF::URDFLoader &) {
                      throw std::runtime_error("default_density is moved to URDF config");
                    })
      .def("load",
           [](URDF::URDFLoader &loader, std::string const &filename, py::dict &dict) {
             auto config = parseURDFConfig(dict);
             return loader.load(filename, config);
           },
           py::return_value_policy::reference, py::arg("filename"), py::arg("config") = py::dict())
      .def("load_kinematic",
           [](URDF::URDFLoader &loader, std::string const &filename, py::dict &dict) {
             auto config = parseURDFConfig(dict);
             return loader.load(filename, config);
           },
           py::return_value_policy::reference, py::arg("filename"), py::arg("config") = py::dict())
      .def("load_from_string",
           [](URDF::URDFLoader &loader, std::string const &urdf, std::string const &srdf,
              py::dict &dict) {
             auto config = parseURDFConfig(dict);
             return loader.loadFromXML(urdf, srdf, config);
           },
           py::return_value_policy::reference, py::arg("urdf_string"), py::arg("srdf_string"),
           py::arg("config") = py::dict())
      .def("load_file_as_articulation_builder",
           [](URDF::URDFLoader &loader, std::string const &filename, py::dict &dict) {
             auto config = parseURDFConfig(dict);
             return loader.loadFileAsArticulationBuilder(filename, config);
           },
           py::return_value_policy::reference, py::arg("filename"),
           py::arg("config") = py::dict());

#ifdef _USE_PINOCCHIO
  PyPinocchioModel
      .def("compute_forward_kinematics", &PinocchioModel::computeForwardKinematics,
           py::arg("qpos"))
      .def("get_link_pose", &PinocchioModel::getLinkPose, py::arg("link_index"))
      .def("compute_inverse_kinematics", &PinocchioModel::computeInverseKinematics,
           py::arg("link_index"), py::arg("pose"), py::arg("eps") = 1e-4,
           py::arg("max_iterations") = 1000, py::arg("dt") = 0.1, py::arg("damp") = 1e-6)
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

#ifdef _USE_VULKAN
  PySapienVulkanRenderer.def(py::init<bool>(), py::arg("offscreen_only") = false)
      .def_static("set_shader_dir", &svulkan::VulkanContext::setDefaultShaderDir,
                  py::arg("spv_dir"));
  PySapienVulkanCamera
      .def("get_position_rgba",
           [](Renderer::SapienVulkanCamera &cam) {
             return py::array_t<float>(
                 {static_cast<int>(cam.getHeight()), static_cast<int>(cam.getWidth()), 4},
                 cam.getPositionRGBA().data());
           })
      .def("get_model_matrix",
           [](Renderer::SapienVulkanCamera &c) { return mat42array(c.getModelMatrix()); })
      .def("get_projection_matrix",
           [](Renderer::SapienVulkanCamera &c) { return mat42array(c.getProjectionMatrix()); })
      .def("set_mode_orthographic", &Renderer::SapienVulkanCamera::changeModeToOrthographic,
           py::arg("scaling") = 1.f)
      .def("set_mode_perspective", &Renderer::SapienVulkanCamera::changeModeToPerspective,
           py::arg("fovy") = glm::radians(35.f));

#ifdef ON_SCREEN
  PySapienVulkanController.def(py::init<Renderer::SapienVulkanRenderer *>(), py::arg("renderer"))
      .def_property_readonly("is_closed", &Renderer::SapienVulkanController::isClosed)
      .def("render", &Renderer::SapienVulkanController::render)
      .def("set_current_scene", &Renderer::SapienVulkanController::setScene, py::arg("scene"))

      // UI control
      .def("select_actor",
           [](Renderer::SapienVulkanController &c, SActorBase *actor) {
             c.selectActor(actor->getId());
           },
           py::arg("actor"))
      .def("focus_actor",
           [](Renderer::SapienVulkanController &c, SActorBase *actor) {
             c.focusActor(actor->getId());
           },
           py::arg("actor"))
      .def("view_from_camera",
           [](Renderer::SapienVulkanController &c, uint32_t camera_index) {
             c.viewFromCamera(camera_index);
           },
           py::arg("camera_index"))
      .def("pause", &Renderer::SapienVulkanController::pause, py::arg("p") = true)
      .def("set_free_camera_position", &Renderer::SapienVulkanController::setFreeCameraPosition,
           py::arg("x"), py::arg("y"), py::arg("z"))
      .def("set_free_camera_rotation", &Renderer::SapienVulkanController::setFreeCameraRotation,
           py::arg("yaw"), py::arg("pitch"), py::arg("roll"))
      .def("set_default_control", &Renderer::SapienVulkanController::setDefaultControl,
           py::arg("mouse"), py::arg("keyboard"))
      .def("close", &Renderer::SapienVulkanController::close)

      // input
      .def("mouse_down", &Renderer::SapienVulkanController::mouseDown, py::arg("key_code") = 0)
      .def("mouse_click", &Renderer::SapienVulkanController::mouseClick, py::arg("key_code") = 0)
      .def("key_down", &Renderer::SapienVulkanController::keyDown, py::arg("key"))
      .def("key_press", &Renderer::SapienVulkanController::keyPressed, py::arg("key"))
      .def_property_readonly("mouse_pos", &Renderer::SapienVulkanController::getMousePos)
      .def_property_readonly("mouse_delta", &Renderer::SapienVulkanController::getMouseDelta)
      .def_property_readonly("wheel_delta", &Renderer::SapienVulkanController::getMouseWheelDelta);

#endif
#endif
}
