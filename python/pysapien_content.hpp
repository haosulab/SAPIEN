#pragma once

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "sapien/actor_builder.h"
#include "sapien/awaitable.hpp"
#include "sapien/renderer/render_interface.h"
#include "sapien/sapien_actor.h"
#include "sapien/sapien_actor_base.h"
#include "sapien/sapien_contact.h"
#include "sapien/sapien_drive.h"
#include "sapien/sapien_entity_particle.h"
#include "sapien/sapien_gear.h"
#include "sapien/sapien_material.h"
#include "sapien/sapien_scene.h"
#include "sapien/simulation.h"

#include "sapien/articulation/articulation_builder.h"
#include "sapien/articulation/sapien_articulation.h"
#include "sapien/articulation/sapien_articulation_base.h"
#include "sapien/articulation/sapien_joint.h"
#include "sapien/articulation/sapien_kinematic_articulation.h"
#include "sapien/articulation/sapien_kinematic_joint.h"
#include "sapien/articulation/sapien_link.h"
#include "sapien/articulation/urdf_loader.h"
#include "sapien/event_system/event_system.h"

#ifdef SAPIEN_KUAFU
#include "sapien/renderer/kuafu_renderer.hpp"
#endif

#include "sapien/renderer/render_config.h"
#include "sapien/renderer/svulkan2_pointbody.h"
#include "sapien/renderer/svulkan2_renderer.h"
#include "sapien/renderer/svulkan2_rigidbody.h"
#include "sapien/renderer/svulkan2_scene.h"
#include "sapien/renderer/svulkan2_shape.h"
#include "sapien/renderer/svulkan2_window.h"

#include "sapien/renderer/server/client.h"
#include "sapien/renderer/server/server.h"

#include "sapien/articulation/pinocchio_model.h"
#include "sapien/profiler.hpp"

#include "sapien/utils/pose.hpp"

using namespace pybind11::literals;
using namespace sapien;
namespace py = pybind11;

class ProfilerBlock {
  std::string mName;

public:
  ProfilerBlock(std::string const &name) : mName(name) {}
  void enter() { EASY_NONSCOPED_BLOCK(mName); }
  void exit(const py::object &type, const py::object &value, const py::object &traceback) {
    EASY_END_BLOCK;
  }
};

#ifdef SAPIEN_DLPACK
static py::capsule wrapDLTensor(DLManagedTensor *tensor) {
  auto capsule_destructor = [](PyObject *data) {
    DLManagedTensor *tensor = (DLManagedTensor *)PyCapsule_GetPointer(data, "dltensor");
    if (tensor) {
      tensor->deleter(const_cast<DLManagedTensor *>(tensor));
    } else {
      PyErr_Clear();
    }
  };
  return py::capsule(tensor, "dltensor", capsule_destructor);
}

using dl_vector = std::vector<DLManagedTensor *>;

class AwaitableDLVectorWrapper : public std::enable_shared_from_this<AwaitableDLVectorWrapper> {

public:
  AwaitableDLVectorWrapper(std::shared_ptr<IAwaitable<dl_vector>> awaitable)
      : mAwaitable(awaitable) {}
  std::vector<py::capsule> wait() {
    auto ts = mAwaitable->wait();
    std::vector<py::capsule> result;
    result.reserve(ts.size());
    for (auto t : ts) {
      result.push_back(wrapDLTensor(t));
    }
    return result;
  }
  bool ready() { return mAwaitable->ready(); }

private:
  std::shared_ptr<IAwaitable<dl_vector>> mAwaitable;
};
#endif

template <typename T> void declare_awaitable(py::module &m, std::string const &typestr) {
  using Class = IAwaitable<T>;
  std::string pyclass_name = std::string("Awaitable") + typestr;
  py::class_<Class, std::shared_ptr<Class>>(m, pyclass_name.c_str())
      .def("wait", &Class::wait)
      .def("ready", &Class::ready);
}

py::array_t<float> getFloatImageFromCamera(SCamera &cam, std::string const &name) {
  uint32_t width = cam.getWidth();
  uint32_t height = cam.getHeight();
  auto image = cam.getRendererCamera()->getFloatImage(name);
  uint32_t channel = image.size() / (width * height);
  if (channel == 1) {
    return py::array_t<float>({height, width}, image.data());
  } else {
    return py::array_t<float>({height, width, channel}, image.data());
  }
}

py::array_t<uint32_t> getUintImageFromCamera(SCamera &cam, std::string const &name) {
  uint32_t width = cam.getWidth();
  uint32_t height = cam.getHeight();
  auto image = cam.getRendererCamera()->getUintImage(name);
  uint32_t channel = image.size() / (width * height);
  if (channel == 1) {
    return py::array_t<uint32_t>({height, width}, image.data());
  } else {
    return py::array_t<uint32_t>({height, width, channel}, image.data());
  }
}

py::array_t<uint8_t> getUint8ImageFromCamera(SCamera &cam, std::string const &name) {
  uint32_t width = cam.getWidth();
  uint32_t height = cam.getHeight();
  auto image = cam.getRendererCamera()->getUint8Image(name);
  uint32_t channel = image.size() / (width * height);
  if (channel == 1) {
    return py::array_t<uint8_t>({height, width}, image.data());
  } else {
    return py::array_t<uint8_t>({height, width, channel}, image.data());
  }
}

py::array getImageFromCamera(SCamera &cam, std::string const &name) {
  std::string format = cam.getRendererCamera()->getImageFormat(name);
  if (format == "i4") {
    return getUintImageFromCamera(cam, name);
  } else if (format == "f4") {
    return getFloatImageFromCamera(cam, name);
  } else if (format == "u1") {
    return getUint8ImageFromCamera(cam, name);
  }
  throw std::runtime_error("unexpected image format " + format);
}

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
        throw std::runtime_error("Please use collision rather than shape!");
      }
      if (dict2.contains("collision")) {
        auto shapeDict = dict2["collision"].cast<py::dict>();
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

py::array_t<PxReal> mat32array(glm::mat3 const &mat) {
  float arr[] = {mat[0][0], mat[1][0], mat[2][0], mat[0][1], mat[1][1],
                 mat[2][1], mat[0][2], mat[1][2], mat[2][2]};
  return py::array_t<PxReal>({3, 3}, arr);
}

static auto getFilterMode(std::string mode) {
  if (mode == "linear") {
    return Renderer::IPxrTexture::FilterMode::eLINEAR;
  } else if (mode == "nearest") {
    return Renderer::IPxrTexture::FilterMode::eNEAREST;
  }
  throw std::invalid_argument("Unknown filter mode " + mode);
}

static auto getAddressMode(std::string mode) {
  if (mode == "repeat") {
    return Renderer::IPxrTexture::AddressMode::eREPEAT;
  } else if (mode == "border") {
    return Renderer::IPxrTexture::AddressMode::eBORDER;
  } else if (mode == "edge") {
    return Renderer::IPxrTexture::AddressMode::eEDGE;
  } else {
    throw std::invalid_argument("Unknown address mode " + mode);
  }
}

#define DEPRECATE_WARN(OLD, NEW)                                                                  \
  PyErr_WarnEx(PyExc_DeprecationWarning, #OLD " is deprecated, use " #NEW " instead.", 1)

void buildSapien(py::module &m) {
  m.doc() = "SAPIEN core module";

  declare_awaitable<void>(m, "Void");

#ifdef SAPIEN_DLPACK
  py::class_<AwaitableDLVectorWrapper, std::shared_ptr<AwaitableDLVectorWrapper>>(
      m, "AwaitableDLList")
      .def("wait", &AwaitableDLVectorWrapper::wait)
      .def("ready", &AwaitableDLVectorWrapper::ready);
#endif

  auto PyMultistepCallback = py::class_<SceneMultistepCallback>(m, "SceneMultistepCallback");

  // collision geometry and shape
  auto PyGeometry = py::class_<SGeometry>(m, "CollisionGeometry");
  auto PyBoxGeometry = py::class_<SBoxGeometry, SGeometry>(m, "BoxGeometry");
  auto PySphereGeometry = py::class_<SSphereGeometry, SGeometry>(m, "SphereGeometry");
  auto PyCapsuleGeometry = py::class_<SCapsuleGeometry, SGeometry>(m, "CapsuleGeometry");
  auto PyPlaneGeometry = py::class_<SPlaneGeometry, SGeometry>(m, "PlaneGeometry");
  auto PyConvexMeshGeometry = py::class_<SConvexMeshGeometry, SGeometry>(m, "ConvexMeshGeometry");
  auto PyNonconvexMeshGeometry =
      py::class_<SNonconvexMeshGeometry, SGeometry>(m, "NonconvexMeshGeometry");

  auto PyCollisionShape = py::class_<SCollisionShape>(m, "CollisionShape");

  auto PyURDFLoader = py::class_<URDF::URDFLoader>(m, "URDFLoader");
  auto PyPhysicalMaterial =
      py::class_<SPhysicalMaterial, std::shared_ptr<SPhysicalMaterial>>(m, "PhysicalMaterial");
  auto PyPose = py::class_<PxTransform>(m, "Pose");
  auto PyRenderMaterial =
      py::class_<Renderer::IPxrMaterial, std::shared_ptr<Renderer::IPxrMaterial>>(
          m, "RenderMaterial");
  auto PyRenderTexture = py::class_<Renderer::IPxrTexture, std::shared_ptr<Renderer::IPxrTexture>>(
      m, "RenderTexture");

  auto PyRenderShape =
      py::class_<Renderer::IPxrRenderShape, std::shared_ptr<Renderer::IPxrRenderShape>>(
          m, "RenderShape");

  auto PyRenderer = py::class_<Renderer::IPxrRenderer, std::shared_ptr<Renderer::IPxrRenderer>>(
      m, "IPxrRenderer");
  auto PyRenderScene = py::class_<Renderer::IPxrScene>(m, "RenderScene");
  auto PyRenderBody = py::class_<Renderer::IPxrRigidbody>(m, "RenderBody");
  auto PyRenderParticleBody = py::class_<Renderer::IPxrPointBody>(m, "RenderParticleBody");
  auto PyVulkanParticleBody =
      py::class_<Renderer::SVulkan2PointBody, Renderer::IPxrPointBody>(m, "VulkanParticleBody");

  // auto PyISensor = py::class_<Renderer::ISensor>(m, "ISensor");
  // auto PyICamera = py::class_<Renderer::ICamera, Renderer::ISensor>(m, "ICamera");

  auto PyEngine = py::class_<Simulation, std::shared_ptr<Simulation>>(m, "Engine");
  auto PySceneConfig = py::class_<SceneConfig>(m, "SceneConfig");
  auto PyScene = py::class_<SScene>(m, "Scene");
  auto PyConstraint = py::class_<SDrive>(m, "Constraint");
  auto PyDrive = py::class_<SDrive6D, SDrive>(m, "Drive");
  auto PyGear = py::class_<SGear>(m, "Gear");

  auto PyEntity = py::class_<SEntity>(m, "Entity");
  auto PyActorBase = py::class_<SActorBase, SEntity>(m, "ActorBase");
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
  auto PyArticulationBase = py::class_<SArticulationBase, SEntity>(m, "ArticulationBase");
  auto PyArticulationDrivable =
      py::class_<SArticulationDrivable, SArticulationBase>(m, "ArticulationDrivable");
  auto PyArticulation = py::class_<SArticulation, SArticulationDrivable>(m, "Articulation");
  py::class_<SKArticulation, SArticulationDrivable>(m, "KinematicArticulation");

  auto PyContact = py::class_<SContact>(m, "Contact");
  auto PyTrigger = py::class_<STrigger>(m, "Trigger");
  auto PyContactPoint = py::class_<SContactPoint>(m, "ContactPoint");

  auto PyActorBuilder = py::class_<ActorBuilder, std::shared_ptr<ActorBuilder>>(m, "ActorBuilder");
  auto PyShapeRecord = py::class_<ActorBuilder::ShapeRecord>(m, "ShapeRecord");
  auto PyVisualRecord = py::class_<ActorBuilder::VisualRecord>(m, "VisualRecord");
  auto PyLinkBuilder =
      py::class_<LinkBuilder, ActorBuilder, std::shared_ptr<LinkBuilder>>(m, "LinkBuilder");
  auto PyJointRecord = py::class_<LinkBuilder::JointRecord>(m, "JointRecord");
  auto PyArticulationBuilder =
      py::class_<ArticulationBuilder, std::shared_ptr<ArticulationBuilder>>(m,
                                                                            "ArticulationBuilder");
  auto PyRenderMesh =
      py::class_<Renderer::IRenderMesh, std::shared_ptr<Renderer::IRenderMesh>>(m, "RenderMesh");
  auto PyVulkanRenderMesh =
      py::class_<Renderer::SVulkan2Mesh, Renderer::IRenderMesh,
                 std::shared_ptr<Renderer::SVulkan2Mesh>>(m, "VulkanRenderMesh");

  auto PySubscription = py::class_<Subscription>(m, "Subscription");

  auto PyPinocchioModel = py::class_<PinocchioModel>(m, "PinocchioModel");

  auto PyVulkanRigidbody =
      py::class_<Renderer::SVulkan2Rigidbody, Renderer::IPxrRigidbody>(m, "VulkanRigidbody");
  auto PyVulkanRenderer =
      py::class_<Renderer::SVulkan2Renderer, Renderer::IPxrRenderer,
                 std::shared_ptr<Renderer::SVulkan2Renderer>>(m, "SapienRenderer");

  // auto PyVulkanCamera = py::class_<Renderer::SVulkan2Camera, Renderer::ICamera>(m,
  // "VulkanCamera");

  auto PyVulkanWindow = py::class_<Renderer::SVulkan2Window>(m, "VulkanWindow");
  auto PyVulkanScene = py::class_<Renderer::SVulkan2Scene, Renderer::IPxrScene>(m, "VulkanScene");

  auto PyLightEntity = py::class_<SLight, SEntity>(m, "LightEntity");
  auto PyPointLightEntity = py::class_<SPointLight, SLight>(m, "PointLightEntity");
  auto PyDirectionalLightEntity =
      py::class_<SDirectionalLight, SLight>(m, "DirectionalLightEntity");
  auto PySpotLightEntity = py::class_<SSpotLight, SLight>(m, "SpotLightEntity");
  auto PyActiveLightEntity = py::class_<SActiveLight, SLight>(m, "ActiveLightEntity");

  auto PyParticleEntity = py::class_<SEntityParticle, SEntity>(m, "ParticleEntity");
  auto PyCameraEntity = py::class_<SCamera, SEntity>(m, "CameraEntity");

  auto PyRenderConfig = py::class_<Renderer::RenderConfig>(m, "RenderConfig");
  m.def("get_global_render_config", &Renderer::GetRenderConfig,
        py::return_value_policy::reference);
  PyRenderConfig.def_readwrite("viewer_shader_dir", &Renderer::RenderConfig::viewerShaderDirectory)
      .def_readwrite("camera_shader_dir", &Renderer::RenderConfig::cameraShaderDirectory)
      .def_readwrite("rt_samples_per_pixel", &Renderer::RenderConfig::rayTracingSamplesPerPixel)
      .def_readwrite("rt_path_depth", &Renderer::RenderConfig::rayTracingPathDepth)
      .def_readwrite("rt_use_denoiser", &Renderer::RenderConfig::rayTracingUseDenoiser)
      .def(
          "set_render_target_format",
          [](Renderer::RenderConfig &config, std::string name, std::string format) {
            config.renderTargetFormats[name] = format;
          },
          py::arg("name"), py::arg("format"))
      .def(
          "unset_render_target_format",
          [](Renderer::RenderConfig &config, std::string name) {
            config.renderTargetFormats.erase(name);
          },
          py::arg("name"))
      .def(
          "get_render_target_format",
          [](Renderer::RenderConfig &config, std::string name) {
            return config.renderTargetFormats.at(name);
          },
          py::arg("name"))
      .def(
          "has_render_target_format",
          [](Renderer::RenderConfig &config, std::string name) {
            return config.renderTargetFormats.contains(name);
          },
          py::arg("name"));

  //======== Kuafu ========//
#ifdef SAPIEN_KUAFU
  auto PyKuafuConfig = py::class_<Renderer::KuafuConfig>(m, "KuafuConfig");
  PyKuafuConfig.def(py::init<>())
      .def_readwrite("use_viewer", &Renderer::KuafuConfig::mUseViewer)
      .def_readwrite("viewer_width", &Renderer::KuafuConfig::mViewerWidth)
      .def_readwrite("viewer_height", &Renderer::KuafuConfig::mViewerHeight)
      .def_readwrite("assets_path", &Renderer::KuafuConfig::mAssetsPath)
      .def_readwrite("spp", &Renderer::KuafuConfig::mPerPixelSampleRate)
      .def_readwrite("max_bounces", &Renderer::KuafuConfig::mPathDepth)
      .def_readwrite("accumulate_frames", &Renderer::KuafuConfig::mAccumulateFrames)
      .def_readwrite("use_denoiser", &Renderer::KuafuConfig::mUseDenoiser)
      .def_readwrite("max_textures", &Renderer::KuafuConfig::mMaxTextures)
      .def_readwrite("max_materials", &Renderer::KuafuConfig::mMaxMaterials)
      .def_readwrite("max_geometries", &Renderer::KuafuConfig::mMaxGeometry)
      .def_readwrite("max_geometry_instances", &Renderer::KuafuConfig::mMaxGeometryInstances);

  auto PyKuafuRenderer = py::class_<Renderer::KuafuRenderer, Renderer::IPxrRenderer,
                                    std::shared_ptr<Renderer::KuafuRenderer>>(m, "KuafuRenderer");
  PyKuafuRenderer
      .def(py::init<Renderer::KuafuConfig>(), py::arg("config") = Renderer::KuafuConfig())
      .def_static("_set_default_assets_path", &Renderer::KuafuRenderer::setDefaultAssetsPath,
                  py::arg("assets_path"))
      .def_static("set_log_level", &Renderer::KuafuRenderer::setLogLevel, py::arg("level"))
      .def_property_readonly("is_running", &Renderer::KuafuRenderer::isRunning);

  // auto PyKuafuCamera = py::class_<Renderer::KuafuCamera, Renderer::ICamera>(m, "KuafuCamera");
  // PyKuafuCamera.def("set_full_perspective", &Renderer::KuafuCamera::setFullPerspective,
  //                   "Set camera into perspective projection mode with full camera parameters",
  //                   py::arg("fx"), py::arg("fy"), py::arg("cx"), py::arg("cy"),
  //                   py::arg("width"), py::arg("height"), py::arg("skew"));

#endif

  auto PyRenderClient =
      py::class_<Renderer::server::ClientRenderer, Renderer::IPxrRenderer,
                 std::shared_ptr<Renderer::server::ClientRenderer>>(m, "RenderClient");
  auto PyRenderServer = py::class_<Renderer::server::RenderServer>(m, "RenderServer");
  auto PyRenderServerBuffer =
      py::class_<Renderer::server::VulkanCudaBuffer>(m, "RenderServerBuffer");

  PyRenderClient.def(py::init<std::string, uint64_t>(), py::arg("address"),
                     py::arg("process_index"));

  PyRenderServer
      .def_static("_set_shader_dir", &Renderer::server::setDefaultShaderDirectory,
                  py::arg("shader_dir"))
      .def(py::init<uint32_t, uint32_t, uint32_t, std::string, bool>(),
           py::arg("max_num_materials") = 5000, py::arg("max_num_textures") = 5000,
           py::arg("default_mipmap_levels") = 1, py::arg("device") = "",
           py::arg("do_not_load_texture") = false)
      .def("start", &Renderer::server::RenderServer::start, py::arg("address"))
      .def("stop", &Renderer::server::RenderServer::stop)
      .def("wait_all", &Renderer::server::RenderServer::waitAll, py::arg("timeout") = UINT64_MAX)
      .def("wait_scenes", &Renderer::server::RenderServer::waitScenes, py::arg("scenes"),
           py::arg("timeout") = UINT64_MAX)
      // .def("allocate_buffer", &Renderer::server::RenderServer::allocateBuffer, py::arg("type"),
      //      py::arg("shape"), py::return_value_policy::reference)
      .def("auto_allocate_buffers", &Renderer::server::RenderServer::autoAllocateBuffers,
           py::arg("render_targets"), py::return_value_policy::reference)
      .def("summary", &Renderer::server::RenderServer::summary);

  PyRenderServerBuffer
      .def_property_readonly("nbytes", &Renderer::server::VulkanCudaBuffer::getSize)
      .def_property_readonly("type", &Renderer::server::VulkanCudaBuffer::getType)
      .def_property_readonly("shape",
                             [](Renderer::server::VulkanCudaBuffer &buffer) {
                               py::tuple shape = py::cast(buffer.getShape());
                               return shape;
                             })
#ifdef SAPIEN_CUDA
      .def_property_readonly("pointer",
                             [](Renderer::server::VulkanCudaBuffer &buffer) {
                               return reinterpret_cast<uintptr_t>(buffer.getCudaPtr());
                             })
      .def_property_readonly(
          "__cuda_array_interface__",
          [](Renderer::server::VulkanCudaBuffer &buffer) {
            py::tuple shape = py::cast(buffer.getShape());
            return py::dict(
                "shape"_a = shape, "typestr"_a = buffer.getType(),
                "data"_a = py::make_tuple(reinterpret_cast<uintptr_t>(buffer.getCudaPtr()), false),
                "version"_a = 2);
          })
#endif
      ;

  //======== Internal ========//

  PyPhysicalMaterial
      .def_property_readonly("static_friction", &SPhysicalMaterial::getStaticFriction)
      .def_property_readonly("dynamic_friction", &SPhysicalMaterial::getDynamicFriction)
      .def_property_readonly("restitution", &SPhysicalMaterial::getRestitution)
      .def("get_static_friction", &SPhysicalMaterial::getStaticFriction)
      .def("get_dynamic_friction", &SPhysicalMaterial::getDynamicFriction)
      .def("get_restitution", &SPhysicalMaterial::getRestitution)
      .def("set_static_friction", &SPhysicalMaterial::setStaticFriction, py::arg("coef"))
      .def("set_dynamic_friction", &SPhysicalMaterial::setDynamicFriction, py::arg("coef"))
      .def("set_restitution", &SPhysicalMaterial::setRestitution, py::arg("coef"));

  PyPose
      .def(py::init([](py::array_t<PxReal> p, py::array_t<PxReal> q) {
             if (p.size() == 16) {
               return utils::fromTransFormationMatrix(p);
             }
             if (p.size() == 3 && q.size() == 4) {
               return new PxTransform({p.at(0), p.at(1), p.at(2)},
                                      {q.at(1), q.at(2), q.at(3), q.at(0)});
             }
             throw std::invalid_argument("failed to create Pose: invalid array size");
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
      .def(py::self * py::self)
      .def(py::pickle(
          [](const PxTransform &p) {
            return py::make_tuple(p.p.x, p.p.y, p.p.z, p.q.x, p.q.y, p.q.z, p.q.w);
          },
          [](py::tuple t) {
            if (t.size() != 7) {
              throw std::runtime_error("Invalid state!");
            }
            return PxTransform(
                {t[0].cast<float>(), t[1].cast<float>(), t[2].cast<float>()},
                {t[3].cast<float>(), t[4].cast<float>(), t[5].cast<float>(), t[6].cast<float>()});
          }));

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
  PyNonconvexMeshGeometry
      .def_property_readonly("scale",
                             [](SNonconvexMeshGeometry &g) { return vec32array(g.scale); })
      .def_property_readonly(
          "rotation",
          [](SNonconvexMeshGeometry &g) {
            return make_array<PxReal>({g.rotation.w, g.rotation.x, g.rotation.y, g.rotation.z});
          })
      .def_property_readonly("vertices",
                             [](SNonconvexMeshGeometry &g) {
                               int nRows = g.vertices.size() / 3;
                               int nCols = 3;
                               return py::array_t<PxReal>({nRows, nCols},
                                                          {sizeof(PxReal) * nCols, sizeof(PxReal)},
                                                          g.vertices.data());
                             })
      .def_property_readonly(
          "indices", [](SNonconvexMeshGeometry &g) { return make_array<uint32_t>(g.indices); });

  PyCollisionShape
      .def_property_readonly("actor", &SCollisionShape::getActor,
                             py::return_value_policy::reference)

      .def("get_collision_groups", &SCollisionShape::getCollisionGroups)
      .def("set_collision_groups", &SCollisionShape::setCollisionGroups,
           R"doc(
collision groups determine the collision behavior of objects. Let A.gx denote the collision group x of collision shape A. Collision shape A and B will collide iff the following condition holds:

((A.g0 & B.g1) or (A.g1 & B.g0)) and (not ((A.g2 & B.g2) and ((A.g3 & 0xffff) == (B.g3 & 0xffff))))

Here is some explanation: g2 is the "ignore group" and g3 is the "id group". The only the lower 16 bits of the id group is used since the upper 16 bits are reserved for other purposes in the future. When 2 collision shapes have the same ID (g3), then if any of their g2 bits match, their collisions are definitely ignored.

If after testing g2 and g3, the objects may collide, g0 and g1 come into play. g0 is the "contact type group" and g1 is the "contact affinity group". Collision shapes collide only when a bit in the contact type of the first shape matches a bit in the contact affinity of the second shape.)doc",
           py::arg("group0"), py::arg("group1"), py::arg("group2"), py::arg("group3"))
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

  //======== Render Interface ========//
  PyRenderMaterial
      .def(
          "set_base_color",
          [](Renderer::IPxrMaterial &mat, py::array_t<float> color) {
            mat.setBaseColor({color.at(0), color.at(1), color.at(2), color.at(3)});
          },
          py::arg("rgba"))
      .def(
          "set_emission",
          [](Renderer::IPxrMaterial &mat, py::array_t<float> color) {
            mat.setEmission({color.at(0), color.at(1), color.at(2), color.at(3)});
          },
          py::arg("rgbs"))
      .def("set_specular", &Renderer::IPxrMaterial::setSpecular, py::arg("specular"))
      .def("set_metallic", &Renderer::IPxrMaterial::setMetallic, py::arg("metallic"))
      .def("set_roughness", &Renderer::IPxrMaterial::setRoughness, py::arg("roughness"))
      .def("set_transmission", &Renderer::IPxrMaterial::setTransmission, py::arg("transmission"))
      .def("set_ior", &Renderer::IPxrMaterial::setIOR, py::arg("ior"))

      .def("set_emission_texture", &Renderer::IPxrMaterial::setEmissionTexture, py::arg("texture"))
      .def("set_diffuse_texture", &Renderer::IPxrMaterial::setDiffuseTexture, py::arg("texture"))
      .def("set_metallic_texture", &Renderer::IPxrMaterial::setMetallicTexture, py::arg("texture"))
      .def("set_roughness_texture", &Renderer::IPxrMaterial::setRoughnessTexture,
           py::arg("texture"))
      .def("set_transmission_texture", &Renderer::IPxrMaterial::setTransmissionTexture,
           py::arg("texture"))
      .def("set_normal_texture", &Renderer::IPxrMaterial::setNormalTexture, py::arg("texture"))

      .def("set_emission_texture_from_file",
           &Renderer::IPxrMaterial::setEmissionTextureFromFilename, py::arg("path"))
      .def("set_diffuse_texture_from_file", &Renderer::IPxrMaterial::setDiffuseTextureFromFilename,
           py::arg("path"))
      .def("set_metallic_texture_from_file",
           &Renderer::IPxrMaterial::setMetallicTextureFromFilename, py::arg("path"))
      .def("set_roughness_texture_from_file",
           &Renderer::IPxrMaterial::setRoughnessTextureFromFilename, py::arg("path"))
      .def("set_transmission_texture_from_file",
           &Renderer::IPxrMaterial::setTransmissionTextureFromFilename, py::arg("path"))
      .def("set_normal_texture_from_file", &Renderer::IPxrMaterial::setNormalTextureFromFilename,
           py::arg("path"))

      .def_property("base_color", &Renderer::IPxrMaterial::getBaseColor,
                    [](Renderer::IPxrMaterial &mat, py::array_t<float> color) {
                      mat.setBaseColor({color.at(0), color.at(1), color.at(2), color.at(3)});
                    })
      .def_property("emission", &Renderer::IPxrMaterial::getEmission,
                    [](Renderer::IPxrMaterial &mat, py::array_t<float> color) {
                      mat.setEmission({color.at(0), color.at(1), color.at(2), color.at(3)});
                    })
      .def_property("specular", &Renderer::IPxrMaterial::getSpecular,
                    &Renderer::IPxrMaterial::setSpecular)
      .def_property("metallic", &Renderer::IPxrMaterial::getMetallic,
                    &Renderer::IPxrMaterial::setMetallic)
      .def_property("roughness", &Renderer::IPxrMaterial::getRoughness,
                    &Renderer::IPxrMaterial::setRoughness)
      .def_property("transmission", &Renderer::IPxrMaterial::getTransmission,
                    &Renderer::IPxrMaterial::setTransmission)
      .def_property("ior", &Renderer::IPxrMaterial::getIOR, &Renderer::IPxrMaterial::setIOR)

      .def_property("emission_texture", &Renderer::IPxrMaterial::getEmissionTexture,
                    &Renderer::IPxrMaterial::setDiffuseTexture)
      .def_property("diffuse_texture", &Renderer::IPxrMaterial::getDiffuseTexture,
                    &Renderer::IPxrMaterial::setDiffuseTexture)
      .def_property("metallic_texture", &Renderer::IPxrMaterial::getMetallicTexture,
                    &Renderer::IPxrMaterial::setMetallicTexture)
      .def_property("roughness_texture", &Renderer::IPxrMaterial::getRoughnessTexture,
                    &Renderer::IPxrMaterial::setRoughnessTexture)
      .def_property("normal_texture", &Renderer::IPxrMaterial::getNormalTexture,
                    &Renderer::IPxrMaterial::setDiffuseTexture)
      .def_property("transmission_texture", &Renderer::IPxrMaterial::getTransmissionTexture,
                    &Renderer::IPxrMaterial::setTransmissionTexture)

      .def_property("emission_texture_filename",
                    &Renderer::IPxrMaterial::getEmissionTextureFilename,
                    &Renderer::IPxrMaterial::setDiffuseTextureFromFilename)
      .def_property("diffuse_texture_filename", &Renderer::IPxrMaterial::getDiffuseTextureFilename,
                    &Renderer::IPxrMaterial::setDiffuseTextureFromFilename)
      .def_property("metallic_texture_filename",
                    &Renderer::IPxrMaterial::getMetallicTextureFilename,
                    &Renderer::IPxrMaterial::setMetallicTextureFromFilename)
      .def_property("roughness_texture_filename",
                    &Renderer::IPxrMaterial::getRoughnessTextureFilename,
                    &Renderer::IPxrMaterial::setRoughnessTextureFromFilename)
      .def_property("normal_texture_filename", &Renderer::IPxrMaterial::getNormalTextureFilename,
                    &Renderer::IPxrMaterial::setDiffuseTextureFromFilename)
      .def_property("transmission_texture_filename",
                    &Renderer::IPxrMaterial::getTransmissionTextureFilename,
                    &Renderer::IPxrMaterial::setTransmissionTextureFromFilename);

  //     // TODO: implement those together with UV
  //     // .def_readwrite("specular_texture", &Renderer::PxrMaterial::specular_texture)
  //     // .def_readwrite("normal_texture", &Renderer::PxrMaterial::normal_texture);

  PyRenderTexture.def_property_readonly("width", &Renderer::IPxrTexture::getWidth)
      .def_property_readonly("height", &Renderer::IPxrTexture::getHeight)
      .def_property_readonly("channels", &Renderer::IPxrTexture::getChannels)
      .def_property_readonly("mipmap_levels", &Renderer::IPxrTexture::getMipmapLevels)
      .def_property_readonly("dtype",
                             [](Renderer::IPxrTexture &tex) {
                               py::module np = py::module::import("numpy");
                               switch (tex.getType()) {
                               case Renderer::IPxrTexture::Type::eBYTE:
                                 return np.attr("uint8");
                               case Renderer::IPxrTexture::Type::eINT:
                                 return np.attr("int32");
                               case Renderer::IPxrTexture::Type::eHALF:
                                 return np.attr("float16");
                               case Renderer::IPxrTexture::Type::eFLOAT:
                                 return np.attr("float32");
                               case Renderer::IPxrTexture::Type::eOTHER:
                                 return np.attr("object");
                               }
                               return np.attr("object");
                             })
      .def_property_readonly("address_mode",
                             [](Renderer::IPxrTexture &tex) {
                               switch (tex.getAddressMode()) {
                               case Renderer::IPxrTexture::AddressMode::eREPEAT:
                                 return "repeat";
                               case Renderer::IPxrTexture::AddressMode::eBORDER:
                                 return "border";
                               case Renderer::IPxrTexture::AddressMode::eEDGE:
                                 return "edge";
                               case Renderer::IPxrTexture::AddressMode::eMIRROR:
                                 return "mirror";
                               }
                               return "unknown";
                             })
      .def_property_readonly("filter_mode",
                             [](Renderer::IPxrTexture &tex) {
                               switch (tex.getFilterMode()) {
                               case Renderer::IPxrTexture::FilterMode::eLINEAR:
                                 return "linear";
                               case Renderer::IPxrTexture::FilterMode::eNEAREST:
                                 return "nearest";
                               }
                               return "unknown";
                             })
      .def_property_readonly("filename", &Renderer::IPxrTexture::getFilename);

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
      .def_readwrite("enable_adaptive_force", &SceneConfig::enableAdaptiveForce)
      .def_readwrite("disable_collision_visual", &SceneConfig::disableCollisionVisual)
      .def("__repr__", [](SceneConfig &) { return "SceneConfig()"; });

  //======== Simulation ========//
  PyEngine
      .def(py::init([](uint32_t nthread, PxReal toleranceLength, PxReal toleranceSpeed) {
             return Simulation::getInstance(nthread, toleranceLength, toleranceSpeed);
           }),
           py::arg("thread_count") = 0, py::arg("tolerance_length") = 0.1f,
           py::arg("tolerance_speed") = 0.2f)
      .def("create_scene", &Simulation::createScene, py::arg("config") = SceneConfig())
      .def_property("renderer", &Simulation::getRenderer, &Simulation::setRenderer)
      .def("get_renderer", &Simulation::getRenderer)
      .def("set_renderer", &Simulation::setRenderer, py::arg("renderer"))
      .def("set_log_level", &Simulation::setLogLevel, py::arg("level"))
      .def("create_physical_material", &Simulation::createPhysicalMaterial,
           py::arg("static_friction"), py::arg("dynamic_friction"), py::arg("restitution"));

  PyScene.def_property_readonly("_ptr", [](SScene &s) { return (void *)&s; })
      .def_property_readonly("name", &SScene::getName)
      .def_property_readonly("engine", &SScene::getSimulation)
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
      .def("remove_drive", &SScene::removeDrive, py::arg("drive"))
      .def("find_actor_by_id", &SScene::findActorById, py::arg("id"),
           py::return_value_policy::reference)
      .def("find_articulation_link_by_link_id", &SScene::findArticulationLinkById, py::arg("id"),
           py::return_value_policy::reference)

      .def("add_camera", &SScene::addCamera, py::arg("name"), py::arg("width"), py::arg("height"),
           py::arg("fovy"), py::arg("near"), py::arg("far"), py::return_value_policy::reference)
      .def(
          "add_mounted_camera",
          [](SScene &scene, std::string const &name, SActorBase *actor, PxTransform const &pose,
             uint32_t width, uint32_t height, float fovy, float near, float far) {
            auto cam = scene.addCamera(name, width, height, fovy, near, far);
            cam->setParent(actor);
            cam->setLocalPose(pose);
            return cam;
          },
          py::arg("name"), py::arg("actor"), py::arg("pose"), py::arg("width"), py::arg("height"),
          py::arg("fovy"), py::arg("near"), py::arg("far"), py::return_value_policy::reference)
      .def(
          "add_mounted_camera",
          [](SScene &scene, std::string const &name, SActorBase *actor, PxTransform const &pose,
             uint32_t width, uint32_t height, float fovx, float fovy, float near, float far) {
            // spdlog::get("SAPIEN")->warn(
            //     "add_mounted_camera with fovx has been deprecated and will be "
            //     "removed in the next release.");
            auto cam = scene.addCamera(name, width, height, fovy, near, far);
            cam->setParent(actor);
            cam->setLocalPose(pose);
            return cam;
          },
          py::arg("name"), py::arg("actor"), py::arg("pose"), py::arg("width"), py::arg("height"),
          py::arg("fovx"), py::arg("fovy"), py::arg("near"), py::arg("far"),
          py::return_value_policy::reference)
      .def("get_cameras", &SScene::getCameras, py::return_value_policy::reference)
      .def(
          "get_mounted_cameras",
          [](SScene &scene) {
            // spdlog::get("SAPIEN")->warn("get_mounted_cameras has been deprecated and will be "
            //                             "removed in the next release, "
            //                             "please use equivalent function get_cameras instead.");
            return scene.getCameras();
          },
          py::return_value_policy::reference)
      .def("remove_camera", &SScene::removeCamera, py::arg("camera"))

      // .def("remove_mounted_camera", &SScene::removeMountedCamera, py::arg("camera"))
      // .def("get_mounted_actors", &SScene::getMountedActors,
      // py::return_value_policy::reference) .def("find_mounted_camera",
      // &SScene::findMountedCamera, py::arg("name"), py::arg("actor") = nullptr,
      // py::return_value_policy::reference)

      .def("step", &SScene::step)
      .def("step_async",
           [](SScene &scene) {
             return std::static_pointer_cast<IAwaitable<void>>(
                 std::make_shared<AwaitableFuture<void>>(scene.stepAsync()));
           })
      .def("multistep_async",
           [](SScene &scene, int steps, void *callback) {
             return std::static_pointer_cast<IAwaitable<void>>(
                 std::make_shared<AwaitableFuture<void>>(
                     scene.multistepAsync(steps, (SceneMultistepCallback *)callback)));
           })
      .def("update_render", &SScene::updateRender)
      .def("_update_render_and_take_pictures", &SScene::updateRenderAndTakePictures)
      .def("update_render_async",
           [](SScene &scene) {
             return std::static_pointer_cast<IAwaitable<void>>(
                 std::make_shared<AwaitableFuture<void>>(scene.updateRenderAsync()));
           })
      .def(
          "add_ground",
          [](SScene &s, float altitude, bool render, std::shared_ptr<SPhysicalMaterial> material,
             std::shared_ptr<Renderer::IPxrMaterial> renderMaterial,
             py::array_t<float> const &renderSize) {
            s.addGround(altitude, render, material, renderMaterial, {renderSize.at(0), renderSize.at(1)});
          },
          py::arg("altitude"), py::arg("render") = true, py::arg("material") = nullptr,
          py::arg("render_material") = nullptr,
          py::arg("render_half_size") = make_array<float>({10.f, 10.f}),
          py::return_value_policy::reference)
      .def("get_contacts", &SScene::getContacts, py::return_value_policy::reference)
      .def("get_all_actors", &SScene::getAllActors, py::return_value_policy::reference)
      .def("get_all_articulations", &SScene::getAllArticulations,
           py::return_value_policy::reference)
      .def("get_all_lights", &SScene::getAllLights, py::return_value_policy::reference)
      // drive, constrains, and joints
      .def("create_drive", &SScene::createDrive, py::arg("actor1"), py::arg("pose1"),
           py::arg("actor2"), py::arg("pose2"), py::return_value_policy::reference)
      .def("create_gear", &SScene::createGear, py::arg("actor1"), py::arg("pose1"),
           py::arg("actor2"), py::arg("pose2"), py::return_value_policy::reference)
      .def_property_readonly("render_id_to_visual_name", &SScene::findRenderId2VisualName)

      // renderer
      .def_property_readonly("renderer_scene", &SScene::getRendererScene,
                             py::return_value_policy::reference)
      .def("get_renderer_scene", &SScene::getRendererScene, py::return_value_policy::reference)
      .def("generate_unique_render_id", &SScene::generateUniqueRenderId)
      // lights
      .def_property_readonly("ambient_light",
                             [](SScene &scene) {
                               auto light = scene.getAmbientLight();
                               return make_array(std::vector<float>{light[0], light[1], light[2]});
                             })
      .def(
          "set_ambient_light",
          [](SScene &scene, py::array_t<float> const &color) {
            scene.setAmbientLight({color.at(0), color.at(1), color.at(2)});
          },
          py::arg("color"))
      .def(
          "add_point_light",
          [](SScene &scene, py::array_t<float> const &position, py::array_t<float> const &color,
             bool shadow, float near, float far, uint32_t shadowMapSize) {
            return scene.addPointLight({position.at(0), position.at(1), position.at(2)},
                                       {color.at(0), color.at(1), color.at(2)}, shadow, near, far,
                                       shadowMapSize);
          },
          py::arg("position"), py::arg("color"), py::arg("shadow") = false, py::arg("near") = 0.1,
          py::arg("far") = 10, py::arg("shadow_map_size") = 2048,
          py::return_value_policy::reference)
      .def(
          "add_directional_light",
          [](SScene &scene, py::array_t<float> const &direction, py::array_t<float> const &color,
             bool shadow, py::array_t<float> const &position, float scale, float near, float far,
             uint32_t shadowMapSize) {
            return scene.addDirectionalLight({direction.at(0), direction.at(1), direction.at(2)},
                                             {color.at(0), color.at(1), color.at(2)}, shadow,
                                             {position.at(0), position.at(1), position.at(2)},
                                             scale, near, far, shadowMapSize);
          },
          py::arg("direction"), py::arg("color"), py::arg("shadow") = false,
          py::arg("position") = make_array<float>({0.f, 0.f, 0.f}), py::arg("scale") = 10.f,
          py::arg("near") = -10.f, py::arg("far") = 10.f, py::arg("shadow_map_size") = 2048,
          py::return_value_policy::reference)
      .def(
          "add_spot_light",
          [](SScene &scene, py::array_t<float> const &position,
             py::array_t<float> const &direction, float fovInner, float fovOuter,
             py::array_t<float> const &color, bool shadow, float near, float far,
             uint32_t shadowMapSize) {
            return scene.addSpotLight({position.at(0), position.at(1), position.at(2)},
                                      {direction.at(0), direction.at(1), direction.at(2)},
                                      fovInner, fovOuter, {color.at(0), color.at(1), color.at(2)},
                                      shadow, near, far, shadowMapSize);
          },
          py::arg("position"), py::arg("direction"), py::arg("inner_fov"), py::arg("outer_fov"),
          py::arg("color"), py::arg("shadow") = false, py::arg("near") = 0.1f,
          py::arg("far") = 10.f, py::arg("shadow_map_size") = 2048,
          py::return_value_policy::reference)
      .def(
          "add_active_light",
          [](SScene &scene, PxTransform const &pose, py::array_t<float> const &color, float fov,
             std::string const &path, float shadowNear, float shadowFar, uint32_t shadowMapSize) {
            return scene.addActiveLight(pose, {color.at(0), color.at(1), color.at(2)}, fov, path,
                                        shadowNear, shadowFar, shadowMapSize);
          },
          py::arg("pose"), py::arg("color"), py::arg("fov"), py::arg("tex_path"),
          py::arg("near") = 0.1f, py::arg("far") = 10.f, py::arg("shadow_map_size") = 2048,
          py::return_value_policy::reference)
      .def("remove_light", &SScene::removeLight, py::arg("light"))
      .def("set_environment_map", &SScene::setEnvironmentMap, py::arg("filename"))
      .def("set_environment_map_from_files", &SScene::setEnvironmentMapFromFiles, py::arg("px"),
           py::arg("nx"), py::arg("py"), py::arg("ny"), py::arg("pz"), py::arg("nz"))
      .def("add_particle_entity", &SScene::addParticleEntity, py::arg("positions"),
           py::return_value_policy::reference)
      .def("remove_particle_entity", &SScene::removeParticleEntity, py::arg("entity"))

      // save
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
  PyDrive.def("set_x_limit", &SDrive6D::setXLimit, py::arg("low"), py::arg("high"))
      .def("set_y_limit", &SDrive6D::setYLimit, py::arg("low"), py::arg("high"))
      .def("set_z_limit", &SDrive6D::setZLimit, py::arg("low"), py::arg("high"))
      .def("set_x_twist_limit", &SDrive6D::setXTwistLimit, py::arg("low"), py::arg("high"))
      .def("set_yz_cone_limit", &SDrive6D::setYZConeLimit, py::arg("y"), py::arg("z"))
      .def("set_yz_pyramid_limit", &SDrive6D::setYZPyramidLimit, py::arg("ylow"), py::arg("yhigh"),
           py::arg("zlow"), py::arg("zhigh"))
      .def("set_distance_limit", &SDrive6D::setDistanceLimit, py::arg("distance"))
      .def("set_x_twist_properties", &SDrive6D::setXTwistDriveProperties, py::arg("stiffness"),
           py::arg("damping"), py::arg("force_limit") = PX_MAX_F32,
           py::arg("is_acceleration") = true)
      .def("set_yz_swing_properties", &SDrive6D::setYZSwingDriveProperties, py::arg("stiffness"),
           py::arg("damping"), py::arg("force_limit") = PX_MAX_F32,
           py::arg("is_acceleration") = true)
      .def("set_slerp_properties", &SDrive6D::setSlerpProperties, py::arg("stiffness"),
           py::arg("damping"), py::arg("force_limit") = PX_MAX_F32,
           py::arg("is_acceleration") = true)
      .def("set_x_properties", &SDrive6D::setXProperties, py::arg("stiffness"), py::arg("damping"),
           py::arg("force_limit") = PX_MAX_F32, py::arg("is_acceleration") = true)
      .def("set_y_properties", &SDrive6D::setYProperties, py::arg("stiffness"), py::arg("damping"),
           py::arg("force_limit") = PX_MAX_F32, py::arg("is_acceleration") = true)
      .def("set_z_properties", &SDrive6D::setZProperties, py::arg("stiffness"), py::arg("damping"),
           py::arg("force_limit") = PX_MAX_F32, py::arg("is_acceleration") = true)
      .def("lock_motion", &SDrive6D::lockMotion, py::arg("tx"), py::arg("ty"), py::arg("tz"),
           py::arg("rx"), py::arg("ry"), py::arg("rz"))
      .def("free_motion", &SDrive6D::freeMotion, py::arg("tx"), py::arg("ty"), py::arg("tz"),
           py::arg("rx"), py::arg("ry"), py::arg("rz"))
      .def("set_target", &SDrive6D::setTarget, py::arg("pose"))
      .def(
          "set_target_velocity",
          [](SDrive6D &d, py::array_t<PxReal> const &linear, py::array_t<PxReal> const &angular) {
            d.setTargetVelocity(array2vec3(linear), array2vec3(angular));
          },
          py::arg("linear"), py::arg("angular"));

  PyGear.def_property("ratio", &SGear::getRatio, &SGear::setRatio);

  PyEntity.def_property_readonly("_ptr", [](SEntity &e) { return (void *)&e; })
      .def_property("name", &SEntity::getName, &SEntity::setName)
      .def("get_name", &SEntity::getName)
      .def("set_name", &SEntity::setName, py::arg("name"))
      .def_property_readonly("pose", &SEntity::getPose)
      .def("get_pose", &SEntity::getPose);

  //======== Actor ========//
  PyActorBase
      .def("__repr__",
           [](SActorBase &actor) {
             std::ostringstream oss;
             oss << "Actor(name=\"" << actor.getName() << "\", id=\"" << actor.getId() << "\")";
             return oss.str();
           })
      .def_property_readonly(
          "type",
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
          },
          "One of \"static\", \"kinematic\", \"dynamic\", \"link\", \"kinematic_link\"")

      .def_property_readonly("id", &SActorBase::getId)
      .def("get_id", &SActorBase::getId)
      .def("get_scene", &SActorBase::getScene, py::return_value_policy::reference)
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
      .def("on_trigger", &SActorBase::onTrigger, py::arg("func"))
      .def("get_builder", &SActorBase::getBuilder);

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
      .def("set_damping", &SActorDynamicBase::setDamping, py::arg("linear"), py::arg("angular"))
      .def("set_ccd", &SActorDynamicBase::setCCDEnabled, py::arg("enable"))
      .def_property("ccd", &SActorDynamicBase::getCCDEnabled, &SActorDynamicBase::setCCDEnabled);

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
      .def("get_pose_in_parent", &SJointBase::getParentPose)
      .def("get_pose_in_child", &SJointBase::getChildPose)
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

  PyArticulationBase
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
      .def("get_root_pose", &SArticulationBase::getRootPose)
      .def("set_root_pose", &SArticulationBase::setRootPose, py::arg("pose"))
      .def("set_pose", &SArticulationBase::setRootPose, py::arg("pose"), "same as set_root_pose")
      .def("create_pinocchio_model", &SArticulationBase::createPinocchioModel,
           "Create the kinematic and dynamic model of this articulation implemented by the "
           "Pinocchio library. Allowing computing forward/inverse kinematics/dynamics.")
      .def("export_urdf", &SArticulationBase::exportURDF, py::arg("cache_dir") = std::string())
      .def("get_builder", &SArticulationBase::getBuilder);

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

  PyArticulation.def_property_readonly("fixed", &SArticulation::isBaseFixed)
      .def("get_drive_velocity_target",
           [](SArticulation &a) {
             auto target = a.getDriveVelocityTarget();
             return py::array_t<PxReal>(target.size(), target.data());
           })
      .def(
          "set_drive_velocity_target",
          [](SArticulation &a, const py::array_t<PxReal> &arr) {
            a.setDriveVelocityTarget(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
          },
          py::arg("drive_velocity_target"))

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

      .def(
          "compute_generalized_external_force",
          [](SArticulation &a, Eigen::Matrix<PxReal, Eigen::Dynamic, 3, Eigen::RowMajor> f,
             Eigen::Matrix<PxReal, Eigen::Dynamic, 3, Eigen::RowMajor> t) {
            std::vector<physx::PxVec3> force;
            std::vector<physx::PxVec3> torque;
            for (uint32_t i = 0; i < f.rows(); ++i) {
              force.push_back(physx::PxVec3{f(i, 0), f(i, 1), f(i, 2)});
            }
            for (uint32_t i = 0; i < t.rows(); ++i) {
              torque.push_back(physx::PxVec3{t(i, 0), t(i, 1), t(i, 2)});
            }
            auto qf = a.computeGeneralizedExternalForce(force, torque);
            return py::array_t<PxReal>(qf.size(), qf.data());
          },
          py::arg("forces"), py::arg("torques"))

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
          "actor0", [](SContact &contact) { return contact.actors[0]; },
          py::return_value_policy::reference)
      .def_property_readonly(
          "actor1", [](SContact &contact) { return contact.actors[1]; },
          py::return_value_policy::reference)
      .def_property_readonly(
          "collision_shape0", [](SContact &contact) { return contact.collisionShapes[0]; },
          py::return_value_policy::reference)
      .def_property_readonly(
          "collision_shape1", [](SContact &contact) { return contact.collisionShapes[1]; },
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

  PyActorBuilder.def("set_scene", &ActorBuilder::setScene)
      .def(
          "add_collision_from_file",
          [](ActorBuilder &a, std::string const &filename, PxTransform const &pose,
             py::array_t<PxReal> const &scale, std::shared_ptr<SPhysicalMaterial> material,
             PxReal density, PxReal patchRadius, PxReal minPatchRadius, bool isTrigger) {
            return a.addConvexShapeFromFile(filename, pose, array2vec3(scale), material, density,
                                            patchRadius, minPatchRadius, isTrigger);
          },
          R"doc(
Add a collision shape from file (see assimp for supported formats).
If the shape in the file is not convex, it will be converted by the PhysX backend.)doc",
          py::arg("filename"), py::arg("pose") = PxTransform(PxIdentity),
          py::arg("scale") = make_array<PxReal>({1, 1, 1}), py::arg("material") = nullptr,
          py::arg("density") = 1000, py::arg("patch_radius") = 0.f,
          py::arg("min_patch_radius") = 0.f, py::arg("is_trigger") = false)
      .def(
          "add_nonconvex_collision_from_file",
          [](ActorBuilder &a, std::string const &filename, PxTransform const &pose,
             py::array_t<PxReal> const &scale, std::shared_ptr<SPhysicalMaterial> material,
             PxReal patchRadius, PxReal minPatchRadius, bool isTrigger) {
            return a.addNonConvexShapeFromFile(filename, pose, array2vec3(scale), material,
                                               patchRadius, minPatchRadius, isTrigger);
          },
          R"doc(Add a nonconvex collision shape from a file. If it is not a trigger, then it is only valid for static and kinematic actors.)doc",
          py::arg("filename"), py::arg("pose") = PxTransform(PxIdentity),
          py::arg("scale") = make_array<PxReal>({1, 1, 1}), py::arg("material") = nullptr,
          py::arg("patch_radius") = 0.f, py::arg("min_patch_radius") = 0.f,
          py::arg("is_trigger") = false)
      .def(
          "add_multiple_collisions_from_file",
          [](ActorBuilder &a, std::string const &filename, PxTransform const &pose,
             py::array_t<PxReal> const &scale, std::shared_ptr<SPhysicalMaterial> material,
             PxReal density, PxReal patchRadius, PxReal minPatchRadius, bool isTrigger) {
            return a.addMultipleConvexShapesFromFile(filename, pose, array2vec3(scale), material,
                                                     density, patchRadius, minPatchRadius,
                                                     isTrigger);
          },
          R"doc(
Add multiple collisions shapes from files. Also see @add_collision_from_file.
Different from @add_collision_from_file, all connected components in the file will be converted to be convex.)doc",
          py::arg("filename"), py::arg("pose") = PxTransform(PxIdentity),
          py::arg("scale") = make_array<PxReal>({1, 1, 1}), py::arg("material") = nullptr,
          py::arg("density") = 1000, py::arg("patch_radius") = 0.f,
          py::arg("min_patch_radius") = 0.f, py::arg("is_trigger") = false)
      .def(
          "add_box_collision",
          [](ActorBuilder &a, PxTransform const &pose, py::array_t<PxReal> const &halfSize,
             std::shared_ptr<SPhysicalMaterial> material, PxReal density, PxReal patchRadius,
             PxReal minPatchRadius, bool isTrigger) {
            return a.addBoxShape(pose, array2vec3(halfSize), material, density, patchRadius,
                                 minPatchRadius, isTrigger);
          },
          py::arg("pose") = PxTransform(PxIdentity),
          py::arg("half_size") = make_array<PxReal>({1, 1, 1}), py::arg("material") = nullptr,
          py::arg("density") = 1000, py::arg("patch_radius") = 0.f,
          py::arg("min_patch_radius") = 0.f, py::arg("is_trigger") = false)
      .def("add_capsule_collision", &ActorBuilder::addCapsuleShape,
           "Add a capsule collision shape. The height is along the x-axis.",
           py::arg("pose") = PxTransform(PxIdentity), py::arg("radius") = 1,
           py::arg("half_length") = 1, py::arg("material") = nullptr, py::arg("density") = 1000,
           py::arg("patch_radius") = 0.f, py::arg("min_patch_radius") = 0.f,
           py::arg("is_trigger") = false)
      .def("add_sphere_collision", &ActorBuilder::addSphereShape,
           py::arg("pose") = PxTransform(PxIdentity), py::arg("radius") = 1,
           py::arg("material") = nullptr, py::arg("density") = 1000, py::arg("patch_radius") = 0.f,
           py::arg("min_patch_radius") = 0.f, py::arg("is_trigger") = false)
      .def(
          "add_box_visual",
          [](ActorBuilder &a, PxTransform const &pose, py::array_t<PxReal> const &halfSize,
             py::array_t<PxReal> color, std::string const &name) {
            return a.addBoxVisual(pose, array2vec3(halfSize), array2vec3(color), name);
          },
          py::arg("pose") = PxTransform(PxIdentity),
          py::arg("half_size") = make_array<PxReal>({1, 1, 1}),
          py::arg("color") = make_array<PxReal>({1, 1, 1}), py::arg("name") = "")
      .def(
          "add_box_visual",
          [](ActorBuilder &a, PxTransform const &pose, py::array_t<PxReal> const &halfSize,
             std::shared_ptr<Renderer::IPxrMaterial> &mat, std::string const &name) {
            return a.addBoxVisualWithMaterial(pose, array2vec3(halfSize), mat, name);
          },
          py::arg("pose") = PxTransform(PxIdentity),
          py::arg("half_size") = make_array<PxReal>({1, 1, 1}), py::arg("material") = nullptr,
          py::arg("name") = "")
      .def(
          "add_capsule_visual",
          [](ActorBuilder &a, PxTransform const &pose, PxReal radius, PxReal halfLength,
             py::array_t<PxReal> color, std::string const &name) {
            return a.addCapsuleVisual(pose, radius, halfLength, array2vec3(color), name);
          },
          "Add a capsule visual shape. The height is along the x-axis.",
          py::arg("pose") = PxTransform(PxIdentity), py::arg("radius") = 1,
          py::arg("half_length") = 1, py::arg("color") = make_array<PxReal>({1, 1, 1}),
          py::arg("name") = "")
      .def(
          "add_capsule_visual",
          [](ActorBuilder &a, PxTransform const &pose, PxReal radius, PxReal halfLength,
             std::shared_ptr<Renderer::IPxrMaterial> &mat, std::string const &name) {
            return a.addCapsuleVisualWithMaterial(pose, radius, halfLength, mat, name);
          },
          "Add a capsule visual shape. The height is along the x-axis.",
          py::arg("pose") = PxTransform(PxIdentity), py::arg("radius") = 1,
          py::arg("half_length") = 1, py::arg("material") = nullptr, py::arg("name") = "")
      .def(
          "add_sphere_visual",
          [](ActorBuilder &a, PxTransform const &pose, PxReal radius, py::array_t<PxReal> color,
             std::string const &name) {
            return a.addSphereVisual(pose, radius, array2vec3(color), name);
          },
          py::arg("pose") = PxTransform(PxIdentity), py::arg("radius") = 1,
          py::arg("color") = make_array<PxReal>({1, 1, 1}), py::arg("name") = "")
      .def(
          "add_sphere_visual",
          [](ActorBuilder &a, PxTransform const &pose, PxReal radius,
             std::shared_ptr<Renderer::IPxrMaterial> &mat,
             std::string const &name) { a.addSphereVisualWithMaterial(pose, radius, mat, name); },
          py::arg("pose") = PxTransform(PxIdentity), py::arg("radius") = 1,
          py::arg("material") = nullptr, py::arg("name") = "")
      .def(
          "add_visual_from_file",
          [](ActorBuilder &a, std::string const &filename, PxTransform const &pose,
             py::array_t<PxReal> scale, std::shared_ptr<Renderer::IPxrMaterial> &mat,
             std::string const &name) {
            return a.addVisualFromFile(filename, pose, array2vec3(scale), mat, name);
          },
          py::arg("filename"), py::arg("pose") = PxTransform(PxIdentity),
          py::arg("scale") = make_array<PxReal>({1, 1, 1}), py::arg("material") = nullptr,
          py::arg("name") = "")
      .def(
          "add_visual_from_mesh",
          [](ActorBuilder &a, std::shared_ptr<Renderer::IRenderMesh> mesh, PxTransform const &pose,
             py::array_t<PxReal> scale, std::shared_ptr<Renderer::IPxrMaterial> &mat,
             std::string const &name) {
            return a.addVisualFromMeshWithMaterial(mesh, pose, array2vec3(scale), mat, name);
          },
          py::arg("mesh"), py::arg("pose") = PxTransform(PxIdentity),
          py::arg("scale") = make_array<PxReal>({1, 1, 1}), py::arg("material") = nullptr,
          py::arg("name") = "")

      .def("remove_all_collisions", &ActorBuilder::removeAllShapes)
      .def("remove_all_visuals", &ActorBuilder::removeAllVisuals)
      .def("remove_collision_at", &ActorBuilder::removeShapeAt, py::arg("index"))
      .def("remove_visual_at", &ActorBuilder::removeVisualAt, py::arg("index"))
      .def("get_collisions", &ActorBuilder::getShapes, py::return_value_policy::reference)
      .def("get_visuals", &ActorBuilder::getVisuals, py::return_value_policy::reference)
      .def(
          "set_mass_and_inertia",
          [](ActorBuilder &a, PxReal mass, PxTransform const &cMassPose,
             py::array_t<PxReal> inertia) {
            return a.setMassAndInertia(mass, cMassPose, array2vec3(inertia));
          },
          R"doc(
Set the mass and inertia.

Args:
  mass: the (scalar) mass of the actor
  inertia_pose: 
    the position is the center of mass;
    the rotation (quaternion) is the principle axis of inertia, relative to actor frame
  inertia: principle moments of inertia (a 3D vector)

References:
  https://en.wikipedia.org/wiki/Moment_of_inertia#Principal_axes
)doc",
          py::arg("mass"), py::arg("inertia_pose"), py::arg("inertia"))
      .def("set_collision_groups", &ActorBuilder::setCollisionGroup,
           "see CollisionShape.set_collision_groups", py::arg("group0"), py::arg("group1"),
           py::arg("group2"), py::arg("group3"))
      .def("reset_collision_groups", &ActorBuilder::resetCollisionGroup)
      .def(
          "build", [](ActorBuilder &a, std::string const &name) { return a.build(false, name); },
          py::arg("name") = "", py::return_value_policy::reference)
      .def(
          "build_kinematic",
          [](ActorBuilder &a, std::string const &name) { return a.build(true, name); },
          py::arg("name") = "", py::return_value_policy::reference)
      .def("build_static", &ActorBuilder::buildStatic, py::return_value_policy::reference,
           py::arg("name") = "");

  PyShapeRecord.def_readonly("filename", &ActorBuilder::ShapeRecord::filename)
      .def_property_readonly("type",
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
                               case sapien::ActorBuilder::ShapeRecord::NonConvexMesh:
                                 return "Nonconvex";
                               }
                               return "";
                             })
      .def_property_readonly(
          "scale", [](ActorBuilder::ShapeRecord const &r) { return vec32array(r.scale); })
      .def_readonly("radius", &ActorBuilder::ShapeRecord::radius)
      .def_readonly("length", &ActorBuilder::ShapeRecord::length)
      .def_readonly("pose", &ActorBuilder::ShapeRecord::pose)
      .def_readonly("density", &ActorBuilder::ShapeRecord::density)
      .def_readonly("material", &ActorBuilder::ShapeRecord::material,
                    py::return_value_policy::reference);

  PyVisualRecord.def_readonly("filename", &ActorBuilder::VisualRecord::filename)
      .def_property_readonly("type",
                             [](ActorBuilder::VisualRecord const &r) {
                               switch (r.type) {
                               case sapien::ActorBuilder::VisualRecord::File:
                                 return "File";
                               case sapien::ActorBuilder::VisualRecord::Box:
                                 return "Box";
                               case sapien::ActorBuilder::VisualRecord::Capsule:
                                 return "Capsule";
                               case sapien::ActorBuilder::VisualRecord::Sphere:
                                 return "Sphere";
                               case sapien::ActorBuilder::VisualRecord::Mesh:
                                 return "Mesh";
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
          R"doc(
Set the properties of the joint.

Args:
  joint_type: ["revolute", "prismatic", "fixed"]
  limits: [[min1, max1], ...], the length is the DoF
  pose_in_parent: joint pose in parent frame. 
    The x-axis is the rotation axis for revolute, or the translation axis for prismatic.
  pose_in_child: joint pose in child frame. See also @pose_in_parent.
  friction: joint friction
  damping: joint damping
)doc",
          py::arg("joint_type"), py::arg("limits"),
          py::arg("pose_in_parent") = PxTransform(PxIdentity),
          py::arg("pose_in_child") = PxTransform(PxIdentity), py::arg("friction") = 0,
          py::arg("damping") = 0)
      .def("get_joint", &LinkBuilder::getJoint, py::return_value_policy::reference);

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
      .def_readonly("pose_in_parent", &LinkBuilder::JointRecord::parentPose)
      .def_readonly("pose_in_child", &LinkBuilder::JointRecord::childPose)
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
          [](ArticulationBuilder &b, std::shared_ptr<LinkBuilder> parent) {
            return b.createLinkBuilder(parent);
          },
          py::arg("parent") = nullptr, py::return_value_policy::reference)
      .def("build", &ArticulationBuilder::build, py::arg("fix_root_link") = false,
           py::return_value_policy::reference)
      .def("build_kinematic", &ArticulationBuilder::buildKinematic,
           py::return_value_policy::reference)
      .def("get_link_builders", &ArticulationBuilder::getLinkBuilders,
           py::return_value_policy::reference);

  PyURDFLoader.def(py::init<SScene *>(), py::arg("scene"))
      .def_readwrite("fix_root_link", &URDF::URDFLoader::fixRootLink)
      .def_readwrite("load_multiple_collisions_from_file",
                     &URDF::URDFLoader::multipleMeshesInOneFile)
      .def_readwrite("collision_is_visual", &URDF::URDFLoader::collisionIsVisual)
      .def_readwrite("scale", &URDF::URDFLoader::scale)
      .def_readwrite("package_dir", &URDF::URDFLoader::packageDir)
      .def(
          "load",
          [](URDF::URDFLoader &loader, std::string const &filename, py::dict &dict) {
            auto config = parseURDFConfig(dict);
            return loader.load(filename, config);
          },
          R"doc(
Load articulation from URDF.
Gazebo cameras are also loaded.

Args:
  filename: path to URDF
  config: a dict to specify any on-the-fly modification of articulation
    It follows the following schema (the inner parameter overrides the outer one):
    - material: PhysicalMaterial
    - density: float
    - link: dict[str, dict]
      - ${link_name}: dict
        - material: PhysicalMaterial
        - density: float
        - patch_radius: float
        - min_patch_radius: float
        - collision: dict[int, dict]
          - ${collision_index}: dict
            - material: PhysicalMaterial
            - density: float
            - patch_radius: float
            - min_patch_radius: float
)doc",
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

  PyPinocchioModel
      .def("compute_forward_kinematics", &PinocchioModel::computeForwardKinematics,
           "Compute and cache forward kinematics. After computation, use get_link_pose to "
           "retrieve the computed pose for a specific link.",
           py::arg("qpos"))
      .def("get_link_pose", &PinocchioModel::getLinkPose,
           "Given link index, get link pose (in articulation base frame) from forward kinematics. "
           "Must be called after compute_forward_kinematics.",
           py::arg("link_index"))
      .def("compute_inverse_kinematics", &PinocchioModel::computeInverseKinematics,
           R"doc(
Compute inverse kinematics with CLIK algorithm.
Details see https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b-examples_i-inverse-kinematics.html
Args:
    link_index: index of the link
    pose: target pose of the link in articulation base frame
    initial_qpos: initial qpos to start CLIK
    active_qmask: dof sized integer array, 1 to indicate active joints and 0 for inactive joints, default to all 1s
    max_iterations: number of iterations steps
    dt: iteration step "speed"
    damp: iteration step "damping"
Returns:
    result: qpos from IK
    success: whether IK is successful
    error: se3 norm error
)doc",
           py::arg("link_index"), py::arg("pose"), py::arg("initial_qpos") = Eigen::VectorXd{},
           py::arg("active_qmask") = Eigen::VectorXi{}, py::arg("eps") = 1e-4,
           py::arg("max_iterations") = 1000, py::arg("dt") = 0.1, py::arg("damp") = 1e-6)
      .def("compute_forward_dynamics", &PinocchioModel::computeForwardDynamics, py::arg("qpos"),
           py::arg("qvel"), py::arg("qf"))
      .def("compute_inverse_dynamics", &PinocchioModel::computeInverseDynamics, py::arg("qpos"),
           py::arg("qvel"), py::arg("qacc"))
      .def("compute_generalized_mass_matrix", &PinocchioModel::computeGeneralizedMassMatrix,
           py::arg("qpos"))
      .def("compute_coriolis_matrix", &PinocchioModel::computeCoriolisMatrix, py::arg("qpos"),
           py::arg("qvel"))

      .def("compute_full_jacobian", &PinocchioModel::computeFullJacobian,
           "Compute and cache Jacobian for all links", py::arg("qpos"))
      .def("get_link_jacobian", &PinocchioModel::getLinkJacobian,
           R"doc(
Given link index, get the Jacobian. Must be called after compute_full_jacobian.

Args:
  link_index: index of the link
  local: True for world(spatial) frame; False for link(body) frame
)doc",
           py::arg("link_index"), py::arg("local") = false)
      .def("compute_single_link_local_jacobian", &PinocchioModel::computeSingleLinkLocalJacobian,
           "Compute the link(body) Jacobian for a single link. It is faster than "
           "compute_full_jacobian followed by get_link_jacobian",
           py::arg("qpos"), py::arg("link_index"));

  PyVulkanRenderer
      .def_static("set_log_level", &Renderer::SVulkan2Renderer::setLogLevel, py::arg("level"))
      .def(py::init<bool, uint32_t, uint32_t, uint32_t, std::string, std::string, bool>(),
           py::arg("offscreen_only") = false, py::arg("max_num_materials") = 5000,
           py::arg("max_num_textures") = 5000, py::arg("default_mipmap_levels") = 1,
           py::arg("device") = "", py::arg("culling") = "back",
           py::arg("do_not_load_texture") = false,
           R"doc(
Create the VulkanRenderer for rasterization-based rendering.

Args:
  offscreen_only: tell the renderer the user does not need to present onto a screen. The renderer will not try to select a GPU with present abilities.
  max_num_materials: tell the maximum number of materials that will exist at the same time. Increase this number if descriptor pool is out of memory.
  max_num_textures: specify the maximum number of textures that will exist at the same time. Increase this number if descriptor pool is out of memory.
  default_mipmap_levels: set the mip map levels for loaded textures.
  device: One the the following:
    'cuda:x' where x is a decimal number, the renderer tries to render using this cuda device. Present request is ignored
    'cuda', the renderer tries to render using a cuda-visible device. If present is requested, it will be prioritized
    'pci:x', where x is a hexadecimal number, the renderer picks the device with given PCI bus number
    '', if present is requested, first try to find cuda+present, next present only, and then turn off present. If present is turned off, first try to find cuda, next any graphics device.)doc")
      // .def_static("_set_viewer_shader_dir", &Renderer::setDefaultViewerShaderDirectory,
      //             py::arg("shader_dir"))
      // .def_static("_set_camera_shader_dir", &Renderer::setDefaultCameraShaderDirectory,
      //             py::arg("shader_dir"))
      .def(
          "create_window",
          [](std::shared_ptr<Renderer::SVulkan2Renderer> renderer, int width, int height,
             std::string const &shaderDir) {
            return new Renderer::SVulkan2Window(renderer, width, height, shaderDir);
          },
          py::arg("width") = 800, py::arg("height") = 600, py::arg("shader_dir") = "")
      .def(
          "create_ktx_environment_map",
          [](Renderer::SVulkan2Renderer &renderer, std::string px, std::string nx, std::string py,
             std::string ny, std::string pz, std::string nz, std::string out) {
            auto cubemap = renderer.getContext()->getResourceManager()->CreateCubemapFromFiles(
                {px, nx, py, ny, pz, nz}, 5);
            cubemap->load();
            cubemap->uploadToDevice();
            cubemap->exportKTX(out);
          },
          py::arg("px"), py::arg("nx"), py::arg("py"), py::arg("ny"), py::arg("pz"), py::arg("nz"),
          py::arg("out"))
      .def_property_readonly(
          "_internal_context",
          [](Renderer::SVulkan2Renderer &renderer) { return renderer.getContext().get(); },
          py::return_value_policy::reference)
      .def("clear_cached_resources",
           [](Renderer::SVulkan2Renderer &renderer) {
             renderer.getContext()->getResourceManager()->clearCachedResources();
           })
      .def(
          "_release_gpu_memory_unsafe",
          [](Renderer::SVulkan2Renderer &renderer) {
            renderer.getContext()->getResourceManager()->releaseGPUResourcesUnsafe();
          },
          "A very unsafe way to release cached gpu (but not CPU) resources. It MUST be called "
          "when no rendering is running, and all cameras and windows become invalid after "
          "calling "
          "this function.");
//       .def(
//           "set_default_texture_format",
//           [](Renderer::SVulkan2Renderer &renderer, py::dict const &dict) {
//             auto config = renderer.getDefaultRendererConfig();
//             vk::Format color1 = vk::Format::eR32Sfloat;
//             vk::Format color4 = vk::Format::eR32G32B32A32Sfloat;
//             if (dict.contains("color_format_1")) {
//               auto color1str = dict["color_format_1"].cast<std::string>();
//               if (color1str == "u4") {
//                 color1 = vk::Format::eR8Unorm;
//               } else if (color1str == "f4") {
//                 color1 = vk::Format::eR32Sfloat;
//               } else {
//                 throw std::runtime_error(color1str + " is not supported for color_format_1");
//               }
//             }
//             if (dict.contains("color_format_4")) {
//               auto color4str = dict["color_format_4"].cast<std::string>();
//               if (color4str == "4u4") {
//                 color1 = vk::Format::eR8G8B8A8Unorm;
//               } else if (color4str == "4f4") {
//                 color1 = vk::Format::eR32G32B32A32Sfloat;
//               } else {
//                 throw std::runtime_error(color4str + " is not supported for color_format_1");
//               }
//             }
//             config->colorFormat1 = color1;
//             config->colorFormat4 = color4;
//             config->depthFormat = vk::Format::eD32Sfloat; // depth format must be float32
//             if (dict.contains("texture_format")) {
//               auto formats = dict["texture_format"].cast<py::dict>();
//               for (auto kv : formats) {
//                 auto name = kv.first.cast<std::string>();
//                 auto formatstr = kv.second.cast<std::string>();
//                 if (formatstr == "u1") {
//                   config->textureFormat[name] = vk::Format::eR8Unorm;
//                 } else if (formatstr == "f4") {
//                   config->textureFormat[name] = vk::Format::eR32Sfloat;
//                 } else if (formatstr == "4u1") {
//                   config->textureFormat[name] = vk::Format::eR8G8B8A8Unorm;
//                 } else if (formatstr == "4f4") {
//                   config->textureFormat[name] = vk::Format::eR32G32B32A32Sfloat;
//                 } else {
//                   throw std::runtime_error("invalid texture format " + formatstr);
//                 }
//               }
//             }
//           },
//           py::arg("config"), R"doc(
// Set the default texture format with a config dict. The dict takes 3 keys,
// ["color_format_1", "color_format_4", "texture_format"].
// "color_format_1" determines the default texture format for single-channel color textures
//     "u1": unorm texture (using uint8 to represent 0-1 values).
//     "f4": signed float32 texture (default)
// "color_format_4" determines the default texture format for 4-channel color textures
//     "4u1": unorm rgba texture
//     "4f4" signed float32 rgba texture (default)
// "texture_format" takes an dictionary, whose keys are texture names and values are texture foramts.
// The values can be one of ["u1", "f4", "4u1", "4f4"])doc");

  PyVulkanRigidbody.def_property_readonly("_internal_objects",
                                          &Renderer::SVulkan2Rigidbody::getVisualObjects,
                                          py::return_value_policy::reference);

  PyLightEntity.def("set_pose", &SLight::setPose, py::arg("pose"))
      .def_property_readonly("pose", &SLight::getPose)
      .def(
          "set_color",
          [](SLight &light, py::array_t<float> color) {
            light.setColor({color.at(0), color.at(1), color.at(2)});
          },
          py::arg("color"))
      .def_property_readonly("color", [](SLight &light) { return vec32array(light.getColor()); })
      .def_property("shadow", &SLight::getShadowEnabled, &SLight::setShadowEnabled);

  // Light Entity
  PyPointLightEntity
      .def(
          "set_position",
          [](SPointLight &light, py::array_t<float> position) {
            light.setPosition({position.at(0), position.at(1), position.at(2)});
          },
          py::arg("position"))
      .def_property_readonly("position",
                             [](SPointLight &light) { return vec32array(light.getPosition()); })
      .def("set_shadow_parameters", &SPointLight::setShadowParameters, py ::arg("near"),
           py::arg("far"))
      .def_property_readonly("shadow_near", &SPointLight::getShadowNear)
      .def_property_readonly("shadow_far", &SPointLight::getShadowFar);

  PyDirectionalLightEntity
      .def(
          "set_direction",
          [](SDirectionalLight &light, py::array_t<float> direction) {
            light.setDirection({direction.at(0), direction.at(1), direction.at(2)});
          },
          py::arg("direction"))
      .def_property_readonly(
          "direction", [](SDirectionalLight &light) { return vec32array(light.getDirection()); })
      .def("set_shadow_parameters", &SDirectionalLight::setShadowParameters, py::arg("half_size"),
           py ::arg("near"), py::arg("far"))
      .def_property_readonly("shadow_near", &SDirectionalLight::getShadowNear)
      .def_property_readonly("shadow_far", &SDirectionalLight::getShadowFar)
      .def_property_readonly("shadow_half_size", &SDirectionalLight::getShadowHalfSize);

  PySpotLightEntity
      .def(
          "set_position",
          [](SSpotLight &light, py::array_t<float> position) {
            light.setPosition({position.at(0), position.at(1), position.at(2)});
          },
          py::arg("position"))
      .def_property_readonly("position",
                             [](SSpotLight &light) { return vec32array(light.getPosition()); })
      .def(
          "set_direction",
          [](SSpotLight &light, py::array_t<float> direction) {
            light.setDirection({direction.at(0), direction.at(1), direction.at(2)});
          },
          py::arg("direction"))
      .def_property_readonly("direction",
                             [](SSpotLight &light) { return vec32array(light.getDirection()); })
      .def("set_shadow_parameters", &SSpotLight::setShadowParameters, py ::arg("near"),
           py::arg("far"))
      .def_property_readonly("shadow_near", &SSpotLight::getShadowNear)
      .def_property_readonly("shadow_far", &SSpotLight::getShadowFar)
      .def("set_fov", &SSpotLight::setFov)
      .def_property_readonly("fov", &SSpotLight::getFov);

  PyActiveLightEntity
      .def(
          "set_position",
          [](SActiveLight &light, py::array_t<float> position) {
            light.setPosition({position.at(0), position.at(1), position.at(2)});
          },
          py::arg("position"))
      .def_property_readonly("position",
                             [](SActiveLight &light) { return vec32array(light.getPosition()); })
      .def("set_fov", &SActiveLight::setFov)
      .def_property_readonly("fov", &SActiveLight::getFov)
      .def("set_shadow_parameters", &SActiveLight::setShadowParameters, py ::arg("near"),
           py::arg("far"))
      .def_property_readonly("shadow_near", &SActiveLight::getShadowNear)
      .def_property_readonly("shadow_far", &SActiveLight::getShadowFar);

  PyParticleEntity.def_property_readonly("visual_body",
                                         [](SEntityParticle &p) { return p.getVisualBody(); });

  PyCameraEntity.def_property("parent", &SCamera::getParent, &SCamera::setParent)
      .def("set_parent", &SCamera::setParent, py::arg("parent"), py::arg("keep_pose"))
      .def("set_local_pose", &SCamera::setLocalPose, py::arg("pose"))
      .def(
          "set_pose",
          [](SCamera &c, PxTransform const &pose) {
            if (c.getParent()) {
              throw std::runtime_error(
                  "set_pose is not allowed for a mounted camera. Call set_local_pose instead.");
            }
            c.setLocalPose(pose);
          },
          py::arg("pose"))
      .def_property_readonly("local_pose", &SCamera::getLocalPose)

      .def_property_readonly("width", &SCamera::getWidth)
      .def_property_readonly("height", &SCamera::getHeight)

      .def_property("near", &SCamera::getNear, &SCamera::setNear)
      .def_property("far", &SCamera::getFar, &SCamera::setFar)

      .def_property_readonly("fovx", &SCamera::getFovX)
      .def_property_readonly("fovy", &SCamera::getFovY)
      .def("set_fovx", &SCamera::setFovX, py::arg("fov"), py::arg("compute_y") = true)
      .def("set_fovy", &SCamera::setFovY, py::arg("fov"), py::arg("compute_x") = true)

      .def_property_readonly("fx", &SCamera::getFocalLengthX)
      .def_property_readonly("fy", &SCamera::getFocalLengthY)
      .def("set_focal_lengths", &SCamera::setFocalLengths, py::arg("fx"), py::arg("fy"))

      .def_property_readonly("cx", &SCamera::getPrincipalPointX)
      .def_property_readonly("cy", &SCamera::getPrincipalPointY)
      .def("set_principal_point", &SCamera::setPrincipalPoint, py::arg("cx"), py::arg("cy"))

      .def("set_perspective_parameters", &SCamera::setPerspectiveParameters, py::arg("near"),
           py::arg("far"), py::arg("fx"), py::arg("fy"), py::arg("cx"), py::arg("cy"),
           py::arg("skew"))

      .def_property("skew", &SCamera::getSkew, &SCamera::setSkew)

      .def("take_picture", &SCamera::takePicture)
#ifdef SAPIEN_DLPACK
      .def(
          "take_picture_and_get_dl_tensors_async",
          [](SCamera &cam, std::vector<std::string> const &names) {
            return std::make_shared<AwaitableDLVectorWrapper>(
                cam.takePictureAndGetDLTensorsAsync(names));
          },
          py::arg("names"))
#endif
      .def("get_float_texture", &getFloatImageFromCamera, py::arg("texture_name"))
      .def("get_uint32_texture", &getUintImageFromCamera, py::arg("texture_name"))
      .def("get_uint8_texture", &getUint8ImageFromCamera, py::arg("texture_name"))
      .def("get_texture", &getImageFromCamera, py::arg("texture_name"))

      .def("get_color_rgba", [](SCamera &c) { return getFloatImageFromCamera(c, "Color"); })
      .def("get_position_rgba", [](SCamera &c) { return getFloatImageFromCamera(c, "Position"); })
      .def("get_albedo_rgba", [](SCamera &c) { return getFloatImageFromCamera(c, "Albedo"); })
      .def("get_normal_rgba", [](SCamera &c) { return getFloatImageFromCamera(c, "Normal"); })
      .def("get_visual_actor_segmentation",
           [](SCamera &c) { return getUintImageFromCamera(c, "Segmentation"); })
#ifdef SAPIEN_DLPACK
      .def(
          "get_dl_tensor",
          [](SCamera &cam, std::string const &name) {
            return wrapDLTensor(cam.getRendererCamera()->getDLImage(name));
          },
          "Get raw GPU memory for a render target in the dl format. It can be wrapped into "
          "PyTorch or Tensorflow using their API",
          py::arg("texture_name"))
#endif
      .def(
          "get_camera_matrix", [](SCamera &c) { return mat42array(c.getCameraMatrix()); },
          "Get 4x4 intrinsic camera matrix in OpenCV format.")

      .def(
          "get_intrinsic_matrix", [](SCamera &c) { return mat32array(c.getIntrinsicMatrix()); },
          "Get 3x3 intrinsic camera matrix in OpenCV format.")
      .def(
          "get_extrinsic_matrix", [](SCamera &c) { return mat42array(c.getExtrinsicMatrix()); },
          "Get 4x4 extrinsic camera matrix in OpenCV format.")

      .def(
          "get_model_matrix", [](SCamera &c) { return mat42array(c.getModelMatrix()); },
          "Get model matrix (inverse of extrinsic matrix) used in rendering (Y up, Z back)")
      .def(
          "get_projection_matrix", [](SCamera &c) { return mat42array(c.getProjectionMatrix()); },
          "Get projection matrix in used in rendering (right-handed NDC with [-1,1] XY and "
          "[0,1] "
          "Z)");

  PyVulkanWindow.def("show", &Renderer::SVulkan2Window::show)
      .def("hide", &Renderer::SVulkan2Window::hide)
      .def_property_readonly("should_close", &Renderer::SVulkan2Window::windowCloseRequested)
      .def("set_camera_parameters", &Renderer::SVulkan2Window::setCameraParameters,
           py::arg("near"), py::arg("far"), py::arg("fovy"))
      .def("set_intrinsic_parameters", &Renderer::SVulkan2Window::setCameraIntrinsicParameters,
           py::arg("near"), py::arg("far"), py::arg("fx"), py::arg("fy"), py::arg("cx"),
           py::arg("cy"), py::arg("skew"))
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
      .def(
          "set_camera_property",
          [](Renderer::SVulkan2Window &window, std::string name, float property) {
            window.setCameraProperty(name, property);
          },
          py::arg("key"), py::arg("value"))
      .def(
          "set_camera_property",
          [](Renderer::SVulkan2Window &window, std::string name, int property) {
            window.setCameraProperty(name, property);
          },
          py::arg("key"), py::arg("value"))
      .def("set_shader_dir", &Renderer::SVulkan2Window::setShader, py::arg("shader_dir"))
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
      .def("get_camera_projection_matrix",
           [](Renderer::SVulkan2Window &window) {
             glm::mat4 proj = glm::transpose(window.getCameraProjectionMatrix());
             return py::array_t<float>({4, 4}, &proj[0][0]);
           })
      .def_property_readonly(
          "_internal_renderer",
          [](Renderer::SVulkan2Window &window) { return window.getInternalRenderer(); },
          py::return_value_policy::reference)
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
      .def_property_readonly("display_target_names",
                             &Renderer::SVulkan2Window::getDisplayTargetNames,
                             "Names for available display targets that can be displayed "
                             "in the render function")
      .def("get_target_size", &Renderer::SVulkan2Window::getRenderTargetSize, py::arg("name"))
      .def("render", &Renderer::SVulkan2Window::render, py::arg("target_name"),
           py::arg("ui_windows") = std::vector<std::shared_ptr<svulkan2::ui::Window>>())
      .def("resize", &Renderer::SVulkan2Window::resize, py::arg("width"), py::arg("height"))
      .def_property_readonly("fps", &Renderer::SVulkan2Window::getFPS)
      .def_property_readonly("size", &Renderer::SVulkan2Window::getWindowSize)
      .def_property_readonly("fovy", &Renderer::SVulkan2Window::getCameraFovy)
      .def_property_readonly("near", &Renderer::SVulkan2Window::getCameraNear)
      .def_property_readonly("far", &Renderer::SVulkan2Window::getCameraFar)
      .def_property("cursor", &Renderer::SVulkan2Window::getCursorEnabled,
                    &Renderer::SVulkan2Window::setCursorEnabled)

      // Download images from window
      .def(
          "get_float_texture",
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
          "get_uint32_texture",
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
          "get_float_texture_pixel",
          [](Renderer::SVulkan2Window &window, std::string const &name, uint32_t x, uint32_t y) {
            auto v = window.downloadFloatTargetPixel(name, x, y);
            return py::array_t<float>(static_cast<int>(v.size()), v.data());
          },
          py::arg("name"), py::arg("x"), py::arg("y"))
      .def(
          "get_uint32_texture_pixel",
          [](Renderer::SVulkan2Window &window, std::string const &name, uint32_t x, uint32_t y) {
            auto v = window.downloadUint32TargetPixel(name, x, y);
            return py::array_t<uint32_t>(static_cast<int>(v.size()), v.data());
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

  PyVulkanScene.def_property_readonly(
      "_internal_scene", [](Renderer::SVulkan2Scene &scene) { return scene.getScene(); });

  PyRenderer.def("create_material", &Renderer::IPxrRenderer::createMaterial)
      .def(
          "create_mesh",
          [](Renderer::IPxrRenderer &renderer,
             Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &vertices,
             Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor> const &indices) {
            return renderer.createMesh(
                std::vector<float>(vertices.data(), vertices.data() + vertices.size()),
                std::vector<uint32_t>(indices.data(), indices.data() + indices.size()));
          },
          py::arg("vertices"), py::arg("indices"))
      .def(
          "create_texture_from_file",
          [](Renderer::IPxrRenderer &renderer, std::string const &filename, uint32_t mipLevels,
             std::string const &filterMode, std::string const &addressMode) {
            Renderer::IPxrTexture::FilterMode::Enum fm = getFilterMode(filterMode);
            Renderer::IPxrTexture::AddressMode::Enum am = getAddressMode(addressMode);
            return renderer.createTexture(filename, mipLevels, fm, am);
          },
          py::arg("filename"), py::arg("mipmap_levels") = 1, py::arg("filter_mode") = "linear",
          py::arg("address_mode") = "repeat")
      .def(
          "create_texture_from_array",
          [](Renderer::IPxrRenderer &renderer, py::array_t<uint8_t> array, uint32_t mipLevels,
             std::string const &filterMode, std::string const &addressMode) {
            Renderer::IPxrTexture::FilterMode::Enum fm = getFilterMode(filterMode);
            Renderer::IPxrTexture::AddressMode::Enum am = getAddressMode(addressMode);
            if (array.ndim() != 2 && array.ndim() != 3) {
              throw std::runtime_error(
                  "failed to create texture: array shape must be [h, w, c] or [h, w]");
            }
            int height = array.shape(0);
            int width = array.shape(1);

            return renderer.createTexture(
                std::vector<uint8_t>(array.data(), array.data() + array.size()), width, height,
                mipLevels, fm, am);
          },
          py::arg("array"), py::arg("mipmap_levels") = 1, py::arg("filter_mode") = "linear",
          py::arg("address_mode") = "repeat");

  PyRenderScene
      .def_property_readonly("ambient_light",
                             [](Renderer::IPxrScene &scene) {
                               auto light = scene.getAmbientLight();
                               return make_array(std::vector<float>{light[0], light[1], light[2]});
                             })
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

  PyRenderParticleBody
      .def("set_visibility", &Renderer::IPxrPointBody::setVisibility, py::arg("visibility"))
      .def("set_shading_mode", &Renderer::IPxrPointBody::setRenderMode, py::arg("mode"))
      .def("set_attribute", &Renderer::IPxrPointBody::setAttribute, py::arg("name"),
           py::arg("value"))
      .def("set_rendered_point_count", &Renderer::IPxrPointBody::setRenderedVertexCount,
           py::arg("n"))
      .def_property("rendered_point_count", &Renderer::IPxrPointBody::getRenderedVertexCount,
                    &Renderer::IPxrPointBody::setRenderedVertexCount);

#ifdef SAPIEN_DLPACK
  PyVulkanParticleBody.def_property_readonly("dl_vertices", [](Renderer::SVulkan2PointBody &b) {
    return wrapDLTensor(b.getDLVertices());
  });
#endif

  PyRenderBody.def_property_readonly("name", &Renderer::IPxrRigidbody::getName)
      .def_property_readonly("visual_id", &Renderer::IPxrRigidbody::getUniqueId)
      .def_property_readonly("actor_id", &Renderer::IPxrRigidbody::getSegmentationId)
      .def_property_readonly("local_pose", &Renderer::IPxrRigidbody::getInitialPose,
                             "Local pose is pre-multiplied when calling set_pose")
      .def("get_name", &Renderer::IPxrRigidbody::getName)
      .def("set_name", &Renderer::IPxrRigidbody::setName, py::arg("name"))
      .def("set_visual_id", &Renderer::IPxrRigidbody::setUniqueId, py::arg("id"))
      .def("get_visual_id", &Renderer::IPxrRigidbody::getUniqueId)
      .def("get_actor_id", &Renderer::IPxrRigidbody::getSegmentationId)
      .def("set_custom_data", &Renderer::IPxrRigidbody::setSegmentationCustomData,
           py::arg("custom_data"))
      .def("get_render_shapes", &Renderer::IPxrRigidbody::getRenderShapes)
      .def("set_pose", &Renderer::IPxrRigidbody::update, py::arg("pose"))
      .def("set_visibility", &Renderer::IPxrRigidbody::setVisibility, py::arg("visibility"))
      .def("set_visible", &Renderer::IPxrRigidbody::setVisible, py::arg("is_visible"))
      .def_property("shade_flat", &Renderer::IPxrRigidbody::getShadeFlat,
                    &Renderer::IPxrRigidbody::setShadeFlat)

      .def_property_readonly("type",
                             [](Renderer::IPxrRigidbody &body) {
                               switch (body.getType()) {
                               case physx::PxGeometryType::eBOX:
                                 return "box";
                               case physx::PxGeometryType::ePLANE:
                                 return "plane";
                               case physx::PxGeometryType::eSPHERE:
                                 return "sphere";
                               case physx::PxGeometryType::eCAPSULE:
                                 return "capsule";
                               default:
                                 return "mesh";
                               }
                             })
      .def_property_readonly("local_pose", &Renderer::IPxrRigidbody::getInitialPose)

      // attribute access for different types
      .def_property_readonly("scale",
                             [](Renderer::IPxrRigidbody &body) {
                               if (body.getType() != physx::PxGeometryType::eTRIANGLEMESH &&
                                   body.getType() != physx::PxGeometryType::eCONVEXMESH) {
                                 throw std::runtime_error(
                                     "Visual body scale is only valid for mesh.");
                               }
                               return vec32array(body.getScale());
                             })
      .def_property_readonly("radius",
                             [](Renderer::IPxrRigidbody &body) {
                               if (body.getType() != physx::PxGeometryType::eSPHERE &&
                                   body.getType() != physx::PxGeometryType::eCAPSULE) {
                                 throw std::runtime_error(
                                     "Visual body radius is only valid for sphere or capsule.");
                               }
                               return body.getScale().y;
                             })
      .def_property_readonly("half_lengths",
                             [](Renderer::IPxrRigidbody &body) {
                               if (body.getType() != physx::PxGeometryType::eBOX) {
                                 throw std::runtime_error(
                                     "Visual body half_lengths is only valid for box.");
                               }
                               return vec32array(body.getScale());
                             })
      .def_property_readonly("half_length", [](Renderer::IPxrRigidbody &body) {
        if (body.getType() != physx::PxGeometryType::eCAPSULE) {
          throw std::runtime_error("Visual body half_length is only valid for capsule.");
        }
        return body.getScale().x;
      });

  PyRenderShape
      // .def_readonly("type", &Renderer::RenderShape::type)
      // .def_readonly("pose", &Renderer::RenderShape::pose)
      // .def_readonly("visual_id", &Renderer::RenderShape::objId)
      // .def_property_readonly("scale",
      //                        [](Renderer::RenderShape &shape) { return
      //                        vec32array(shape.scale);
      //                        })
      .def_property_readonly("mesh", &Renderer::IPxrRenderShape::getGeometry)
      .def_property_readonly("material", &Renderer::IPxrRenderShape::getMaterial)
      .def("set_material", &Renderer::IPxrRenderShape::setMaterial, py::arg("material"));

  PyRenderMesh
      .def_property_readonly("vertices",
                             [](Renderer::IRenderMesh &geom) {
                               auto vertices = geom.getVertices();
                               return py::array_t<float>(
                                   {static_cast<int>(vertices.size() / 3), 3},
                                   {sizeof(float) * 3, sizeof(float)}, vertices.data());
                             })
      .def_property_readonly("normals",
                             [](Renderer::IRenderMesh &geom) {
                               auto normals = geom.getNormals();
                               return py::array_t<float>({static_cast<int>(normals.size() / 3), 3},
                                                         {sizeof(float) * 3, sizeof(float)},
                                                         normals.data());
                             })
      .def_property_readonly("uvs",
                             [](Renderer::IRenderMesh &geom) {
                               auto uvs = geom.getUVs();
                               return py::array_t<float>({static_cast<int>(uvs.size() / 2), 2},
                                                         {sizeof(float) * 2, sizeof(float)},
                                                         uvs.data());
                             })
      .def_property_readonly("tangents",
                             [](Renderer::IRenderMesh &geom) {
                               auto tangents = geom.getTangents();
                               return py::array_t<float>(
                                   {static_cast<int>(tangents.size() / 3), 3},
                                   {sizeof(float) * 3, sizeof(float)}, tangents.data());
                             })
      .def_property_readonly("bitangents",
                             [](Renderer::IRenderMesh &geom) {
                               auto bitangents = geom.getBitangents();
                               return py::array_t<float>(
                                   {static_cast<int>(bitangents.size() / 3), 3},
                                   {sizeof(float) * 3, sizeof(float)}, bitangents.data());
                             })
      .def_property_readonly(
          "indices", [](Renderer::IRenderMesh &geom) { return make_array(geom.getIndices()); })
      .def("set_vertices",
           [](Renderer::IRenderMesh &geom,
              Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> vertices) {
             geom.setVertices(std::vector(vertices.data(), vertices.data() + vertices.size()));
           })
      .def("set_indices",
           [](Renderer::IRenderMesh &geom,
              Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor> indices) {
             geom.setIndices(std::vector(indices.data(), indices.data() + indices.size()));
           })
      .def("set_normals",
           [](Renderer::IRenderMesh &geom,
              Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> normals) {
             geom.setNormals(std::vector(normals.data(), normals.data() + normals.size()));
           })
      .def("set_uvs", [](Renderer::IRenderMesh &geom,
                         Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> uvs) {
        geom.setUVs(std::vector(uvs.data(), uvs.data() + uvs.size()));
      });

#ifdef SAPIEN_DLPACK
  PyVulkanRenderMesh.def_property_readonly("dl_vertices", [](Renderer::SVulkan2Mesh &mesh) {
    return wrapDLTensor(mesh.getDLVertices());
  });
#endif

  m.def("add_profiler_event", &AddProfilerEvent, py::arg("name"));
  py::class_<ProfilerBlock>(m, "ProfilerBlock")
      .def(py::init<std::string>(), py::arg("name"))
      .def("__enter__", &ProfilerBlock::enter)
      .def("__exit__", &ProfilerBlock::exit);

#ifdef SAPIEN_DLPACK
  auto dlpack = m.def_submodule("dlpack");

  dlpack.def("dl_shape", [](py::capsule data) {
    DLManagedTensor *tensor = (DLManagedTensor *)data.get_pointer();
    return std::vector<int64_t>(tensor->dl_tensor.shape,
                                tensor->dl_tensor.shape + tensor->dl_tensor.ndim);
  });

  dlpack.def("dl_ptr", [](py::capsule data) {
    DLManagedTensor *tensor = (DLManagedTensor *)data.get_pointer();
    return (intptr_t)tensor->dl_tensor.data;
  });

  dlpack.def(
      "dl_to_numpy_cuda_async_unchecked",
      [](py::capsule data, py::array_t<float> array) {
        DLManagedTensor *tensor = (DLManagedTensor *)data.get_pointer();
        int64_t size = 1;
        for (int i = 0; i < tensor->dl_tensor.ndim; ++i) {
          size *= tensor->dl_tensor.shape[i];
        };
        int64_t bytes = size * (tensor->dl_tensor.dtype.bits / 8);
        cudaMemcpyAsync(py::detail::array_proxy(array.ptr())->data, tensor->dl_tensor.data, bytes,
                        cudaMemcpyDeviceToHost);
      },
      py::arg("dl_tensor"), py::arg("result").noconvert());

  dlpack.def("dl_cuda_sync", []() { cudaStreamSynchronize(0); });
#endif
}
