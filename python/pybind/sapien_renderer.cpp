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
#include "sapien/sapien_renderer/sapien_renderer.h"
#include "array.hpp"
#include "format.hpp"
#include "sapien_type_caster.h"
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <svulkan2/core/instance.h>

namespace py = pybind11;

using namespace pybind11::literals;
using namespace sapien;
using namespace sapien::sapien_renderer;

namespace pybind11::detail {

template <> struct type_caster<CameraMode> {
  PYBIND11_TYPE_CASTER(CameraMode, _("typing.Literal['perspective', 'orthographic']"));

  bool load(py::handle src, bool convert) {
    std::string name = py::cast<std::string>(src);
    if (name == "perspective" || name == "pers") {
      value = CameraMode::ePerspective;
      return true;
    } else if (name == "orthographic" || name == "ortho") {
      value = CameraMode::eOrthographic;
      return true;
    }
    return false;
  }

  static py::handle cast(CameraMode const &src, py::return_value_policy policy,
                         py::handle parent) {
    switch (src) {
    case CameraMode::ePerspective:
      return py::str("perspective").release();
    case CameraMode::eOrthographic:
      return py::str("orthographic").release();
    default:
      return py::str("none").release();
    }
  }
};

template <> struct type_caster<svulkan2::renderer::RTRenderer::DenoiserType> {
  PYBIND11_TYPE_CASTER(svulkan2::renderer::RTRenderer::DenoiserType,
                       _("typing.Literal['none', 'oidn', 'optix']"));

  bool load(py::handle src, bool convert) {
    std::string name = py::cast<std::string>(src);
    if (name == "none") {
      value = svulkan2::renderer::RTRenderer::DenoiserType::eNONE;
      return true;
    } else if (name == "oidn") {
      value = svulkan2::renderer::RTRenderer::DenoiserType::eOIDN;
      return true;
    } else if (name == "optix") {
      value = svulkan2::renderer::RTRenderer::DenoiserType::eOPTIX;
      return true;
    }
    return false;
  }

  static py::handle cast(svulkan2::renderer::RTRenderer::DenoiserType const &src,
                         py::return_value_policy policy, py::handle parent) {
    switch (src) {
    case svulkan2::renderer::RTRenderer::DenoiserType::eOPTIX:
      return py::str("optix").release();
    case svulkan2::renderer::RTRenderer::DenoiserType::eOIDN:
      return py::str("oidn").release();
    case svulkan2::renderer::RTRenderer::DenoiserType::eNONE:
    default:
      return py::str("none").release();
    }
  }
};
} // namespace pybind11::detail

py::array downloadSapienTexture(SapienRenderTexture &texture) {
  vk::Format format = texture.getFormat();
  int channels = getFormatChannels(format);
  int dim = texture.getDim();

  std::vector<py::ssize_t> shape;
  if (dim == 1) {
    shape.push_back(texture.getWidth());
  } else if (dim == 2) {
    shape.push_back(texture.getHeight());
    shape.push_back(texture.getWidth());
  } else if (dim == 3) {
    shape.push_back(texture.getDepth());
    shape.push_back(texture.getHeight());
    shape.push_back(texture.getWidth());
  }
  shape.push_back(channels);

  size_t size = texture.getWidth() * texture.getHeight() * texture.getDepth() *
                svulkan2::getFormatSize(format);
  std::vector<char> data(size);

  texture.download(data.data());
  return py::array(py::dtype(getFormatTypestr(format)), shape, data.data());
}

py::array downloadSapienTexture2D(SapienRenderTexture2D &texture) {
  return downloadSapienTexture(texture);
}

void uploadSapienTexture(SapienRenderTexture &texture, py::array array) {
  if (!(array.flags() & py::array::c_style)) {
    throw std::runtime_error("array is not contiguous.");
  }

  vk::Format format = texture.getFormat();
  int channels = getFormatChannels(format);
  int dim = texture.getDim();

  std::vector<py::ssize_t> shape;
  if (dim == 1) {
    shape.push_back(texture.getWidth());
  } else if (dim == 2) {
    shape.push_back(texture.getHeight());
    shape.push_back(texture.getWidth());
  } else if (dim == 3) {
    shape.push_back(texture.getDepth());
    shape.push_back(texture.getHeight());
    shape.push_back(texture.getWidth());
  }
  shape.push_back(channels);

  // check array matches texture
  if (!array.dtype().is(py::dtype(getFormatTypestr(format)))) {
    throw std::runtime_error("array format does not match texture format");
  }
  if (array.ndim() != static_cast<int>(shape.size())) {
    throw std::runtime_error("array dim does not match texture");
  }
  for (int i = 0; i < array.ndim(); ++i) {
    if (array.shape(0) != shape.at(0)) {
      throw std::runtime_error("array shape does not match texture");
    }
  }

  texture.upload(array.request().ptr);
}

void uploadSapienTexture2D(SapienRenderTexture2D &texture, py::array array) {
  uploadSapienTexture(texture, array);
}

static std::shared_ptr<SapienRenderTexture>
CreateSapienTexture(py::array array, int dim, vk::Format format, uint32_t mipLevels,
                    SapienRenderTexture::FilterMode filterMode,
                    SapienRenderTexture::AddressMode addressMode, bool srgb) {

  if (array.ndim() < 1 || array.ndim() > 4) {
    throw std::runtime_error("invalid numpy array dimension");
  }

  py::dtype dtype(getFormatTypestr(format));
  int channels = getFormatChannels(format);

  std::vector<int> shape;
  for (int i = 0; i < array.ndim(); ++i) {
    shape.push_back(array.shape(i));
  }

  if (channels == 1 && array.ndim() == dim) {
    shape.push_back(1);
  }
  if (static_cast<int>(shape.size()) != dim + 1) {
    throw std::runtime_error("numpy array dimension does not match provided dim");
  }

  if (shape.at(shape.size() - 1) != channels) {
    throw std::runtime_error("numpy array is incompatible with the provided format");
  }

  uint32_t width = 1;
  uint32_t height = 1;
  uint32_t depth = 1;
  if (shape.size() == 2) {
    width = shape[0];
  }
  if (shape.size() == 3) {
    height = shape[0];
    width = shape[1];
  }
  if (shape.size() == 4) {
    depth = shape[0];
    height = shape[1];
    width = shape[2];
  }

  if (shape.size() > 4) {
    // unreachable
    throw std::runtime_error("corrupted shape");
  }

  if (!(array.flags() & py::array::c_style)) {
    throw std::runtime_error("array is not contiguous.");
  }

  if (!array.dtype().is(dtype)) {
    throw std::runtime_error("array dtype [" + std::string(py::str(array.dtype())) +
                             "] is incompatible with texture format [" +
                             FormatToString.at(format) + "]. You may use [" +
                             std::string(py::str(dtype)) + "] instead.");
    // FIXME: figure out how to properly cast the array

    // printf("casting array!\n");
    // auto newArray = py::detail::npy_api::get().PyArray_FromAny_(
    //     array.release().ptr(), dtype.release().ptr(), 0, 0,
    //     py::detail::npy_api::NPY_ARRAY_ENSUREARRAY_ |
    //         py::detail::npy_api::NPY_ARRAY_C_CONTIGUOUS_ |
    //         py::detail::npy_api::NPY_ARRAY_FORCECAST_,
    //     nullptr);
    // array = py::reinterpret_steal<py::array>(newArray);
  }

  char *data = reinterpret_cast<char *>(array.request().ptr);
  std::vector<char> rawData = {data, data + array.nbytes()};

  return std::make_shared<SapienRenderTexture>(width, height, depth, format, rawData, dim,
                                               mipLevels, filterMode, addressMode, srgb);
}

// TODO: merge with previous function
static std::shared_ptr<SapienRenderTexture2D>
CreateSapienTexture2D(py::array array, vk::Format format, uint32_t mipLevels,
                      SapienRenderTexture::FilterMode filterMode,
                      SapienRenderTexture::AddressMode addressMode, bool srgb) {

  if (array.ndim() != 2 && array.ndim() != 3) {
    throw std::runtime_error("invalid numpy array dimension");
  }

  py::dtype dtype(getFormatTypestr(format));
  int channels = getFormatChannels(format);

  if (array.ndim() == 2 && channels != 1) {
    throw std::runtime_error("numpy array dimension is not compatible");
  }
  if (array.ndim() == 3 && channels != array.shape(2)) {
    throw std::runtime_error("numpy array dimension is not compatible");
  }

  uint32_t width = array.shape(1);
  uint32_t height = array.shape(0);

  if (!(array.flags() & py::array::c_style)) {
    throw std::runtime_error("array is not contiguous.");
  }

  if (!array.dtype().is(dtype)) {
    throw std::runtime_error("array dtype [" + std::string(py::str(array.dtype())) +
                             "] is incompatible with texture format [" +
                             FormatToString.at(format) + "]. You may use [" +
                             std::string(py::str(dtype)) + "] instead.");
  }

  char *data = reinterpret_cast<char *>(array.request().ptr);
  std::vector<char> rawData = {data, data + array.nbytes()};

  return std::make_shared<SapienRenderTexture2D>(width, height, format, rawData, mipLevels,
                                                 filterMode, addressMode, srgb);
}

namespace pybind11::detail {

template <> struct type_caster<vk::Format> {
  PYBIND11_TYPE_CASTER(vk::Format, _("str"));

  bool load(py::handle src, bool convert) {
    std::string name = py::cast<std::string>(src);
    std::transform(name.begin(), name.end(), name.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    if (!StringToFormat.contains(name)) {
      return false;
    }
    value = StringToFormat.at(name);
    return true;
  }

  static py::handle cast(vk::Format const &src, py::return_value_policy policy,
                         py::handle parent) {
    if (!FormatToString.contains(src)) {
      throw std::runtime_error("unsupported image format");
    }
    return py::str(FormatToString.at(src)).release();
  }
};

template <> struct type_caster<vk::CullModeFlagBits> {
  PYBIND11_TYPE_CASTER(vk::CullModeFlagBits, _("typing.Literal['back', 'front', 'none', 'both']"));

  bool load(py::handle src, bool convert) {
    std::string name = py::cast<std::string>(src);
    if (name == "back") {
      value = vk::CullModeFlagBits::eBack;
      return true;
    } else if (name == "front") {
      value = vk::CullModeFlagBits::eFront;
      return true;
    } else if (name == "none") {
      value = vk::CullModeFlagBits::eNone;
      return true;
    } else if (name == "both" || name == "front_and_back") {
      value = vk::CullModeFlagBits::eFrontAndBack;
      return true;
    }
    return false;
  }

  static py::handle cast(vk::CullModeFlagBits const &src, py::return_value_policy policy,
                         py::handle parent) {
    switch (src) {
    case vk::CullModeFlagBits::eBack:
      return py::str("back").release();
    case vk::CullModeFlagBits::eFront:
      return py::str("front").release();
    case vk::CullModeFlagBits::eFrontAndBack:
      return py::str("both").release();
    case vk::CullModeFlagBits::eNone:
      return py::str("none").release();
    }
    throw std::runtime_error("invalid culling");
  }
};

template <> struct type_caster<vk::FrontFace> {
  PYBIND11_TYPE_CASTER(vk::FrontFace, _("typing.Literal['counterclockwise', 'clockwise']"));

  bool load(py::handle src, bool convert) {
    std::string name = py::cast<std::string>(src);
    if (name == "clockwise") {
      value = vk::FrontFace::eClockwise;
      return true;
    } else if (name == "counterclockwise") {
      value = vk::FrontFace::eCounterClockwise;
      return true;
    }
    return false;
  }

  static py::handle cast(vk::FrontFace const &src, py::return_value_policy policy,
                         py::handle parent) {
    switch (src) {
    case vk::FrontFace::eClockwise:
      return py::str("clockwise").release();
    case vk::FrontFace::eCounterClockwise:
      return py::str("counterclockwise").release();
    }
    throw std::runtime_error("invalid front face");
  }
};

template <> struct type_caster<SapienRenderTexture::FilterMode> {
  PYBIND11_TYPE_CASTER(SapienRenderTexture::FilterMode, _("typing.Literal['nearest', 'linear']"));

  bool load(py::handle src, bool convert) {
    std::string name = py::cast<std::string>(src);
    if (name == "nearest") {
      value = SapienRenderTexture::FilterMode::eNEAREST;
      return true;
    } else if (name == "linear") {
      value = SapienRenderTexture::FilterMode::eLINEAR;
      return true;
    }
    return false;
  }

  static py::handle cast(SapienRenderTexture::FilterMode const &src,
                         py::return_value_policy policy, py::handle parent) {
    switch (src) {
    case SapienRenderTexture::FilterMode::eNEAREST:
      return py::str("nearest").release();
    case SapienRenderTexture::FilterMode::eLINEAR:
      return py::str("linear").release();
    }
    throw std::runtime_error("invalid filter mode");
  }
};

template <> struct type_caster<SapienRenderTexture::AddressMode> {
  PYBIND11_TYPE_CASTER(SapienRenderTexture::AddressMode,
                       _("typing.Literal['repeat', 'border', 'edge', 'mirror']"));

  bool load(py::handle src, bool convert) {
    std::string name = py::cast<std::string>(src);
    if (name == "repeat") {
      value = SapienRenderTexture::AddressMode::eREPEAT;
      return true;
    } else if (name == "border") {
      value = SapienRenderTexture::AddressMode::eBORDER;
      return true;
    } else if (name == "edge") {
      value = SapienRenderTexture::AddressMode::eEDGE;
      return true;
    } else if (name == "mirror") {
      value = SapienRenderTexture::AddressMode::eMIRROR;
      return true;
    }
    return false;
  }

  static py::handle cast(SapienRenderTexture::AddressMode const &src,
                         py::return_value_policy policy, py::handle parent) {
    switch (src) {
    case SapienRenderTexture::AddressMode::eREPEAT:
      return py::str("repeat").release();
    case SapienRenderTexture::AddressMode::eBORDER:
      return py::str("border").release();
    case SapienRenderTexture::AddressMode::eEDGE:
      return py::str("edge").release();
    case SapienRenderTexture::AddressMode::eMIRROR:
      return py::str("mirror").release();
    }
    throw std::runtime_error("invalid address mode");
  }
};

} // namespace pybind11::detail

void init_sapien_renderer(py::module &sapien) {
  auto m = sapien.def_submodule("render");

  auto PyRenderSceneLoaderNode = py::classh<RenderSceneLoaderNode>(m, "RenderSceneLoaderNode");

  ////////// global //////////

  m.def("_internal_set_shader_search_path", &SapienRendererDefault::internalSetShaderSearchPath,
        py::arg("path"))
      .def("set_imgui_ini_filename", &SapienRendererDefault::setImguiIniFilename,
           py::arg("filename"))
      .def("set_vr_action_manifest_filename", &SapienRendererDefault::setVRActionManifestFilename,
           py::arg("filename"))
      .def("enable_vr", &SapienRendererDefault::enableVR,
           "Enable VR via Steam. Must be called before creating RenderSystem or sapien Scene.")
      .def("get_vr_enabled", &SapienRendererDefault::getVREnabled)
      .def(
          "_force_vr_shutdown",
          []() { SapienRenderEngine::Get()->getContext()->getInstance2()->shutdownVR(); },
          "SteamVR will be permanently broken (even across processes) when the process is killed "
          "without shutting down VR. This function is used to force shutting down VR when you "
          "know SAPIEN will be killed without a clean shutdown.")

      .def("set_viewer_shader_dir", &SapienRendererDefault::setViewerShaderDirectory,
           py::arg("dir"))
      .def("set_camera_shader_dir", &SapienRendererDefault::setCameraShaderDirectory,
           py::arg("dir"))
      .def("set_ray_tracing_samples_per_pixel",
           &SapienRendererDefault::setRayTracingSamplesPerPixel, py::arg("spp"))
      .def("set_ray_tracing_path_depth", &SapienRendererDefault::setRayTracingPathDepth,
           py::arg("depth"))
      .def("set_ray_tracing_denoiser", &SapienRendererDefault::setRayTracingDenoiser,
           py::arg("name"))
      .def("set_ray_tracing_dof_aperture", &SapienRendererDefault::setRayTracingDoFAperture,
           py::arg("radius"))
      .def("set_ray_tracing_dof_plane", &SapienRendererDefault::setRayTracingDoFPlane,
           py::arg("depth"))
      .def("set_msaa", &SapienRendererDefault::setMSAA, py::arg("msaa"))
      .def("set_picture_format", &SapienRendererDefault::setRenderTargetFormat, py::arg("name"),
           py::arg("format"))

      .def("get_imgui_ini_filename", &SapienRendererDefault::getImguiIniFilename)
      .def("get_vr_action_manifest_filename", &SapienRendererDefault::getVRActionManifestFilename)
      .def("get_viewer_shader_dir", &SapienRendererDefault::getViewerShaderDirectory)
      .def("get_camera_shader_dir", &SapienRendererDefault::getCameraShaderDirectory)
      .def("get_ray_tracing_samples_per_pixel",
           &SapienRendererDefault::getRayTracingSamplesPerPixel)
      .def("get_ray_tracing_path_depth", &SapienRendererDefault::getRayTracingPathDepth)
      .def("get_ray_tracing_denoiser", &SapienRendererDefault::getRayTracingDenoiser)
      .def("get_ray_tracing_dof_aperture", &SapienRendererDefault::getRayTracingDoFAperture)
      .def("get_ray_tracing_dof_plane", &SapienRendererDefault::getRayTracingDoFPlane)
      .def("get_msaa", &SapienRendererDefault::getMSAA)

      .def("set_log_level", &svulkan2::logger::setLogLevel, py::arg("level"))
      .def(
          "clear_cache",
          [](bool models, bool images, bool shaders) {
            SapienRenderEngine::Get(nullptr)->getResourceManager()->clearCachedResources(
                models, images, shaders);
          },
          py::arg("models") = true, py::arg("images") = true, py::arg("shaders") = false)

      .def(
          "load_scene",
          [](std::string const &filename, bool applyScale) {
            return LoadScene(filename, applyScale);
          },
          py::arg("filename"), py::arg("apply_scale") = true)
      .def("get_device_summary", []() { return SapienRenderEngine::Get(nullptr)->getSummary(); })

      .def("set_global_config", &SapienRendererDefault::setGlobalConfig,
           py::arg("max_num_materials") = 128, py::arg("max_num_textures") = 512,
           py::arg("default_mipmap_levels") = 1, py::arg("do_not_load_texture") = false,
           "Sets global properties for SAPIEN renderer. This function should only be called "
           "before creating any renderer-related objects.");

  ////////// end global //////////

  auto PySapienRenderer = py::classh<SapienRenderEngine>(m, "SapienRenderer");
  auto PyRenderTexture = py::classh<SapienRenderTexture>(m, "RenderTexture");
  auto PyRenderTexture2D = py::classh<SapienRenderTexture2D>(m, "RenderTexture2D");

  // auto PyRenderTargetCuda = py::classh<SapienRenderImageCuda>(m, "RenderImageCuda");

  auto PyRenderCubemap = py::classh<SapienRenderCubemap>(m, "RenderCubemap");
  auto PyRenderMaterial = py::classh<SapienRenderMaterial>(m, "RenderMaterial");
  auto PyRenderShapeTriangleMeshPart =
      py::classh<RenderShapeTriangleMeshPart>(m, "RenderShapeTriangleMeshPart");

  auto PyRenderShape = py::classh<RenderShape>(m, "RenderShape");
  auto PyRenderShapePrimitive =
      py::classh<RenderShapePrimitive, RenderShape>(m, "RenderShapePrimitive");
  auto PyRenderShapePlane =
      py::classh<RenderShapePlane, RenderShapePrimitive>(m, "RenderShapePlane");
  auto PyRenderShapeBox = py::classh<RenderShapeBox, RenderShapePrimitive>(m, "RenderShapeBox");
  auto PyRenderShapeSphere =
      py::classh<RenderShapeSphere, RenderShapePrimitive>(m, "RenderShapeSphere");
  auto PyRenderShapeCapsule =
      py::classh<RenderShapeCapsule, RenderShapePrimitive>(m, "RenderShapeCapsule");
  auto PyRenderShapeCylinder =
      py::classh<RenderShapeCylinder, RenderShapePrimitive>(m, "RenderShapeCylinder");
  auto PyRenderShapeTriangleMesh =
      py::classh<RenderShapeTriangleMesh, RenderShape>(m, "RenderShapeTriangleMesh");

  auto PyRenderSystem = py::classh<SapienRendererSystem, System>(m, "RenderSystem");

  auto PyRenderSystemGroup = py::classh<BatchedRenderSystem>(m, "RenderSystemGroup");
  auto PyCameraGroup = py::classh<BatchedCamera>(m, "RenderCameraGroup");

  auto PyRenderBodyComponent =
      py::classh<SapienRenderBodyComponent, Component>(m, "RenderBodyComponent");
  auto PyRenderPointCloudComponent =
      py::classh<PointCloudComponent, Component>(m, "RenderPointCloudComponent");
  auto PyRenderCameraComponent =
      py::classh<SapienRenderCameraComponent, Component>(m, "RenderCameraComponent");
  auto PyRenderLightComponent =
      py::classh<SapienRenderLightComponent, Component>(m, "RenderLightComponent");
  auto PyRenderPointLightComponent =
      py::classh<SapienRenderPointLightComponent, SapienRenderLightComponent>(
          m, "RenderPointLightComponent");
  auto PyRenderDirectionalLightComponent =
      py::classh<SapienRenderDirectionalLightComponent, SapienRenderLightComponent>(
          m, "RenderDirectionalLightComponent");
  auto PyRenderSpotLightComponent =
      py::classh<SapienRenderSpotLightComponent, SapienRenderLightComponent>(
          m, "RenderSpotLightComponent");
  auto PyRenderTexturedLightComponent =
      py::classh<SapienRenderTexturedLightComponent, SapienRenderSpotLightComponent>(
          m, "RenderTexturedLightComponent");
  auto PyRenderParallelogramLightComponent =
      py::classh<SapienRenderParallelogramLightComponent, SapienRenderLightComponent>(
          m, "RenderParallelogramLightComponent");
  auto PyRenderCudaMeshComponent =
      py::classh<CudaDeformableMeshComponent, Component>(m, "RenderCudaMeshComponent");

  auto PyRenderWindow = py::classh<SapienRendererWindow>(m, "RenderWindow");
  auto PyRenderVRDisplay = py::classh<SapienVRDisplay>(m, "RenderVRDisplay");

  PySapienRenderer.def(py::init(&SapienRenderEngine::Get), py::arg("device") = nullptr)
      .def_property_readonly("_internal_context", &SapienRenderEngine::getContext);

  PyRenderSystemGroup
      .def(py::init<std::vector<std::shared_ptr<SapienRendererSystem>>>(), py::arg("systems"))
      .def("create_camera_group", &BatchedRenderSystem::createCameraBatch, py::arg("cameras"),
           py::arg("picture_names"))
      .def("set_cuda_stream", &BatchedRenderSystem::setCudaStream, py::arg("stream"))
      .def("set_cuda_poses", &BatchedRenderSystem::setPoseSource, py::arg("pose_buffer"))
      .def("update_render", &BatchedRenderSystem::update,
           R"doc(
This function performs CUDA operations to transfer poses from the CUDA buffer provided by :func:`set_cuda_poses` into render systems.
It updates the transformation matrices of objects and cameras.

This function waits for any pending CUDA operations on cuda stream provided by :func:`set_cuda_stream`.
)doc");

  PyCameraGroup.def("take_picture", &BatchedCamera::takePicture)
      .def("get_picture_cuda", &BatchedCamera::getPictureCuda, py::arg("name"));

  PyRenderSystem
      .def(py::init([](std::shared_ptr<Device> device) {
             return std::make_shared<SapienRendererSystem>(device);
           }),
           py::arg("device") = nullptr)
      .def(py::init([](std::string const &device) {
             return std::make_shared<SapienRendererSystem>(findDevice(device));
           }),
           py::arg("device"))
      .def_property_readonly("device", &SapienRendererSystem::getDevice)
      .def_property("ambient_light", &SapienRendererSystem::getAmbientLight,
                    &SapienRendererSystem::setAmbientLight)
      .def("get_ambient_light", &SapienRendererSystem::getAmbientLight)
      .def("set_ambient_light", &SapienRendererSystem::setAmbientLight, py::arg("color"))

      .def_property_readonly("cameras", &SapienRendererSystem::getCameraComponents)
      .def("get_cameras", &SapienRendererSystem::getCameraComponents)

      .def_property_readonly("render_bodies", &SapienRendererSystem::getRenderBodyComponents)
      .def("get_render_bodies", &SapienRendererSystem::getRenderBodyComponents)

      .def_property_readonly("point_clouds", &SapienRendererSystem::getPointCloudComponents)
      .def("get_point_clouds", &SapienRendererSystem::getPointCloudComponents)

      .def_property_readonly("lights", &SapienRendererSystem::getLightComponents)
      .def("get_lights", &SapienRendererSystem::getLightComponents)

      .def_property("cubemap", &SapienRendererSystem::getCubemap,
                    &SapienRendererSystem::setCubemap)
      .def("get_cubemap", &SapienRendererSystem::getCubemap)
      .def("set_cubemap", &SapienRendererSystem::setCubemap, py::arg("cubemap"))

      // .def("disable_auto_upload", &SapienRendererSystem::disableAutoUpload)

      .def_property_readonly("cuda_object_transforms",
                             &SapienRendererSystem::getTransformCudaArray)

      .def_property_readonly("_internal_scene", &SapienRendererSystem::getScene);

  PyRenderTexture
      .def(py::init(&CreateSapienTexture), py::arg("array"), py::arg("dim"), py::arg("format"),
           py::arg("mipmap_levels") = 1,
           py::arg("filter_mode") = SapienRenderTexture::FilterMode::eLINEAR,
           py::arg("address_mode") = SapienRenderTexture::AddressMode::eREPEAT,
           py::arg("srgb") = false)
      .def_property_readonly("width", &SapienRenderTexture::getWidth)
      .def("get_width", &SapienRenderTexture::getWidth)
      .def_property_readonly("height", &SapienRenderTexture::getHeight)
      .def("get_height", &SapienRenderTexture::getHeight)
      .def_property_readonly("depth", &SapienRenderTexture::getDepth)
      .def("get_depth", &SapienRenderTexture::getDepth)
      .def_property_readonly("channels", &SapienRenderTexture::getChannels)
      .def("get_channels", &SapienRenderTexture::getChannels)
      .def_property_readonly("mipmap_levels", &SapienRenderTexture::getMipmapLevels)
      .def("get_mipmap_levels", &SapienRenderTexture::getMipmapLevels)
      .def_property_readonly("address_mode", &SapienRenderTexture::getAddressMode)
      .def("get_address_mode", &SapienRenderTexture::getAddressMode)
      .def_property_readonly("filter_mode", &SapienRenderTexture::getFilterMode)
      .def("get_filter_mode", &SapienRenderTexture::getFilterMode)
      .def_property_readonly("format", &SapienRenderTexture::getFormat)
      .def("get_format", &SapienRenderTexture::getFormat)
      .def_property_readonly("is_srgb", &SapienRenderTexture::getIsSrgb)
      .def("download", &downloadSapienTexture)
      .def("upload", &uploadSapienTexture, py::arg("data"))
      .def("__eq__", [](SapienRenderTexture const &self, SapienRenderTexture const &other) {
        return self.getTexture() == other.getTexture();
      });

  PyRenderTexture2D
      .def(py::init(&CreateSapienTexture2D), py::arg("array"), py::arg("format"),
           py::arg("mipmap_levels") = 1,
           py::arg("filter_mode") = SapienRenderTexture::FilterMode::eLINEAR,
           py::arg("address_mode") = SapienRenderTexture::AddressMode::eREPEAT,
           py::arg("srgb") = false)
      .def(py::init<std::string, uint32_t, SapienRenderTexture::FilterMode,
                    SapienRenderTexture::AddressMode, bool>(),
           py::arg("filename"), py::arg("mipmap_levels") = 1,
           py::arg("filter_mode") = SapienRenderTexture::FilterMode::eLINEAR,
           py::arg("address_mode") = SapienRenderTexture::AddressMode::eREPEAT,
           py::arg("srgb") = true,
           "Create texture from file. The srgb parameter only affects files in uint8 format; it "
           "should be true for color textures (diffuse, emission) and false for others (normal, "
           "roughness)")
      .def_property_readonly("width", &SapienRenderTexture2D::getWidth)
      .def("get_width", &SapienRenderTexture2D::getWidth)
      .def_property_readonly("height", &SapienRenderTexture2D::getHeight)
      .def("get_height", &SapienRenderTexture2D::getHeight)
      .def_property_readonly("channels", &SapienRenderTexture2D::getChannels)
      .def("get_channels", &SapienRenderTexture2D::getChannels)
      .def_property_readonly("mipmap_levels", &SapienRenderTexture2D::getMipmapLevels)
      .def("get_mipmap_levels", &SapienRenderTexture2D::getMipmapLevels)
      .def_property_readonly("is_srgb", &SapienRenderTexture2D::getIsSrgb)

      .def_property_readonly("address_mode", &SapienRenderTexture2D::getAddressMode)
      .def("get_address_mode", &SapienRenderTexture2D::getAddressMode)
      .def_property_readonly("filter_mode", &SapienRenderTexture2D::getFilterMode)
      .def("get_filter_mode", &SapienRenderTexture2D::getFilterMode)
      .def_property_readonly("filename", &SapienRenderTexture2D::getFilename)
      .def("get_filename", &SapienRenderTexture2D::getFilename)
      .def_property_readonly("format", &SapienRenderTexture2D::getFormat)
      .def("get_format", &SapienRenderTexture2D::getFormat)
      .def("download", &downloadSapienTexture2D)
      .def("upload", &uploadSapienTexture2D, py::arg("data"))
      .def("__eq__", [](SapienRenderTexture2D const &self, SapienRenderTexture2D const &other) {
        return self.getTexture() == other.getTexture();
      });

  PyRenderCubemap.def(py::init<std::string const &>(), py::arg("filename"))
      .def(py::init<std::string const &, std::string const &, std::string const &,
                    std::string const &, std::string const &, std::string const &>(),
           py::arg("px"), py::arg("nx"), py::arg("py"), py::arg("ny"), py::arg("pz"),
           py::arg("nz"))
      .def("export", &SapienRenderCubemap::exportKtx, py::arg("filename"))
      .def_property_readonly("_internal_cubemap", &SapienRenderCubemap::getCubemap);

  PyRenderMaterial
      .def(py::init<std::array<float, 4>, std::array<float, 4>, float, float, float, float, float,
                    float>(),
           py::arg("emission") = std::array<float, 4>{0.f, 0.f, 0.f, 0.f},
           py::arg("base_color") = std::array<float, 4>{1.f, 1.f, 1.f, 1.f},
           py::arg("specular") = 0.f, py::arg("roughness") = 1.f, py::arg("metallic") = 0.f,
           py::arg("transmission") = 0.f, py::arg("ior") = 1.45f,
           py::arg("transmission_roughness") = 0.f)

      // property
      .def_property("base_color", &SapienRenderMaterial::getBaseColor,
                    &SapienRenderMaterial::setBaseColor)
      .def("get_base_color", &SapienRenderMaterial::getBaseColor)
      .def("set_base_color", &SapienRenderMaterial::setBaseColor, py::arg("color"))
      .def_property("emission", &SapienRenderMaterial::getEmission,
                    &SapienRenderMaterial::setEmission)
      .def("get_emission", &SapienRenderMaterial::getEmission)
      .def("set_emission", &SapienRenderMaterial::setEmission, py::arg("emission"))
      .def_property("specular", &SapienRenderMaterial::getSpecular,
                    &SapienRenderMaterial::setSpecular)
      .def("get_specular", &SapienRenderMaterial::getSpecular)
      .def("set_specular", &SapienRenderMaterial::setSpecular, py::arg("specular"))
      .def_property("metallic", &SapienRenderMaterial::getMetallic,
                    &SapienRenderMaterial::setMetallic)
      .def("get_metallic", &SapienRenderMaterial::getMetallic)
      .def("set_metallic", &SapienRenderMaterial::setMetallic, py::arg("metallic"))
      .def_property("roughness", &SapienRenderMaterial::getRoughness,
                    &SapienRenderMaterial::setRoughness)
      .def("get_roughness", &SapienRenderMaterial::getRoughness)
      .def("set_roughness", &SapienRenderMaterial::setRoughness, py::arg("roughness"))
      .def_property("transmission", &SapienRenderMaterial::getTransmission,
                    &SapienRenderMaterial::setTransmission)
      .def("get_transmission", &SapienRenderMaterial::getTransmission)
      .def("set_transmission", &SapienRenderMaterial::setTransmission, py::arg("transmission"))
      .def_property("ior", &SapienRenderMaterial::getIOR, &SapienRenderMaterial::setIOR)
      .def("get_ior", &SapienRenderMaterial::getIOR)
      .def("set_ior", &SapienRenderMaterial::setIOR, py::arg("ior"))
      .def_property("transmission_roughness", &SapienRenderMaterial::getTransmissionRoughness,
                    &SapienRenderMaterial::setTransmissionRoughness)
      .def("get_transmission_roughness", &SapienRenderMaterial::getTransmissionRoughness)
      .def("set_transmission_roughness", &SapienRenderMaterial::setTransmissionRoughness,
           py::arg("roughness"))

      .def_property("emission_texture", &SapienRenderMaterial::getEmissionTexture,
                    &SapienRenderMaterial::setEmissionTexture)
      .def("get_emission_texture", &SapienRenderMaterial::getEmissionTexture)
      .def("set_emission_texture", &SapienRenderMaterial::setEmissionTexture, py::arg("texture"))

      .def_property("base_color_texture", &SapienRenderMaterial::getBaseColorTexture,
                    &SapienRenderMaterial::setBaseColorTexture)
      .def("get_base_color_texture", &SapienRenderMaterial::getBaseColorTexture)
      .def("set_base_color_texture", &SapienRenderMaterial::setBaseColorTexture,
           py::arg("texture"))

      .def_property("metallic_texture", &SapienRenderMaterial::getMetallicTexture,
                    &SapienRenderMaterial::setMetallicTexture)
      .def("get_metallic_texture", &SapienRenderMaterial::getMetallicTexture)
      .def("set_metallic_texture", &SapienRenderMaterial::setMetallicTexture, py::arg("texture"))
      .def_property("roughness_texture", &SapienRenderMaterial::getRoughnessTexture,
                    &SapienRenderMaterial::setRoughnessTexture)
      .def("get_roughness_texture", &SapienRenderMaterial::getRoughnessTexture)
      .def("set_roughness_texture", &SapienRenderMaterial::setRoughnessTexture, py::arg("texture"))
      .def_property("normal_texture", &SapienRenderMaterial::getNormalTexture,
                    &SapienRenderMaterial::setNormalTexture)
      .def("get_normal_texture", &SapienRenderMaterial::getNormalTexture)
      .def("set_normal_texture", &SapienRenderMaterial::setNormalTexture, py::arg("texture"))
      .def_property("transmission_texture", &SapienRenderMaterial::getTransmissionTexture,
                    &SapienRenderMaterial::setTransmissionTexture)
      .def("get_transmission_texture", &SapienRenderMaterial::getTransmissionTexture)
      .def("set_transmission_texture", &SapienRenderMaterial::setTransmissionTexture,
           py::arg("texture"))
      .def("__eq__", [](SapienRenderMaterial &self, SapienRenderMaterial &other) {
        return self.getMaterial() == other.getMaterial();
      });

  PyRenderShape.def_property("local_pose", &RenderShape::getLocalPose, &RenderShape::setLocalPose)
      .def("get_local_pose", &RenderShape::getLocalPose)
      .def("set_local_pose", &RenderShape::setLocalPose, py::arg("pose"))

      .def_property("front_face", &RenderShape::getFrontFace, &RenderShape::setFrontFace)
      .def("get_front_face", &RenderShape::getFrontFace)
      .def("set_front_face", &RenderShape::setFrontFace, py::arg("front_face"))

      .def_property_readonly("per_scene_id", &RenderShape::getRenderId)
      .def("get_per_scene_id", &RenderShape::getRenderId)
      .def_property("name", &RenderShape::getName, &RenderShape::setName)
      .def("get_name", &RenderShape::getName)
      .def("set_name", &RenderShape::setName, py::arg("name"))

      // TODO: set material
      .def_property_readonly("material", &RenderShape::getMaterial)
      .def("get_material", &RenderShape::getMaterial)

      .def("set_gpu_pose_batch_index", &RenderShape::setGpuBatchedPoseIndex, py::arg("index"))

      .def_property_readonly("parts", &RenderShape::getParts)
      .def("get_parts", &RenderShape::getParts)
      .def("clone", &RenderShape::clone);

  PyRenderShapePrimitive.def_property_readonly("vertices", &RenderShapePrimitive::getVertices)
      .def("get_vertices", &RenderShapePrimitive::getVertices)
      .def_property_readonly("triangles", &RenderShapePrimitive::getTriangles)
      .def("get_triangles", &RenderShapePrimitive::getTriangles)
      .def_property_readonly("vertex_normal", &RenderShapePrimitive::getNormal)
      .def("get_vertex_normal", &RenderShapePrimitive::getNormal)
      .def_property_readonly("vertex_uv", &RenderShapePrimitive::getUV)
      .def("get_vertex_uv", &RenderShapePrimitive::getUV);

  PyRenderShapePlane
      .def(py::init<Vec3, std::shared_ptr<SapienRenderMaterial>>(), py::arg("scale"),
           py::arg("material"))
      .def_property_readonly("scale", &RenderShapePlane::getScale)
      .def("get_scale", &RenderShapePlane::getScale);

  PyRenderShapeBox
      .def(py::init<Vec3, std::shared_ptr<SapienRenderMaterial>>(), py::arg("half_size"),
           py::arg("material"))
      .def_property_readonly("half_size", &RenderShapeBox::getHalfLengths)
      .def("get_half_size", &RenderShapeBox::getHalfLengths);

  PyRenderShapeSphere
      .def(py::init<float, std::shared_ptr<SapienRenderMaterial>>(), py::arg("radius"),
           py::arg("material"))
      .def_property_readonly("radius", &RenderShapeSphere::getRadius)
      .def("get_radius", &RenderShapeSphere::getRadius);

  PyRenderShapeCapsule
      .def(py::init<float, float, std::shared_ptr<SapienRenderMaterial>>(), py::arg("radius"),
           py::arg("half_length"), py::arg("material"))
      .def_property_readonly("half_length", &RenderShapeCapsule::getHalfLength)
      .def("get_half_length", &RenderShapeCapsule::getHalfLength)
      .def_property_readonly("radius", &RenderShapeCapsule::getRadius)
      .def("get_radius", &RenderShapeCapsule::getRadius);

  PyRenderShapeCylinder
      .def(py::init<float, float, std::shared_ptr<SapienRenderMaterial>>(), py::arg("radius"),
           py::arg("half_length"), py::arg("material"))
      .def_property_readonly("half_length", &RenderShapeCylinder::getHalfLength)
      .def("get_half_length", &RenderShapeCylinder::getHalfLength)
      .def_property_readonly("radius", &RenderShapeCylinder::getRadius)
      .def("get_radius", &RenderShapeCylinder::getRadius);

  PyRenderShapeTriangleMeshPart
      .def_property_readonly("vertices", &RenderShapeTriangleMeshPart::getVertices)
      .def("get_vertices", &RenderShapeTriangleMeshPart::getVertices)
      .def_property_readonly("triangles", &RenderShapeTriangleMeshPart::getTriangles)
      .def("get_triangles", &RenderShapeTriangleMeshPart::getTriangles)
      .def_property_readonly("cuda_vertices",
                             &RenderShapeTriangleMeshPart::getVertexBufferCudaArray)
      .def("get_vertex_normal", &RenderShapeTriangleMeshPart::getNormal)
      .def("set_vertex_normal", &RenderShapeTriangleMeshPart::setNormal, py::arg("normal"))
      .def("get_vertex_uv", &RenderShapeTriangleMeshPart::getUV)
      .def("set_vertex_uv", &RenderShapeTriangleMeshPart::setUV, py::arg("uv"))

      .def("get_cuda_vertices", &RenderShapeTriangleMeshPart::getVertexBufferCudaArray)
      .def_property_readonly("cuda_triangles",
                             &RenderShapeTriangleMeshPart::getIndexBufferCudaArray)
      .def("get_cuda_triangles", &RenderShapeTriangleMeshPart::getIndexBufferCudaArray)
      .def_property_readonly("material", &RenderShapeTriangleMeshPart::getMaterial)
      .def("get_material", &RenderShapeTriangleMeshPart::getMaterial)
      .def("__eq__", [](RenderShapeTriangleMeshPart &self, RenderShapeTriangleMeshPart &other) {
        return self.getShape() == other.getShape();
      });

  PyRenderShapeTriangleMesh
      .def(py::init<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &,
                    Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor> const &,
                    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &,
                    Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor> const &,
                    std::shared_ptr<SapienRenderMaterial>>(),
           py::arg("vertices"), py::arg("triangles"), py::arg("normals"), py::arg("uvs"),
           py::arg("material"))
      .def(py::init<std::string const &, Vec3, std::shared_ptr<SapienRenderMaterial>>(),
           py::arg("filename"), py::arg("scale") = Vec3(1.f), py::arg("material") = nullptr)
      .def_property_readonly("filename", &RenderShapeTriangleMesh::getFilename)
      .def("get_filename", &RenderShapeTriangleMesh::getFilename)

      .def_property("scale", &RenderShapeTriangleMesh::getScale,
                    &RenderShapeTriangleMesh::setScale)
      .def("get_scale", &RenderShapeTriangleMesh::getScale)
      .def("set_scale", &RenderShapeTriangleMesh::setScale, py::arg("scale"),
           "Note: this function only works when the shape is not added to scene");

  PyRenderBodyComponent.def(py::init<>())
      .def("attach", &SapienRenderBodyComponent::attachRenderShape, py::arg("shape"))
      .def_property_readonly("render_shapes", &SapienRenderBodyComponent::getRenderShapes)
      .def_property_readonly("_internal_node", &SapienRenderBodyComponent::internalGetNode)
      .def_property("visibility", &SapienRenderBodyComponent::getVisibility,
                    &SapienRenderBodyComponent::setVisibility)
      .def("set_property",
           py::overload_cast<const std::string &, Vec3 const &>(
               &SapienRenderBodyComponent::setProperty),
           py::arg("name"), py::arg("value"))
      .def("set_property",
           py::overload_cast<const std::string &, float>(&SapienRenderBodyComponent::setProperty),
           py::arg("name"), py::arg("value"))
      .def("set_property",
           py::overload_cast<const std::string &, int>(&SapienRenderBodyComponent::setProperty),
           py::arg("name"), py::arg("value"))
      .def("set_texture", &SapienRenderBodyComponent::setTexture, py::arg("name"),
           py::arg("texture"))
      .def("set_texture_array", &SapienRenderBodyComponent::setTextureArray, py::arg("name"),
           py::arg("textures"))

      .def_property("shading_mode", &SapienRenderBodyComponent::getShadingMode,
                    &SapienRenderBodyComponent::setShadingMode)

      .def("compute_global_aabb_tight", &SapienRenderBodyComponent::computeGlobalAABBTight)
      .def("get_global_aabb_fast", &SapienRenderBodyComponent::getGlobalAABBFast)

      .def_property_readonly("is_render_id_disabled",
                             &SapienRenderBodyComponent::getRenderIdDisabled)
      .def("disable_render_id", &SapienRenderBodyComponent::disableRenderId)
      .def("enable_render_id", &SapienRenderBodyComponent::enableRenderId)
      .def("clone", &SapienRenderBodyComponent::clone);

  PyRenderPointCloudComponent.def(py::init<uint32_t>(), py::arg("capacity") = 0)
      .def("set_vertices", &PointCloudComponent::setVertices, py::arg("vertices"))
      .def("get_vertices", &PointCloudComponent::getVertices,
           "Get previously set vertices. This function does not reflect any changes directly made "
           "to the GPU.")
      .def("set_attribute", &PointCloudComponent::setAttribute, py::arg("name"),
           py::arg("attribute"))
      .def("get_cuda_vertices", &PointCloudComponent::getCudaArray)
      .def("get_cuda_aabb", &PointCloudComponent::getCudaAABBArray,
           "this function is a temporary hack to help update the AABBs used for ray tracing BLAS. "
           "returns None if ray tracing has not been initialized");

  PyRenderCameraComponent
      .def(py::init<uint32_t, uint32_t, std::string const &>(), py::arg("width"),
           py::arg("height"), py::arg("shader_dir") = "")

      .def_property("local_pose", &SapienRenderCameraComponent::getLocalPose,
                    &SapienRenderCameraComponent::setLocalPose)
      .def("get_local_pose", &SapienRenderCameraComponent::getLocalPose)
      .def("set_local_pose", &SapienRenderCameraComponent::setLocalPose, py::arg("pose"))

      .def_property_readonly("global_pose", &SapienRenderCameraComponent::getGlobalPose)
      .def("get_global_pose", &SapienRenderCameraComponent::getGlobalPose)

      .def_property_readonly("width", &SapienRenderCameraComponent::getWidth)
      .def("get_width", &SapienRenderCameraComponent::getWidth)

      .def_property_readonly("height", &SapienRenderCameraComponent::getHeight)
      .def("get_height", &SapienRenderCameraComponent::getHeight)

      .def_property("near", &SapienRenderCameraComponent::getNear,
                    &SapienRenderCameraComponent::setNear)
      .def("get_near", &SapienRenderCameraComponent::getNear)
      .def("set_near", &SapienRenderCameraComponent::setNear, py::arg("near"))
      .def_property("far", &SapienRenderCameraComponent::getFar,
                    &SapienRenderCameraComponent::setFar)
      .def("get_far", &SapienRenderCameraComponent::getFar)
      .def("set_far", &SapienRenderCameraComponent::setFar, py::arg("far"))

      .def_property_readonly("fovx", &SapienRenderCameraComponent::getFovX)
      .def_property_readonly("fovy", &SapienRenderCameraComponent::getFovY)
      .def("set_fovx", &SapienRenderCameraComponent::setFovX, py::arg("fov"),
           py::arg("compute_y") = true)
      .def("set_fovy", &SapienRenderCameraComponent::setFovY, py::arg("fov"),
           py::arg("compute_x") = true)

      .def_property_readonly("fx", &SapienRenderCameraComponent::getFocalLengthX)
      .def_property_readonly("fy", &SapienRenderCameraComponent::getFocalLengthY)
      .def("set_focal_lengths", &SapienRenderCameraComponent::setFocalLengths, py::arg("fx"),
           py::arg("fy"))

      .def_property_readonly("cx", &SapienRenderCameraComponent::getPrincipalPointX)
      .def_property_readonly("cy", &SapienRenderCameraComponent::getPrincipalPointY)
      .def("set_principal_point", &SapienRenderCameraComponent::setPrincipalPoint, py::arg("cx"),
           py::arg("cy"))

      .def_property("skew", &SapienRenderCameraComponent::getSkew,
                    &SapienRenderCameraComponent::setSkew)
      .def("get_skew", &SapienRenderCameraComponent::getSkew)
      .def("set_skew", &SapienRenderCameraComponent::setSkew, py::arg("skew"))

      .def_property_readonly("mode", &SapienRenderCameraComponent::getMode)
      .def("get_mode", &SapienRenderCameraComponent::getMode)
      .def("set_perspective_parameters", &SapienRenderCameraComponent::setPerspectiveParameters,
           py::arg("near"), py::arg("far"), py::arg("fx"), py::arg("fy"), py::arg("cx"),
           py::arg("cy"), py::arg("skew"))

      .def("set_orthographic_parameters",
           py::overload_cast<float, float, float>(
               &SapienRenderCameraComponent::setOrthographicParameters),
           py::arg("near"), py::arg("far"), py::arg("top"))
      .def("set_orthographic_parameters",
           py::overload_cast<float, float, float, float, float, float>(
               &SapienRenderCameraComponent::setOrthographicParameters),
           py::arg("near"), py::arg("far"), py::arg("left"), py::arg("right"), py::arg("bottom"),
           py::arg("top"))
      .def_property_readonly("ortho_left", &SapienRenderCameraComponent::getOrthoLeft)
      .def_property_readonly("ortho_right", &SapienRenderCameraComponent::getOrthoRight)
      .def_property_readonly("ortho_bottom", &SapienRenderCameraComponent::getOrthoBottom)
      .def_property_readonly("ortho_top", &SapienRenderCameraComponent::getOrthoTop)

      .def("get_intrinsic_matrix", &SapienRenderCameraComponent::getIntrinsicMatrix,
           "Get 3x3 intrinsic camera matrix in OpenCV format.")
      .def("get_extrinsic_matrix", &SapienRenderCameraComponent::getExtrinsicMatrix,
           "Get 3x4 extrinsic camera matrix in OpenCV format.")
      .def("get_model_matrix", &SapienRenderCameraComponent::getModelMatrix,
           "Get model matrix (inverse of extrinsic matrix) used in rendering (Y up, Z back)")
      .def("get_projection_matrix", &SapienRenderCameraComponent::getProjectionMatrix,
           "Get projection matrix in used in rendering (right-handed NDC with [-1,1] XY and "
           "[0,1] "
           "Z)")

      .def(
          "set_property",
          py::overload_cast<const std::string &, float>(&SapienRenderCameraComponent::setProperty),
          py::arg("name"), py::arg("value"))
      .def("set_property",
           py::overload_cast<const std::string &, int>(&SapienRenderCameraComponent::setProperty),
           py::arg("name"), py::arg("value"))
      .def("set_texture", &SapienRenderCameraComponent::setTexture, py::arg("name"),
           py::arg("texture"))
      .def("set_texture_array", &SapienRenderCameraComponent::setTextureArray, py::arg("name"),
           py::arg("textures"))

      .def("take_picture", &SapienRenderCameraComponent::takePicture)
      .def("get_picture_names", &SapienRenderCameraComponent::getImageNames)

      .def(
          "get_picture",
          [](SapienRenderCameraComponent &c, std::string const &name) {
            return CpuArrayHandle(c.getImage(name));
          },
          py::arg("name"))

      // .def("gpu_init", &SapienRenderCameraComponent::gpuInit,
      //      "Do rendering once to ensure all GPU resources for this camera is initialized")
      .def_property_readonly(
          "_cuda_buffer", &SapienRenderCameraComponent::getCudaBuffer,
          "Debug only. Get the CUDA buffer containing GPU data for this camera, including "
          "transformaion matrices, sizes, and user-defined shader fields.")
      .def("set_gpu_pose_batch_index", &SapienRenderCameraComponent::setGpuBatchedPoseIndex,
           py::arg("index"))

      .def(
          "get_picture_cuda",
          [](SapienRenderCameraComponent &c, std::string const &name) {
            return CudaArrayHandle(c.getImageCuda(name));
          },
          py::arg("name"),
          R"doc(
This function transfers the rendered image into a CUDA buffer.

Usage:

# use torch backend
sapien.set_cuda_tensor_backend("torch")  # called once per process
image: torch.Tensor = camera.get_picture_cuda()

# use default backend
image = camera.get_picture_cuda()
torch_tensor = torch.as_tensor(image)

Warning: The camera must not be destroyed when the GPU tensor is in use by the
consumer library. Make a copy if needed.
)doc")

      .def_property_readonly(
          "_internal_renderer",
          [](SapienRenderCameraComponent &cam) { return &cam.getInternalRenderer(); },
          py::return_value_policy::reference);

  PyRenderLightComponent
      .def_property("local_pose", &SapienRenderLightComponent::getLocalPose,
                    &SapienRenderLightComponent::setLocalPose)
      .def("get_local_pose", &SapienRenderLightComponent::getLocalPose)
      .def("set_local_pose", &SapienRenderLightComponent::setLocalPose, py::arg("pose"))

      .def_property_readonly("global_pose", &SapienRenderLightComponent::getGlobalPose)
      .def("get_global_pose", &SapienRenderLightComponent::getGlobalPose)

      .def_property("color", &SapienRenderLightComponent::getColor,
                    &SapienRenderLightComponent::setColor)
      .def("get_color", &SapienRenderLightComponent::getColor)
      .def("set_color", &SapienRenderLightComponent::setColor, py::arg("color"))

      .def_property("shadow", &SapienRenderLightComponent::getShadowEnabled,
                    &SapienRenderLightComponent::setShadowEnabled)
      .def("enable_shadow", &SapienRenderLightComponent::enableShadow)
      .def("disable_shadow", &SapienRenderLightComponent::disableShadow)

      .def_property("shadow_near", &SapienRenderLightComponent::getShadowNear,
                    &SapienRenderLightComponent::setShadowNear)
      .def("get_shadow_near", &SapienRenderLightComponent::getShadowNear)
      .def("set_shadow_near", &SapienRenderLightComponent::setShadowNear, py::arg("near"))

      .def_property("shadow_far", &SapienRenderLightComponent::getShadowFar,
                    &SapienRenderLightComponent::setShadowFar)
      .def("get_shadow_far", &SapienRenderLightComponent::getShadowFar)
      .def("set_shadow_far", &SapienRenderLightComponent::setShadowFar, py::arg("far"))

      .def_property("shadow_map_size", &SapienRenderLightComponent::getShadowMapSize,
                    &SapienRenderLightComponent::setShadowMapSize)
      .def("get_shadow_map_size", &SapienRenderLightComponent::getShadowMapSize)
      .def("set_shadow_map_size", &SapienRenderLightComponent::setShadowMapSize, py::arg("size"));

  PyRenderPointLightComponent.def(py::init<>());
  PyRenderDirectionalLightComponent.def(py::init<>())
      .def_property("shadow_half_size", &SapienRenderDirectionalLightComponent::getShadowHalfSize,
                    &SapienRenderDirectionalLightComponent::setShadowHalfSize)
      .def("get_shadow_half_size", &SapienRenderDirectionalLightComponent::getShadowHalfSize)
      .def("set_shadow_half_size", &SapienRenderDirectionalLightComponent::setShadowHalfSize,
           py::arg("size"));

  PyRenderSpotLightComponent.def(py::init<>())
      .def_property("inner_fov", &SapienRenderSpotLightComponent::getFovInner,
                    &SapienRenderSpotLightComponent::setFovInner)
      .def("get_inner_fov", &SapienRenderSpotLightComponent::getFovInner)
      .def("set_inner_fov", &SapienRenderSpotLightComponent::setFovInner, py::arg("fov"))

      .def_property("outer_fov", &SapienRenderSpotLightComponent::getFovOuter,
                    &SapienRenderSpotLightComponent::setFovOuter)
      .def("get_outer_fov", &SapienRenderSpotLightComponent::getFovOuter)
      .def("set_outer_fov", &SapienRenderSpotLightComponent::setFovOuter, py::arg("fov"));

  PyRenderTexturedLightComponent.def(py::init<>())
      .def_property("texture", &SapienRenderTexturedLightComponent::getTexture,
                    &SapienRenderTexturedLightComponent::setTexture)
      .def("get_texture", &SapienRenderTexturedLightComponent::getTexture)
      .def("set_texture", &SapienRenderTexturedLightComponent::setTexture, py::arg("texture"));

  PyRenderParallelogramLightComponent.def(py::init<>())
      .def("set_shape", &SapienRenderParallelogramLightComponent::setShape, py::arg("half_width"),
           py::arg("half_height"), py::arg("angle") = std::numbers::pi_v<float> / 2.f)

      .def_property_readonly("half_width", &SapienRenderParallelogramLightComponent::getHalfWidth)
      .def("get_half_width", &SapienRenderParallelogramLightComponent::getHalfWidth)
      .def_property_readonly("half_height",
                             &SapienRenderParallelogramLightComponent::getHalfHeight)
      .def("get_half_height", &SapienRenderParallelogramLightComponent::getHalfHeight)
      .def_property_readonly("angle", &SapienRenderParallelogramLightComponent::getAngle)
      .def("get_angle", &SapienRenderParallelogramLightComponent::getAngle);

  PyRenderCudaMeshComponent
      .def(py::init<uint32_t, uint32_t>(), py::arg("max_vertex_count"),
           py::arg("max_triangle_count"))
      .def_property("vertex_count", &CudaDeformableMeshComponent::getVertexCount,
                    &CudaDeformableMeshComponent::setVertexCount)
      .def("get_vertex_count", &CudaDeformableMeshComponent::getVertexCount)
      .def("set_vertex_count", &CudaDeformableMeshComponent::setVertexCount, py::arg("count"))
      .def("set_triangles", &CudaDeformableMeshComponent::setTriangles, py::arg("triangles"))

      .def_property("triangle_count", &CudaDeformableMeshComponent::getTriangleCount,
                    &CudaDeformableMeshComponent::setTriangleCount)
      .def("get_triangle_count", &CudaDeformableMeshComponent::getTriangleCount)
      .def("set_triangle_count", &CudaDeformableMeshComponent::setTriangleCount, py::arg("count"))

      .def_property_readonly("cuda_vertices",
                             &CudaDeformableMeshComponent::getVertexBufferCudaArray)
      .def("get_cuda_vertices", &CudaDeformableMeshComponent::getVertexBufferCudaArray)

      .def_property_readonly("cuda_triangles",
                             &CudaDeformableMeshComponent::getIndexBufferCudaArray)
      .def("get_cuda_triangles", &CudaDeformableMeshComponent::getIndexBufferCudaArray)

      .def_property("material", &CudaDeformableMeshComponent::getMaterial,
                    &CudaDeformableMeshComponent::setMaterial)
      .def("get_material", &CudaDeformableMeshComponent::getMaterial)
      .def("set_material", &CudaDeformableMeshComponent::setMaterial, py::arg("material"))
      .def("notify_vertex_updated", &CudaDeformableMeshComponent::notifyVertexUpdated,
           py::arg("cuda_stream") = 0);

  PyRenderWindow
      .def(py::init<int, int, std::string>(), py::arg("width"), py::arg("height"),
           py::arg("shader_dir"))
      .def("show", &SapienRendererWindow::show)
      .def("hide", &SapienRendererWindow::hide)
      .def_property_readonly("should_close", &SapienRendererWindow::windowCloseRequested)
      .def("set_camera_parameters", &SapienRendererWindow::setCameraParameters, py::arg("near"),
           py::arg("far"), py::arg("fovy"))
      .def("set_intrinsic_parameters", &SapienRendererWindow::setCameraIntrinsicParameters,
           py::arg("near"), py::arg("far"), py::arg("fx"), py::arg("fy"), py::arg("cx"),
           py::arg("cy"), py::arg("skew"))

      .def("set_camera_orthographic_parameters", &SapienRendererWindow::setCameraOrthoParameters,
           py::arg("near"), py::arg("far"), py::arg("top"))
      .def_property_readonly("camera_mode", &SapienRendererWindow::getCameraMode)
      .def_property_readonly("ortho_top", &SapienRendererWindow::getCameraOrthoTop)

      .def("set_camera_pose", &SapienRendererWindow::setCameraPose, py::arg("pose"))
      .def("set_camera_position", &SapienRendererWindow::setCameraPosition, py::arg("position"))
      .def("set_camera_rotation", &SapienRendererWindow::setCameraRotation, py::arg("quat"))

      .def("get_camera_property_int", &SapienRendererWindow::getCameraPropertyInt, py::arg("key"))
      .def("get_camera_property_float", &SapienRendererWindow::getCameraPropertyFloat,
           py::arg("key"))
      .def("set_camera_property",
           py::overload_cast<const std::string &, float>(&SapienRendererWindow::setCameraProperty),
           py::arg("key"), py::arg("value"))
      .def("set_camera_property",
           py::overload_cast<const std::string &, int>(&SapienRendererWindow::setCameraProperty),
           py::arg("key"), py::arg("value"))

      .def("set_camera_texture", &SapienRendererWindow::setCameraTexture, py::arg("name"),
           py::arg("texture"))
      .def("set_camera_texture_array", &SapienRendererWindow::setCameraTextureArray,
           py::arg("name"), py::arg("textures"))

      .def("set_shader_dir", &SapienRendererWindow::setShader, py::arg("shader_dir"))
      .def("get_content_scale", &SapienRendererWindow::getContentScale)
      .def("set_content_scale", &SapienRendererWindow::setContentScale, py::arg("scale"))
      .def("get_camera_position", &SapienRendererWindow::getCameraPosition)
      .def("get_camera_rotation", &SapienRendererWindow::getCameraRotation)
      .def("get_camera_pose", &SapienRendererWindow::getCameraPose)
      .def("get_camera_projection_matrix",
           [](SapienRendererWindow &window) {
             glm::mat4 proj = glm::transpose(window.getCameraProjectionMatrix());
             return py::array_t<float>({4, 4}, &proj[0][0]);
           })
      .def("get_camera_model_matrix", &SapienRendererWindow::getCameraModelMatrix)

      .def("update_render", &SapienRendererWindow::updateRender,
           "Equivalent to calling the update_render function for all added scene")
      .def_property_readonly(
          "_internal_renderer",
          [](SapienRendererWindow &window) { return window.getInternalRenderer(); },
          py::return_value_policy::reference)
      .def_property_readonly("_internal_scene", &SapienRendererWindow::getInternalScene,
                             py::return_value_policy::reference)

      .def("set_scene", &SapienRendererWindow::setScene, py::arg("scene"))
      .def("set_scenes", &SapienRendererWindow::setScenes, py::arg("scenes"), py::arg("offsets"))
      .def_property_readonly("display_picture_names", &SapienRendererWindow::getDisplayTargetNames,
                             "Names for available display targets that can be displayed "
                             "in the render function")
      .def("get_picture_size", &SapienRendererWindow::getRenderTargetSize, py::arg("name"))
      .def("render", &SapienRendererWindow::render, py::arg("target_name"),
           py::arg("ui_windows") = std::vector<std::shared_ptr<svulkan2::ui::Widget>>())
      .def("resize", &SapienRendererWindow::resize, py::arg("width"), py::arg("height"))
      .def_property_readonly("fps", &SapienRendererWindow::getFPS)
      .def_property_readonly("size", &SapienRendererWindow::getWindowSize)
      .def_property_readonly("fovy", &SapienRendererWindow::getCameraFovy)
      .def_property_readonly("near", &SapienRendererWindow::getCameraNear)
      .def_property_readonly("far", &SapienRendererWindow::getCameraFar)
      .def_property("cursor", &SapienRendererWindow::getCursorEnabled,
                    &SapienRendererWindow::setCursorEnabled)

      .def_property("denoiser", &SapienRendererWindow::getDenoiserType,
                    &SapienRendererWindow::setDenoiserType)

      .def(
          "get_picture",
          [](SapienRendererWindow &w, std::string const &name) {
            return CpuArrayHandle(w.getImage(name));
          },
          py::arg("name"))
      .def("get_picture_pixel", &SapienRendererWindow::getImagePixel, py::arg("name"),
           py::arg("x"), py::arg("y"))

      .def_property_readonly("shift", &SapienRendererWindow::isShiftDown)
      .def_property_readonly("alt", &SapienRendererWindow::isAltDown)
      .def_property_readonly("ctrl", &SapienRendererWindow::isCtrlDown)
      .def_property_readonly("super", &SapienRendererWindow::isSuperDown)

      .def("key_down", &SapienRendererWindow::isKeyDown, py::arg("key"))
      .def("key_press", &SapienRendererWindow::isKeyPressed, py::arg("key"))
      .def("mouse_down", &SapienRendererWindow::isMouseKeyDown, py::arg("key"))
      .def("mouse_click", &SapienRendererWindow::isMouseKeyClicked, py::arg("key"))
      .def_property_readonly("mouse_position", &SapienRendererWindow::getMousePosition)
      .def_property_readonly("mouse_delta", &SapienRendererWindow::getMouseDelta)
      .def_property_readonly("mouse_wheel_delta", &SapienRendererWindow::getMouseWheelDelta)
      .def("set_drop_callback", &SapienRendererWindow::setDropCallback, py::arg("callback"))
      .def("unset_drop_callback", &SapienRendererWindow::unsetDropCallback)
      .def("set_focus_callback", &SapienRendererWindow::setFocusCallback, py::arg("callback"))
      .def("unset_focus_callback", &SapienRendererWindow::unsetFocusCallback);

  PyRenderVRDisplay.def(py::init<>())
      .def("set_scene", &SapienVRDisplay::setScene, py::arg("scene"))
      .def("set_camera_parameters", &SapienVRDisplay::setCameraParameters, py::arg("near"),
           py::arg("far"))

      .def_property("root_pose", &SapienVRDisplay::getRootPose, &SapienVRDisplay::setRootPose)
      .def("get_root_pose", &SapienVRDisplay::getRootPose)
      .def("set_root_pose", &SapienVRDisplay::setRootPose, py::arg("pose"))

      .def("get_controller_ids", &SapienVRDisplay::getControllerIds)
      .def("fetch_poses", &SapienVRDisplay::fetchPoses, "fetches poses of HMD and controllers")
      .def(
          "get_hmd_pose", &SapienVRDisplay::getHMDPose,
          "Gets the local pose of the head set. It should be called immediately after fetch_poses")
      .def(
          "get_controller_pose", &SapienVRDisplay::getControllerPose, py::arg("id"),
          "Gets the local pose of a controller. It should be called immediately after fetch_poses")
      .def("get_left_hand_root_pose", &SapienVRDisplay::getLeftHandRootPose)
      .def("get_right_hand_root_pose", &SapienVRDisplay::getRightHandRootPose)
      .def("get_left_hand_skeletal_poses", &SapienVRDisplay::getLeftHandSkeletalPoses)
      .def("get_right_hand_skeletal_poses", &SapienVRDisplay::getRightHandSkeletalPoses)

      .def("update_render", &SapienVRDisplay::updateRender,
           "update_render implicitly calls fetch_poses to make sure the HMD pose is up-to-date")
      .def("render", &SapienVRDisplay::render)
      .def_property_readonly("_internal_scene", &SapienVRDisplay::getInternalScene,
                             py::return_value_policy::reference)

      .def("get_controller_button_pressed", &SapienVRDisplay::getControllerButtonPressed,
           py::arg("id"))
      .def("get_controller_button_touched", &SapienVRDisplay::getControllerButtonTouched,
           py::arg("id"))
      .def("get_controller_axis_state", &SapienVRDisplay::getControllerAxisState, py::arg("id"),
           py::arg("axis"));

  PyRenderSceneLoaderNode.def_readonly("name", &RenderSceneLoaderNode::name)
      .def_readonly("pose", &RenderSceneLoaderNode::pose)
      .def_readonly("mesh", &RenderSceneLoaderNode::mesh)
      .def_readonly("light", &RenderSceneLoaderNode::light)
      .def_readonly("scale", &RenderSceneLoaderNode::scale)
      .def_readonly("children", &RenderSceneLoaderNode::children)
      .def("flatten", &RenderSceneLoaderNode::flatten);
}
