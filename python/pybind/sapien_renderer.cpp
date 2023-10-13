#include "sapien/component/sapien_renderer/sapien_renderer.h"
#include "array.hpp"
#include "format.hpp"
#include "sapien_type_caster.h"
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/smart_holder.h>
#include <pybind11/stl.h>

namespace py = pybind11;

using namespace pybind11::literals;
using namespace sapien;
using namespace sapien::component;

namespace pybind11::detail {
template <> struct type_caster<svulkan2::renderer::RTRenderer::DenoiserType> {
  PYBIND11_TYPE_CASTER(svulkan2::renderer::RTRenderer::DenoiserType,
                       _("typing.Liternal['none', 'oidn', 'optix']"));

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

  ////////// global //////////

  m.def("_internal_set_shader_search_path", &SapienRendererDefault::internalSetShaderSearchPath)
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
            SapienRenderEngine::Get()->getResourceManager()->clearCachedResources(models, images,
                                                                                  shaders);
          },
          py::arg("models") = true, py::arg("images") = true, py::arg("shaders") = false)

      .def("load_scene", &LoadScene, py::arg("filename"));

  ////////// end global //////////

  auto PySapienRenderer = py::class_<SapienRenderEngine>(m, "SapienRenderer");

  auto PyRenderTexture = py::class_<SapienRenderTexture>(m, "RenderTexture");

  auto PyRenderTexture2D = py::class_<SapienRenderTexture2D>(m, "RenderTexture2D");

  auto PyRenderTargetCuda = py::class_<SapienRenderImageCuda>(m, "RenderImageCuda");

  auto PyRenderCubemap = py::class_<SapienRenderCubemap>(m, "RenderCubemap");

  auto PyRenderMaterial = py::class_<SapienRenderMaterial>(m, "RenderMaterial");

  auto PyRenderShapeTriangleMeshPart =
      py::class_<RenderShapeTriangleMeshPart>(m, "RenderShapeTriangleMeshPart");

  auto PyRenderShape = py::class_<RenderShape>(m, "RenderShape");
  auto PyRenderShapePlane = py::class_<RenderShapePlane, RenderShape>(m, "RenderShapePlane");
  auto PyRenderShapeBox = py::class_<RenderShapeBox, RenderShape>(m, "RenderShapeBox");
  auto PyRenderShapeSphere = py::class_<RenderShapeSphere, RenderShape>(m, "RenderShapeSphere");
  auto PyRenderShapeCapsule = py::class_<RenderShapeCapsule, RenderShape>(m, "RenderShapeCapsule");
  auto PyRenderShapeTriangleMesh =
      py::class_<RenderShapeTriangleMesh, RenderShape>(m, "RenderShapeTriangleMesh");

  auto PyRenderSystem = py::class_<SapienRendererSystem, System>(m, "RenderSystem");

  auto PyRenderBodyComponent =
      py::class_<SapienRenderBodyComponent, Component>(m, "RenderBodyComponent");
  auto PyRenderCameraComponent =
      py::class_<SapienRenderCameraComponent, Component>(m, "RenderCameraComponent");
  auto PyRenderLightComponent =
      py::class_<SapienRenderLightComponent, Component>(m, "RenderLightComponent");
  auto PyRenderPointLightComponent =
      py::class_<SapienRenderPointLightComponent, SapienRenderLightComponent>(
          m, "RenderPointLightComponent");
  auto PyRenderDirectionalLightComponent =
      py::class_<SapienRenderDirectionalLightComponent, SapienRenderLightComponent>(
          m, "RenderDirectionalLightComponent");
  auto PyRenderSpotLightComponent =
      py::class_<SapienRenderSpotLightComponent, SapienRenderLightComponent>(
          m, "RenderSpotLightComponent");
  auto PyRenderTexturedLightComponent =
      py::class_<SapienRenderTexturedLightComponent, SapienRenderSpotLightComponent>(
          m, "RenderTexturedLightComponent");
  auto PyRenderParallelogramLightComponent =
      py::class_<SapienRenderParallelogramLightComponent, SapienRenderLightComponent>(
          m, "RenderParallelogramLightComponent");
  auto PyRenderCudaMeshComponent =
      py::class_<CudaDeformableMeshComponent, Component>(m, "RenderCudaMeshComponent");

  auto PyRenderWindow = py::class_<SapienRendererWindow>(m, "RenderWindow");

  PySapienRenderer
      .def(py::init(&SapienRenderEngine::Get), py::arg("offscreen_only") = false,
           py::arg("max_num_materials") = 128, py::arg("max_num_textures") = 512,
           py::arg("default_mipmap_levels") = 1, py::arg("default_device") = "",
           py::arg("do_not_load_texture") = false)
      .def_property_readonly("_internal_context", &SapienRenderEngine::getContext);

  PyRenderSystem.def(py::init<>())
      .def_property("ambient_light", &SapienRendererSystem::getAmbientLight,
                    &SapienRendererSystem::setAmbientLight)
      .def("get_ambient_light", &SapienRendererSystem::getAmbientLight)
      .def("set_ambient_light", &SapienRendererSystem::setAmbientLight, py::arg("color"))
      .def_property_readonly("_internal_scene", &SapienRendererSystem::getScene)
      .def_property_readonly("cameras", &SapienRendererSystem::getCameraComponents)
      .def_property_readonly("render_bodies", &SapienRendererSystem::getRenderBodyComponents)
      .def_property_readonly("lights", &SapienRendererSystem::getLightComponents)
      .def_property("cubemap", &SapienRendererSystem::getCubemap,
                    &SapienRendererSystem::setCubemap)
      .def("get_cubemap", &SapienRendererSystem::getCubemap)
      .def("set_cubemap", &SapienRendererSystem::setCubemap, py::arg("cubemap"));

  PyRenderTexture
      .def(py::init(&CreateSapienTexture), py::arg("array"), py::arg("dim"), py::arg("format"),
           py::arg("mipmap_levels") = 1,
           py::arg("filter_mode") = SapienRenderTexture::FilterMode::eLINEAR,
           py::arg("address_mode") = SapienRenderTexture::AddressMode::eREPEAT,
           py::arg("srgb") = false)
      .def_property_readonly("width", &SapienRenderTexture::getWidth)
      .def_property_readonly("height", &SapienRenderTexture::getHeight)
      .def_property_readonly("depth", &SapienRenderTexture::getDepth)
      .def_property_readonly("channels", &SapienRenderTexture::getChannels)
      .def_property_readonly("mipmap_levels", &SapienRenderTexture::getMipmapLevels)
      .def_property_readonly("address_mode", &SapienRenderTexture::getAddressMode)
      .def_property_readonly("filter_mode", &SapienRenderTexture::getFilterMode)
      .def_property_readonly("format", &SapienRenderTexture::getFormat)
      .def_property_readonly("srgb", &SapienRenderTexture::getIsSrgb);

  PyRenderTexture2D
      .def(py::init<std::string, uint32_t, SapienRenderTexture::FilterMode,
                    SapienRenderTexture::AddressMode>(),
           py::arg("filename"), py::arg("mipmap_levels") = 1,
           py::arg("filter_mode") = SapienRenderTexture::FilterMode::eLINEAR,
           py::arg("address_mode") = SapienRenderTexture::AddressMode::eREPEAT)
      .def_property_readonly("width", &SapienRenderTexture2D::getWidth)
      .def_property_readonly("height", &SapienRenderTexture2D::getHeight)
      .def_property_readonly("channels", &SapienRenderTexture2D::getChannels)
      .def_property_readonly("mipmap_levels", &SapienRenderTexture2D::getMipmapLevels)

      .def_property_readonly("address_mode", &SapienRenderTexture2D::getAddressMode)
      .def_property_readonly("filter_mode", &SapienRenderTexture2D::getFilterMode)
      .def_property_readonly("filename", &SapienRenderTexture2D::getFilename)
      .def_property_readonly("format", &SapienRenderTexture2D::getFormat);

  PyRenderTargetCuda.def_readonly("shape", &SapienRenderImageCuda::shape)
      .def_readonly("strides", &SapienRenderImageCuda::strides)
      .def_readonly("cuda_id", &SapienRenderImageCuda::cudaId)
      .def_readonly("typstr", &SapienRenderImageCuda::type)
      .def_property_readonly(
          "ptr",
          [](SapienRenderImageCuda &array) { return reinterpret_cast<intptr_t>(array.ptr); })
      .def_property_readonly("__cuda_array_interface__", [](SapienRenderImageCuda &array) {
        py::tuple shape = py::cast(array.shape);
        py::tuple strides = py::cast(array.strides);

        return py::dict("shape"_a = shape, "strides"_a = strides, "typestr"_a = array.type,
                        "data"_a = py::make_tuple(reinterpret_cast<intptr_t>(array.ptr), false),
                        "version"_a = 2);
      });

  PyRenderCubemap.def(py::init<std::string const &>(), py::arg("filename"))
      .def(py::init<std::string const &, std::string const &, std::string const &,
                    std::string const &, std::string const &, std::string const &>(),
           py::arg("px"), py::arg("nx"), py::arg("py"), py::arg("ny"), py::arg("pz"),
           py::arg("nz"))
      .def("export", &SapienRenderCubemap::exportKtx, py::arg("filename"));

  PyRenderMaterial
      .def(py::init<std::array<float, 4>, std::array<float, 4>, float, float, float, float, float,
                    float>(),
           py::arg("emission") = std::array<float, 4>{0.f, 0.f, 0.f, 0.f},
           py::arg("base_color") = std::array<float, 4>{1.f, 1.f, 1.f, 1.f},
           py::arg("specular") = 0.f, py::arg("roughness") = 1.f, py::arg("metallic") = 0.f,
           py::arg("transmission") = 0.f, py::arg("ior") = 1.45f,
           py::arg("transmission_roughness") = 0.f)

      // getter
      .def("get_base_color", &SapienRenderMaterial::getBaseColor)
      .def("get_emission", &SapienRenderMaterial::getEmission)
      .def("get_specular", &SapienRenderMaterial::getSpecular)
      .def("get_metallic", &SapienRenderMaterial::getMetallic)
      .def("get_roughness", &SapienRenderMaterial::getRoughness)
      .def("get_transmission", &SapienRenderMaterial::getTransmission)
      .def("get_ior", &SapienRenderMaterial::getIOR)
      .def("get_transmission_roughness", &SapienRenderMaterial::getTransmissionRoughness)

      .def("get_emission_texture", &SapienRenderMaterial::getEmissionTexture)
      .def("get_diffuse_texture", &SapienRenderMaterial::getDiffuseTexture)
      .def("get_metallic_texture", &SapienRenderMaterial::getMetallicTexture)
      .def("get_roughness_texture", &SapienRenderMaterial::getRoughnessTexture)
      .def("get_normal_texture", &SapienRenderMaterial::getNormalTexture)
      .def("get_transmission_texture", &SapienRenderMaterial::getTransmissionTexture)

      // setter
      .def("set_base_color", &SapienRenderMaterial::setBaseColor)
      .def("set_emission", &SapienRenderMaterial::setEmission)
      .def("set_specular", &SapienRenderMaterial::setSpecular)
      .def("set_metallic", &SapienRenderMaterial::setMetallic)
      .def("set_roughness", &SapienRenderMaterial::setRoughness)
      .def("set_transmission", &SapienRenderMaterial::setTransmission)
      .def("set_ior", &SapienRenderMaterial::setIOR)
      .def("set_transmission_roughness", &SapienRenderMaterial::setTransmissionRoughness)

      .def("set_emission_texture", &SapienRenderMaterial::setEmissionTexture)
      .def("set_diffuse_texture", &SapienRenderMaterial::setDiffuseTexture)
      .def("set_metallic_texture", &SapienRenderMaterial::setMetallicTexture)
      .def("set_roughness_texture", &SapienRenderMaterial::setRoughnessTexture)
      .def("set_normal_texture", &SapienRenderMaterial::setNormalTexture)
      .def("set_transmission_texture", &SapienRenderMaterial::setTransmissionTexture)

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
                    &SapienRenderMaterial::setDiffuseTexture)
      .def("get_emission_texture", &SapienRenderMaterial::getEmissionTexture)
      .def("set_emission_texture", &SapienRenderMaterial::setDiffuseTexture, py::arg("texture"))
      .def_property("diffuse_texture", &SapienRenderMaterial::getDiffuseTexture,
                    &SapienRenderMaterial::setDiffuseTexture)
      .def("get_diffuse_texture", &SapienRenderMaterial::getDiffuseTexture)
      .def("set_diffuse_texture", &SapienRenderMaterial::setDiffuseTexture, py::arg("texture"))
      .def_property("metallic_texture", &SapienRenderMaterial::getMetallicTexture,
                    &SapienRenderMaterial::setMetallicTexture)
      .def("get_metallic_texture", &SapienRenderMaterial::getMetallicTexture)
      .def("set_metallic_texture", &SapienRenderMaterial::setMetallicTexture, py::arg("texture"))
      .def_property("roughness_texture", &SapienRenderMaterial::getRoughnessTexture,
                    &SapienRenderMaterial::setRoughnessTexture)
      .def("get_roughness_texture", &SapienRenderMaterial::getRoughnessTexture)
      .def("set_roughness_texture", &SapienRenderMaterial::setRoughnessTexture, py::arg("texture"))
      .def_property("normal_texture", &SapienRenderMaterial::getNormalTexture,
                    &SapienRenderMaterial::setDiffuseTexture)
      .def("get_normal_texture", &SapienRenderMaterial::getNormalTexture)
      .def("set_normal_texture", &SapienRenderMaterial::setDiffuseTexture, py::arg("texture"))
      .def_property("transmission_texture", &SapienRenderMaterial::getTransmissionTexture,
                    &SapienRenderMaterial::setTransmissionTexture)
      .def("get_transmission_texture", &SapienRenderMaterial::getTransmissionTexture)
      .def("set_transmission_texture", &SapienRenderMaterial::setTransmissionTexture,
           py::arg("texture"));

  PyRenderShape.def_property("local_pose", &RenderShape::getLocalPose, &RenderShape::setLocalPose);
  PyRenderShapePlane
      .def(py::init<Vec3, std::shared_ptr<SapienRenderMaterial>>(), py::arg("scale"),
           py::arg("material"))
      .def_property_readonly("scale", &RenderShapePlane::getScale);
  PyRenderShapeBox
      .def(py::init<Vec3, std::shared_ptr<SapienRenderMaterial>>(), py::arg("half_size"),
           py::arg("material"))
      .def_property_readonly("half_size", &RenderShapeBox::getHalfLengths);
  PyRenderShapeSphere
      .def(py::init<float, std::shared_ptr<SapienRenderMaterial>>(), py::arg("radius"),
           py::arg("material"))
      .def_property_readonly("radius", &RenderShapeSphere::getRadius);
  PyRenderShapeCapsule
      .def(py::init<float, float, std::shared_ptr<SapienRenderMaterial>>(), py::arg("radius"),
           py::arg("half_length"), py::arg("material"))
      .def_property_readonly("half_length", &RenderShapeCapsule::getHalfLength)
      .def_property_readonly("radius", &RenderShapeCapsule::getRadius);

  PyRenderShapeTriangleMeshPart
      .def_property_readonly("material", &RenderShapeTriangleMeshPart::getMaterial)
      .def_property_readonly("vertices", &RenderShapeTriangleMeshPart::getVertices)
      .def_property_readonly("triangles", &RenderShapeTriangleMeshPart::getTriangles)
      .def_property_readonly("cuda_vertices",
                             &RenderShapeTriangleMeshPart::getVertexBufferCudaArray)
      .def_property_readonly("cuda_triangles",
                             &RenderShapeTriangleMeshPart::getIndexBufferCudaArray);

  PyRenderShapeTriangleMesh
      .def(py::init<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &,
                    Eigen::Matrix<uint32_t, Eigen::Dynamic, 3, Eigen::RowMajor> const &,
                    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &,
                    std::shared_ptr<SapienRenderMaterial>>(),
           py::arg("vertices"), py::arg("triangles"), py::arg("normals"), py::arg("material"))
      .def(py::init<std::string const &, Vec3, std::shared_ptr<SapienRenderMaterial>>(),
           py::arg("filename"), py::arg("scale") = Vec3(1.f), py::arg("material") = nullptr)
      .def_property_readonly("filename", &RenderShapeTriangleMesh::getFilename)

      .def_property("scale", &RenderShapeTriangleMesh::getScale,
                    &RenderShapeTriangleMesh::setScale)
      .def("get_scale", &RenderShapeTriangleMesh::getScale)
      .def("set_scale", &RenderShapeTriangleMesh::setScale, py::arg("scale"),
           "Note: this function only works when the shape is not added to scene")

      .def_property_readonly("parts", &RenderShapeTriangleMesh::getParts);

  PyRenderBodyComponent.def(py::init<>())
      .def("attach", &SapienRenderBodyComponent::attachRenderShape)
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
                    &SapienRenderBodyComponent::setShadingMode);

  PyRenderCameraComponent
      .def(py::init<uint32_t, uint32_t, std::string const &>(), py::arg("width"),
           py::arg("height"), py::arg("shader_dir") = "")
      .def_property("local_pose", &SapienRenderCameraComponent::getLocalPose,
                    &SapienRenderCameraComponent::setLocalPose)
      .def_property_readonly("global_pose", &SapienRenderCameraComponent::getGlobalPose)
      .def_property_readonly("width", &SapienRenderCameraComponent::getWidth)
      .def_property_readonly("height", &SapienRenderCameraComponent::getHeight)

      .def_property("near", &SapienRenderCameraComponent::getNear,
                    &SapienRenderCameraComponent::setNear)
      .def_property("far", &SapienRenderCameraComponent::getFar,
                    &SapienRenderCameraComponent::setFar)

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
      .def("set_perspective_parameters", &SapienRenderCameraComponent::setPerspectiveParameters,
           py::arg("near"), py::arg("far"), py::arg("fx"), py::arg("fy"), py::arg("cx"),
           py::arg("cy"), py::arg("skew"))

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

      .def("get_picture_cuda", &SapienRenderCameraComponent::getImageCuda, py::arg("name"),
           R"doc(
This function transfers the rendered image into a CUDA buffer.
The returned object implements __cuda_array_interface__

Usage:

image = camera.getImageCuda()
torch_tensor = torch.as_tensor(image)

Warning: The camera must not be destroyed when the GPU tensor is in use by the
consumer library. Make a copy if needed.
)doc");

  PyRenderLightComponent
      .def_property("local_pose", &SapienRenderLightComponent::getLocalPose,
                    &SapienRenderLightComponent::setLocalPose)
      .def_property_readonly("global_pose", &SapienRenderLightComponent::getGlobalPose)
      .def_property("color", &SapienRenderLightComponent::getColor,
                    &SapienRenderLightComponent::setColor)
      .def_property("shadow", &SapienRenderLightComponent::getShadowEnabled,
                    &SapienRenderLightComponent::setShadowEnabled)
      .def_property("shadow_near", &SapienRenderLightComponent::getShadowNear,
                    &SapienRenderLightComponent::setShadowNear)
      .def_property("shadow_far", &SapienRenderLightComponent::getShadowFar,
                    &SapienRenderLightComponent::setShadowFar)
      .def_property("shadow_map_size", &SapienRenderLightComponent::getShadowMapSize,
                    &SapienRenderLightComponent::setShadowMapSize);

  PyRenderPointLightComponent.def(py::init<>());
  PyRenderDirectionalLightComponent.def(py::init<>())
      .def_property("shadow_half_size", &SapienRenderDirectionalLightComponent::getShadowHalfSize,
                    &SapienRenderDirectionalLightComponent::setShadowHalfSize);
  PyRenderSpotLightComponent.def(py::init<>())
      .def_property("inner_fov", &SapienRenderSpotLightComponent::getFovInner,
                    &SapienRenderSpotLightComponent::setFovInner)
      .def_property("outer_fov", &SapienRenderSpotLightComponent::getFovOuter,
                    &SapienRenderSpotLightComponent::setFovOuter);
  PyRenderTexturedLightComponent.def(py::init<>())
      .def_property("texture", &SapienRenderTexturedLightComponent::getTexture,
                    &SapienRenderTexturedLightComponent::setTexture);
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
      .def("set_vertex_count", &CudaDeformableMeshComponent::setVertexCount)

      .def_property("triangle_count", &CudaDeformableMeshComponent::getTriangleCount,
                    &CudaDeformableMeshComponent::setTriangleCount)
      .def("get_triangle_count", &CudaDeformableMeshComponent::getTriangleCount)
      .def("set_triangle_count", &CudaDeformableMeshComponent::setTriangleCount)

      .def_property_readonly("cuda_vertices",
                             &CudaDeformableMeshComponent::getVertexBufferCudaArray)
      .def("get_cuda_vertices", &CudaDeformableMeshComponent::getVertexBufferCudaArray)

      .def_property_readonly("cuda_triangles",
                             &CudaDeformableMeshComponent::getIndexBufferCudaArray)
      .def("get_cuda_triangles", &CudaDeformableMeshComponent::getIndexBufferCudaArray)

      .def_property("material", &CudaDeformableMeshComponent::getMaterial,
                    &CudaDeformableMeshComponent::setMaterial)
      .def("getmaterial", &CudaDeformableMeshComponent::getMaterial)
      .def("set_material", &CudaDeformableMeshComponent::setMaterial, py::arg("material"))
      .def("set_data_source", &CudaDeformableMeshComponent::setDataSource,
           py::arg("vertex_provider"));

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
      .def("set_camera_pose", &SapienRendererWindow::setCameraPose)
      .def("set_camera_position", &SapienRendererWindow::setCameraPosition, py::arg("position"))
      .def("set_camera_rotation", &SapienRendererWindow::setCameraRotation, py::arg("quat"))

      .def("get_camera_property_int", &SapienRendererWindow::getCameraPropertyInt)
      .def("get_camera_property_float", &SapienRendererWindow::getCameraPropertyFloat)
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
      .def("get_camera_position", &SapienRendererWindow::getCameraPosition)
      .def("get_camera_rotation", &SapienRendererWindow::getCameraRotation)
      .def("get_camera_pose", &SapienRendererWindow::getCameraPose)
      .def("get_camera_projection_matrix",
           [](SapienRendererWindow &window) {
             glm::mat4 proj = glm::transpose(window.getCameraProjectionMatrix());
             return py::array_t<float>({4, 4}, &proj[0][0]);
           })
      .def_property_readonly(
          "_internal_renderer",
          [](SapienRendererWindow &window) { return window.getInternalRenderer(); },
          py::return_value_policy::reference)
      .def("set_scene", &SapienRendererWindow::setScene, py::arg("scene"))
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

      // // Download images from window
      // .def(
      //     "get_float_texture",
      //     [](SapienRendererWindow &window, std::string const &name) {
      //       auto [image, sizes] = window.downloadFloatTarget(name);
      //       if (sizes[2] == 1) {
      //         return py::array_t<float>({static_cast<int>(sizes[0]),
      //         static_cast<int>(sizes[1])},
      //                                   image.data());
      //       } else {
      //         return py::array_t<float>({static_cast<int>(sizes[0]), static_cast<int>(sizes[1]),
      //                                    static_cast<int>(sizes[2])},
      //                                   image.data());
      //       }
      //     },
      //     py::arg("name"))
      // .def(
      //     "get_uint32_texture",
      //     [](SapienRendererWindow &window, std::string const &name) {
      //       auto [image, sizes] = window.downloadUint32Target(name);
      //       if (sizes[2] == 1) {
      //         return py::array_t<uint32_t>(
      //             {static_cast<int>(sizes[0]), static_cast<int>(sizes[1])}, image.data());
      //       } else {
      //         return py::array_t<uint32_t>({static_cast<int>(sizes[0]),
      //         static_cast<int>(sizes[1]),
      //                                       static_cast<int>(sizes[2])},
      //                                      image.data());
      //       }
      //     },
      //     py::arg("name"))
      // .def(
      //     "get_float_texture_pixel",
      //     [](SapienRendererWindow &window, std::string const &name, uint32_t x, uint32_t y) {
      //       auto v = window.downloadFloatTargetPixel(name, x, y);
      //       return py::array_t<float>(static_cast<int>(v.size()), v.data());
      //     },
      //     py::arg("name"), py::arg("x"), py::arg("y"))
      // .def(
      //     "get_uint32_texture_pixel",
      //     [](SapienRendererWindow &window, std::string const &name, uint32_t x, uint32_t y) {
      //       auto v = window.downloadUint32TargetPixel(name, x, y);
      //       return py::array_t<uint32_t>(static_cast<int>(v.size()), v.data());
      //     },
      //     py::arg("name"), py::arg("x"), py::arg("y"))
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
}