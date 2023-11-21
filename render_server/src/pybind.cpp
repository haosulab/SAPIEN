#define PYBIND11_USE_SMART_HOLDER_AS_DEFAULT 1
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <sapien/component.h>

#include "camera_component.h"
#include "client_system.h"
#include "render_body_component.h"
#include "server.h"

#include "sapien_type_caster.h"

namespace py = pybind11;

using namespace py::literals;
using namespace sapien::render_server;
PYBIND11_MODULE(pysapien_render_server, m) {

  auto PyRenderServer = py::class_<RenderServer>(m, "RenderServer");
  auto PyRenderServerBuffer = py::class_<VulkanCudaBuffer>(m, "RenderServerBuffer");

  auto PyRenderClientSystem = py::class_<ClientSystem>(m, "RenderClientSystem");
  auto PyRenderClientCameraComponent =
      py::class_<ClientCameraComponent, sapien::Component>(m, "RenderClientCameraComponent");
  auto PyRenderClientBodyComponent =
      py::class_<ClientRenderBodyComponent, sapien::Component>(m, "RenderClientBodyComponent");

  auto PyRenderClientShape = py::class_<ClientRenderShape>(m, "RenderClientShape");
  auto PyRenderClientShapeTriangleMesh =
      py::class_<ClientRenderShapeTriangleMesh, ClientRenderShape>(
          m, "RenderClientShapeTriangleMesh");

  PyRenderClientSystem
      .def(py::init<std::string const &, uint64_t>(), py::arg("address"), py::arg("process_index"))

      .def_property_readonly("process_index", &ClientSystem::getIndex)
      .def("get_process_index", &ClientSystem::getIndex)
      .def("update_render_and_take_pictures", &ClientSystem::updateRenderAndTakePictures)
      .def("set_ambient_light", &ClientSystem::setAmbientLight, py::arg("color"))
      .def("add_point_light", &ClientSystem::addPointLight, py::arg("position"), py::arg("color"),
           py::arg("shadow") = false, py::arg("shadow_near") = 0.01f,
           py::arg("shadow_far") = 100.f, py::arg("shadow_map_size") = 1024)
      .def("set_ambient_light", &ClientSystem::addDirectionalLight, py::arg("direction"),
           py::arg("color"), py::arg("shadow") = false, py::arg("position") = sapien::Vec3(0.f),
           py::arg("shadow_scale") = 5.f, py::arg("shadow_near") = 0.01f,
           py::arg("shadow_far") = 100.f, py::arg("shadow_map_size") = 1024)

      ;

  PyRenderServer.def_static("_set_shader_dir", &setDefaultShaderDirectory, py::arg("shader_dir"))
      .def(py::init<uint32_t, uint32_t, uint32_t, std::string const &, bool>(),
           py::arg("max_num_materials") = 500, py::arg("max_num_textures") = 500,
           py::arg("default_mipmap_levels") = 1, py::arg("device") = "cuda",
           py::arg("do_not_load_texture") = false)
      .def("start", &RenderServer::start, py::arg("address"))
      .def("stop", &RenderServer::stop)
      .def("wait_all", &RenderServer::waitAll, py::arg("timeout") = UINT64_MAX)
      .def("wait_scenes", &RenderServer::waitScenes, py::arg("scenes"),
           py::arg("timeout") = UINT64_MAX)
      .def("auto_allocate_buffers", &RenderServer::autoAllocateBuffers, py::arg("render_targets"),
           py::return_value_policy::reference)
      .def("summary", &RenderServer::summary);

  PyRenderServerBuffer.def_property_readonly("nbytes", &VulkanCudaBuffer::getSize)
      .def_property_readonly("type", &VulkanCudaBuffer::getType)
      .def_property_readonly("shape",
                             [](VulkanCudaBuffer &buffer) {
                               py::tuple shape = py::cast(buffer.getShape());
                               return shape;
                             })
#ifdef SAPIEN_CUDA
      .def_property_readonly("pointer",
                             [](VulkanCudaBuffer &buffer) {
                               return reinterpret_cast<uintptr_t>(buffer.getCudaPtr());
                             })
      .def_property_readonly(
          "__cuda_array_interface__",
          [](VulkanCudaBuffer &buffer) {
            py::tuple shape = py::cast(buffer.getShape());
            return py::dict(
                "shape"_a = shape, "typestr"_a = buffer.getType(),
                "data"_a = py::make_tuple(reinterpret_cast<uintptr_t>(buffer.getCudaPtr()), false),
                "version"_a = 2);
          })
#endif
      ;

  PyRenderClientCameraComponent
      .def(py::init<uint32_t, uint32_t, std::string const &>(), py::arg("width"),
           py::arg("height"), py::arg("shader_dir") = "")
      .def_property("local_pose", &ClientCameraComponent::getLocalPose,
                    &ClientCameraComponent::setLocalPose)
      .def("get_local_pose", &ClientCameraComponent::getLocalPose)
      .def("set_local_pose", &ClientCameraComponent::setLocalPose, py::arg("pose"))
      .def("set_perspective_parameters", &ClientCameraComponent::setPerspectiveParameters,
           py::arg("near"), py::arg("far"), py::arg("fx"), py::arg("fy"), py::arg("cx"),
           py::arg("cy"), py::arg("skew"))
      .def("take_picture", &ClientCameraComponent::takePicture);

  PyRenderClientBodyComponent.def(py::init<>())
      .def("attach", &ClientRenderBodyComponent::attachRenderShape);

  PyRenderClientShape
      .def_property("local_pose", &ClientRenderShape::getLocalPose,
                    &ClientRenderShape::setLocalPose)
      .def("get_local_pose", &ClientRenderShape::getLocalPose)
      .def("set_local_pose", &ClientRenderShape::setLocalPose, py::arg("pose"))
      .def_property_readonly("per_scene_id", &ClientRenderShape::getRenderId)
      .def("get_per_scene_id", &ClientRenderShape::getRenderId);

  PyRenderClientShapeTriangleMesh
      .def(py::init<std::string const &, sapien::Vec3 const &>(), py::arg("filename"),
           py::arg("scale") = sapien::Vec3(1.f))
      .def_property_readonly("filename", &ClientRenderShapeTriangleMesh::getFilename)
      .def("get_filename", &ClientRenderShapeTriangleMesh::getFilename)
      .def_property_readonly("scale", &ClientRenderShapeTriangleMesh::getScale)
      .def("get_scale", &ClientRenderShapeTriangleMesh::getScale);
}
