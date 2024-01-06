#include "./array.hpp"
#include "./python_component.hpp"
#include "generator.hpp"
#include "sapien/component.h"
#include "sapien/entity.h"
#include "sapien/logger.h"
#include "sapien/math/math.h"
#include "sapien/physx/physx_system.h"
#include "sapien/profiler.h"
#include "sapien/sapien_renderer/sapien_renderer.h"
#include "sapien/scene.h"
#include "sapien/system.h"
#include "sapien_type_caster.h"
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/smart_holder.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals;
using namespace sapien;

namespace sapien {

static std::string gPythonCudaBackend = "none";
void setPythonCudaBackend(std::string const &name) { gPythonCudaBackend = name; }
std::string getPythonCudaBackend() { return gPythonCudaBackend; }

}; // namespace sapien

class PythonSystem : public System, public py::trampoline_self_life_support {
public:
  using System::System;

  void step() override { PYBIND11_OVERRIDE_PURE(void, System, step, ); }
  std::string getName() const override {
    PYBIND11_OVERRIDE_PURE_NAME(std::string, System, "get_name", getName, );
  }
};

class PythonProfiler {
public:
  PythonProfiler(std::string const &name) { mName = name; }

  void enter() { ProfilerBlockBegin(mName.c_str()); }
  void exit(const std::optional<pybind11::type> &exc_type,
            const std::optional<pybind11::object> &exc_value,
            const std::optional<pybind11::object> &traceback) {
    ProfilerBlockEnd();
  }

private:
  std::string mName;
};

Generator<int> init_sapien(py::module &m) {

  m.def("set_log_level", &sapien::logger::setLogLevel, py::arg("level"));
  m.def(
      "set_cuda_tensor_backend", &sapien::setPythonCudaBackend, py::arg("backend"),
      R"doc(set the backend returned CUDA tensors. Supported backends are "torch" and "jax")doc");
  m.def("get_cuda_tensor_backend", &sapien::getPythonCudaBackend);

  py::class_<PythonProfiler>(m, "Profiler")
      .def(py::init<std::string const &>())
      .def("__enter__", &PythonProfiler::enter)
      .def("__exit__", &PythonProfiler::exit);

  auto PyPose = py::class_<Pose>(m, "Pose");
  auto PyScene = py::class_<Scene>(m, "Scene");
  auto PyEntity = py::class_<Entity>(m, "Entity");

  auto PyComponent = py::class_<Component, PythonComponent>(m, "Component");

  auto PySystem = py::class_<System, PythonSystem>(m, "System");
  auto PyCudaArray = py::class_<PythonCudaArrayHandle>(m, "CudaArray");

  co_yield 0;

  PyPose.def(py::init<Vec3, Quat>(), py::arg("p") = Vec3(), py::arg("q") = Quat())
      .def(py::init(&EigenMat4ToPose))
      .def("to_transformation_matrix", &PoseToEigenMat4)

      .def_readwrite("p", &Pose::p)
      .def(
          "set_p", [](Pose &pose, Vec3 const &p) { pose.p = p; }, py::arg("p"))
      .def("get_p", [](Pose &pose) { return pose.p; })

      .def_readwrite("q", &Pose::q)
      .def(
          "set_q", [](Pose &pose, Quat const &q) { pose.q = q; }, py::arg("q"))
      .def("get_q", [](Pose &pose) { return pose.q; })

      .def_property(
          "rpy", [](Pose &pose) { return QuatToRPY(pose.q); },
          [](Pose &pose, Vec3 const &rpy) { pose.q = RPYToQuat(rpy); })
      .def("get_rpy", [](Pose &pose) { return QuatToRPY(pose.q); })
      .def("set_rpy", [](Pose &pose, Vec3 const &rpy) { pose.q = RPYToQuat(rpy); })

      .def("inv", &Pose::getInverse)
      .def(py::self * py::self)
      .def("__repr__",
           [](Pose const &pose) {
             std::ostringstream oss;
             oss << "Pose([" << pose.p.x << ", " << pose.p.y << ", " << pose.p.z << "], ["
                 << pose.q.w << ", " << pose.q.x << ", " << pose.q.y << ", " << pose.q.z << "])";
             return oss.str();
           })
      .def(py::pickle(
          [](Pose const &p) {
            return py::make_tuple(p.p.x, p.p.y, p.p.z, p.q.w, p.q.x, p.q.y, p.q.z);
          },
          [](py::tuple t) {
            if (t.size() != 7) {
              throw std::runtime_error("Invalid state!");
            }
            return Pose(Vec3{t[0].cast<float>(), t[1].cast<float>(), t[2].cast<float>()},
                        Quat{t[3].cast<float>(), t[4].cast<float>(), t[5].cast<float>(),
                             t[6].cast<float>()});
          }));

  PyScene
      .def(py::init<std::vector<std::shared_ptr<System>> const &>(),
           py::arg("systems")) // TODO: support parameters

      .def_property_readonly("entities", &Scene::getEntities)
      .def("get_entities", &Scene::getEntities)
      .def("add_entity", &Scene::addEntity)
      .def("remove_entity", &Scene::removeEntity, py::arg("entity"))
      .def("add_system", &Scene::addSystem)
      .def("get_system", &Scene::getSystem, py::arg("name"))

      // .def_property_readonly("modules", &Scene::getModules)
      // .def("get_modules", &Scene::getModules)
      // .def("add_module", &Scene::addModule)
      // .def("remove_module", &Scene::removeModule, py::arg("module"))

      .def_property_readonly("physx_system", &Scene::getPhysxSystem)
      .def("get_physx_system", &Scene::getPhysxSystem)
      .def_property_readonly("render_system", &Scene::getSapienRendererSystem)
      .def("get_render_system", &Scene::getSapienRendererSystem)
      .def("pack_poses", [](Scene &s) { return py::bytes(s.packEntityPoses()); })
      .def(
          "unpack_poses", [](Scene &s, py::bytes data) { s.unpackEntityPoses(data); },
          py::arg("data"));

  PyEntity.def(py::init<>())
      .def_property_readonly("per_scene_id", &Entity::getPerSceneId)
      .def("get_per_scene_id", &Entity::getPerSceneId)
      .def_property_readonly("global_id", &Entity::getId)
      .def("get_global_id", &Entity::getId)

      .def_property("name", &Entity::getName, &Entity::setName)
      .def("get_name", &Entity::getName)
      .def("set_name", &Entity::setName)

      .def_property_readonly("scene", &Entity::getScene, py::return_value_policy::reference)
      .def("get_scene", &Entity::getScene)

      .def_property_readonly("components", &Entity::getComponents)
      .def("get_components", &Entity::getComponents)

      .def("add_component", &Entity::addComponent, py::arg("component"))
      .def("remove_component", &Entity::removeComponent, py::arg("component"))

      .def_property("pose", &Entity::getPose, &Entity::setPose)
      .def("get_pose", &Entity::getPose)
      .def("set_pose", &Entity::setPose)

      .def("remove_from_scene", &Entity::removeFromScene)

      .def(
          "find_component_by_type",
          [](Entity &e, py::type type) {
            for (auto &c : e.getComponents()) {
              if (py::isinstance(py::cast(c), type)) {
                return c;
              }
            }
            return std::shared_ptr<Component>(nullptr);
          },
          py::arg("cls"))

      .def("add_to_scene", &Entity::addToScene, py::arg("scene"))
      .def("remove_from_scene", &Entity::removeFromScene);

  PyComponent.def(py::init<>())
      .def_property_readonly("entity", &Component::getEntity)
      .def("get_entity", &Component::getEntity)

      .def_property("name", &Component::getName, &Component::setName)
      .def("get_name", &Component::getName)
      .def("set_name", &Component::setName)

      .def_property(
          "pose",
          [](Component &c) {
            PyErr_WarnEx(PyExc_DeprecationWarning,
                         "component.pose can be ambiguous thus deprecated. It is equivalent to "
                         "component.entity_pose, which should be used instead",
                         1);
            return c.getPose();
          },
          [](Component &c, Pose const &pose) { c.setPose(pose); })
      .def("get_pose",
           [](Component &c) {
             PyErr_WarnEx(
                 PyExc_DeprecationWarning,
                 "component.get_pose can be ambiguous thus deprecated. It is equivalent to "
                 "component.get_entity_pose, which should be used instead",
                 1);
             return c.getPose();
           })
      .def("set_pose",
           [](Component &c, Pose const &pose) {
             PyErr_WarnEx(
                 PyExc_DeprecationWarning,
                 "component.set_pose can be ambiguous thus deprecated. It is equivalent to "
                 "component.set_entity_pose, which should be used instead",
                 1);
             c.setPose(pose);
           })

      .def_property("entity_pose", &Component::getPose, &Component::setPose)
      .def("get_entity_pose", &Component::getPose)
      .def("set_entity_pose", &Component::setPose)

      .def_property_readonly("is_enabled", &Component::getEnabled)
      .def("enable", &Component::enable, "enable the component")
      .def("disable", &Component::disable, "disable the component");

  PySystem.def(py::init<>()).def("step", &System::step);

  PyCudaArray
      .def(py::init<>([](py::object obj) {
        auto interface = obj.attr("__cuda_array_interface__").cast<py::dict>();

        auto shape = interface.attr("shape").cast<py::tuple>().cast<std::vector<int>>();
        auto type = interface.attr("typestr").cast<std::string>();
        py::dtype dtype(type);

        std::vector<int> strides;
        if (py::hasattr(obj, "stride") && !obj.is_none()) {
          strides = interface.attr("strides").cast<py::tuple>().cast<std::vector<int>>();
        } else {
          int acc = dtype.itemsize();
          strides.push_back(acc);
          for (uint32_t i = shape.size() - 1; i >= 1; --i) {
            acc *= shape.at(i);
            strides.push_back(acc);
          }
        }

        auto data = interface.attr("data").cast<py::tuple>();
        void *ptr = reinterpret_cast<void *>(data[0].cast<uintptr_t>());

        return PythonCudaArrayHandle{.shape = shape,
                                     .strides = strides,
                                     .type = type,
                                     .cudaId = 0, // TODO: do we need cuda id?
                                     .ptr = ptr};
      }))
      .def_readonly("shape", &PythonCudaArrayHandle::shape)
      .def_readonly("strides", &PythonCudaArrayHandle::strides)
      .def_readonly("cuda_id", &PythonCudaArrayHandle::cudaId)
      .def_readonly("typstr", &PythonCudaArrayHandle::type)
      .def_property_readonly(
          "ptr",
          [](PythonCudaArrayHandle &array) { return reinterpret_cast<intptr_t>(array.ptr); })
      .def_property_readonly("__cuda_array_interface__", [](PythonCudaArrayHandle &array) {
        py::tuple shape = py::cast(array.shape);
        py::tuple strides = py::cast(array.strides);
        std::string type = array.type;

        // torch does not support uint type except uint8
        if (type != "u1" && type[0] == 'u') {
          type = "i" + type.substr(1);
        }

        return py::dict("shape"_a = shape, "strides"_a = strides, "typestr"_a = type,
                        "data"_a = py::make_tuple(reinterpret_cast<intptr_t>(array.ptr), false),
                        "version"_a = 2);
      });
}
