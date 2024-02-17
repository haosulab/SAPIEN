#include "./array.hpp"
#include "./python_component.hpp"
#include "generator.hpp"
#include "sapien/component.h"
#include "sapien/device.h"
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

  py::cpp_function decorate(py::function func) {
    return [func = func, name = mName](py::args args, py::kwargs const &kwargs) {
      ProfilerBlockBegin(name.c_str());
      auto obj = func(*args, **kwargs);
      ProfilerBlockEnd();
      return obj;
    };
  }

private:
  std::string mName;
};

Generator<int> init_sapien(py::module &m) {

  m.def("set_log_level", &sapien::logger::setLogLevel, py::arg("level"));

  py::class_<PythonProfiler>(m, "Profiler")
      .def(py::init<std::string const &>(), py::arg("name"))
      .def("__enter__", &PythonProfiler::enter)
      .def("__exit__", &PythonProfiler::exit, py::arg("exec_type"), py::arg("exec_value"),
           py::arg("traceback"))
      .def("__call__", &PythonProfiler::decorate, py::arg("func"));

  m.def(
       "profile", [](std::string const &name) { return PythonProfiler(name); }, py::arg("name"))
      .def(
          "profile",
          [](py::function func) {
            return PythonProfiler(py::getattr(func, "__name__").cast<std::string>())
                .decorate(func);
          },
          py::arg("func"));

  auto PyPose = py::class_<Pose>(m, "Pose");
  auto PyScene = py::class_<Scene>(m, "Scene");
  auto PyEntity = py::class_<Entity>(m, "Entity");

  auto PyComponent = py::class_<Component, PythonComponent>(m, "Component");

  auto PySystem = py::class_<System, PythonSystem>(m, "System");
  auto PyCudaArray = py::class_<CudaArrayHandle>(m, "CudaArray");
  auto PyDevice = py::class_<Device>(m, "Device");

  co_yield 0;

  PyPose.def(py::init<Vec3, Quat>(), py::arg("p") = Vec3(), py::arg("q") = Quat())
      .def(py::init(&EigenMat4ToPose), py::arg("matrix"))
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
      .def(
          "set_rpy", [](Pose &pose, Vec3 const &rpy) { pose.q = RPYToQuat(rpy); }, py::arg("rpy"))

      .def("inv", &Pose::getInverse)
      .def(py::self * py::self, py::arg("other"))
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
      .def_property_readonly("id", &Scene::getId)
      .def("get_id", &Scene::getId)
      .def_property_readonly("entities", &Scene::getEntities)
      .def("get_entities", &Scene::getEntities)
      .def("add_entity", &Scene::addEntity, py::arg("entity"))
      .def("remove_entity", &Scene::removeEntity, py::arg("entity"))
      .def("add_system", &Scene::addSystem, py::arg("system"))
      .def("get_system", &Scene::getSystem, py::arg("name"))
      .def_property_readonly("physx_system", &Scene::getPhysxSystem)
      .def("get_physx_system", &Scene::getPhysxSystem)
      .def_property_readonly("render_system", &Scene::getSapienRendererSystem)
      .def("get_render_system", &Scene::getSapienRendererSystem)
      .def("pack_poses", [](Scene &s) { return py::bytes(s.packEntityPoses()); })
      .def(
          "unpack_poses", [](Scene &s, py::bytes data) { s.unpackEntityPoses(data); },
          py::arg("data"))
      .def("clear", &Scene::clear);

  PyEntity.def(py::init<>())
      .def_property_readonly("per_scene_id", &Entity::getPerSceneId)
      .def("get_per_scene_id", &Entity::getPerSceneId)
      .def_property_readonly("global_id", &Entity::getId)
      .def("get_global_id", &Entity::getId)

      .def_property("name", &Entity::getName, &Entity::setName)
      .def("get_name", &Entity::getName)
      .def("set_name", &Entity::setName, py::arg("name"))

      .def_property_readonly("scene", &Entity::getScene, py::return_value_policy::reference)
      .def("get_scene", &Entity::getScene)

      .def_property_readonly("components", &Entity::getComponents)
      .def("get_components", &Entity::getComponents)

      .def("add_component", &Entity::addComponent, py::arg("component"))
      .def("remove_component", &Entity::removeComponent, py::arg("component"))

      .def_property("pose", &Entity::getPose, &Entity::setPose)
      .def("get_pose", &Entity::getPose)
      .def("set_pose", &Entity::setPose, py::arg("pose"))

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
      .def("set_name", &Component::setName, py::arg("name"))

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
      .def(
          "set_pose",
          [](Component &c, Pose const &pose) {
            PyErr_WarnEx(
                PyExc_DeprecationWarning,
                "component.set_pose can be ambiguous thus deprecated. It is equivalent to "
                "component.set_entity_pose, which should be used instead",
                1);
            c.setPose(pose);
          },
          py::arg("pose"))

      .def_property("entity_pose", &Component::getPose, &Component::setPose)
      .def("get_entity_pose", &Component::getPose)
      .def("set_entity_pose", &Component::setPose, py::arg("pose"))

      .def_property_readonly("is_enabled", &Component::getEnabled)
      .def("enable", &Component::enable, "enable the component")
      .def("disable", &Component::disable, "disable the component");

  PySystem.def(py::init<>()).def("step", &System::step);

  PyDevice
      .def(py::init<>([](std::string const &alias) { return findDevice(alias); }),
           py::arg("alias"))
      .def("__repr__", [](Device const &d) { return "Device(\"" + d.getAlias() + "\")"; })
      .def("__str__", &Device::getAlias)
      .def("is_cpu", &Device::isCpu)
      .def("is_cuda", &Device::isCuda)
      .def_readonly("cuda_id", &Device::cudaId)
      .def("can_render", &Device::canRender)
      .def("can_present", &Device::canPresent)
      .def_readonly("name", &Device::name)
      .def_property_readonly("pci_string", &Device::getPciString);

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

             return CudaArrayHandle{.shape = shape,
                                    .strides = strides,
                                    .type = type,
                                    .cudaId = 0, // TODO: do we need cuda id?
                                    .ptr = ptr};
           }),
           py::arg("data"))
      .def_readonly("shape", &CudaArrayHandle::shape)
      .def_readonly("strides", &CudaArrayHandle::strides)
      .def_readonly("cuda_id", &CudaArrayHandle::cudaId)
      .def_readonly("typstr", &CudaArrayHandle::type)
      .def_property_readonly(
          "ptr", [](CudaArrayHandle &array) { return reinterpret_cast<intptr_t>(array.ptr); })
      .def_property_readonly(
          "__cuda_array_interface__",
          [](CudaArrayHandle &array) {
            py::tuple shape = py::cast(array.shape);
            py::tuple strides = py::cast(array.strides);
            std::string type = array.type;

            // torch does not support uint type except uint8
            // if (type != "u1" && type[0] == 'u') {
            //   type = "i" + type.substr(1);
            // }

            return py::dict("shape"_a = shape, "strides"_a = strides, "typestr"_a = type,
                            "data"_a =
                                py::make_tuple(reinterpret_cast<intptr_t>(array.ptr), false),
                            "version"_a = 2);
          })
      .def("torch",
           [](CudaArrayHandle &array) {
             // torch does not support uint except uint8
             CudaArrayHandle newArray = array;
             if (array.type != "u1" && array.type[0] == 'u') {
               newArray.type = "i" + array.type.substr(1);
             }

             py::object obj = py::cast(newArray);
             auto as_tensor = py::module_::import("torch").attr("as_tensor");
             return as_tensor("data"_a = obj, "device"_a = "cuda").release();
           })
      .def("jax",
           [](CudaArrayHandle &array) {
             auto from_dlpack = py::module_::import("jax").attr("dlpack").attr("from_dlpack");
             auto capsule = DLPackToCapsule(array.toDLPack());
             return from_dlpack(capsule).release();
           })
      .def("dlpack", [](CudaArrayHandle &array) -> py::object {
        auto capsule = DLPackToCapsule(array.toDLPack());
        return capsule;
      });
}
