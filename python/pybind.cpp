#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include "actor_builder.h"
#include "articulation_builder.h"
#include "articulation_wrapper.h"
#include "kinematics_articulation_wrapper.h"
#include "optifuser_renderer.h"
#include "render_interface.h"
#include "simulation.h"
#include "urdf_loader.h"
#include "joint_system.h"
#include <vector>

using namespace sapien;
namespace py = pybind11;

class PyISensor : public Renderer::ISensor {
public:
  using Renderer::ISensor::ISensor;
  Renderer::SensorPose getSensorPose() const override {
    PYBIND11_OVERLOAD_PURE(Renderer::SensorPose, Renderer::ISensor, getSensorPose);
  }
  void setSensorPose(const Renderer::SensorPose &pose) override {
    PYBIND11_OVERLOAD_PURE(void, Renderer::ISensor, setSensorPose, pose);
  }
};

class PyICamera : public Renderer::ICamera {
public:
  using Renderer::ICamera::ICamera;
  Renderer::SensorPose getSensorPose() const override {
    PYBIND11_OVERLOAD_PURE(Renderer::SensorPose, Renderer::ICamera, getSensorPose);
  }
  void setSensorPose(const Renderer::SensorPose &pose) override {
    PYBIND11_OVERLOAD_PURE(void, Renderer::ICamera, setSensorPose, pose);
  }
  const std::string &getName() const override {
    PYBIND11_OVERLOAD_PURE(const std::string &, Renderer::ICamera, getName);
  }
  uint32_t getWidth() const override {
    PYBIND11_OVERLOAD_PURE(uint32_t, Renderer::ICamera, getWidth);
  }
  uint32_t getHeight() const override {
    PYBIND11_OVERLOAD_PURE(uint32_t, Renderer::ICamera, getHeight);
  }
  float getFovy() const override { PYBIND11_OVERLOAD_PURE(float, Renderer::ICamera, getFovy); }
  void takePicture() override { PYBIND11_OVERLOAD_PURE(void, Renderer::ICamera, takePicture); }
};
class PyIArticulationBase : public IArticulationBase {
public:
  EArticulationType get_articulation_type() const override {
    PYBIND11_OVERLOAD_PURE(EArticulationType, IArticulationBase, get_articulation_type);
  }
  uint32_t dof() const override {
    PYBIND11_OVERLOAD_PURE(uint32_t, IArticulationBase, dof);
  }

  std::vector<std::string> get_joint_names() const override {
    PYBIND11_OVERLOAD_PURE(std::vector<std::string>, IArticulationBase, get_joint_names);
  }
  std::vector<uint32_t> get_joint_dofs() const override {
    PYBIND11_OVERLOAD_PURE(std::vector<uint32_t>, IArticulationBase, get_joint_dofs);
  }

  std::vector<std::tuple<physx::PxReal, physx::PxReal>> get_joint_limits() const override {
    PYBIND11_OVERLOAD_PURE_NAME(PYBIND11_TYPE(std::vector<std::tuple<physx::PxReal, physx::PxReal>>), PYBIND11_TYPE(IArticulationBase), "get_joint_limits", get_joint_limits);
  }

  std::vector<physx::PxReal> get_qpos() const override {
    PYBIND11_OVERLOAD_PURE(std::vector<physx::PxReal>, IArticulationBase, get_qpos);
  }
  void set_qpos(const std::vector<physx::PxReal> &v) override {
    PYBIND11_OVERLOAD_PURE(void, IArticulationBase, set_qpos, v);
  }

  std::vector<physx::PxReal> get_qvel() const override {
    PYBIND11_OVERLOAD_PURE(std::vector<physx::PxReal>, IArticulationBase, get_qvel);
  }
  void set_qvel(const std::vector<physx::PxReal> &v) override {
    PYBIND11_OVERLOAD_PURE(void, IArticulationBase, set_qvel, v);
  }

  std::vector<physx::PxReal> get_qacc() const override {
    PYBIND11_OVERLOAD_PURE(std::vector<physx::PxReal>, IArticulationBase, get_qacc);
  }
  void set_qacc(const std::vector<physx::PxReal> &v) override {
    PYBIND11_OVERLOAD_PURE(void, IArticulationBase, set_qacc, v);
  }

  std::vector<physx::PxReal> get_qf() const override {
    PYBIND11_OVERLOAD_PURE(std::vector<physx::PxReal>, IArticulationBase, get_qf);
  }
  void set_qf(const std::vector<physx::PxReal> &v) override {
    PYBIND11_OVERLOAD_PURE(void, IArticulationBase, set_qf, v);
  }
};


PYBIND11_MODULE(sapyen, m) {

  py::class_<Simulation>(m, "Simulation")
      .def(py::init<>())
      .def("setTimestep", &Simulation::setTimestep)
      .def("getTimestep", &Simulation::getTimestep)
      .def("setRenderer", &Simulation::setRenderer)
      .def("getRenderer", &Simulation::getRenderer)
      .def("createActorBuilder", &Simulation::createActorBuilder)
      .def("createArticulationBuilder", &Simulation::createArticulationBuilder)
      .def("step", &Simulation::step)
      .def("updateRenderer", &Simulation::updateRenderer)
      .def("addGround", &Simulation::addGround, py::arg("altitude"), py::arg("render") = true, py::arg("material") = nullptr);

  py::class_<PxRigidActor, std::unique_ptr<PxRigidActor, py::nodelete>>(m, "PxRigidActor");
  py::class_<PxRigidStatic, PxRigidActor, std::unique_ptr<PxRigidStatic, py::nodelete>>(m, "PxRigidStatic");
  py::class_<PxRigidBody, PxRigidActor, std::unique_ptr<PxRigidBody, py::nodelete>>(m, "PxRigidBody");
  py::class_<PxRigidDynamic, PxRigidBody, std::unique_ptr<PxRigidDynamic, py::nodelete>>(m, "PxRigidDynamic");

  py::class_<PxMaterial, std::unique_ptr<PxMaterial, py::nodelete>>(m, "PxMaterial");

  // py::class_<Renderer::ISensor> (m, "ISensor")
  //     .def("getSensorPose", &Renderer::ISensor::getSensorPose)
  //     .def("setSensorPose", &Renderer::ISensor::setSensorPose);

  // py::class_<Renderer::ICamera, Renderer::ISensor> (m, "ICamera")
  //     .def("getName", &Renderer::ICamera::getName)
  //     .def("getWidth", &Renderer::ICamera::getWidth)
  //     .def("getHeight", &Renderer::ICamera::getHeight)
  //     .def("getHovy", &Renderer::ICamera::getHovy)
  //     .def("takePicture", &Renderer::ICamera::takePicture)
  //     .def("getColorRGBA", &Renderer::ICamera::getColorRGBA)
  //     .def("getAlbedoRGBA", &Renderer::ICamera::getAlbedoRGBA)
  //     .def("getNormalRGBA", &Renderer::ICamera::getNormalRGBA)
  //     .def("getDepth", &Renderer::ICamera::getDepth)
  //     .def("getSegmentation", &Renderer::ICamera::getSegmentation);

  py::class_<Renderer::ICameraManager>(m, "ICameraManager");

  py::class_<Renderer::IPhysxRenderer, Renderer::ICameraManager>(m, "IPhysxRenderer");

  py::class_<Renderer::OptifuserRenderer, Renderer::IPhysxRenderer>(m, "OptifuserRenderer")
      .def(py::init<>())
      .def("init", &Renderer::OptifuserRenderer::init)
      .def("render", &Renderer::OptifuserRenderer::render)
      .def("destroy", &Renderer::OptifuserRenderer::destroy)
      .def_readwrite("cam", &Renderer::OptifuserRenderer::cam);
  // py::class_<glm::vec3>(m, "vec3", py::buffer_protocol())
  //     .def_buffer([](glm::vec3 &v) -> py::array {
  //         auto buffer = py::buffer_info (
  //             &v,
  //             sizeof(float),
  //             py::format_descriptor<float>::format(),
  //             1,
  //             {3},
  //             {sizeof(float)}
  //         );
  //         auto toReturn = py::array(buffer);
  //         return toReturn;
  //     });
  py::class_<Optifuser::CameraSpec>(m, "CameraSpec")
      .def(py::init<>())
      .def_readwrite("name", &Optifuser::CameraSpec::name)
      .def_property(
          "position",
          [](Optifuser::CameraSpec &c) { return py::array_t<float>(3, (float *)(&c.position)); },
          [](Optifuser::CameraSpec &c, py::array_t<float> arr) {
            c.position = {arr.at(0), arr.at(1), arr.at(2)};
          })
      // TODO: fields of quaternion not wrapped yet
      .def_property("rotation",
                    [](Optifuser::CameraSpec &c) {
                      std::cerr << "rotation getter not wrapped yet" << std::endl;
                    },
                    [](Optifuser::CameraSpec &c) {
                      std::cerr << "rotation setter not wrapped yet" << std::endl;
                    })
      .def_readwrite("near", &Optifuser::CameraSpec::near)
      .def_readwrite("far", &Optifuser::CameraSpec::far)
      .def_readwrite("fovy", &Optifuser::CameraSpec::fovy)
      .def_readwrite("aspect", &Optifuser::CameraSpec::aspect)
      .def("lookAt",
           [](Optifuser::CameraSpec &c, py::array_t<float> dir, py::array_t<float> up) {
             c.lookAt({dir.at(0), dir.at(1), dir.at(2)}, {up.at(0), up.at(1), up.at(2)});
           })
      // TODO: function involving matrix not wrapped yet
      .def("getModelMat",
           [](Optifuser::CameraSpec &c) {
             std::cerr << "getModelMat not wrapped yet" << std::endl;
           })
      .def("getProjectionMat",
           [](Optifuser::CameraSpec &c) {
             std::cerr << "getProjectionMat not wrapped yet" << std::endl;
           })
      .def("perspective", [](Optifuser::CameraSpec &c) {
        std::cerr << "perspective not wrapped yet" << std::endl;
      });

  py::class_<Optifuser::FPSCameraSpec, Optifuser::CameraSpec>(m, "FPSCameraSpec")
      .def(py::init<>())
      .def("update", &Optifuser::FPSCameraSpec::update)
      .def("isSane", &Optifuser::FPSCameraSpec::isSane)
      .def("setForward",
           [](Optifuser::FPSCameraSpec &c, py::array_t<float> dir) {
             c.setForward({dir.at(0), dir.at(1), dir.at(2)});
           })
      .def("setUp",
           [](Optifuser::FPSCameraSpec &c, py::array_t<float> dir) {
             c.setUp({dir.at(0), dir.at(1), dir.at(2)});
           })
      .def("rotateYawPitch", &Optifuser::FPSCameraSpec::rotateYawPitch)
      .def("moveForwardRight", &Optifuser::FPSCameraSpec::moveForwardRight)
      // TODO: function involving quarternion not wrapped yet
      .def("getRotation0", [](Optifuser::FPSCameraSpec &c) {
        std::cerr << "getRotation0 not wrapped yet" << std::endl;
      });

  py::class_<IArticulationBase, PyIArticulationBase> articulationBase(m, "IArticulationBase");
  articulationBase.def(py::init<>())
    .def("get_articulation_type", &IArticulationBase::get_articulation_type)
    .def("dof", &IArticulationBase::dof)
    .def("get_joint_names", &IArticulationBase::get_joint_names)
    .def("get_joint_dofs", [](IArticulationBase& a) {
      auto dofs = a.get_joint_dofs();
      return py::array_t<uint32_t> (dofs.size(), dofs.data());
    })
    .def("get_joint_limits", [](IArticulationBase& a) {
      auto limits = a.get_joint_limits();
      return py::array_t<PxReal> ({(int)limits.size(), 2}, {sizeof(std::tuple<PxReal, PxReal>), sizeof(PxReal)}, (PxReal *)limits.data());
    })
    .def("get_qpos", [](IArticulationBase& a) {
      auto qpos = a.get_qpos();
      return py::array_t<PxReal> (qpos.size(), qpos.data());
    })
    .def("set_qpos", [](IArticulationBase& a, py::array_t<float> arr) {
      a.set_qpos(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
    })
    .def("get_qvel", [](IArticulationBase& a) {
      auto qvel = a.get_qvel();
      return py::array_t<PxReal> (qvel.size(), qvel.data());
    })
    .def("set_qvel", [](IArticulationBase& a, py::array_t<float> arr) {
      a.set_qvel(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
    })
    .def("get_qacc", [](IArticulationBase& a) {
      auto qacc = a.get_qacc();
      return py::array_t<PxReal> (qacc.size(), qacc.data());
    })
    .def("set_qacc", [](IArticulationBase& a, py::array_t<float> arr) {
      a.set_qacc(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
    })
    .def("get_qf", [](IArticulationBase& a) {
      auto qf = a.get_qf();
      return py::array_t<PxReal> (qf.size(), qf.data());
    })
    .def("set_qf", [](IArticulationBase& a, py::array_t<float> arr) {
      a.set_qf(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
    });

  py::class_<IArticulationDrivable, IArticulationBase>(m, "IArticulationDrivable");
  py::enum_<EArticulationType>(articulationBase, "ArticulationType")
      .value("DYNAMIC_ARTICULATION", EArticulationType::DYNAMIC_ARTICULATION)
      .value("KINEMATIC_ARTICULATION", EArticulationType::KINEMATIC_ARTICULATION)
      .value("OBJECT_ARTICULATION", EArticulationType::OBJECT_ARTICULATION);
  py::class_<ArticulationWrapper, IArticulationBase>(m, "ArticulationWrapper")
    .def(py::init<>())
    .def("updateCache", &ArticulationWrapper::updateCache)
    .def("updateArticulation", &ArticulationWrapper::updateArticulation);

  py::class_<PxJointSystem, IArticulationBase>(m, "JointSystem");

  py::class_<URDF::URDFLoader>(m, "URDFLoader")
    .def(py::init<Simulation &>())
    .def_readwrite("fixLoadedObject", &URDF::URDFLoader::fixLoadedObject)
    .def("load", &URDF::URDFLoader::load, py::return_value_policy::reference);
}

