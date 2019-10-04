#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include "render_interface.h"
#include "optifuser_renderer.h"
#include "simulation.h"
#include "articulation_wrapper.h"
#include "kinematics_articulation_wrapper.h"
#include "actor_builder.h"
#include "articulation_builder.h"
#include <vector>

namespace py = pybind11;

class PyISensor: public Renderer::ISensor{
 public:
  using Renderer::ISensor::ISensor;
  Renderer::SensorPose getSensorPose() const override {PYBIND11_OVERLOAD_PURE(Renderer::SensorPose, Renderer::ISensor, getSensorPose);}
  void setSensorPose(const Renderer::SensorPose &pose) override {PYBIND11_OVERLOAD_PURE(void, Renderer::ISensor, setSensorPose, pose);}
};

class PyICamera: public Renderer::ICamera{
 public:
  using Renderer::ICamera::ICamera;
  Renderer::SensorPose getSensorPose() const override {PYBIND11_OVERLOAD_PURE(Renderer::SensorPose, Renderer::ICamera, getSensorPose);}
  void setSensorPose(const Renderer::SensorPose &pose) override {PYBIND11_OVERLOAD_PURE(void, Renderer::ICamera, setSensorPose, pose);}
  const std::string &getName() const override {PYBIND11_OVERLOAD_PURE(const std::string &, Renderer::ICamera, getName);}
  uint32_t getWidth() const override {PYBIND11_OVERLOAD_PURE(uint32_t, Renderer::ICamera, getWidth);}
  uint32_t getHeight() const override {PYBIND11_OVERLOAD_PURE(uint32_t, Renderer::ICamera, getHeight);}
  float getFovy() const override {PYBIND11_OVERLOAD_PURE(float, Renderer::ICamera, getFovy);}
  void takePicture() override {PYBIND11_OVERLOAD_PURE(void, Renderer::ICamera, takePicture);}

};




//class PyAnimal : public Animal {
//public:
/* Inherit the constructors */
//    using Animal::Animal;

/* Trampoline (need one for each virtual function) */
//    std::string go(int n_times) override {
//        PYBIND11_OVERLOAD_PURE(
//            std::string, /* Return type */
//            Animal,      /* Parent class */
//            go,          /* Name of function in C++ (must match Python name) */
//            n_times      /* Argument(s) */
//        );
//    }
//};
PxArticulationWrapper a;

class Animal {
 public:
  std::vector<int> Q;
  std::vector<PxRigidActor *> mRigidActors;
  PxPhysics *mPhysicsSDK = nullptr;
  PxFoundation *mFoundation = nullptr;
  PxCooking *mCooking = nullptr;
  PxScene *mScene = nullptr;
  PxReal mTimestep = 1.0f / 60.0f;
  Renderer::IPhysxRenderer *mRenderer = nullptr;
  PxDefaultCpuDispatcher *mCpuDispatcher = nullptr;
  PxMaterial *mDefaultMaterial = nullptr;
  CollisionGroupManager collisionManager;

  std::map<physx_id_t, PxTransform> mCameraId2InitialPose;
  std::map<physx_id_t, PxRigidActor*> mMountedCamera2MountedActor;

  std::map<physx_id_t, PxTransform> mRenderId2InitialPose;
  std::map<physx_id_t, PxRigidActor*> mRenderId2Parent;
  std::map<physx_id_t, IArticulationBase*> mRenderId2Articulation;

  //struct PxArticulationWrapper wrapper;

  std::vector<std::unique_ptr<struct PxArticulationWrapper>> mDynamicArticulationWrappers;
  std::vector<std::unique_ptr<class PxKinematicsArticulationWrapper>> mKinematicArticulationWrappers;
};

PYBIND11_MODULE(pybind, m) {

  py::class_<PxSimulation>(m, "PxSimulation")
      .def(py::init<>())
      .def("setTimestep", &PxSimulation::setTimestep)
      .def("getTimestep", &PxSimulation::getTimestep)
      .def("setRenderer", &PxSimulation::setRenderer)
      .def("getRenderer", &PxSimulation::getRenderer)
      .def("createActorBuilder", &PxSimulation::createActorBuilder)
      .def("createArticulationBuilder", &PxSimulation::createArticulationBuilder)
      .def("step", &PxSimulation::step)
      .def("updateRenderer", &PxSimulation::updateRenderer)
      .def("addGround", &PxSimulation::addGround);
    
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

  py::class_<Renderer::OptifuserRenderer>(m, "OptifuserRenderer")
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
      .def_property("position", 
                    [](Optifuser::CameraSpec &c) {
                      return py::array_t<float>(3, (float *)(&c.position));
                    }, 
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
      .def("lookAt", [](Optifuser::CameraSpec &c, py::array_t<float> dir, py::array_t<float> up) {
                       c.lookAt({dir.at(0), dir.at(1), dir.at(2)}, {up.at(0), up.at(1), up.at(2)});
                     })
      // TODO: function involving matrix not wrapped yet
      .def("getModelMat", [](Optifuser::CameraSpec &c){ 
                            std::cerr << "getModelMat not wrapped yet" << std::endl;
                          })
      .def("getProjectionMat", [](Optifuser::CameraSpec &c){ 
                                 std::cerr << "getProjectionMat not wrapped yet" << std::endl;
                               })
      .def("perspective", [](Optifuser::CameraSpec &c){ 
                            std::cerr << "perspective not wrapped yet" << std::endl;
                          });


  py::class_<Optifuser::FPSCameraSpec, Optifuser::CameraSpec>(m, "FPSCameraSpec")
      .def(py::init<>())
      .def("update", &Optifuser::FPSCameraSpec::update)
      .def("isSane", &Optifuser::FPSCameraSpec::isSane)
      .def("setForward", [](Optifuser::FPSCameraSpec &c, py::array_t<float> dir) {
                           c.setForward({dir.at(0), dir.at(1), dir.at(2)});
                         })
      .def("setUp", [](Optifuser::FPSCameraSpec &c, py::array_t<float> dir) {
                      c.setUp({dir.at(0), dir.at(1), dir.at(2)});
                    })
      .def("rotateYawPitch", &Optifuser::FPSCameraSpec::rotateYawPitch)
      .def("moveForwardRight", &Optifuser::FPSCameraSpec::moveForwardRight)
      // TODO: function involving quarternion not wrapped yet
      .def("getRotation0", [](Optifuser::FPSCameraSpec &c) {
                             std::cerr << "getRotation0 not wrapped yet" << std::endl;
                           });
}
