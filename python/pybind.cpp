#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "PxArticulation.h"
#include "actor_builder.h"
#include "articulation_builder.h"
#include "articulation_wrapper.h"
#include "joint_system.h"
#include "kinematics_articulation_wrapper.h"
#include "optifuser_renderer.h"
#include "render_interface.h"
#include "simulation.h"
#include "urdf_loader.h"
#include <vector>

using namespace sapien;
namespace py = pybind11;

py::array_t<float> mat42array(glm::mat4 const &mat) {
  float arr[] = {mat[0][0], mat[1][0], mat[2][0], mat[3][0], mat[0][1], mat[1][1],
                 mat[2][1], mat[3][1], mat[0][2], mat[1][2], mat[2][2], mat[3][2],
                 mat[0][3], mat[1][3], mat[2][3], mat[3][3]};
  return py::array_t<float>({4, 4}, arr);
}

template <typename T> py::array_t<T> make_array(std::vector<T> const &values) {
  return py::array_t(values.size(), values.data());
}

PxVec3 array2vec3(const py::array_t<float> &arr) { return {arr.at(0), arr.at(1), arr.at(2)}; }

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
  uint32_t dof() const override { PYBIND11_OVERLOAD_PURE(uint32_t, IArticulationBase, dof); }

  std::vector<std::string> get_joint_names() const override {
    PYBIND11_OVERLOAD_PURE(std::vector<std::string>, IArticulationBase, get_joint_names);
  }
  std::vector<uint32_t> get_joint_dofs() const override {
    PYBIND11_OVERLOAD_PURE(std::vector<uint32_t>, IArticulationBase, get_joint_dofs);
  }

  std::vector<std::tuple<physx::PxReal, physx::PxReal>> get_joint_limits() const override {
    PYBIND11_OVERLOAD_PURE_NAME(
        PYBIND11_TYPE(std::vector<std::tuple<physx::PxReal, physx::PxReal>>),
        PYBIND11_TYPE(IArticulationBase), "get_joint_limits", get_joint_limits);
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
      .def("addGround", &Simulation::addGround, py::arg("altitude"), py::arg("render") = true,
           py::arg("material") = nullptr);

  py::class_<PxRigidActor, std::unique_ptr<PxRigidActor, py::nodelete>>(m, "PxRigidActor")
      .def("getGlobalPose", &PxRigidActor::getGlobalPose);
  py::class_<PxRigidStatic, PxRigidActor, std::unique_ptr<PxRigidStatic, py::nodelete>>(
      m, "PxRigidStatic")
      .def("getGlobalPose", &PxRigidStatic::getGlobalPose);

  py::class_<PxRigidBody, PxRigidActor, std::unique_ptr<PxRigidBody, py::nodelete>>(m,
                                                                                    "PxRigidBody")
      .def("getGlobalPose", &PxRigidBody::getGlobalPose)
      .def("getLinearVelocity", [](PxRigidBody &a) {physx::PxVec3 vel = a.getLinearVelocity(); return py::array_t<PxReal>(3, (PxReal *)(&vel)); })
      .def("getAngularVelocity", [](PxRigidBody &a) {physx::PxVec3 vel = a.getAngularVelocity(); return py::array_t<PxReal>(3, (PxReal *)(&vel)); });
  py::class_<PxRigidDynamic, PxRigidBody, std::unique_ptr<PxRigidDynamic, py::nodelete>>(
      m, "PxRigidDynamic")
      .def("getGlobalPose", &PxRigidDynamic::getGlobalPose)
      .def("getLinearVelocity", [](PxRigidDynamic &a) {physx::PxVec3 vel = a.getLinearVelocity(); return py::array_t<PxReal>(3, (PxReal *)(&vel)); })
      .def("getAngularVelocity", [](PxRigidDynamic &a) {physx::PxVec3 vel = a.getAngularVelocity(); return py::array_t<PxReal>(3, (PxReal *)(&vel)); });
  py::class_<PxArticulationLink, PxRigidBody, std::unique_ptr<PxArticulationLink, py::nodelete>>(
      m, "PxArticulationLink")
      .def("getGlobalPose", &PxArticulationLink::getGlobalPose)
      .def("getLinearVelocity", [](PxArticulationLink &a) {physx::PxVec3 vel = a.getLinearVelocity(); return py::array_t<PxReal>(3, (PxReal *)(&vel)); })
      .def("getAngularVelocity", [](PxArticulationLink &a) {physx::PxVec3 vel = a.getAngularVelocity(); return py::array_t<PxReal>(3, (PxReal *)(&vel)); });

  py::class_<PxMaterial, std::unique_ptr<PxMaterial, py::nodelete>>(m, "PxMaterial")
      .def("getStaticFriction", &PxMaterial::getStaticFriction)
      .def("getDynamicFriction", &PxMaterial::getDynamicFriction)
      .def("getRestitution", &PxMaterial::getRestitution)
      .def("setStaticFriction", &PxMaterial::setStaticFriction)
      .def("setDynamicFriction", &PxMaterial::setDynamicFriction)
      .def("setRestitution", &PxMaterial::setRestitution);

  m.def("createMaterial", &PxPhysics::createMaterial);

  py::class_<Renderer::ISensor, PyISensor>(m, "ISensor")
      .def("getSensorPose", &Renderer::ISensor::getSensorPose)
      .def("setSensorPose", &Renderer::ISensor::setSensorPose);

  py::class_<Renderer::ICamera, Renderer::ISensor>(m, "ICamera")
      .def("getName", &Renderer::ICamera::getName)
      .def("getWidth", &Renderer::ICamera::getWidth)
      .def("getHeight", &Renderer::ICamera::getHeight)
      .def("getHovy", &Renderer::ICamera::getFovy)
      .def("takePicture", &Renderer::ICamera::takePicture)
      .def("getColorRGBA",
           [](Renderer::ICamera &cam) {
             return py::array_t<float>(
                 {static_cast<int>(cam.getHeight()), static_cast<int>(cam.getWidth()), 4},
                 cam.getColorRGBA().data());
           })
      .def("getAlbedoRGBA", &Renderer::ICamera::getAlbedoRGBA)
      .def("getNormalRGBA", &Renderer::ICamera::getNormalRGBA)
      .def("getDepth", &Renderer::ICamera::getDepth)
      .def("getSegmentation", &Renderer::ICamera::getSegmentation);

  py::class_<Renderer::ICameraManager>(m, "ICameraManager");

  py::class_<Renderer::IPhysxRenderer, Renderer::ICameraManager>(m, "IPhysxRenderer");

  py::class_<Renderer::OptifuserRenderer, Renderer::IPhysxRenderer>(m, "OptifuserRenderer")
      .def(py::init<>())
      .def("init", &Renderer::OptifuserRenderer::init)
      .def("render", &Renderer::OptifuserRenderer::render)
      .def("destroy", &Renderer::OptifuserRenderer::destroy)
      .def("show_window", &Renderer::OptifuserRenderer::showWindow)
      .def("hide_window", &Renderer::OptifuserRenderer::hideWindow)
      .def_readonly("cam", &Renderer::OptifuserRenderer::cam)
      .def("get_cameras", &Renderer::OptifuserRenderer::getCameras);
  py::class_<Optifuser::CameraSpec>(m, "CameraSpec")
      .def(py::init([]() { return new Optifuser::CameraSpec(); }))
      .def_readwrite("name", &Optifuser::CameraSpec::name)
      .def("set_position",
           [](Optifuser::CameraSpec &c, const py::array_t<float> &arr) {
             c.position = {arr.at(0), arr.at(1), arr.at(2)};
           })
      .def("set_rotation",
           [](Optifuser::CameraSpec &c, const py::array_t<float> &arr) {
             c.rotation = {arr.at(0), arr.at(1), arr.at(2), arr.at(3)};
           })
      .def_property_readonly(
          "position",
          [](Optifuser::CameraSpec &c) { return py::array_t<float>(3, (float *)(&c.position)); })
      .def_property_readonly(
          "rotation",
          [](Optifuser::CameraSpec &c) {
            return make_array<float>({c.rotation.w, c.rotation.x, c.rotation.y, c.rotation.z});
          })
      .def_readwrite("near", &Optifuser::CameraSpec::near)
      .def_readwrite("far", &Optifuser::CameraSpec::far)
      .def_readwrite("fovy", &Optifuser::CameraSpec::fovy)
      .def_readwrite("aspect", &Optifuser::CameraSpec::aspect)
      .def("lookAt",
           [](Optifuser::CameraSpec &c, const py::array_t<float> &dir,
              const py::array_t<float> &up) {
             c.lookAt({dir.at(0), dir.at(1), dir.at(2)}, {up.at(0), up.at(1), up.at(2)});
           })
      .def("getModelMat", [](Optifuser::CameraSpec &c) { return mat42array(c.getModelMat()); })
      .def("getProjectionMat",
           [](Optifuser::CameraSpec &c) { return mat42array(c.getProjectionMat()); });

  py::class_<Optifuser::FPSCameraSpec, Optifuser::CameraSpec>(m, "FPSCameraSpec")
      .def(py::init<>())
      .def("update", &Optifuser::FPSCameraSpec::update)
      .def("isSane", &Optifuser::FPSCameraSpec::isSane)
      .def("setForward",
           [](Optifuser::FPSCameraSpec &c, const py::array_t<float> &dir) {
             c.setForward({dir.at(0), dir.at(1), dir.at(2)});
           })
      .def("setUp",
           [](Optifuser::FPSCameraSpec &c, const py::array_t<float> &dir) {
             c.setUp({dir.at(0), dir.at(1), dir.at(2)});
           })
      .def("rotateYawPitch", &Optifuser::FPSCameraSpec::rotateYawPitch)
      .def("moveForwardRight", &Optifuser::FPSCameraSpec::moveForwardRight)
      .def("getRotation0", [](Optifuser::FPSCameraSpec &c) {
        glm::quat q = c.getRotation0();
        return make_array<float>({q.w, q.x, q.y, q.z});
      });

  py::class_<IArticulationBase, PyIArticulationBase> articulationBase(m, "IArticulationBase");
  articulationBase
      // NOTE: do not expose constructor
      // .def(py::init<>())
      .def("get_articulation_type", &IArticulationBase::get_articulation_type)
      .def("dof", &IArticulationBase::dof)
      .def("get_joint_names", &IArticulationBase::get_joint_names)
      .def("get_joint_dofs",
           [](IArticulationBase &a) {
             auto dofs = a.get_joint_dofs();
             return py::array_t<uint32_t>(dofs.size(), dofs.data());
           })
      .def("get_joint_limits",
           [](IArticulationBase &a) {
             auto limits = a.get_joint_limits();
             return py::array_t<PxReal>({(int)limits.size(), 2},
                                        {sizeof(std::tuple<PxReal, PxReal>), sizeof(PxReal)},
                                        (PxReal *)limits.data());
           })
      .def("get_qpos",
           [](IArticulationBase &a) {
             auto qpos = a.get_qpos();
             return py::array_t<PxReal>(qpos.size(), qpos.data());
           })
      .def("set_qpos",
           [](IArticulationBase &a, const py::array_t<float> &arr) {
             a.set_qpos(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
           })
      .def("get_qvel",
           [](IArticulationBase &a) {
             auto qvel = a.get_qvel();
             return py::array_t<PxReal>(qvel.size(), qvel.data());
           })
      .def("set_qvel",
           [](IArticulationBase &a, const py::array_t<float> &arr) {
             a.set_qvel(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
           })
      .def("get_qacc",
           [](IArticulationBase &a) {
             auto qacc = a.get_qacc();
             return py::array_t<PxReal>(qacc.size(), qacc.data());
           })
      .def("set_qacc",
           [](IArticulationBase &a, const py::array_t<float> &arr) {
             a.set_qacc(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
           })
      .def("get_qf",
           [](IArticulationBase &a) {
             auto qf = a.get_qf();
             return py::array_t<PxReal>(qf.size(), qf.data());
           })
      .def("set_qf", [](IArticulationBase &a, const py::array_t<float> &arr) {
        a.set_qf(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
      });

  py::class_<IArticulationDrivable, IArticulationBase>(m, "IArticulationDrivable");
  py::enum_<EArticulationType>(articulationBase, "ArticulationType")
      .value("DYNAMIC_ARTICULATION", EArticulationType::DYNAMIC_ARTICULATION)
      .value("KINEMATIC_ARTICULATION", EArticulationType::KINEMATIC_ARTICULATION)
      .value("OBJECT_ARTICULATION", EArticulationType::OBJECT_ARTICULATION);
  py::class_<ArticulationWrapper, IArticulationBase>(m, "ArticulationWrapper")
      // NOTE: do not expose constructor
      // .def(py::init<>())
      .def("updateCache", &ArticulationWrapper::updateCache)
      .def("updateArticulation", &ArticulationWrapper::updateArticulation)
      .def("add_force_actuator", &ArticulationWrapper::addForceActuator)
      .def("get_force_actuator_range",
           [](ArticulationWrapper &a) {
             const auto &ar = a.getForceActuatorRanges();
             return py::array_t<PxReal>({(int)ar.size(), 2},
                                        {sizeof(std::array<PxReal, 2>), sizeof(PxReal)},
                                        (PxReal *)ar.data());
           })
      .def("get_force_actuator_name", &ArticulationWrapper::getForceActuatorNames)
      .def("apply_actuator",
           [](ArticulationWrapper &a, py::array_t<float> arr) {
             a.applyActuatorForce(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
           })
      .def("get_cfrc_ext",
           [](ArticulationWrapper &a) {
             const auto cfrc = a.get_cfrc_ext();
             return py::array_t<PxReal>({(int)cfrc.size(), 6},
                                        {sizeof(std::array<PxReal, 6>), sizeof(PxReal)},
                                        (PxReal *)cfrc.data());
           })
      .def("set_root_pose",
           [](ArticulationWrapper &a, const py::array_t<float> &position,
              const py::array_t<float> &quaternion) {
             a.articulation->teleportRootLink(
                 {{position.at(0), position.at(1), position.at(2)},
                  {quaternion.at(1), quaternion.at(2), quaternion.at(3), quaternion.at(0)}},
                 true);
           },
           py::arg("position") = make_array<float>({0, 0, 0}),
           py::arg("quaternion") = make_array<float>({1, 0, 0, 0}))
      .def("set_drive_qpos",
           [](ArticulationWrapper &a, py::array_t<float> qpos) {
             a.set_drive_target(std::vector<PxReal>(qpos.data(), qpos.data() + qpos.size()));
           })
      .def("set_PD", &ArticulationWrapper::set_drive_property, py::arg("p"), py::arg("d"),
           py::arg("forceLimit") = PX_MAX_F32, py::arg("jointIndex") = py::list())
      .def("get_drivable_joint_name", &ArticulationWrapper::get_drive_joint_names)
      .def("balance_passive_force", &ArticulationWrapper::set_force_balance);

  py::class_<JointSystem, IArticulationBase>(m, "JointSystem");

  py::class_<URDF::URDFLoader>(m, "URDFLoader")
      .def(py::init<Simulation &>())
      .def_readwrite("fixLoadedObject", &URDF::URDFLoader::fixLoadedObject)
      .def("load", &URDF::URDFLoader::load, py::return_value_policy::reference)
      .def("loadJointSystem", &URDF::URDFLoader::loadJointSystem,
           py::return_value_policy::reference);

  py::class_<PxTransform>(m, "PxTransform")
      .def(py::init([](py::array_t<float> p, py::array_t<float> q) {
             return new PxTransform({p.at(0), p.at(1), p.at(2)},
                                    {q.at(1), q.at(2), q.at(3), q.at(0)});
           }),
           py::return_value_policy::automatic, py::arg("p") = make_array<float>({0, 0, 0}),
           py::arg("q") = make_array<float>({1, 0, 0, 0}))
      .def_property_readonly(
          "p", [](PxTransform &t) { return py::array_t<PxReal>(3, (PxReal *)(&t.p)); })
      .def_property_readonly("q",
                             [](PxTransform &t) {
                               return make_array<float>({t.q.w, t.q.x, t.q.y, t.q.z});
                             })
      .def("set_p", [](PxTransform &t, const py::array_t<PxReal> &arr) { t.p = array2vec3(arr); })
      .def("set_q", [](PxTransform &t, const py::array_t<PxReal> &arr) {
        t.q = {arr.at(1), arr.at(2), arr.at(3), arr.at(0)}; // NOTE: wxyz to xyzw
      });

  py::class_<PxArticulationJointType> jointtype(m, "PxArticulationJointType");

  py::enum_<PxArticulationJointType::Enum>(jointtype, "ArticulationJointType")
      .value("ePRISMATIC", PxArticulationJointType::ePRISMATIC)
      .value("eREVOLUTE", PxArticulationJointType::eREVOLUTE)
      .value("eSPHERICAL", PxArticulationJointType::eSPHERICAL)
      .value("eFIX", PxArticulationJointType::eFIX)
      .value("eUNDEFINED", PxArticulationJointType::eUNDEFINED)
      .export_values();

  py::class_<ArticulationBuilder>(m, "ArticulationBuilder")
      .def(py::init<Simulation *>())
      .def("addLink",
           [](ArticulationBuilder &a, PxArticulationLink *parent, const PxTransform &pose,
              const std::string &name, const std::string &jointName,
              PxArticulationJointType::Enum jointType, const py::array_t<float> &arr,
              PxTransform const &parentPose, PxTransform const &childPose) {
             std::vector<std::array<float, 2>> limits;
             if (jointType == PxArticulationJointType::eREVOLUTE ||
                 jointType == PxArticulationJointType::ePRISMATIC) {
               limits = {{arr.at(0, 0), arr.at(0, 1)}};
             }
             // TODO: arr to limits
             return a.addLink(parent, pose, name, jointName, jointType, limits, parentPose,
                              childPose);
           },
           py::return_value_policy::reference, py::arg("parent") = (PxArticulationLink *)nullptr,
           py::arg("pose") = PxTransform({0, 0, 0}, PxIdentity), py::arg("name") = "",
           py::arg("jointName") = "", py::arg("jointType") = PxArticulationJointType::eUNDEFINED,
           py::arg("limits") = py::array_t<float>(),
           py::arg("parentPose") = PxTransform({0, 0, 0}, PxIdentity),
           py::arg("childPose") = PxTransform({0, 0, 0}, PxIdentity))
      .def("addBoxShapeToLink",
           [](ArticulationBuilder &a, PxArticulationLink &link, const PxTransform &pose,
              const py::array_t<float> &arr, PxMaterial *material) {
             a.addBoxShapeToLink(link, pose, array2vec3(arr), material);
           },
           py::arg("link"), py::arg("pose") = PxTransform{{0, 0, 0}, PxIdentity},
           py::arg("scale") = make_array<float>({1, 1, 1}), py::arg("material") = nullptr)
      .def("addCapsuleShapeToLink", &ArticulationBuilder::addCapsuleShapeToLink, py::arg("link"),
           py::arg("pose") = PxTransform{{0, 0, 0}, PxIdentity}, py::arg("radius") = 1,
           py::arg("length") = 1, py::arg("material") = nullptr)
      .def("addSphereShapeToLink", &ArticulationBuilder::addSphereShapeToLink, py::arg("link"),
           py::arg("pose") = PxTransform{{0, 0, 0}, PxIdentity}, py::arg("radius") = 1,
           py::arg("material") = nullptr)
      .def("addConvexObjShapeToLink",
           [](ArticulationBuilder &a, PxArticulationLink &link, const std::string &filename,
              const PxTransform &pose, const py::array_t<float> &arr, PxMaterial *material) {
             a.addConvexObjShapeToLink(link, filename, pose, array2vec3(arr), material);
           },
           py::arg("link"), py::arg("filename"),
           py::arg("pose") = PxTransform{{0, 0, 0}, PxIdentity},
           py::arg("scale") = make_array<float>({1, 1, 1}), py::arg("material") = nullptr)
      .def("setLinkMassAndInertia",
           [](ArticulationBuilder &a, PxArticulationLink &link, PxReal mass,
              const PxTransform &cMassPose, const py::array_t<float> &arr) {
             a.setLinkMassAndInertia(link, mass, cMassPose, array2vec3(arr));
           })
      .def("updateLinkMassAndInertia", &ArticulationBuilder::updateLinkMassAndInertia,
           py::arg("link"), py::arg("density") = 1.f)
      .def("addBoxVisualToLink",
           [](ArticulationBuilder &a, PxArticulationLink &link, const PxTransform &pose,
              const py::array_t<float> &scale, const py::array_t<float> &color) {
             a.addBoxVisualToLink(link, pose, array2vec3(scale), array2vec3(color));
           },
           py::arg("link"), py::arg("pose") = PxTransform{{0, 0, 0}, PxIdentity},
           py::arg("scale") = make_array<float>({1, 1, 1}),
           py::arg("color") = make_array<float>({1, 1, 1}))
      .def("addCapsuleVisualToLink",
           [](ArticulationBuilder &a, PxArticulationLink &link, const PxTransform &pose,
              PxReal radius, PxReal length, const py::array_t<PxReal> &color) {
             a.addCapsuleVisualToLink(link, pose, radius, length, array2vec3(color));
           },
           py::arg("link"), py::arg("pose") = PxTransform{{0, 0, 0}, PxIdentity},
           py::arg("radius") = 1, py::arg("length") = 1,
           py::arg("color") = make_array<float>({1, 1, 1}))
      .def("addSphereVisualToLink",
           [](ArticulationBuilder &a, PxArticulationLink &link, const PxTransform &pose,
              PxReal radius, const py::array_t<PxReal> &color) {
             a.addSphereVisualToLink(link, pose, radius, array2vec3(color));
           },
           py::arg("link"), py::arg("pose") = PxTransform{{0, 0, 0}, PxIdentity},
           py::arg("radius") = 1, py::arg("color") = make_array<float>({1, 1, 1}))
      .def("addObjVisualToLink",
           [](ArticulationBuilder &a, PxArticulationLink &link, const std::string &filename,
              const PxTransform &pose, const py::array_t<float> &scale) {
             a.addObjVisualToLink(link, filename, pose, array2vec3(scale));
           },
           py::arg("link"), py::arg("filename"),
           py::arg("pose") = PxTransform({0, 0, 0}, PxIdentity),
           py::arg("scale") = make_array<float>({1, 1, 1}))
      .def("build", &ArticulationBuilder::build, py::arg("fixBase") = true,
           py::arg("balanceForce") = false, py::return_value_policy::reference);
}
