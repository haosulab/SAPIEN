#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "PxArticulation.h"
#include "actor_builder.h"
#include "articulation_builder.h"
#include "articulation_interface.h"
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

  std::vector<std::array<physx::PxReal, 2>> get_qlimits() const override {
    PYBIND11_OVERLOAD_PURE_NAME(PYBIND11_TYPE(std::vector<std::array<physx::PxReal, 2>>),
                                PYBIND11_TYPE(IArticulationBase), "get_qlimits", get_qlimits);
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
// class PyIArticulationDrivable : public IArticulationDrivable, PyIArticulationBase {
// public:
//  void set_drive_target(const std::vector<PxReal> &v) override {
//    PYBIND11_OVERLOAD_PURE(void, IArticulationDrivable, set_drive_target);
//  }
//  void move_base(const PxTransform &T) override {
//    PYBIND11_OVERLOAD_PURE(void, IArticulationDrivable, move_base);
//  }
//  std::vector<std::string> get_qnames() const override {
//    PYBIND11_OVERLOAD_PURE(std::vector<std::string>, IArticulationDrivable,
//    get_qnames);
//  }
//};

PYBIND11_MODULE(sapyen, m) {
  py::class_<Simulation>(m, "Simulation")
      .def(py::init<>())
      .def("set_time_step", &Simulation::setTimestep)
      .def("get_time_step", &Simulation::getTimestep)
      .def("set_renderer", &Simulation::setRenderer)
      .def("get_renderer", &Simulation::getRenderer)
      .def("create_actor_builder", &Simulation::createActorBuilder)
      .def("create_articulation_builder", &Simulation::createArticulationBuilder)
      .def("create_urdf_loader", [](Simulation &s) { return new URDF::URDFLoader(s); })
      .def("create_controllable_articulation", &Simulation::createControllableArticulationWrapper,
           py::return_value_policy::reference)
      .def("step", &Simulation::step)
      .def("dump", &Simulation::dump)
      .def("pack",
           [](Simulation &a, const py::array_t<float> &arr) {
             a.pack(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
           })
      .def("update_renderer", &Simulation::updateRenderer)
      .def("add_ground", &Simulation::addGround, py::arg("altitude"), py::arg("render") = true,
           py::arg("material") = nullptr)
      .def("create_material", &Simulation::createPhysicalMaterial)
      .def("add_mounted_camera", &Simulation::addMountedCamera)
      .def(
          "get_render_name_dict", [](Simulation &sim) { return sim.mRenderId2VisualName; },
          py::return_value_policy::copy);

  py::class_<PxRigidActor, std::unique_ptr<PxRigidActor, py::nodelete>>(m, "PxRigidActor")
      .def("get_global_pose", &PxRigidActor::getGlobalPose)
      .def("set_global_pose", &PxRigidActor::setGlobalPose, py::arg("pose"),
           py::arg("autoawake") = true);
  py::class_<PxRigidStatic, PxRigidActor, std::unique_ptr<PxRigidStatic, py::nodelete>>(
      m, "PxRigidStatic")
      .def("get_global_pose", &PxRigidStatic::getGlobalPose);

  py::class_<PxRigidBody, PxRigidActor, std::unique_ptr<PxRigidBody, py::nodelete>>(m,
                                                                                    "PxRigidBody")
      .def("get_global_pose", &PxRigidBody::getGlobalPose)
      .def("get_local_mass_center", &PxRigidBody::getCMassLocalPose)
      .def("add_force",
           [](PxRigidBody &a, const py::array_t<float> &arr) {
             a.addForce(PxVec3(arr.at(0), arr.at(1), arr.at(2)));
             a.addTorque(PxVec3(arr.at(3), arr.at(4), arr.at(5)));
           })
      .def("get_global_mass_center",
           [](PxRigidBody &a) {
             auto globalPose = a.getGlobalPose();
             auto globalMassCenter = globalPose.transform(a.getCMassLocalPose());
             return globalMassCenter;
           })
      .def("get_linear_velocity",
           [](PxRigidBody &a) {
             physx::PxVec3 vel = a.getLinearVelocity();
             return py::array_t<PxReal>(3, (PxReal *)(&vel));
           })
      .def("get_angular_velocity", [](PxRigidBody &a) {
        physx::PxVec3 vel = a.getAngularVelocity();
        return py::array_t<PxReal>(3, (PxReal *)(&vel));
      });
  py::class_<PxRigidDynamic, PxRigidBody, std::unique_ptr<PxRigidDynamic, py::nodelete>>(
      m, "PxRigidDynamic")
      .def("get_global_pose", &PxRigidDynamic::getGlobalPose)
      .def("get_linear_velocity",
           [](PxRigidDynamic &a) {
             physx::PxVec3 vel = a.getLinearVelocity();
             return py::array_t<PxReal>(3, (PxReal *)(&vel));
           })
      .def("get_angular_velocity", [](PxRigidDynamic &a) {
        physx::PxVec3 vel = a.getAngularVelocity();
        return py::array_t<PxReal>(3, (PxReal *)(&vel));
      });
  py::class_<PxArticulationLink, PxRigidBody, std::unique_ptr<PxArticulationLink, py::nodelete>>(
      m, "PxArticulationLink")
      .def("get_global_pose", &PxArticulationLink::getGlobalPose)
      .def("get_linear_velocity",
           [](PxArticulationLink &a) {
             physx::PxVec3 vel = a.getLinearVelocity();
             return py::array_t<PxReal>(3, (PxReal *)(&vel));
           })
      .def("get_angular_velocity", [](PxArticulationLink &a) {
        physx::PxVec3 vel = a.getAngularVelocity();
        return py::array_t<PxReal>(3, (PxReal *)(&vel));
      });

  py::class_<PxMaterial, std::unique_ptr<PxMaterial, py::nodelete>>(m, "PxMaterial")
      .def("get_static_friction", &PxMaterial::getStaticFriction)
      .def("get_dynamic_friction", &PxMaterial::getDynamicFriction)
      .def("get_restitution", &PxMaterial::getRestitution)
      .def("set_static_friction", &PxMaterial::setStaticFriction)
      .def("set_dynamic_friction", &PxMaterial::setDynamicFriction)
      .def("set_restitution", &PxMaterial::setRestitution);

  py::class_<Renderer::ISensor, PyISensor>(m, "ISensor")
      .def("get_sensor_pose", &Renderer::ISensor::getSensorPose)
      .def("set_sensor_pose", &Renderer::ISensor::setSensorPose);

  py::class_<Renderer::ICamera, Renderer::ISensor>(m, "ICamera")
      .def("get_name", &Renderer::ICamera::getName)
      .def("get_width", &Renderer::ICamera::getWidth)
      .def("get_height", &Renderer::ICamera::getHeight)
      .def("get_hovy", &Renderer::ICamera::getFovy)
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

  py::class_<Renderer::ICameraManager>(m, "ICameraManager");

  py::class_<Renderer::IPhysxRenderer, Renderer::ICameraManager>(m, "IPhysxRenderer");

  py::class_<Renderer::OptifuserRenderer, Renderer::IPhysxRenderer>(m, "OptifuserRenderer")
      .def(py::init<>())
      .def("render", &Renderer::OptifuserRenderer::render)
      .def("show_window", &Renderer::OptifuserRenderer::showWindow)
      .def("hide_window", &Renderer::OptifuserRenderer::hideWindow)
      .def_readonly("cam", &Renderer::OptifuserRenderer::cam)
      .def("get_camera_count",
           [](Renderer::OptifuserRenderer &r) { return r.getCameras().size(); })
      .def(
          "get_camera", [](Renderer::OptifuserRenderer &r, int i) { return r.getCameras().at(i); },
          py::return_value_policy::reference)
      .def(
          "add_point_light",
          [](Renderer::OptifuserRenderer &r, py::array_t<float> const &position,
             py::array_t<float> const &color) {
            r.addPointLight({position.at(0), position.at(1), position.at(2)},
                            {color.at(0), color.at(1), color.at(2)});
          },
          py::arg("position"), py::arg("color"))
      .def(
          "set_ambient_light",
          [](Renderer::OptifuserRenderer &r, py::array_t<float> const &color) {
            r.setAmbientLight({color.at(0), color.at(1), color.at(2)});
          },
          py::arg("color"))
      .def(
          "set_shadow_light",
          [](Renderer::OptifuserRenderer &r, py::array_t<float> const &direction,
             py::array_t<float> const &color) {
            r.setShadowLight({direction.at(0), direction.at(1), direction.at(2)},
                             {color.at(0), color.at(1), color.at(2)});
          },
          py::arg("direction"), py::arg("color"));

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
      .def("get_model_mat", [](Optifuser::CameraSpec &c) { return mat42array(c.getModelMat()); })
      .def("get_projection_mat",
           [](Optifuser::CameraSpec &c) { return mat42array(c.getProjectionMat()); });
  py::class_<Renderer::MountedCamera, Optifuser::CameraSpec, Renderer::ICamera>(m,
                                                                                "mounted_camera")
      .def("get_camera_matrix",
           [](Renderer::MountedCamera &a) { return mat42array(a.getCameraMatrix()); });

  py::class_<Optifuser::FPSCameraSpec, Optifuser::CameraSpec>(m, "FPSCameraSpec")
      .def(py::init<>())
      .def("update", &Optifuser::FPSCameraSpec::update)
      .def("is_sane", &Optifuser::FPSCameraSpec::isSane)
      .def("set_forward",
           [](Optifuser::FPSCameraSpec &c, const py::array_t<float> &dir) {
             c.setForward({dir.at(0), dir.at(1), dir.at(2)});
           })
      .def("set_up",
           [](Optifuser::FPSCameraSpec &c, const py::array_t<float> &dir) {
             c.setUp({dir.at(0), dir.at(1), dir.at(2)});
           })
      .def("rotate_yaw_pitch", &Optifuser::FPSCameraSpec::rotateYawPitch)
      .def("move_forward_right", &Optifuser::FPSCameraSpec::moveForwardRight)
      .def("get_rotation0", [](Optifuser::FPSCameraSpec &c) {
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
      .def("get_link_names", &IArticulationBase::get_link_names)
      .def("get_link_ids", &IArticulationBase::get_link_ids)
      .def("get_link_joint_indices", &IArticulationBase::get_link_joint_indices)
      .def("get_links", &IArticulationBase::get_links, py::return_value_policy::reference)
      .def("get_joint_dofs",
           [](IArticulationBase &a) {
             auto dofs = a.get_joint_dofs();
             return py::array_t<uint32_t>(dofs.size(), dofs.data());
           })
      .def("get_qlimits",
           [](IArticulationBase &a) {
             auto limits = a.get_qlimits();
             return py::array_t<PxReal>({(int)limits.size(), 2},
                                        {sizeof(std::array<PxReal, 2>), sizeof(PxReal)},
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
      .def("set_qf",
           [](IArticulationBase &a, const py::array_t<float> &arr) {
             a.set_qf(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
           })
      .def("get_link_joint_pose", &IArticulationBase::get_link_joint_pose)
      .def("get_qnames", &IArticulationDrivable::get_qnames)
      .def("get_joint_types", &IArticulationDrivable::get_joint_types);

  py::class_<IArticulationDrivable, IArticulationBase>(m, "IArticulationDrivable")
      .def(
          "set_root_pose",
          [](IArticulationDrivable &a, const py::array_t<float> &position,
             const py::array_t<float> &quaternion) {
            a.move_base(
                {{position.at(0), position.at(1), position.at(2)},
                 {quaternion.at(1), quaternion.at(2), quaternion.at(3), quaternion.at(0)}});
          },
          py::arg("position") = make_array<float>({0, 0, 0}),
          py::arg("quaternion") = make_array<float>({0, 0, 0, 1}))
      .def("set_drive_qpos", [](ArticulationWrapper &a, py::array_t<float> qpos) {
        a.set_drive_target(std::vector<PxReal>(qpos.data(), qpos.data() + qpos.size()));
      });

  py::enum_<EArticulationType>(articulationBase, "ArticulationType")
      .value("DYNAMIC_ARTICULATION", EArticulationType::DYNAMIC_ARTICULATION)
      .value("KINEMATIC_ARTICULATION", EArticulationType::KINEMATIC_ARTICULATION)
      .value("OBJECT_ARTICULATION", EArticulationType::OBJECT_ARTICULATION);
  py::class_<ArticulationWrapper, IArticulationDrivable>(m, "ArticulationWrapper")
      .def("update_cache", &ArticulationWrapper::updateCache)
      .def("update_articulation", &ArticulationWrapper::updateArticulation)
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
      .def("set_pd", &ArticulationWrapper::set_drive_property, py::arg("p"), py::arg("d"),
           py::arg("force_limit") = PX_MAX_F32, py::arg("joint_index") = py::list())
      .def("get_drivable_joint_name", &IArticulationBase::get_qnames)
      .def("balance_passive_force", &ArticulationWrapper::set_force_balance);

  py::class_<JointSystem, IArticulationBase>(m, "JointSystem");
  py::class_<KinematicsArticulationWrapper, IArticulationDrivable>(
      m, "KinematicsArticulationWrapper");

  py::class_<ControllableArticulationWrapper>(m, "ControllableArticulation");

  py::class_<URDF::URDFLoader>(m, "URDFLoader")
      .def(py::init<Simulation &>())
      .def_readwrite("fix_loaded_object", &URDF::URDFLoader::fixLoadedObject)
      .def_readwrite("balance_passive_force", &URDF::URDFLoader::balancePassiveForce)
      .def("load", &URDF::URDFLoader::load, py::return_value_policy::reference,
           py::arg("filename"), py::arg("material") = nullptr)
      .def("load_kinematic", &URDF::URDFLoader::loadKinematic, py::return_value_policy::reference)
      .def("load_joint_system", &URDF::URDFLoader::loadJointSystem,
           py::return_value_policy::reference);

  py::class_<PxTransform>(m, "Pose")
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
      .def("inv", &PxTransform::getInverse)
      .def("__repr__",
           [](const PxTransform &pose) {
             std::ostringstream oss;
             oss << "Position: x: " << pose.p.x << ", y: " << pose.p.y << ", z: " << pose.p.z
                 << "\n";
             oss << "Quaternion: w: " << pose.q.w << ", x: " << pose.q.x << ", y: " << pose.q.y
                 << ", z: " << pose.q.z << "\n";
             std::string repr = oss.str();
             return repr;
           })
      .def("transform", [](PxTransform &t, PxTransform &src) { return t.transform(src); })
      .def("set_p", [](PxTransform &t, const py::array_t<PxReal> &arr) { t.p = array2vec3(arr); })
      .def("set_q", [](PxTransform &t, const py::array_t<PxReal> &arr) {
        t.q = {arr.at(1), arr.at(2), arr.at(3), arr.at(0)}; // NOTE: wxyz to xyzw
      });

  py::class_<PxArticulationJointType> jointtype(m, "PxArticulationJointType");

  py::enum_<PxArticulationJointType::Enum>(jointtype, "ArticulationJointType")
      .value("PRISMATIC", PxArticulationJointType::ePRISMATIC)
      .value("REVOLUTE", PxArticulationJointType::eREVOLUTE)
      .value("SPHERICAL", PxArticulationJointType::eSPHERICAL)
      .value("FIX", PxArticulationJointType::eFIX)
      .value("UNDEFINED", PxArticulationJointType::eUNDEFINED)
      .export_values();

  py::class_<ArticulationBuilder>(m, "ArticulationBuilder")
      .def(py::init<Simulation *>())
      .def(
          "add_link",
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
          py::arg("joint_name") = "", py::arg("joint_type") = PxArticulationJointType::eUNDEFINED,
          py::arg("limits") = py::array_t<float>(),
          py::arg("parent_pose") = PxTransform({0, 0, 0}, PxIdentity),
          py::arg("child_pose") = PxTransform({0, 0, 0}, PxIdentity))
      .def(
          "add_box_shape_to_link",
          [](ArticulationBuilder &a, PxArticulationLink &link, const PxTransform &pose,
             const py::array_t<float> &arr,
             PxMaterial *material) { a.addBoxShapeToLink(link, pose, array2vec3(arr), material); },
          py::arg("link"), py::arg("pose") = PxTransform{{0, 0, 0}, PxIdentity},
          py::arg("scale") = make_array<float>({1, 1, 1}), py::arg("material") = nullptr)
      .def("add_capsule_shape_to_link", &ArticulationBuilder::addCapsuleShapeToLink,
           py::arg("link"), py::arg("pose") = PxTransform{{0, 0, 0}, PxIdentity},
           py::arg("radius") = 1, py::arg("length") = 1, py::arg("material") = nullptr)
      .def("add_sphere_shape_to_link", &ArticulationBuilder::addSphereShapeToLink, py::arg("link"),
           py::arg("pose") = PxTransform{{0, 0, 0}, PxIdentity}, py::arg("radius") = 1,
           py::arg("material") = nullptr)
      .def(
          "add_convex_obj_shape_to_link",
          [](ArticulationBuilder &a, PxArticulationLink &link, const std::string &filename,
             const PxTransform &pose, const py::array_t<float> &arr, PxMaterial *material) {
            a.addConvexObjShapeToLink(link, filename, pose, array2vec3(arr), material);
          },
          py::arg("link"), py::arg("filename"),
          py::arg("pose") = PxTransform{{0, 0, 0}, PxIdentity},
          py::arg("scale") = make_array<float>({1, 1, 1}), py::arg("material") = nullptr)
      .def("set_link_mass_and_inertia",
           [](ArticulationBuilder &a, PxArticulationLink &link, PxReal mass,
              const PxTransform &cMassPose, const py::array_t<float> &arr) {
             a.setLinkMassAndInertia(link, mass, cMassPose, array2vec3(arr));
           })
      .def("update_link_mass_and_inertia", &ArticulationBuilder::updateLinkMassAndInertia,
           py::arg("link"), py::arg("density") = 1.f)
      .def(
          "add_box_visual_to_link",
          [](ArticulationBuilder &a, PxArticulationLink &link, const PxTransform &pose,
             const py::array_t<float> &scale, const py::array_t<float> &color,
             const std::string &name) {
            a.addBoxVisualToLink(link, pose, array2vec3(scale), array2vec3(color), name);
          },
          py::arg("link"), py::arg("pose") = PxTransform{{0, 0, 0}, PxIdentity},
          py::arg("scale") = make_array<float>({1, 1, 1}),
          py::arg("color") = make_array<float>({1, 1, 1}), py::arg("name") = "")
      .def(
          "add_capsule_visual_to_link",
          [](ArticulationBuilder &a, PxArticulationLink &link, const PxTransform &pose,
             PxReal radius, PxReal length, const py::array_t<PxReal> &color,
             const std::string &name) {
            a.addCapsuleVisualToLink(link, pose, radius, length, array2vec3(color), name);
          },
          py::arg("link"), py::arg("pose") = PxTransform{{0, 0, 0}, PxIdentity},
          py::arg("radius") = 1, py::arg("length") = 1,
          py::arg("color") = make_array<float>({1, 1, 1}), py::arg("name") = "")
      .def(
          "add_sphere_visual_to_link",
          [](ArticulationBuilder &a, PxArticulationLink &link, const PxTransform &pose,
             PxReal radius, const py::array_t<PxReal> &color, const std::string &name) {
            a.addSphereVisualToLink(link, pose, radius, array2vec3(color), name);
          },
          py::arg("link"), py::arg("pose") = PxTransform{{0, 0, 0}, PxIdentity},
          py::arg("radius") = 1, py::arg("color") = make_array<float>({1, 1, 1}),
          py::arg("name") = "")
      .def(
          "add_obj_visual_to_link",
          [](ArticulationBuilder &a, PxArticulationLink &link, const std::string &filename,
             const PxTransform &pose, const py::array_t<float> &scale, const std::string &name) {
            a.addObjVisualToLink(link, filename, pose, array2vec3(scale), name);
          },
          py::arg("link"), py::arg("filename"),
          py::arg("pose") = PxTransform({0, 0, 0}, PxIdentity),
          py::arg("scale") = make_array<float>({1, 1, 1}), py::arg("name") = "")
      .def("build", &ArticulationBuilder::build, py::arg("fix_base") = true,
           py::arg("balanceForce") = false, py::return_value_policy::reference);

  py::class_<ActorBuilder>(m, "ActorBuilder")
      .def(
          "add_box_shape",
          [](ActorBuilder &a, PxTransform const &pose, py::array_t<float> const &size,
             PxMaterial *material, PxReal density) {
            a.addBoxShape(pose, {size.at(0), size.at(1), size.at(2)}, material, density);
          },
          py::arg("pose") = PxTransform({0, 0, 0}, {0, 0, 0, 1}),
          py::arg("size") = make_array<float>({1, 1, 1}), py::arg("material") = nullptr,
          py::arg("density") = 1000.f)
      .def("build", &ActorBuilder::build, py::arg("is_static") = false,
           py::arg("is_kinematic") = false, py::arg("name") = "", py::arg("add_to_scene") = true);
  // .def("add_sphere_shape", &ActorBuilder::addSphereShape)
  //     .def("add__shape", &ActorBuilder::addBoxShape)
  //     .def("add_box_shape", &ActorBuilder::addBoxShape)
}
