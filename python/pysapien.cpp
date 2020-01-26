#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "actor_builder.h"
#include "renderer/render_interface.h"
#include "sapien_actor.h"
#include "sapien_actor_base.h"
#include "sapien_contact.h"
#include "sapien_scene.h"
#include "simulation.h"

#include "articulation/articulation_builder.h"
#include "articulation/sapien_articulation.h"
#include "articulation/sapien_articulation_base.h"
#include "articulation/sapien_joint.h"
#include "articulation/sapien_link.h"
#include "articulation/urdf_loader.h"

#include "renderer/optifuser_controller.h"
#include "renderer/optifuser_renderer.h"

using namespace sapien;
namespace py = pybind11;

PxVec3 array2vec3(const py::array_t<float> &arr) { return {arr.at(0), arr.at(1), arr.at(2)}; }

template <typename T> py::array_t<T> make_array(std::vector<T> const &values) {
  return py::array_t(values.size(), values.data());
}

py::array_t<PxReal> vec32array(PxVec3 const &vec) {
  std::vector<PxReal> v = {vec.x, vec.y, vec.z};
  return make_array(v);
}

py::array_t<float> mat42array(glm::mat4 const &mat) {
  float arr[] = {mat[0][0], mat[1][0], mat[2][0], mat[3][0], mat[0][1], mat[1][1],
                 mat[2][1], mat[3][1], mat[0][2], mat[1][2], mat[2][2], mat[3][2],
                 mat[0][3], mat[1][3], mat[2][3], mat[3][3]};
  return py::array_t<float>({4, 4}, arr);
}

PYBIND11_MODULE(pysapien, m) {

  //======== Internal ========//
  py::enum_<PxSolverType::Enum>(m, "SolverType")
      .value("PGS", PxSolverType::ePGS)
      .value("TGS", PxSolverType::eTGS)
      .export_values();

  py::enum_<PxArticulationJointType::Enum>(m, "ArticulationJointType")
      .value("PRISMATIC", PxArticulationJointType::ePRISMATIC)
      .value("REVOLUTE", PxArticulationJointType::eREVOLUTE)
      .value("SPHERICAL", PxArticulationJointType::eSPHERICAL)
      .value("FIX", PxArticulationJointType::eFIX)
      .value("UNDEFINED", PxArticulationJointType::eUNDEFINED)
      .export_values();

  py::class_<PxMaterial, std::unique_ptr<PxMaterial, py::nodelete>>(m, "PxMaterial")
      .def("get_static_friction", &PxMaterial::getStaticFriction)
      .def("get_dynamic_friction", &PxMaterial::getDynamicFriction)
      .def("get_restitution", &PxMaterial::getRestitution)
      .def("set_static_friction", &PxMaterial::setStaticFriction, py::arg("coef"))
      .def("set_dynamic_friction", &PxMaterial::setDynamicFriction, py::arg("coef"))
      .def("set_restitution", &PxMaterial::setRestitution, py::arg("coef"));

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
      .def("set_p", [](PxTransform &t, const py::array_t<PxReal> &arr) { t.p = array2vec3(arr); },
           py::arg("p"))
      .def("set_q",
           [](PxTransform &t, const py::array_t<PxReal> &arr) {
             t.q = {arr.at(1), arr.at(2), arr.at(3), arr.at(0)}; // NOTE: wxyz to xyzw
           },
           py::arg("q"))
      .def(py::self * py::self);

  //======== Render Interface ========//
  py::class_<Renderer::IPxrRenderer>(m, "IPxrRenderer");
  py::class_<Renderer::IPxrScene>(m, "IPxrScene");

  py::class_<Renderer::ISensor>(m, "ISensor")
      .def("setInitialPose", &Renderer::ISensor::setInitialPose, py::arg("pose"))
      .def("getPose", &Renderer::ISensor::getPose)
      .def("setPose", &Renderer::ISensor::setPose, py::arg("pose"));

  py::class_<Renderer::ICamera, Renderer::ISensor>(m, "ICamera")
      .def("get_name", &Renderer::ICamera::getName)
      .def("get_width", &Renderer::ICamera::getWidth)
      .def("get_height", &Renderer::ICamera::getHeight)
      .def("get_fovy", &Renderer::ICamera::getFovy)
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

  py::class_<Renderer::OptifuserRenderer, Renderer::IPxrRenderer>(m, "OptifuserRenderer")
      .def_static("set_default_shader_config", Renderer::OptifuserRenderer::setDefaultShaderConfig,
                  py::arg("glsl_dir"), py::arg("glsl_version"))
      .def(py::init<std::string const &, std::string const &>(), py::arg("glsl_dir") = "",
           py::arg("glsl_version") = "");

  py::class_<Renderer::OptifuserController>(m, "OptifuserController")
      .def(py::init<Renderer::OptifuserRenderer *>(), py::arg("renderer"))
      .def("show_window", &Renderer::OptifuserController::showWindow)
      .def("hide_window", &Renderer::OptifuserController::hideWindow)
      .def("set_current_scene", &Renderer::OptifuserController::setCurrentScene, py::arg("scene"))
      .def("render", &Renderer::OptifuserController::render)
      .def("set_camera_position", &Renderer::OptifuserController::setCameraPosition, py::arg("x"),
           py::arg("y"), py::arg("z"))
      .def("set_camera_rotation", &Renderer::OptifuserController::setCameraRotation,
           py::arg("yaw"), py::arg("pitch"))
      .def("focus", &Renderer::OptifuserController::focus, py::arg("actor"))
      .def_property_readonly("should_quit", &Renderer::OptifuserController::shouldQuit);

  py::class_<Optifuser::CameraSpec>(m, "CameraSpec")
      .def(py::init([]() { return new Optifuser::CameraSpec(); }))
      .def_readwrite("name", &Optifuser::CameraSpec::name)
      .def("set_position",
           [](Optifuser::CameraSpec &c, const py::array_t<float> &arr) {
             c.position = {arr.at(0), arr.at(1), arr.at(2)};
           },
           py::arg("position"))
      .def("set_rotation",
           [](Optifuser::CameraSpec &c, const py::array_t<float> &arr) {
             c.setRotation({arr.at(0), arr.at(1), arr.at(2), arr.at(3)});
           },
           py::arg("rotation"))
      .def_property_readonly(
          "position",
          [](Optifuser::CameraSpec &c) { return py::array_t<float>(3, (float *)(&c.position)); })
      .def_property_readonly("rotation",
                             [](Optifuser::CameraSpec &c) {
                               auto rot = c.getRotation();
                               return make_array<float>({rot.w, rot.x, rot.y, rot.z});
                             })
      .def_readwrite("near", &Optifuser::CameraSpec::near)
      .def_readwrite("far", &Optifuser::CameraSpec::far)
      .def_readwrite("fovy", &Optifuser::CameraSpec::fovy)
      .def_readwrite("aspect", &Optifuser::CameraSpec::aspect)
      .def("lookAt",
           [](Optifuser::CameraSpec &c, const py::array_t<float> &dir,
              const py::array_t<float> &up) {
             c.lookAt({dir.at(0), dir.at(1), dir.at(2)}, {up.at(0), up.at(1), up.at(2)});
           },
           py::arg("direction"), py::arg("up"))
      .def("get_model_matrix",
           [](Optifuser::CameraSpec &c) { return mat42array(c.getModelMat()); })
      .def("get_projection_matrix",
           [](Optifuser::CameraSpec &c) { return mat42array(c.getProjectionMat()); });

  py::class_<Optifuser::FPSCameraSpec, Optifuser::CameraSpec>(m, "FPSCameraSpec")
      .def("update", &Optifuser::FPSCameraSpec::update)
      .def("is_sane", &Optifuser::FPSCameraSpec::isSane)
      .def("set_forward",
           [](Optifuser::FPSCameraSpec &c, const py::array_t<float> &dir) {
             c.setForward({dir.at(0), dir.at(1), dir.at(2)});
           },
           py::arg("forward"))
      .def("set_up",
           [](Optifuser::FPSCameraSpec &c, const py::array_t<float> &dir) {
             c.setUp({dir.at(0), dir.at(1), dir.at(2)});
           },
           py::arg("up"))
      .def("rotate_yaw_pitch", &Optifuser::FPSCameraSpec::rotateYawPitch, py::arg("delta_yaw"),
           py::arg("delta_pitch"))
      .def("move_forward_right", &Optifuser::FPSCameraSpec::moveForwardRight,
           py::arg("delta_forward"), py::arg("delta_right"))
      .def("get_rotation0", [](Optifuser::FPSCameraSpec &c) {
        glm::quat q = c.getRotation0();
        return make_array<float>({q.w, q.x, q.y, q.z});
      });

  py::class_<Renderer::OptifuserCamera, Optifuser::CameraSpec, Renderer::ICamera>(
      m, "OptifuserCamera")
      .def("get_camera_matrix",
           [](Renderer::OptifuserCamera &c) { return mat42array(c.getCameraMatrix()); })

#ifdef _USE_OPTIX
      .def("take_raytraced_picture",
           [](Renderer::OptifuserCamera &cam, uint32_t samplesPerPixel, uint32_t reflectionCount) {
             return py::array_t<float>(
                 {static_cast<int>(cam.getHeight()), static_cast<int>(cam.getWidth()), 4},
                 cam.takeRaytracedPicture(samplesPerPixel, reflectionCount).data());
           },
           py::arg("samples_per_pixel") = 128, py::arg("reflection_count") = 4)
#endif
      ;

  //======== Simulation ========//
  py::class_<Simulation>(m, "Simulation")
      .def(py::init<>())
      .def("set_renderer", &Simulation::setRenderer, py::arg("renderer"))
      .def("get_renderer", &Simulation::getRenderer, py::return_value_policy::reference)
      .def("create_physical_material", &Simulation::createPhysicalMaterial,
           py::return_value_policy::reference)
      .def("create_scene",
           [](Simulation &sim, py::array_t<PxReal> const &gravity, PxSolverType::Enum solverType,
              bool enableCCD, bool enablePCM) {
             PxSceneFlags flags = PxSceneFlags();
             if (enableCCD) {
               flags |= PxSceneFlag::eENABLE_CCD;
             }
             if (enablePCM) {
               flags |= PxSceneFlag::eENABLE_PCM;
             }
             return sim.createScene(array2vec3(gravity), solverType, flags);
           },
           py::arg("gravity") = make_array<PxReal>({0, 0, -9.8}),
           py::arg("solver_type") = PxSolverType::ePGS, py::arg("enable_ccd") = false,
           py::arg("enable_pcm") = false);

  py::class_<SScene>(m, "Scene")
      .def_property_readonly("name", &SScene::getName)
      .def("set_timestep", &SScene::setTimestep, py::arg("second"))
      .def("get_timestep", &SScene::getTimestep)
      .def_property("timestep", &SScene::getTimestep, &SScene::setTimestep)
      .def("create_actor_builder", &SScene::createActorBuilder)
      .def("create_articulation_builder", &SScene::createArticulationBuilder)
      .def("create_urdf_loader", &SScene::createURDFLoader)
      .def("remove_actor", &SScene::removeActor, py::arg("actor"))
      .def("remove_articulation", &SScene::removeArticulation, py::arg("articulation"))
      .def("find_actor_by_id", &SScene::findActorById, py::arg("id"),
           py::return_value_policy::reference)
      .def("find_articulation_link_by_link_id", &SScene::findArticulationLinkById, py::arg("id"),
           py::return_value_policy::reference)

      .def("add_mounted_camera", &SScene::addMountedCamera, py::arg("name"), py::arg("actor"),
           py::arg("pose") = PxTransform(PxIdentity), py::arg("width"), py::arg("height"),
           py::arg("fovx"), py::arg("fovy"), py::arg("near"), py::arg("far"),
           py::return_value_policy::reference)
      .def("get_mounted_cameras", &SScene::getMountedCameras, py::return_value_policy::reference)
      .def("get_mounted_actors", &SScene::getMountedActors, py::return_value_policy::reference)
      .def("remove_mounted_camera", &SScene::removeMountedCamera, py::arg("camera"))
      .def("find_mounted_camera", &SScene::findMountedCamera, py::arg("name"),
           py::arg("actor") = nullptr, py::return_value_policy::reference)

      .def("step", &SScene::step)
      .def("update_render", &SScene::updateRender)
      .def("add_ground", &SScene::addGround, py::arg("altitude"), py::arg("render") = true,
           py::arg("material") = nullptr)
      .def("get_contacts", &SScene::getContacts)

      .def("set_shadow_light",
           [](SScene &s, py::array_t<PxReal> const &direction, py::array_t<PxReal> const &color) {
             s.setShadowLight(array2vec3(direction), array2vec3(color));
           },
           py::arg("direction"), py::arg("color"))
      .def("set_ambient_light",
           [](SScene &s, py::array_t<PxReal> const &color) {
             s.setAmbientLight(array2vec3(color));
           },
           py::arg("clor"))
      .def("add_point_light",
           [](SScene &s, py::array_t<PxReal> const &position, py::array_t<PxReal> const &color) {
             s.addPointLight(array2vec3(position), array2vec3(color));
           },
           py::arg("position"), py::arg("color"))
      .def("add_directional_light",
           [](SScene &s, py::array_t<PxReal> const &direction, py::array_t<PxReal> const &color) {
             s.addDirectionalLight(array2vec3(direction), array2vec3(color));
           },
           py::arg("direction"), py::arg("color"))
      .def("get_renderer_scene", &SScene::getRendererScene);

  //======== Actor ========//

  py::class_<SActorBase>(m, "ActorBase")
      .def_property("name", &SActorBase::getName, &SActorBase::setName)
      .def("get_name", &SActorBase::getName)
      .def("set_name", &SActorBase::setName, py::arg("name"))
      .def_property_readonly("id", &SActorBase::getId)
      .def("get_id", &SActorBase::getId)
      .def("get_scene", &SActorBase::getScene, py::return_value_policy::reference)
      .def_property_readonly("pose", &SActorBase::getPose)
      .def_property_readonly("col1", &SActorBase::getCollisionGroup1)
      .def_property_readonly("col2", &SActorBase::getCollisionGroup2)
      .def_property_readonly("col3", &SActorBase::getCollisionGroup3)

      // TODO: check if it is okay to return a vector of REFERENCED raw pointers
      // .def_property_readonly("render_bodies", &SActorBase::getRenderBodies)
      ;

  py::class_<SActorDynamicBase, SActorBase>(m, "ActorDynamicBase")
      .def_property_readonly("velocity",
                             [](SActorDynamicBase &a) { return vec32array(a.getVel()); })
      .def_property_readonly("angular_velocity",
                             [](SActorDynamicBase &a) { return vec32array(a.getAngularVel()); })
      .def_property_readonly("mass", &SActorDynamicBase::getMass)
      .def_property_readonly("inertia",
                             [](SActorDynamicBase &a) { return vec32array(a.getInertia()); })
      .def_property_readonly("cmass_local_pose", &SActorDynamicBase::getCMassLocalPose)
      .def("add_force_at_point",
           [](SActorDynamicBase &a, py::array_t<PxReal> const &force,
              py::array_t<PxReal> const &point) {
             a.addForceAtPoint(array2vec3(force), array2vec3(point));
           },
           py::arg("force"), py::arg("point"));

  py::class_<SActorStatic, SActorBase>(m, "ActorStatic")
      .def("set_pose", &SActorStatic::setPose, py::arg("pose"));

  py::class_<SActor, SActorDynamicBase>(m, "Actor")
      .def("set_pose", &SActor::setPose, py::arg("pose"));

  py::class_<SLinkBase, SActorDynamicBase>(m, "LinkBase")
      .def("get_index", &SLinkBase::getIndex)
      .def("get_articulation", &SLinkBase::getArticulation, py::return_value_policy::reference);

  py::class_<SLink, SLinkBase>(m, "Link").def("get_articulation", &SLink::getArticulation,
                                              py::return_value_policy::reference);

  //======== End Actor ========//

  //======== Joint ========//
  py::class_<SJointBase>(m, "JointBase")
      .def_property("name", &SJointBase::getName, &SJointBase::setName)
      .def("get_name", &SJointBase::getName)
      .def("set_name", &SJointBase::setName, py::arg("name"))
      .def("get_parent_link", &SJointBase::getParentLink, py::return_value_policy::reference)
      .def("get_child_link", &SJointBase::getChildLink, py::return_value_policy::reference)
      .def("get_dof", &SJointBase::getDof)
      .def("get_limits",
           [](SJointBase &j) {
             auto limits = j.getLimits();
             return py::array_t<PxReal>({(int)limits.size(), 2},
                                        {sizeof(std::array<PxReal, 2>), sizeof(PxReal)},
                                        reinterpret_cast<PxReal *>(limits.data()));
           })
      .def("set_limits",
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
           py::arg("limits"));

  py::class_<SJoint, SJointBase>(m, "Joint")
      .def("set_friction", &SJoint::setFriction, py::arg("friction"))
      .def("set_drive_property", &SJoint::setDriveProperty, py::arg("stiffness"),
           py::arg("damping"), py::arg("force_limit") = PX_MAX_F32)
      .def("set_drive_velocity_target", [](SJoint &j, PxReal v) { j.setDriveVelocityTarget(v); },
           py::arg("velocity"))
      .def("set_drive_target", [](SJoint &j, PxReal p) { j.setDriveTarget(p); }, py::arg("target"))
      // TODO wrapper for array-valued targets
      .def("get_global_pose", &SJoint::getGlobalPose);

  //======== End Joint ========//

  //======== Articulation ========//
  py::enum_<EArticulationType>(m, "ArticulationType")
      .value("DYNAMIC", EArticulationType::DYNAMIC)
      .value("KINEMATIC", EArticulationType::KINEMATIC)
      .export_values();

  py::class_<SArticulationBase>(m, "ArticulationBase")
      .def_property("name", &SArticulationBase::getName, &SArticulationBase::setName)
      .def("get_name", &SArticulationBase::getName)
      .def("set_name", &SArticulationBase::setName, py::arg("name"))
      .def("get_base_links", &SArticulationBase::getBaseLinks, py::return_value_policy::reference)
      .def("get_base_joints", &SArticulationBase::getBaseJoints,
           py::return_value_policy::reference)
      .def_property_readonly("type", &SArticulationBase::getType)
      .def_property_readonly("dof", &SArticulationBase::dof)
      .def("get_qpos",
           [](SArticulationBase &a) {
             auto qpos = a.getQpos();
             return py::array_t<PxReal>(qpos.size(), qpos.data());
           })
      .def("set_qpos",
           [](SArticulationBase &a, const py::array_t<float> &arr) {
             a.setQpos(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
           },
           py::arg("qpos"))

      .def("get_qvel",
           [](SArticulationBase &a) {
             auto qvel = a.getQvel();
             return py::array_t<PxReal>(qvel.size(), qvel.data());
           })
      .def("set_qvel",
           [](SArticulationBase &a, const py::array_t<float> &arr) {
             a.setQvel(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
           },
           py::arg("qvel"))
      .def("get_qacc",
           [](SArticulationBase &a) {
             auto qacc = a.getQacc();
             return py::array_t<PxReal>(qacc.size(), qacc.data());
           })
      .def("set_qacc",
           [](SArticulationBase &a, const py::array_t<float> &arr) {
             a.setQacc(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
           },
           py::arg("qacc"))
      .def("get_qf",
           [](SArticulationBase &a) {
             auto qf = a.getQf();
             return py::array_t<PxReal>(qf.size(), qf.data());
           })
      .def("set_qf",
           [](SArticulationBase &a, const py::array_t<float> &arr) {
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
      .def("set_qlimits",
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

      .def("set_root_pose", &SArticulationBase::setRootPose, py::arg("pose"));

  py::class_<SArticulationDrivable, SArticulationBase>(m, "ArticulationDrivable")
      .def("get_drive_target",
           [](SArticulationDrivable &a) {
             auto target = a.getDriveTarget();
             return py::array_t<PxReal>(target.size(), target.data());
           })
      .def("set_drive_target",
           [](SArticulationDrivable &a, const py::array_t<float> &arr) {
             a.setDriveTarget(std::vector<PxReal>(arr.data(), arr.data() + arr.size()));
           },
           py::arg("drive_target"));

  py::class_<SArticulation, SArticulationDrivable>(m, "Articulation")
      .def("get_links", &SArticulation::getSLinks, py::return_value_policy::reference)
      .def("get_joints", &SArticulation::getSJoints, py::return_value_policy::reference)
      .def("set_root_velocity",
           [](SArticulation &a, py::array_t<float> v) { a.setRootVelocity(array2vec3(v)); },
           py::arg("vel"))
      .def("set_root_angular_velocity",
           [](SArticulation &a, py::array_t<float> v) { a.setRootAngularVelocity(array2vec3(v)); },
           py::arg("vel"));

  //======== End Articulation ========//

  py::class_<SContact>(m, "Contact")
      .def_property_readonly("actor1", [](SContact &contact) { return contact.actors[0]; },
                             py::return_value_policy::reference)
      .def_property_readonly("actor2", [](SContact &contact) { return contact.actors[1]; },
                             py::return_value_policy::reference)
      .def_property_readonly(
          "point",
          [](SContact &contact) {
            return make_array<PxReal>({contact.point.x, contact.point.y, contact.point.z});
          })
      .def_property_readonly(
          "normal",
          [](SContact &contact) {
            return make_array<PxReal>({contact.normal.x, contact.normal.y, contact.normal.z});
          })
      .def_property_readonly(
          "impulse",
          [](SContact &contact) {
            return make_array<PxReal>({contact.impulse.x, contact.impulse.y, contact.impulse.z});
          })
      .def_readonly("separation", &SContact::separation);

  //======== Builders ========

  py::class_<ActorBuilder>(m, "ActorBuilder")
      .def("add_convex_shape_from_file",
           [](ActorBuilder &a, std::string const &filename, PxTransform const &pose,
              py::array_t<PxReal> const &scale, PxMaterial *material, PxReal density) {
             a.addConvexShapeFromFile(filename, pose, array2vec3(scale), material, density);
           },
           py::arg("filename"), py::arg("pose") = PxTransform(PxIdentity),
           py::arg("scale") = make_array<PxReal>({1, 1, 1}), py::arg("material") = nullptr,
           py::arg("density") = 1000)
      .def("add_multiple_convex_shapes_from_file",
           [](ActorBuilder &a, std::string const &filename, PxTransform const &pose,
              py::array_t<PxReal> const &scale, PxMaterial *material, PxReal density) {
             a.addMultipleConvexShapesFromFile(filename, pose, array2vec3(scale), material,
                                               density);
           },
           py::arg("filename"), py::arg("pose") = PxTransform(PxIdentity),
           py::arg("scale") = make_array<PxReal>({1, 1, 1}), py::arg("material") = nullptr,
           py::arg("density") = 1000)
      .def("add_box_shape",
           [](ActorBuilder &a, PxTransform const &pose, py::array_t<PxReal> const &size,
              PxMaterial *material,
              PxReal density) { a.addBoxShape(pose, array2vec3(size), material, density); },
           py::arg("pose") = PxTransform(PxIdentity),
           py::arg("size") = make_array<PxReal>({1, 1, 1}), py::arg("material") = nullptr,
           py::arg("density") = 1000)
      .def("add_capsule_shape", &ActorBuilder::addCapsuleShape,
           py::arg("pose") = PxTransform(PxIdentity), py::arg("radius") = 1,
           py::arg("half_length") = 1, py::arg("material") = nullptr, py::arg("density") = 1000)
      .def("add_sphere_shape", &ActorBuilder::addSphereShape,
           py::arg("pose") = PxTransform(PxIdentity), py::arg("radius") = 1,
           py::arg("material") = nullptr, py::arg("density") = 1000)

      .def("add_box_visual",
           [](ActorBuilder &a, PxTransform const &pose, py::array_t<PxReal> const &size,
              py::array_t<PxReal> color, std::string const &name) {
             a.addBoxVisual(pose, array2vec3(size), array2vec3(color), name);
           },
           py::arg("pose") = PxTransform(PxIdentity),
           py::arg("size") = make_array<PxReal>({1, 1, 1}),
           py::arg("color") = make_array<PxReal>({1, 1, 1}), py::arg("name") = "")
      .def("add_capsule_visual",
           [](ActorBuilder &a, PxTransform const &pose, PxReal radius, PxReal halfLength,
              py::array_t<PxReal> color, std::string const &name) {
             a.addCapsuleVisual(pose, radius, halfLength, array2vec3(color), name);
           },
           py::arg("pose") = PxTransform(PxIdentity), py::arg("radius") = 1,
           py::arg("half_length") = 1, py::arg("color") = make_array<PxReal>({1, 1, 1}),
           py::arg("name") = "")
      .def("add_sphere_visual",
           [](ActorBuilder &a, PxTransform const &pose, PxReal radius, py::array_t<PxReal> color,
              std::string const &name) {
             a.addSphereVisual(pose, radius, array2vec3(color), name);
           },
           py::arg("pose") = PxTransform(PxIdentity), py::arg("radius") = 1,
           py::arg("color") = make_array<PxReal>({1, 1, 1}), py::arg("name") = "")
      .def("add_visual_from_file",
           [](ActorBuilder &a, std::string const &filename, PxTransform const &pose,
              py::array_t<PxReal> scale, std::string const &name) {
             a.addVisualFromFile(filename, pose, array2vec3(scale), name);
           },
           py::arg("filename"), py::arg("pose") = PxTransform(PxIdentity),
           py::arg("scale") = make_array<PxReal>({1, 1, 1}), py::arg("name") = "")

      .def("set_collision_group", &ActorBuilder::setCollisionGroup)
      .def("add_collision_group", &ActorBuilder::addCollisionGroup)
      .def("reset_collision_group", &ActorBuilder::resetCollisionGroup)
      .def("set_mass_and_inertia",
           [](ActorBuilder &a, PxReal mass, PxTransform const &cMassPose,
              py::array_t<PxReal> inertia) {
             a.setMassAndInertia(mass, cMassPose, array2vec3(inertia));
           })
      .def("set_scene", &ActorBuilder::setScene)
      .def("build", &ActorBuilder::build, py::arg("is_kinematic") = false, py::arg("name") = "",
           py::return_value_policy::reference)
      .def("build_static", &ActorBuilder::buildStatic, py::return_value_policy::reference);

  py::class_<LinkBuilder, ActorBuilder>(m, "LinkBuilder")
      .def("get_index", &LinkBuilder::getIndex)
      .def("set_parent", &LinkBuilder::setParent)
      .def("set_name", &LinkBuilder::setName)
      .def("set_joint_name", &LinkBuilder::setJointName)
      .def("set_joint_properties",
           [](LinkBuilder &b, PxArticulationJointType::Enum jointType,
              py::array_t<PxReal> const &limits, PxTransform const &parentPose,
              PxTransform const &childPose, PxReal friction, PxReal damping) {
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
             b.setJointProperties(jointType, l, parentPose, childPose, friction, damping);
           },
           py::arg("joint_type"), py::arg("limits"),
           py::arg("parent_pose") = PxTransform(PxIdentity),
           py::arg("child_pose") = PxTransform(PxIdentity), py::arg("friction") = 0,
           py::arg("damping") = 0);

  py::class_<ArticulationBuilder>(m, "ArticulationBuilder")
      .def("set_scene", &ArticulationBuilder::setScene, py::arg("scene"))
      .def("get_scene", &ArticulationBuilder::getScene)
      .def("create_link_builder",
           [](ArticulationBuilder &b, LinkBuilder *parent) { return b.createLinkBuilder(parent); },
           py::arg("parent") = nullptr, py::return_value_policy::reference)
      .def("build", &ArticulationBuilder::build, py::return_value_policy::reference,
           py::arg("fix_base") = false);

  py::class_<URDF::URDFLoader>(m, "URDFLoader")
      .def(py::init<SScene *>(), py::arg("scene"))
      .def_readwrite("fix_root_link", &URDF::URDFLoader::fixRootLink)
      .def_readwrite("scale", &URDF::URDFLoader::scale)
      .def_readwrite("default_density", &URDF::URDFLoader::defaultDensity)
      .def("load", &URDF::URDFLoader::load, py::return_value_policy::reference,
           py::arg("filename"), py::arg("default_material") = nullptr);
}
