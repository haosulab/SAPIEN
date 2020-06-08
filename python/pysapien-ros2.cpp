#include "pysapien_content.hpp"
#include "spdlog/sinks/stdout_color_sinks.h"

#include "manager/robot_descriptor.h"
#include "manager/robot_loader.h"
#include "manager/robot_manager.h"
#include "manager/scene_manager.h"

using namespace sapien::ros2;
namespace py = pybind11;

PYBIND11_MODULE(pysapien_ros2, m) {
  /* Build core module of sapien */
  auto m_core = m.def_submodule("core");
  buildSapien(m_core);

  /* Build ros2 module of sapien */
  auto m_ros2 = m.def_submodule("ros2");
  //======== Module Function ========//
  m_ros2.def("set_resources_directory", &RobotManager::setResourcesDirectory, py::arg("path"))
      .def("get_resources_directory", &RobotManager::getResourcesDirectory)
      .def("init_spd_logger", []() { auto logger = spdlog::stderr_color_mt("SAPIEN_ROS2"); })
      .def("set_ros2_logging_level",
           [](const std::string &level) {
             if (level == "debug") {
               rcutils_logging_set_default_logger_level(10);
             } else if (level == "info") {
               rcutils_logging_set_default_logger_level(20);
             } else if (level == "warn" || level == "warning") {
               rcutils_logging_set_default_logger_level(30);
             } else if (level == "err" || level == "error") {
               rcutils_logging_set_default_logger_level(40);
             } else if (level == "fatal") {
               rcutils_logging_set_default_logger_level(50);
             } else {
               std::cerr << "Invalid log level \"" << level << "\"" << std::endl;
             }
           })
      .def(
          "rclcpp_init",
          [](const std::vector<std::string> &args) {
            std::vector<char *> str;
            str.reserve(args.size());
            for (auto &s : args)
              str.push_back(const_cast<char *>(s.c_str()));
            int size = str.size();
            rclcpp::init(size, str.data());
          },
          py::arg("args"));

  //======== Manager ========//
  auto PySceneManager = py::class_<SceneManager>(m_ros2, "SceneManager");
  auto PyRobotManager = py::class_<RobotManager>(m_ros2, "RobotManager");
  auto PyRobotLoader = py::class_<RobotLoader>(m_ros2, "RobotLoader");
  auto PyRobotDescriptor = py::class_<RobotDescriptor>(m_ros2, "RobotDescriptor");
  auto PyKinematicsSolverType = py::enum_<KinematicsSolver>(m_ros2, "KinematicsSolverType");
  auto PyKinematicsConfig = py::class_<KinematicsConfig>(m_ros2, "KinematicsConfig");
  auto PyMotionPlanningConfig = py::class_<MotionPlanningConfig>(m_ros2, "MotionPlanningConfig");

  PySceneManager
      .def(py::init<sapien::SScene *, std::string const &>(), py::arg("scene"),
           py::arg("scene_name"))
      .def("start", &SceneManager::start)
      .def("create_robot_loader", &SceneManager::createRobotLoader);

  PyRobotManager
      .def("set_drive_property", &RobotManager::setDriveProperty, py::arg("stiffness"),
           py::arg("damping"), py::arg("force_limit") = PX_MAX_F32,
           py::arg("joint_index") = std::vector<uint32_t>())
      .def("balance_passive_force", &RobotManager::balancePassiveForce, py::arg("gravity") = true,
           py::arg("coriolis_centrifugal") = true, py::arg("external") = true)
      .def("set_kinematics_config", &RobotManager::setKinematicsConfig,
           py::arg("config") = KinematicsConfig())
      .def("set_motion_planning_config", &RobotManager::setMotionPlanningConfig,
           py::arg("config") = MotionPlanningConfig())
      .def("get_group_names", &RobotManager::getGroupNames)
      .def("get_kinematics_config", &RobotManager::getKinematicsConfig)
      .def("get_motion_planning_config", &RobotManager::getMotionPlanningConfig)
      .def("create_joint_publisher", &RobotManager::createJointPublisher, py::arg("frequency"))
      .def("build_motion_planner", &RobotManager::buildMotionPlanner, py::arg("group_name"),
           py::arg("service_name"))
      .def("build_joint_velocity_controller", &RobotManager::buildJointVelocityController,
           py::return_value_policy::reference, py::arg("joint_names"), py::arg("service_name"),
           py::arg("latency") = 0)
      .def("build_cartesian_velocity_controller", &RobotManager::buildCartesianVelocityController,
           py::return_value_policy::reference, py::arg("group_name"), py::arg("service_name"),
           py::arg("latency") = 0);

  PyRobotLoader.def(py::init<SceneManager *>(), py::arg("scene_manager"))
      .def_property("fix_root_link", &RobotLoader::getFixRootLink, &RobotLoader::setFixRootLink)
      .def_property("collision_is_visual", &RobotLoader::getCollisionIsVisual,
                    &RobotLoader::setCollisionIsVisual)
      .def_property("default_density", &RobotLoader::getDefaultDensity,
                    &RobotLoader::setDefaultDensity)
      .def("load_robot_and_manager", &RobotLoader::loadRobotAndManager,
           py::arg("robot_descriptor"), py::arg("robot_name"),
           py::arg("material") = (physx::PxMaterial *)nullptr, py::return_value_policy::reference);

  PyRobotDescriptor
      .def(py::init<bool, std::string const &, std::string const &, std::string const &>(),
           py::arg("is_path"), py::arg("urdf"), py::arg("srdf"), py::arg("substitute_path") = "")
      .def(py::init<std::string const &, std::string const &, std::string const &, bool>(),
           py::arg("ros_package_name"), py::arg("urdf_relative_path"),
           py::arg("srdf_relative_path"), py::arg("use_share_directory"))
      .def("get_urdf", &RobotDescriptor::getURDF)
      .def("get_srdf", &RobotDescriptor::getSRDF)
      .def("get_standard_urdf", &RobotDescriptor::getStandardURDF);

  //======== Controller ========//
  auto PyMoveType = py::enum_<MoveType>(m_ros2, "MoveType");

  auto PyJointVelocityController =
      py::class_<JointVelocityController>(m_ros2, "JointVelocityController");
  auto PyCartesianVelocityController =
      py::class_<CartesianVelocityController>(m_ros2, "CartesianVelocityController");
  auto PyMotionPlanner = py::class_<MotionPlanner>(m_ros2, "MotionPlanner");
  auto PyMotionPlan = py::class_<MotionPlan>(m_ros2, "MotionPlan");

  // The enum_::export_values() function exports the enum entries into the parent scope,
  // which should be skipped for newer C++11-style strongly typed enums.
  PyMoveType.value("WORLD_TRANSLATE", MoveType::WorldTranslate)
      .value("WORLD_ROTATE", MoveType::WorldRotate)
      .value("LOCAL_TRANSLATE", MoveType::LocalTranslate)
      .value("LOCAL_ROTATE", MoveType::LocalRotate)
      .value("BODY_TWIST", MoveType::BodyTwist)
      .value("SPATIAL_TWIST", MoveType::SpatialTwist);

  PyKinematicsSolverType.value("KDL", KinematicsSolver::KDL);

  PyKinematicsConfig.def(py::init<>())
      .def_readwrite("kinematics_solver_timeout", &KinematicsConfig::kinematics_solver_timeout)
      .def_readwrite("kinematics_solver_attempts", &KinematicsConfig::kinematics_solver_attempts)
      .def_readwrite("kinematics_solver_search_resolution",
                     &KinematicsConfig::kinematics_solver_search_resolution)
      .def_readwrite("kinematics_solver_type", &KinematicsConfig::kinematics_solver);

  PyMotionPlanningConfig.def(py::init<>())
      .def_readwrite("max_acceleration_scaling_factor",
                     &MotionPlanningConfig::max_acceleration_scaling_factor)
      .def_readwrite("start_state_max_bounds_error",
                     &MotionPlanningConfig::start_state_max_bounds_error)
      .def_readwrite("request_adapter", &MotionPlanningConfig::request_adapter)
      .def_readwrite("planning_plugin", &MotionPlanningConfig::planning_plugin)
      .def_readwrite("planning_attempts", &MotionPlanningConfig::planning_attempts)
      .def_readwrite("max_velocity_scaling_factor",
                     &MotionPlanningConfig::max_velocity_scaling_factor);

  PyJointVelocityController
      .def("move_joint",
           py::overload_cast<const std::vector<std::string> &, float, bool>(
               &JointVelocityController::moveJoint),
           py::arg("joint_names"), py::arg("velocity"), py::arg("continuous") = true,
           "Move joints with given names, same velocity for all joints.")
      .def("move_joint",
           py::overload_cast<const std::vector<std::string> &, const std::vector<float> &, bool>(
               &JointVelocityController::moveJoint),
           py::arg("joint_names"), py::arg("velocities"), py::arg("continuous") = true,
           "Move joints with given names, velocity is specified for each joints.")
      .def(
          "move_joint",
          py::overload_cast<const std::vector<float> &, bool>(&JointVelocityController::moveJoint),
          py::arg("velocity"), py::arg("continuous") = true,
          "Move joints with velocity, given default order.");

  PyCartesianVelocityController
      .def("move_twist",
           [](CartesianVelocityController &c, const py::array_t<double> &vec, MoveType type) {
             if (vec.size() != 6) {
               std::stringstream ss;
               ss << "Cartesian velocity command should have size 3, but given " << vec.size();
               throw std::runtime_error(ss.str());
             }
             std::array<double, 6> v = {vec.at(0), vec.at(1), vec.at(2),
                                        vec.at(3), vec.at(4), vec.at(5)};
             c.moveTwist(v, type);
           })
      .def("move_cartesian",
           [](CartesianVelocityController &c, const py::array_t<double> &vec, MoveType type) {
             if (vec.size() != 3) {
               std::stringstream ss;
               ss << "Cartesian velocity command should have size 3, but given " << vec.size();
               throw std::runtime_error(ss.str());
             }
             std::array<double, 3> v = {*vec.data(), *(vec.data() + 1), *(vec.data() + 2)};
             c.moveCartesian(v, type);
           });

  PyMotionPlan.def_readonly("joint_names", &MotionPlan::jointNames)
      .def("__repr__",
           [](MotionPlan &plan) {
             std::ostringstream oss;
             oss << "Motion Plan contains [" << plan.duration.cols() << "] trajectory points";
             return oss.str();
           })
      .def_readonly("duration", &MotionPlan::duration)
      .def_readonly("position", &MotionPlan::position)
      .def_readonly("velocity", &MotionPlan::velocity)
      .def_readonly("acceleration", &MotionPlan::acceleration);

  PyMotionPlanner.def("set_start_state", &MotionPlanner::setStartState, py::arg("qpos"))
      .def("set_goal_state", py::overload_cast<Eigen::VectorXd &>(&MotionPlanner::setGoalState),
           py::arg("qpos"))
      .def("set_goal_state",
           py::overload_cast<const PxTransform &, const std::string &>(
               &MotionPlanner::setGoalState),
           py::arg("pose"), py::arg("link_name") = "")
      .def("set_start_state_to_current_state", &MotionPlanner::setStartStateToCurrentState)
      .def("plan", &MotionPlanner::plan);
}
