#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "manager/robot_manager.h"
#include "manager/scene_manager.h"

using namespace sapien::ros2;
namespace py = pybind11;

PYBIND11_MODULE(pysapien_ros2, m_ros2) {
  //======== Module Function ========//
  m_ros2.def(
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

  PySceneManager
      .def(py::init<sapien::SScene *, std::string const &>(), py::arg("scene"),
           py::arg("scene_name"))
      .def("start", &SceneManager::start)
      .def("build_robot_manager", &SceneManager::buildRobotManager,
           py::return_value_policy::reference, py::arg("articulation"), py::arg("robot_name"));

  PyRobotManager
      .def("set_drive_property", &RobotManager::setDriveProperty, py::arg("stiffness"),
           py::arg("damping"), py::arg("force_limit") = PX_MAX_F32,
           py::arg("joint_index") = std::vector<uint32_t>())
      .def("balance_passive_force", &RobotManager::balancePassiveForce, py::arg("gravity") = true,
           py::arg("coriolis_centrifugal") = true, py::arg("external") = true)
      .def("create_joint_publisher", &RobotManager::createJointPublisher, py::arg("frequency"))
      .def("build_joint_velocity_controller", &RobotManager::buildJointVelocityController,
           py::return_value_policy::reference, py::arg("joint_names"), py::arg("service_name"),
           py::arg("latency") = 0)
      .def("build_cartesian_velocity_controller", &RobotManager::buildCartesianVelocityController,
           py::return_value_policy::reference, py::arg("group_name"), py::arg("service_name"),
           py::arg("latency") = 0);

  //======== Controller ========//

  auto PyMoveType = py::enum_<MoveType>(m_ros2, "MoveType");
  auto PyJointVelocityController =
      py::class_<JointVelocityController>(m_ros2, "JointVelocityController");
  auto PyCartesianVelocityController =
      py::class_<CartesianVelocityController>(m_ros2, "CartesianVelocityController");

  // The enum_::export_values() function exports the enum entries into the parent scope,
  // which should be skipped for newer C++11-style strongly typed enums.
  PyMoveType.value("WORLD_TRANSLATE", MoveType::WorldTranslate)
      .value("WORLD_ROTATE", MoveType::WorldTranslate)
      .value("LOCAL_TRANSLATE", MoveType::WorldTranslate)
      .value("LOCAL_ROTATE", MoveType::WorldTranslate);

  //  PyJointVelocityController
  //      .def(
  //          "move_joint",
  //          [](JointVelocityController &a, const std::vector<std::string> &jointNames,
  //             float velocity, bool continuous) { a.moveJoint(jointNames, velocity, continuous);
  //             },
  //          py::arg("joint_names"), py::arg("velocity"), py::arg("continuous") = true)
  //      .def(
  //          "move_joint",
  //          [](JointVelocityController &a, const std::vector<std::string> &jointNames,
  //             const std::vector<float> &velocity,
  //             bool continuous) { a.moveJoint(jointNames, velocity, continuous); },
  //          py::arg("joint_names"), py::arg("velocities"), py::arg("continuous") = true)
  //      .def(
  //          "move_joint",
  //          [](JointVelocityController &a, const std::vector<float> &velocity, bool continuous) {
  //            a.moveJoint(velocity, continuous);
  //          },
  //          py::arg("velocity"), py::arg("continuous") = true);

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

  PyCartesianVelocityController.def(
      "move_cartesian",
      [](CartesianVelocityController &c, const py::array_t<float> &vec, MoveType type) {
        if (vec.size() != 3) {
          std::stringstream ss;
          ss << "Cartesian velocity command should have size 3, but given " << vec.size();
          throw std::runtime_error(ss.str());
        }
        std::array<float, 3> v = {*vec.data(), *(vec.data() + 1), *(vec.data() + 2)};
        c.moveCartesian(v, type);
      });
}
