#include <moveit/move_group_interface/move_group_interface.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "controllable_articulation_wrapper.h"
#include "controller/cartesian_velocity_controller.h"
#include "controller/controller_manger.h"
#include "controller/velocity_control_service.h"
#include "device/movo.hpp"
#include "device/movo_free_base.hpp"
#include "device/single_gripper.hpp"
#include "device/xarm6.hpp"
#include "device/panda.hpp"

using namespace sapien::robot;
namespace py = pybind11;

PYBIND11_MODULE(sapyen_robot, m) {
  py::module m1 = m.def_submodule("ros", "Pybinding of original ROS functionality");
  // ROS Pre-requisite
  m1.def(
      "init",
      [](const std::vector<std::string> &args, const std::string &name, uint32_t options) {
        std::vector<char *> str;
        str.reserve(args.size());
        for (auto &s : args)
          str.push_back(const_cast<char *>(s.c_str()));
        int size = str.size();
        ros::init(size, str.data(), name, options);
      },
      py::arg("args"), py::arg("name"), py::arg("options") = 0);

  py::class_<ControllerManger>(m, "ControllerManger")
      .def(py::init<std::string, sapien::ControllableArticulationWrapper *>())
      .def("start", &ControllerManger::start)
      .def("move_base", &ControllerManger::moveBase)
      .def("add_joint_state_publisher", &ControllerManger::createJointPubNode)
      .def("create_joint_velocity_controller", &ControllerManger::createJointVelocityController,
           py::return_value_policy::reference)
      .def("create_cartesian_velocity_controller",
           &ControllerManger::createCartesianVelocityController,
           py::return_value_policy::reference)
      .def("create_group_planner", &ControllerManger::createGroupPlanner,
           py::return_value_policy::reference)
      .def("add_group_trajectory_controller", &ControllerManger::addGroupTrajectoryController);

  py::class_<JointVelocityController>(m, "JointVelocityController")
      .def("move_joint", [](JointVelocityController &a, const std::vector<std::string> &jointNames,
                            float velocity) { a.moveJoint(jointNames, velocity); })
      .def("move_joint",
           [](JointVelocityController &a, const std::vector<std::string> &jointNames,
              const std::vector<float> &velocity) { a.moveJoint(jointNames, velocity); })
      .def("move_joint", [](JointVelocityController &a, const std::vector<float> &velocity) {
        a.moveJoint(velocity);
      });

  py::class_<CartesianVelocityController>(m, "CartesianVelocityController")
      .def("move_local_translate",
           [](CartesianVelocityController &a, const py::array_t<float> &arr, bool continuous) {
             std::array<float, 3> array{};
             for (int j = 0; j < 3; ++j) {
               array[j] = arr.at(j);
             }
             a.moveRelative(array, MoveType::LocalTranslate, continuous);
           })
      .def("move_world_translate",
           [](CartesianVelocityController &a, const py::array_t<float> &arr, bool continuous) {
             std::array<float, 3> array{};
             for (int j = 0; j < 3; ++j) {
               array[j] = arr.at(j);
             }
             a.moveRelative(array, MoveType::WorldTranslate, continuous);
           })
      .def("move_local_rotate",
           [](CartesianVelocityController &a, const py::array_t<float> &arr, bool continuous) {
             std::array<float, 3> array{};
             for (int j = 0; j < 3; ++j) {
               array[j] = arr.at(j);
             }
             a.moveRelative(array, MoveType::LocalRotate, continuous);
           })
      .def("move_world_rotate",
           [](CartesianVelocityController &a, const py::array_t<float> &arr, bool continuous) {
             std::array<float, 3> array{};
             for (int j = 0; j < 3; ++j) {
               array[j] = arr.at(j);
             }
             a.moveRelative(array, MoveType::WorldRotate, continuous);
           })
      .def("get_velocity", &CartesianVelocityController::getVelocity);

  py::class_<MoveGroupPlanner>(m, "MoveGroupPlanner").def("go", &MoveGroupPlanner::go);

  py::class_<PS3RobotControl>(m, "PS3RobotControl")
      .def(py::init<ControllerManger *>())
      .def("set_gripper_velocity", &PS3RobotControl::set_gripper_velocity)
      .def("set_translation_velocity", &PS3RobotControl::set_translation_velocity)
      .def("set_rotation_velocity", &PS3RobotControl::set_rotation_velocity)
      .def("set_arm_velocity", &PS3RobotControl::set_arm_velocity)
      .def("set_arm_angular_velocity", &PS3RobotControl::set_arm_angular_velocity)
      .def("get_gripper_velocity", &PS3RobotControl::get_gripper_velocity)
      .def("get_translation_velocity", &PS3RobotControl::get_translation_velocity)
      .def("get_rotation_velocity", &PS3RobotControl::get_rotation_velocity)
      .def("get_arm_velocity", &PS3RobotControl::get_arm_velocity)
      .def("get_arm_angular_velocity", &PS3RobotControl::get_arm_angular_velocity)
      .def("set_normal_mode", [](PS3RobotControl &a) { a.set_mode(PS3Mode::NORMAL); })
      .def("set_demonstration_mode",
           [](PS3RobotControl &a) { a.set_mode(PS3Mode::DEMONSTRATION); })
      .def("set_replay_mode", [](PS3RobotControl &a) { a.set_mode(PS3Mode::REPLAY); })
      .def("step", &PS3RobotControl::step)
      .def("start_record", &PS3RobotControl::start_record)
      .def("record_current_step", &PS3RobotControl::record_single_step)
      .def("apply_cache",
           [](PS3RobotControl &a, const py::array_t<int> &arr) {
             a.set_cache(
                 std::vector<int>(arr.data(), arr.data() + PS3_AXIS_COUNT + PS3_BUTTON_COUNT));
           })
      .def("get_cache", [](PS3RobotControl &a) {
        auto cache = a.get_cache();
        return py::array_t<int>(cache.size(), cache.data());
      });

  py::class_<KinovaGripperPS3, PS3RobotControl>(m, "SingleGripperPS3")
      .def(py::init<ControllerManger *>())
      .def("step", &KinovaGripperPS3::step);

  py::class_<MOVOPS3, PS3RobotControl>(m, "MOVOPS3")
      .def(py::init<ControllerManger *>())
      .def("get_wheel_velocity", &MOVOPS3::get_wheel_velocity)
      .def("get_head_velocity", &MOVOPS3::get_head_velocity)
      .def("get_body_velocity", &MOVOPS3::get_body_velocity)
      .def("set_wheel_velocity", &MOVOPS3::set_wheel_velocity)
      .def("set_head_velocity", &MOVOPS3::set_head_velocity)
      .def("set_body_velocity", &MOVOPS3::set_body_velocity)
      .def("step", &MOVOPS3::step);

  py::class_<XArm6PS3, PS3RobotControl>(m, "XArm6PS3")
      .def(py::init<ControllerManger *>())
      .def("step", &XArm6PS3::step);

  py::class_<PandaPS3, PS3RobotControl>(m, "PandaPS3")
      .def(py::init<ControllerManger *>())
      .def("step", &PandaPS3::step);

  py::class_<MOVOFreeBasePS3, PS3RobotControl>(m, "MOVOFreeBasePS3")
      .def(py::init<ControllerManger *>())
      .def("get_wheel_velocity", &MOVOFreeBasePS3::get_wheel_velocity)
      .def("get_head_velocity", &MOVOFreeBasePS3::get_head_velocity)
      .def("get_body_velocity", &MOVOFreeBasePS3::get_body_velocity)
      .def("set_wheel_velocity", &MOVOFreeBasePS3::set_wheel_velocity)
      .def("set_head_velocity", &MOVOFreeBasePS3::set_head_velocity)
      .def("set_body_velocity", &MOVOFreeBasePS3::set_body_velocity)
      .def("step", &MOVOFreeBasePS3::step);
}
