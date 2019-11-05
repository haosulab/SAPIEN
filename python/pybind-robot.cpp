#include <moveit/move_group_interface/move_group_interface.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "controllable_articulation_wrapper.h"
#include "controller/cartesian_velocity_controller.h"
#include "controller/controller_manger.h"
#include "controller/velocity_control_service.h"
#include "device/movo_ps3.h"
#include "device/single_gripper_ps3.hpp"
#include "device/xarm6_ps3.h"

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
      .def("move_joint", &JointVelocityController::moveJoint);

  py::class_<CartesianVelocityController>(m, "CartesianVelocityController")
      .def("get_velocity", &CartesianVelocityController::getVelocity);

  py::class_<MoveGroupPlanner>(m, "MoveGroupPlanner").def("go", &MoveGroupPlanner::go);

  py::class_<KinovaGripperPS3>(m, "SingleKinovaGripper")
      .def(py::init<ControllerManger *>())
      .def("set_gripper_velocity", &KinovaGripperPS3::set_gripper_velocity)
      .def("set_translation_velocity", &KinovaGripperPS3::set_translation_velocity)
      .def("set_rotation_velocity", &KinovaGripperPS3::set_rotation_velocity)
      .def("step", &KinovaGripperPS3::step)
      .def("start_record", &KinovaGripperPS3::start_record)
      .def("set_normal_mode", [](KinovaGripperPS3 &a) { a.set_mode(PS3Mode::NORMAL); })
      .def("set_demonstration_mode",
           [](KinovaGripperPS3 &a) { a.set_mode(PS3Mode::DEMONSTRATION); })
      .def("set_replay_mode", [](KinovaGripperPS3 &a) { a.set_mode(PS3Mode::REPLAY); })
      .def("apply_cache",
           [](KinovaGripperPS3 &a, const py::array_t<int> &arr) {
             a.set_cache(
                 std::vector<int>(arr.data(), arr.data() + PS3_AXIS_COUNT + PS3_BUTTON_COUNT));
           })
      .def("get_cache", [](KinovaGripperPS3 &a) {
        auto cache = a.get_cache();
        return py::array_t<int>(cache.size(), cache.data());
      });

  py::class_<MOVOPS3>(m, "MOVOPS3")
      .def(py::init<ControllerManger *>())
      .def("step", &MOVOPS3::step)
      .def("apply_cache",
           [](MOVOPS3 &a, const py::array_t<int> &arr) {
             a.set_cache(
                 std::vector<int>(arr.data(), arr.data() + PS3_AXIS_COUNT + PS3_BUTTON_COUNT));
           })
      .def("get_cache",
           [](MOVOPS3 &a) {
             auto cache = a.get_cache();
             return py::array_t<int>(cache.size(), cache.data());
           })
      .def("set_normal_mode", [](MOVOPS3 &a) { a.set_mode(PS3Mode::NORMAL); })
      .def("set_demonstration_mode", [](MOVOPS3 &a) { a.set_mode(PS3Mode::DEMONSTRATION); })
      .def("set_replay_mode", [](MOVOPS3 &a) { a.set_mode(PS3Mode::REPLAY); });
  py::class_<XArm6PS3>(m, "XArm6PS3")
      .def(py::init<ControllerManger *>())
      .def("step", &XArm6PS3::step)
      .def("start_record", &XArm6PS3::start_record)
      .def("apply_cache",
           [](XArm6PS3 &a, const py::array_t<int> &arr) {
             a.set_cache(
                 std::vector<int>(arr.data(), arr.data() + PS3_AXIS_COUNT + PS3_BUTTON_COUNT));
           })
      .def("get_cache",
           [](XArm6PS3 &a) {
             auto cache = a.get_cache();
             return py::array_t<int>(cache.size(), cache.data());
           })
      .def("set_normal_mode", [](XArm6PS3 &a) { a.set_mode(PS3Mode::NORMAL); })
      .def("set_demonstration_mode", [](XArm6PS3 &a) { a.set_mode(PS3Mode::DEMONSTRATION); })
      .def("set_replay_mode", [](XArm6PS3 &a) { a.set_mode(PS3Mode::REPLAY); });
}
