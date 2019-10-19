#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

//#include "pybind.cpp"
#include "controllable_articulation_wrapper.h"
#include "controller/cartesian_velocity_controller.h"
#include "controller/controller_manger.h"
#include "controller/joint_pub_node.h"
#include "controller/joint_trajectory_controller.h"
#include "controller/velocity_control_service.h"
#include "device/movo_ps3.h"

using namespace sapien::robot;
namespace py = pybind11;

PYBIND11_MODULE(sapyen_robot, m) {
  py::module m1 = m.def_submodule("ros", "Pybinding of original ROS functionality");
  // ROS Pre-requisite
  py::class_<ros::Publisher>(m1, "ROSPublisher");
  py::class_<ros::Subscriber>(m1, "ROSSubscriber");
  py::class_<ros::NodeHandle>(m1, "ROSNodeHandle");
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
      .def(py::init<std::string, sapien::ControllableArticulationWrapper *>());

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
}
