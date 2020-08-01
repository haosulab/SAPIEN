#include "pysapien_content.hpp"
#include "spdlog/sinks/stdout_color_sinks.h"

#include "manager/robot_loader.h"
#include "manager/robot_manager.h"
#include "manager/scene_manager.h"

using namespace sapien::ros1;
namespace py = pybind11;

PYBIND11_MODULE(pysapien_ros1, m) {
  /* Build core module of sapien */
  auto m_core = m.def_submodule("core");
  buildSapien(m_core);

  /* Build ros1 module of sapien */
  auto m_ros1 = m.def_submodule("ros1");
  //======== Module Function ========//
  m_ros1.def("init_spd_logger", []() { auto logger = spdlog::stderr_color_mt("SAPIEN_ROS1"); })
      .def(
          "ros_init",
          [](const std::string &name, const std::vector<std::string> &args) {
            std::vector<char *> str;
            str.reserve(args.size());
            for (auto &s : args)
              str.push_back(const_cast<char *>(s.c_str()));
            int size = str.size();
            ros::init(size, str.data(), name);
          },
          py::arg("name"), py::arg("args"));

  //======== Manager ========//
  auto PySceneManager = py::class_<SceneManager>(m_ros1, "SceneManager");
  auto PyRobotManager = py::class_<RobotManager>(m_ros1, "RobotManager");
  auto PyRobotLoader = py::class_<RobotLoader>(m_ros1, "RobotLoader");

  PySceneManager
      .def(py::init<sapien::SScene *, std::string const &, uint8_t>(), py::arg("scene"),
           py::arg("scene_name"), py::arg("num_thread") = 4)
      .def("start", &SceneManager::start)
      .def("stop", &SceneManager::stop)
      .def("start_all_ros_camera", &SceneManager::startAllCamera, py::arg("frequency"))
      .def("create_robot_loader", &SceneManager::createRobotLoader);

  PyRobotManager
      .def("set_drive_property", &RobotManager::setDriveProperty, py::arg("stiffness"),
           py::arg("damping"), py::arg("force_limit") = PX_MAX_F32,
           py::arg("joint_index") = std::vector<uint32_t>())
      .def("balance_passive_force", &RobotManager::balancePassiveForce, py::arg("gravity") = true,
           py::arg("coriolis_centrifugal") = true, py::arg("external") = true);

  PyRobotLoader.def(py::init<SceneManager *>(), py::arg("scene_manager"))
      .def_property("fix_root_link", &RobotLoader::getFixRootLink, &RobotLoader::setFixRootLink)
      .def_property("collision_is_visual", &RobotLoader::getCollisionIsVisual,
                    &RobotLoader::setCollisionIsVisual)
      .def(
          "load_from_parameter_server",
          [](RobotLoader &loader, const std::string &robotName, py::dict &dict, double frequency,
             const std::string &URDFParamName = "", const std::string &SRDFName = "") {
            auto config = parseURDFConfig(dict);
            return loader.loadFromParameterServer(robotName, config, frequency);
          },
          py::arg("robot_name"), py::arg("urdf_config"), py::arg("frequency"),
          py::arg("urdf_param_name") = "", py::arg("srdf_param_name") = "",
          py::return_value_policy::reference);
}
