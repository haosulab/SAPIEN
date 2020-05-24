#pragma once
#include <rclcpp/rclcpp.hpp>
namespace sapien::ros2 {

#define SET_PARAMETERS(name, value)                                                               \
  {                                                                                               \
    if (node->has_parameter(name)) {                                                              \
      rclcpp::Parameter param(name, value);                                                       \
      node->set_parameter(param);                                                                 \
    } else {                                                                                      \
      node->declare_parameter(name, rclcpp::ParameterValue(value));                               \
    }                                                                                             \
  }
enum KinematicsSolver { KDL };

struct KinematicsConfig {
  //  panda_arm : kinematics_solver : kdl_kinematics_plugin / KDLKinematicsPlugin

  std::string kinematics_solver = "kdl_kinematics_plugin/KDLKinematicsPlugin";
  float kinematics_solver_search_resolution = 0.005;
  float kinematics_solver_timeout = 0.05;
  int kinematics_solver_attempts = 5;

  std::string enum2string(KinematicsSolver solver) const {
    switch (solver) {
    case KDL:
      return "kdl_kinematics_plugin / KDLKinematicsPlugin";
    }
  }

  void publishKinematicsConfig(rclcpp::Node::SharedPtr &node, const std::string &groupName) const {
    std::string ns = groupName + ".";
    SET_PARAMETERS(ns + "kinematics_solver", kinematics_solver)
    SET_PARAMETERS(ns + "kinematics_solver_search_resolution", kinematics_solver_search_resolution)
    SET_PARAMETERS(ns + "kinematics_solver_timeout", kinematics_solver_timeout)
    SET_PARAMETERS(ns + "kinematics_solver_attempt", kinematics_solver_attempts)
  }
};

}