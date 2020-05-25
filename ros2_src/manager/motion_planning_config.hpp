#pragma once

#include <experimental/filesystem>
#include <rcl_yaml_param_parser/parser.h>
#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#define SET_PARAMETERS(name, value)                                                               \
  {                                                                                               \
    if (node->has_parameter(name)) {                                                              \
      rclcpp::Parameter param(name, value);                                                       \
      node->set_parameter(param);                                                                 \
    } else {                                                                                      \
      node->declare_parameter(name, rclcpp::ParameterValue(value));                               \
    }                                                                                             \
  }

namespace fs = std::experimental::filesystem;
namespace sapien::ros2 {
struct MotionPlan {
  std::vector<std::string> jointNames;
  Eigen::VectorXd duration;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> position;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> velocity;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> acceleration;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> effort;

  MotionPlan(uint32_t dof, uint32_t PointsNum)
      : duration(PointsNum), position(PointsNum, dof), velocity(PointsNum, dof),
        acceleration(PointsNum, dof), effort(PointsNum, dof) {}
};

struct MotionPlanningConfig {
  /* default planner options */
  int planning_attempts = 10;
  float max_velocity_scaling_factor = 1.0;
  float max_acceleration_scaling_factor = 1.0;

  /* default general OMPL options */
  std::string planning_plugin = "ompl_interface/OMPLPlanner";
  std::string request_adapter = "default_planner_request_adapters/AddTimeOptimalParameterization "
                                "default_planner_request_adapters/FixWorkspaceBounds "
                                "default_planner_request_adapters/FixStartStateBounds "
                                "default_planner_request_adapters/FixStartStateCollision "
                                "default_planner_request_adapters/FixStartStatePathConstraints";
  float start_state_max_bounds_error = 0.1;

  void publishPlannerOption(rclcpp::Node::SharedPtr &node) const {
    const std::string ns("default_planner_options.");
    SET_PARAMETERS(ns + "planning_attempts", planning_attempts);
    SET_PARAMETERS(ns + "max_velocity_scaling_factor", max_velocity_scaling_factor);
    SET_PARAMETERS(ns + "max_acceleration_scaling_factor", max_acceleration_scaling_factor);
  }

  void publishGeneralOMPLConfig(rclcpp::Node::SharedPtr &node) const {
    const std::string ns("ompl.");
    SET_PARAMETERS(ns + "planning_plugin", planning_plugin);
    SET_PARAMETERS(ns + "request_adapters", request_adapter);
    SET_PARAMETERS(ns + "start_state_max_bounds_error", start_state_max_bounds_error);
  }

  bool publishDefaultOMPLPlannerConfig(rclcpp::Node::SharedPtr &node,
                                       const std::string &directory) const {
    std::string filePath = directory + "/motion_planning/default_ompl_planning.yaml";
    auto logger = spdlog::get("SAPIEN_ROS2");
    if (!fs::exists(filePath)) {
      logger->warn("File path {} does not exist", filePath);
      logger->warn("OMPL motion planning fail for the motion planner.");
      return false;
    }
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rcl_params_t *params = rcl_yaml_node_struct_init(allocator);
    rcl_parse_yaml_file(filePath.c_str(), params);
    for (size_t h = 0; h < params->num_nodes; ++h) {
      auto nodeParam = params->params[h];
      auto nodeName = params->node_names[h];
      for (size_t g = 0; g < nodeParam.num_params; ++g) {
        std::string paraName(nodeParam.parameter_names[g]);
        rclcpp::ParameterValue parameterValue;

        rcl_variant_t *paramVar = &(nodeParam.parameter_values[g]);
        if (nullptr != paramVar) {
          if (nullptr != paramVar->bool_value) {
            bool currentValue = paramVar->bool_value ? "true" : "false";
            parameterValue = rclcpp::ParameterValue(currentValue);
          } else if (nullptr != paramVar->string_value) {
            parameterValue = rclcpp::ParameterValue(std::string(paramVar->string_value));
          } else if (nullptr != paramVar->integer_value) {
            parameterValue = rclcpp::ParameterValue(*(paramVar->integer_value));
          } else if (nullptr != paramVar->double_value) {
            parameterValue = rclcpp::ParameterValue(*(paramVar->double_value));
          }
        }

        std::string nodeNameString(nodeName);
        auto pos = nodeNameString.find('/');
        nodeNameString.replace(pos, 1, ".");
        std::string finalParamString = "ompl." + nodeNameString;
        finalParamString += ".";
        finalParamString += paraName;
        if (node->has_parameter(finalParamString)) {
          rclcpp::Parameter parameter(finalParamString, parameterValue);
          node->set_parameter(parameter);
        } else {
          node->declare_parameter(finalParamString, parameterValue);
        }
      }
    }
    return true;
  };
}; // namespace sapien::ros2
}