#pragma once

#include <PxPhysicsAPI.h>
#include <memory>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <rcl_yaml_param_parser/parser.h>
#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

namespace sapien::ros2 {
using namespace moveit::planning_interface;
namespace fs = std::experimental::filesystem;

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
    node->declare_parameter(ns + "planning_attempts", rclcpp::ParameterValue(planning_attempts));
    node->declare_parameter(ns + "max_velocity_scaling_factor",
                            rclcpp::ParameterValue(max_velocity_scaling_factor));
    node->declare_parameter(ns + "max_acceleration_scaling_factor",
                            rclcpp::ParameterValue(max_acceleration_scaling_factor));
  }

  void publishGeneralOMPLConfig(rclcpp::Node::SharedPtr &node) const {
    const std::string ns("ompl.");
    node->declare_parameter(ns + "planning_plugin", rclcpp::ParameterValue(planning_plugin));
    node->declare_parameter(ns + "request_adapters", rclcpp::ParameterValue(request_adapter));
    node->declare_parameter(ns + "start_state_max_bounds_error",
                            rclcpp::ParameterValue(start_state_max_bounds_error));
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
    rcl_yaml_node_struct_print(params);
    return true;
  }
};

class RobotManager;

class MotionPlanner {
  friend RobotManager;

protected:
  rclcpp::Node::SharedPtr mNode;
  rclcpp::Clock::SharedPtr mClock;

  MoveItCppPtr mMoveItCpp;
  PlanningComponentUniquePtr mComponent;
  robot_state::RobotState mRobotState;
  const robot_state::JointModelGroup *mJointModelGroup;

public:
  std::string mEEName;
  std::string mBaseName;

protected:
public:
  MotionPlanner(rclcpp::Node::SharedPtr node, rclcpp::Clock::SharedPtr clock,
                const MoveItCppPtr &moveitCpp, const std::string &groupName,
                const std::string &serviceName)
      : mNode(std::move(node)), mClock(std::move(clock)), mMoveItCpp(moveitCpp),
        mRobotState(moveitCpp->getRobotModel()) {
    auto pipelineNames = moveitCpp->getPlanningPipelineNames(groupName);
    auto logger = spdlog::get("SAPIEN_ROS2");
    logger->info("There are {} pipelines loaded for group {}", pipelineNames.size(), groupName);
    for (const auto &pipelineName : pipelineNames) {
      logger->info(pipelineName);
    }

    mComponent = std::make_unique<PlanningComponent>(PlanningComponent(groupName, mMoveItCpp));
    mJointModelGroup = mMoveItCpp->getRobotModel()->getJointModelGroup(groupName);
    auto state = mComponent->getStartState();
    std::cout << state->getVariablePositions();
    mEEName = mJointModelGroup->getLinkModelNames().back();
    mBaseName = mJointModelGroup->getLinkModelNames()[0];
  };

  /* Set Start and End State */
  inline void setStartStateToCurrentState() { mComponent->setStartStateToCurrentState(); }

  inline bool setStartState(Eigen::VectorXd &qpos) {
    mRobotState.copyJointGroupPositions(mJointModelGroup, qpos);
    return mComponent->setStartState(mRobotState);
  }

  inline bool setGoalState(Eigen::VectorXd &qpos) {
    mRobotState.copyJointGroupPositions(mJointModelGroup, qpos);
    return mComponent->setGoal(mRobotState);
  }

  bool setGoalState(const physx::PxTransform &Pose, const std::string &linkName = "") {
    geometry_msgs::msg::PoseStamped targetPose;
    std::string targetLink = linkName.empty() ? mEEName : linkName;
    targetPose.header.frame_id = mBaseName;
    targetPose.pose.orientation.w = Pose.q.w;
    targetPose.pose.orientation.x = Pose.q.x;
    targetPose.pose.orientation.y = Pose.q.y;
    targetPose.pose.orientation.z = Pose.q.z;
    targetPose.pose.position.x = Pose.p.x;
    targetPose.pose.position.y = Pose.p.y;
    targetPose.pose.position.z = Pose.p.z;
    return mComponent->setGoal(targetPose, targetLink);
  }

  /* Planning */
  MotionPlan plan() {
    auto solution = mComponent->plan();
    auto pointsNum = solution.trajectory->getWayPointCount();
    auto jointNames = solution.trajectory->getGroup()->getActiveJointModelNames();
    auto dof = jointNames.size();

    MotionPlan plan(dof, pointsNum);
    auto duration = solution.trajectory->getWayPointDurations();

    for (size_t j = 0; j < pointsNum; ++j) {
      plan.duration(j) = duration[j];
      auto state = solution.trajectory->getWayPoint(j);
      auto position = state.getVariablePositions();
      plan.position.block(j, 0, 1, dof) = Eigen::Map<Eigen::RowVectorXd>(position, 1, dof);
      auto velocity = state.getVariableVelocities();
      plan.velocity.block(j, 0, 1, dof) = Eigen::Map<Eigen::RowVectorXd>(velocity, 1, dof);
      auto acceleration = state.getVariableAccelerations();
      plan.acceleration.block(j, 0, 1, dof) = Eigen::Map<Eigen::RowVectorXd>(acceleration, 1, dof);
      auto effort = state.getVariableEffort();
      plan.effort.block(j, 0, 1, dof) = Eigen::Map<Eigen::RowVectorXd>(effort, 1, dof);
    }
    plan.jointNames = jointNames;

    return plan;
  }
};

}