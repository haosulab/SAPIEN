#pragma once

#include <PxPhysicsAPI.h>
#include <memory>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>

#include <manager/motion_planning_config.hpp>

namespace sapien::ros2 {
using namespace moveit::planning_interface;

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

    mComponent = std::make_unique<PlanningComponent>(groupName, mMoveItCpp);
    mJointModelGroup = mMoveItCpp->getRobotModel()->getJointModelGroup(groupName);
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