#pragma once

#include <PxPhysicsAPI.h>
#include <memory>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>

namespace sapien::ros2 {
using namespace moveit::planning_interface;

struct MotionPlan;

class MotionPlanner {

protected:
  rclcpp::Node::SharedPtr mNode;
  rclcpp::Clock::SharedPtr mClock;

  MoveItCppPtr mMoveItCpp;
  PlanningComponentUniquePtr mComponent;
//  robot_state::RobotState *mRobotState;
  robot_state::RobotState *mManagerRobotState;
  const robot_state::JointModelGroup *mJointModelGroup;

public:
  std::string mEEName;
  std::string mBaseName;

protected:
public:
  MotionPlanner(rclcpp::Node::SharedPtr node, rclcpp::Clock::SharedPtr clock,
                const MoveItCppPtr &moveitCpp, const std::string &groupName,
                const std::string &serviceName, robot_state::RobotState *robotState);

  /* Set Start and End State */
  inline void setStartStateToCurrentState() { mComponent->setStartState(*mManagerRobotState); }

  inline bool setStartState(Eigen::VectorXd &qpos) {
    robot_state::RobotState state(*mManagerRobotState);
    state.setJointGroupPositions(mJointModelGroup, qpos);
    return mComponent->setStartState(state);
  }

  inline bool setGoalState(Eigen::VectorXd &qpos) {
    robot_state::RobotState state(*mManagerRobotState);
    state.setJointGroupPositions(mJointModelGroup, qpos);
    return mComponent->setGoal(state);
  }

  bool setGoalState(const physx::PxTransform &Pose, const std::string &linkName = "");

  /* Planning */
  MotionPlan plan();
};

}