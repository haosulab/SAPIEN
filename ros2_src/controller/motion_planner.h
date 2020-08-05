#pragma once

#include <PxPhysicsAPI.h>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>

namespace sapien {
class SScene;
class SActorBase;

namespace ros2 {
using namespace moveit::planning_interface;
struct MotionPlan;
class RobotManager;

class MotionPlanner {
  friend RobotManager;

protected:
  rclcpp::Node::SharedPtr mNode;
  rclcpp::Clock::SharedPtr mClock;
  SScene *mScene;
  RobotManager *mManager;

  MoveItCppPtr mMoveItCpp;
  PlanningComponentUniquePtr mComponent;
  robot_state::RobotState mRobotState;
  const robot_state::JointModelGroup *mJointModelGroup;

public:
  std::string mEEName;
  std::string mBaseName;

public:
  MotionPlanner(rclcpp::Node::SharedPtr node, rclcpp::Clock::SharedPtr clock,
                const MoveItCppPtr &moveitCpp, const std::string &groupName,
                const std::string &serviceName="");

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

  bool setGoalState(const physx::PxTransform &Pose, const std::string &linkName = "");

  /* Planning */
  MotionPlan plan();
  bool updateCollisionObjects(SActorBase *actor);

protected:
  geometry_msgs::msg::Pose fillPose(const physx::PxTransform &pose);
};

} // namespace ros2
} // namespace sapien
