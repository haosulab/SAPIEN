#pragma once
#include "controllable_articulation_wrapper.h"
#include "joint_trajectory_controller.h"
#include <actionlib/client/simple_action_client.h>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>

using namespace moveit::planning_interface;
namespace sapien::robot {

class MoveGroupPlanner {
private:
  moveit::planning_interface::MoveGroupInterface::Plan plan = {};
  ros::NodeHandle *nh;
  uint32_t jointNum = 0;

public:
  MoveGroupPlanner(const std::string &groupName, GroupControllerNode *controller,
                   ros::NodeHandle *handle);
  GroupControllerNode *controller;
  moveit::planning_interface::MoveGroupInterface moveGroup;

  // Function to mimic moveit commander behavior
  MoveItErrorCode moveGroupPlan(MoveGroupInterface::Plan &plan);
  bool go(const physx::PxTransform &pose, const std::string &frame);
  //  bool goJoint(std::vector<double> jointPos);
  //  bool goStraight(std::array<float, 3> pos, std::array<float, 4> quat, const std::string
  //  &frame);
};
} // namespace sapien::robot
