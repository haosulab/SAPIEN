#pragma once
#include "controllable_articulation_wrapper.h"
#include "joint_trajectory_controller.h"
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>

namespace sapien::robot {

class MoveGroupPlanner {
private:
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  uint32_t jointNum = 0;

public:
  MoveGroupPlanner(const std::string &groupName, GroupControllerNode *controller);
  moveit::planning_interface::MoveGroupInterface moveGroup;
  GroupControllerNode* controller;

  // Function to mimic moveit commander behavior
  bool go(const physx::PxTransform &pose, const std::string &frame);
  bool goJoint(std::vector<double> jointPos);
  bool goStraight(std::array<float, 3> pos, std::array<float, 4> quat, const std::string &frame);
};
} // namespace sapien::robot
