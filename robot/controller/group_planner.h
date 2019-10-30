#pragma once
#include "controllable_articulation_wrapper.h"
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>

class MoveGroupPlanner {
private:
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  uint32_t jointNum=0;

public:
  MoveGroupPlanner(const std::string &groupName);
  moveit::planning_interface::MoveGroupInterface moveGroup;

  // Function to mimic moveit commander behavior
  bool go(std::array<float, 3> pos, std::array<float, 4> quat, const std::string &frame);
  bool goJoint(std::vector<double> jointPos);
  bool goStraight(std::array<float, 3> pos, std::array<float, 4> quat, const std::string &frame);
};