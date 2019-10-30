//
// Created by sim on 10/29/19.
//

#include "group_planner.h"
#include <geometry_msgs/PoseStamped.h>

MoveGroupPlanner::MoveGroupPlanner(const std::string &groupName) : moveGroup(groupName) {}
bool MoveGroupPlanner::go(std::array<float, 3> pos, std::array<float, 4> quat,
                          const std::string &frame) {
  geometry_msgs::PoseStamped target_pose;
  target_pose.pose.orientation.w = quat[0];
  target_pose.pose.orientation.x = quat[1];
  target_pose.pose.orientation.y = quat[2];
  target_pose.pose.orientation.z = quat[3];
  target_pose.pose.position.x = pos[0];
  target_pose.pose.position.y = pos[1];
  target_pose.pose.position.z = pos[2];
  moveGroup.setPoseTarget(target_pose);

  bool success = (moveGroup.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (not success) {
    ROS_WARN("Motion planning fail for a pose target!");
    return success;
  }
  auto code = moveGroup.move();
  ROS_INFO("Move to the pose goal, result: %d", code.val);
  return code.val == moveit::planning_interface::MoveItErrorCode::SUCCESS;
}
