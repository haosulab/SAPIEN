//
// Created by sim on 10/29/19.
//

#include "group_planner.h"
#include <geometry_msgs/PoseStamped.h>
namespace sapien::robot {

MoveGroupPlanner::MoveGroupPlanner(const std::string &groupName, GroupControllerNode *controller)
    : moveGroup(groupName), controller(controller) {}
bool MoveGroupPlanner::go(const physx::PxTransform &pose, const std::string &frame) {
  std::cout<< "Receive cartesian go command!" << std::endl;
  geometry_msgs::PoseStamped target_pose;
  auto quat = pose.q;
  auto pos = pose.p;
  target_pose.pose.orientation.w = quat.w;
  target_pose.pose.orientation.x = quat.x;
  target_pose.pose.orientation.y = quat.y;
  target_pose.pose.orientation.z = quat.z;
  target_pose.pose.position.x = pos[0];
  target_pose.pose.position.y = pos[1];
  target_pose.pose.position.z = pos[2];
  target_pose.header.frame_id = frame;
  moveGroup.setPoseTarget(target_pose);
  std::cout<< "Plan executed!" << std::endl;

  bool success = (moveGroup.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  std::cout<< "Plan executed!" << std::endl;
  if (not success) {
    ROS_WARN("Motion planning fail for a pose target!");
    return success;
  }
  success = controller->execute(plan);
  std::cout<< "Go command executed!" << std::endl;
  return success;
}
} // namespace sapien::robot
