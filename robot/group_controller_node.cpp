//
// Created by sim on 9/28/19.
//

#include <boost/bind.hpp>

#include "group_controller_node.h"
bool robot_interface::GroupControllerNode::setsEqual(const std::vector<std::string> &a,
                                                     const std::vector<std::string> &b) {
  if (a.size() != b.size())
    return false;

  for (const auto &i : a) {
    if (count(b.begin(), b.end(), i) != 1)
      return false;
  }
  for (const auto &i : b) {
    if (count(a.begin(), a.end(), i) != 1)
      return false;
  }

  return true;
}
void robot_interface::GroupControllerNode::clearGoal() { queue->clear(); }
void robot_interface::GroupControllerNode::executeGoal() {
  ros::Duration now = current_trajectory_.points.begin()->time_from_start;
  ros::Duration end = current_trajectory_.points.end()->time_from_start;

  auto current = current_trajectory_.points.begin();
  auto next = current_trajectory_.points.begin() + 1;

  std::vector<float> currentPosition;
  currentPosition.assign(current->positions.begin(), current->positions.end());
  std::vector<float> step;
  step.resize(jointNum, 0);

  while (now < end && next != current_trajectory_.points.end()) {
    step.resize(jointNum);
    auto percentage = timestep / (next->time_from_start - current->time_from_start).toSec();
    for (std::size_t i = 0; i < jointNum; i++) {
      step[i] = (next->positions[i] - currentPosition[i]) * percentage;
    }
    while (now < next->time_from_start) {
      // TODO: index of controller and physx
      queue->pushValue(currentPosition);
      advance(step, currentPosition);
      now += ros::Duration(timestep);
    }
    current += 1;
    next += 1;
  }
}
void robot_interface::GroupControllerNode::executeCB(GoalHandle gh) {
  ROS_INFO("Receive joint trajectory goal");
  bool success = true;

  // Ensures that the joints in the goal match the joints we are commanding.
  if (!setsEqual(mJointName, gh.getGoal()->trajectory.joint_names)) {
    ROS_ERROR("Joints on incoming goal don't match our joints");
    gh.setRejected();
    return;
  }

  // Cancels the currently active goal.
  if (has_active_goal_) {
    // Stops the controller.
    trajectory_msgs::JointTrajectory empty;
    empty.joint_names = mJointName;
    clearGoal();

    // Marks the current goal as canceled.
    active_goal_.setCanceled();
    has_active_goal_ = false;
  }

  gh.setAccepted();
  active_goal_ = gh;
  has_active_goal_ = true;

  // Sends the trajectory along to the controller
  current_trajectory_ = active_goal_.getGoal()->trajectory;
  executeGoal();
}
void robot_interface::GroupControllerNode::advance(const std::vector<float> &step,
                                                   std::vector<float> &currentPosition) {
  for (std::size_t i = 0; i < jointNum; i++) {
    currentPosition[i] += step[i];
  }
}
void robot_interface::GroupControllerNode::cancleCB(
    robot_interface::GroupControllerNode::GoalHandle gh) {
  if (active_goal_ == gh) {
    // Marks the current goal as canceled.
    clearGoal();
    active_goal_.setCanceled();
    has_active_goal_ = false;
  }
}
robot_interface::GroupControllerNode::GroupControllerNode(
    const std::vector<std::string> &jointName, const std::string &nameSpace,
    std::shared_ptr<ros::NodeHandle> nh)
    : mNodeHandle(nh), has_active_goal_(false),
      mServer(*nh, nameSpace + "/joint_trajectory_action",
              boost::bind(&robot_interface::GroupControllerNode::executeCB, this, _1),
              boost::bind(&robot_interface::GroupControllerNode::cancleCB, this, _1), false) {
  // Gets all of the joints
  mJointName = jointName;
  queue = std::make_unique<ThreadSafeQueue>();
}
void robot_interface::GroupControllerNode::spin() {
  ros::Time started_waiting_for_controller = ros::Time::now();
  while (ros::ok()) {
    ros::spinOnce();
    if (started_waiting_for_controller != ros::Time(0) &&
        ros::Time::now() > started_waiting_for_controller + ros::Duration(30.0)) {
      ROS_WARN("Waited for the controller for 30 seconds, but it never showed up.");
      started_waiting_for_controller = ros::Time(0);
    }
    ros::Duration(0.1).sleep();
    mServer.start();
  }
}
