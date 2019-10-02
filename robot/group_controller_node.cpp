//
// Created by sim on 9/28/19.
//

#include <boost/bind.hpp>

#include "group_controller_node.h"
bool robot_interface::GroupControllerNode::setsEqual(
    const std::vector<std::string> &controllerJoints, const std::vector<std::string> &goal) {
  for (const auto &i : goal) {
    if (count(controllerJoints.begin(), controllerJoints.end(), i) != 1)
      return false;
  }

  return true;
}
void robot_interface::GroupControllerNode::clearGoal() { queue->clear(); }
void robot_interface::GroupControllerNode::executeGoal(std::vector<uint32_t> indexGoal2Controller) {
  auto numPoints = current_trajectory_.points.size();
  if(numPoints < 2){
    ROS_ERROR("Invalid trajectory, number of points should be greater than or equal to 2");
    return;
  }

  ros::Duration now = current_trajectory_.points.begin()->time_from_start;
  ros::Duration end = (current_trajectory_.points.begin() + numPoints - 1)->time_from_start;

  auto current = current_trajectory_.points.begin();
  auto next = current_trajectory_.points.begin() + 1;

  std::vector<float> currentPosition;
  currentPosition.assign(current->positions.begin(), current->positions.end());
  std::vector<float> step(jointNum, 0);

  float bug=0;

  while (now < end && next != current_trajectory_.points.end()) {
    auto percentage = timestep / (next->time_from_start - current->time_from_start).toSec();
    for (std::size_t i = 0; i < jointNum; i++) {
      step[i] = (next->positions[i] - currentPosition[i]) * percentage;
    }
    while (now < next->time_from_start) {
      // Transform the controller index from goal order to controller order
      std::vector<float> tempPosition(jointNum, 0);
      for(size_t i=0;i < indexGoal2Controller.size();++i){
        tempPosition[indexGoal2Controller[i]] = currentPosition[i];
      }
      queue->pushValue(tempPosition);

      advance(step, currentPosition);
      now += ros::Duration(timestep);

      bug = tempPosition[0];
    }
    current += 1;
    next += 1;
  }
  ROS_INFO("Controller: %f", bug);
}
void robot_interface::GroupControllerNode::executeCB(GoalHandle gh) {
  ROS_INFO("Receive joint trajectory goal");
  bool success = true;

  // Ensures that the joints in the goal match the joints we are commanding.
  std::vector<std::string> goalJoints = gh.getGoal()->trajectory.joint_names;
  if (!setsEqual(mJointName, goalJoints)) {
    ROS_ERROR("Joints on incoming goal don't match our joints");
    gh.setRejected();
    return;
  }

  // Cancels the currently active goal.
//  if (has_active_goal_) {
//    // Stops the controller.
//    trajectory_msgs::JointTrajectory empty;
//    empty.joint_names = mJointName;
//    clearGoal();
//
//    // Marks the current goal as canceled.
//    active_goal_.setCanceled();
//    has_active_goal_ = false;
//  }

  std::vector<uint32_t> indexGoal2Controller = {};
  indexGoal2Controller.resize(goalJoints.size());
  for (size_t i = 0; i < goalJoints.size(); ++i) {
    auto index = std::find(mJointName.begin(), mJointName.end(), goalJoints[i]);
    indexGoal2Controller[i] = index - mJointName.begin();
  }
  gh.setAccepted();
  active_goal_ = gh;
  has_active_goal_ = true;

  // TODO: handle goal more careful here
  current_trajectory_ = active_goal_.getGoal()->trajectory;
  executeGoal(indexGoal2Controller);
  mResult.error_code = mResult.SUCCESSFUL;
  mResult.error_string = "";
  gh.setSucceeded(mResult);
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
    const std::vector<std::string> &jointName, const std::string &group,
    const std::string &nameSpace, float timestep, std::shared_ptr<ros::NodeHandle> nh)
    : mNodeHandle(nh), has_active_goal_(false), groupName(group), timestep(timestep),
      mServer(*nh, "physx/" + group + "/follow_joint_trajectory",
              boost::bind(&robot_interface::GroupControllerNode::executeCB, this, _1),
              boost::bind(&robot_interface::GroupControllerNode::cancleCB, this, _1), false) {
  // Gets all of the joints
  mJointName = jointName;
  jointNum = jointName.size();
  queue = std::make_unique<ThreadSafeQueue>();
}
void robot_interface::GroupControllerNode::spin() {
//  ROS_INFO("Controller start with group name: %s", groupName.c_str());
  mServer.start();
  ros::spin();
}
ThreadSafeQueue *robot_interface::GroupControllerNode::getQueue() { return queue.get(); }
