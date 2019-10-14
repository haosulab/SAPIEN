//
// Created by sim on 9/28/19.
//

#include "joint_trajectory_controller.h"
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
namespace sapien::robot {
bool robot::GroupControllerNode::setsEqual(const std::vector<std::string> &controllerJoints,
                                           const std::vector<std::string> &goal) {
  for (const auto &i : goal) {
    if (count(controllerJoints.begin(), controllerJoints.end(), i) != 1)
      return false;
  }

  return true;
}
void robot::GroupControllerNode::clearGoal() { queue->clear(); }
void robot::GroupControllerNode::executeGoal(std::vector<uint32_t> indexGoal2Controller) {
  auto numPoints = current_trajectory_.points.size();
  if (numPoints < 2) {
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

  float bug = 0;

  while (now < end && next != current_trajectory_.points.end()) {
    auto percentage = timestep / (next->time_from_start - current->time_from_start).toSec();
    for (std::size_t i = 0; i < jointNum; i++) {
      step[i] = (next->positions[i] - currentPosition[i]) * percentage;
    }
    while (now < next->time_from_start) {
      // Transform the controller index from goal order to controller order
      std::vector<float> tempPosition(jointNum, 0);
      for (size_t i = 0; i < indexGoal2Controller.size(); ++i) {
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
  has_active_goal_ = false;
}
void robot::GroupControllerNode::executeCB(GoalHandle gh) {
  ROS_INFO("Receive joint trajectory goal");
  // Ensures that the joints in the goal match the joints we are commanding.
  std::vector<std::string> goalJoints = gh.getGoal()->trajectory.joint_names;
  if (!setsEqual(mJointName, goalJoints)) {
    ROS_ERROR("Joints on incoming goal don't match our joints");
    gh.setRejected();
    return;
  }

  //   Cancels the currently active goal.
  if (has_active_goal_) {
    // Stops the controller.
    trajectory_msgs::JointTrajectory empty;
    empty.joint_names = mJointName;
    clearGoal();

    // Marks the current goal as canceled.
    active_goal_.setCanceled();
    has_active_goal_ = false;
  }

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
void robot::GroupControllerNode::advance(const std::vector<float> &step,
                                         std::vector<float> &currentPosition) {
  for (std::size_t i = 0; i < jointNum; i++) {
    currentPosition[i] += step[i];
  }
}
void robot::GroupControllerNode::cancleCB(robot::GroupControllerNode::GoalHandle gh) {
  if (active_goal_ == gh) {
    // Marks the current goal as canceled.
    clearGoal();
    active_goal_.setCanceled();
    has_active_goal_ = false;
  }
}
robot::GroupControllerNode::GroupControllerNode(ControllableArticulationWrapper *wrapper,
                                                const std::string &groupName, float timestep,
                                                ros::NodeHandle *nh, const std::string &robotName)
    : groupName(groupName), mNodeHandle(nh), has_active_goal_(false), timestep(timestep) {
  // Gets all of the joints
  queue = std::make_unique<ThreadSafeQueue>();
  std::string controllerName = "sapien/" + robotName + "/" + groupName + "_controller";
  mServer =
      std::make_unique<JTAS>(*nh, controllerName + "/follow_joint_trajectory",
                             boost::bind(&robot::GroupControllerNode::executeCB, this, _1),
                             boost::bind(&robot::GroupControllerNode::cancleCB, this, _1), false);

  // Fetch value from parameters servers: the yaml file
  // Gets all of the joints
  XmlRpc::XmlRpcValue controllerList;
  if (!mNodeHandle->getParam("/move_group/controller_list", controllerList)) {
    ROS_FATAL("No controller list given. (namespace: %s)", mNodeHandle->getNamespace().c_str());
    exit(1);
  }
  if (controllerList.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_FATAL("Malformed controller specification.  (namespace: %s)",
              mNodeHandle->getNamespace().c_str());
    exit(1);
  }
  bool found = false;
  for (int i = 0; i < controllerList.size(); ++i) {
    std::string tempControllerName = static_cast<std::string>(controllerList[i]["name"]);
    if (tempControllerName != controllerName) {
      continue;
    } else {
      found = true;
      if (static_cast<std::string>(controllerList[i]["type"]) != "FollowJointTrajectory") {
        ROS_FATAL("Only FollowJointTrajectory is supported for this controller, use other "
                  "controller for other types");
        exit(1);
      }
      XmlRpc::XmlRpcValue &joints = controllerList[i]["joints"];
      for (int k = 0; k < joints.size(); ++k) {
        if (joints[k].getType() != XmlRpc::XmlRpcValue::TypeString) {
          ROS_FATAL("Array of joint names should contain all strings.  (namespace: %s)",
                    mNodeHandle->getNamespace().c_str());
          exit(1);
        }
        mJointName.push_back(static_cast<std::string>(joints[k]));
      }
    }
  }
  if (!found) {
    ROS_FATAL_STREAM("Controller list param found, but no controller named "
                     << controllerName << "found! Do you use the same as the one in yaml?");
  }

  jointNum = mJointName.size();

  // Register position controller to the wrapper
  wrapper->add_position_controller(mJointName, queue.get());
  ROS_INFO("Controller start with group name: %s", groupName.c_str());
  mServer->start();
}
} // namespace sapien::robot
