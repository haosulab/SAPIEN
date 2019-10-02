//
// Created by sim on 9/28/19.
//

#pragma once

#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <control_msgs/FollowJointTrajectoryResult.h>
#include <kinematics_articulation_wrapper.h>
#include <ros/ros.h>

namespace robot_interface {
class GroupControllerNode {
private:
  // Define convenient type
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JTAS;
  typedef JTAS::GoalHandle GoalHandle;
  std::string groupName;

  // Controlling constant
  float timestep;
  uint32_t jointNum;

  // Action monitoring
  std::unique_ptr<JTAS> mServer;
  std::vector<std::string> mJointName;
  std::shared_ptr<ros::NodeHandle> mNodeHandle = nullptr;
  control_msgs::FollowJointTrajectoryGoal mGoal;
  control_msgs::FollowJointTrajectoryFeedback mFeedback;
  control_msgs::FollowJointTrajectoryResult mResult;

  // Action internal state
  bool has_active_goal_;
  GoalHandle active_goal_;
  trajectory_msgs::JointTrajectory current_trajectory_;

  // Interface to Physx
  std::unique_ptr<ThreadSafeQueue> queue;

public:
  GroupControllerNode(const std::string &groupName, float timestep,
                      std::shared_ptr<ros::NodeHandle> nh, const std::string &nameSpace = "physx",
                      std::string controllerName = "");

  void spin();
  ThreadSafeQueue *getQueue();
  const std::vector<std::string> getJointNames();

private:
  void executeCB(GoalHandle gh);
  void cancleCB(GoalHandle gh);
  static bool setsEqual(const std::vector<std::string> &controllerJoints,
                        const std::vector<std::string> &goal);

  // Control internal buffer
  void clearGoal();
  void executeGoal(std::vector<uint32_t> indexGoal2Controller);
  void advance(const std::vector<float> &step, std::vector<float> &currentPosition);

  // Buffer from interface
};
} // namespace robot_interface
