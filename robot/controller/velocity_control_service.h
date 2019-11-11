//
// Created by sim on 10/2/19.
//
#pragma once

#include "controllable_articulation_wrapper.h"
#include "sapien_ros_utils/JointVelocityControl.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace sapien::robot {
class JointVelocityController {
private:
  std::vector<std::string> mJointName = {};
  ros::NodeHandle *mNodeHandle = nullptr;
  ros::ServiceServer mServer;
  std::string mServerName;
  sensor_msgs::JointState *mState;
  std::vector<uint32_t> jointIndex2Topic;

  // Interface to physx
  std::unique_ptr<ThreadSafeQueue> mQueue;
  std::unique_ptr<ThreadSafeQueue> mForceQueue;

public:
  bool forceEqual = false;
  float mTimestep;

public:
  JointVelocityController(ControllableArticulationWrapper *wrapper,
                          const std::vector<std::string> &jointName,
                          sensor_msgs::JointState *jointState, const std::string &serviceName,
                          float timestep, ros::NodeHandle *nh, const std::string &robotName);

  bool executeCB(sapien_ros_utils::JointVelocityControl::Request &req,
                 sapien_ros_utils::JointVelocityControl::Response &res);
  void moveJoint(const std::vector<std::string> &jointName, float velocity);
  void moveJoint(const std::vector<std::string> &jointName, const std::vector<float> &velocity);
  void moveJoint(const std::vector<float> &velocity);
};
} // namespace sapien::robot
