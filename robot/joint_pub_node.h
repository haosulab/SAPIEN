//
// Created by sim on 9/25/19.
//
#pragma once

#include "kinematics_articulation_wrapper.h"
#include <ros/publisher.h>
#include <ros/rate.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace robot_interface {

class JointPubNode {
private:
  sensor_msgs::JointState mStates;
  std::vector<std::string> jointName;

  // Buffer from interface
  ThreadSafeQueue *queue;
  std::thread worker;

  uint32_t jointNum;
  std::shared_ptr<ros::NodeHandle> mNodeHandle = nullptr;
  ros::Publisher mPub;
  double pubInterval;
  double updateInterval;

public:
  JointPubNode(PxKinematicsArticulationWrapper *wrapper, double pubFrequency,
               double updateFrequency, const std::string &topicName,
               std::shared_ptr<ros::NodeHandle> nh);

  void spin();

private:
  void updateJointStates();
};

} // namespace robot_interface
