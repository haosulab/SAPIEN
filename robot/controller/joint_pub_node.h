//
// Created by sim on 9/25/19.
//
#pragma once

#include "kinematics_articulation_wrapper.h"
#include <ros/publisher.h>
#include <ros/rate.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace sapien::robot {

class JointPubNode {
public:
  std::unique_ptr<sensor_msgs::JointState> mStates = nullptr;

private:
  std::vector<std::string> jointName;

  // communication and multi-thread related variable
  ThreadSafeQueue *queue;
  std::thread pubWorker;
  std::thread updateWorker;
  bool isCancel = false;

  uint32_t jointNum;
  ros::NodeHandle *mNodeHandle = nullptr;
  std::unique_ptr<ros::Publisher> mPub;
  double pubFrequency;
  double updateFrequency;

public:
  JointPubNode(ControllableArticulationWrapper *wrapper, double pubFrequency,
               double updateFrequency, const std::string &robotName, ros::NodeHandle *nh);

  void cancel();

private:
  void updateJointStates();
  void spin();
};

} // namespace sapien::robot
