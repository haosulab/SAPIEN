//
// Created by sim on 10/2/19.
//
#pragma once

#include <include/VelocityControlService.h>
#include <kinematics_articulation_wrapper.h>
#include <ros/ros.h>

namespace robot_interface {
class VelocityControllerServer {
private:
  std::vector<std::string> mJointName;
    std::shared_ptr<ros::NodeHandle> mNodeHandle = nullptr;
  ros::ServiceServer mServer;
  float mTimestep;
  std::string mServerName;

  // Interface to physx
  std::unique_ptr<ThreadSafeQueue> mQueue;

public:
  VelocityControllerServer(const std::vector<std::string> &jointName, float timestep,
                           const std::string &serviceName, std::shared_ptr<ros::NodeHandle> nh,
                           const std::string &nameSpace = "physx");

  void spin();

  bool executeCB(physxtest::VelocityControlService::Request &req,
                 physxtest::VelocityControlService::Response &res);

  const std::vector<std::string> &getJointNames();
  ThreadSafeQueue *getQueue();
};
} // namespace robot_interface
