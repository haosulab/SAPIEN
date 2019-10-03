//
// Created by sim on 10/2/19.
//

#include "velocity_control_service.h"
robot_interface::VelocityControllerServer::VelocityControllerServer(
    const std::vector<std::string> &jointName, float timestep, const std::string &serviceName,
    std::shared_ptr<ros::NodeHandle> nh, const std::string &nameSpace)
    : mJointName(jointName), mNodeHandle(std::move(nh)), mTimestep(timestep),
      mServerName(nameSpace + "/" + serviceName) {
  mServer = mNodeHandle->advertiseService(
      mServerName, &robot_interface::VelocityControllerServer::executeCB, this);
  mQueue = std::make_unique<ThreadSafeQueue>();
}
bool robot_interface::VelocityControllerServer::executeCB(
    physxtest::VelocityControlService::Request &req,
    physxtest::VelocityControlService::Response &res) {
  // Check joint name
  auto serviceJointName = req.joint_name;
  for (const auto &name : serviceJointName) {
    if (count(mJointName.begin(), mJointName.end(), name) != 1)
      ROS_ERROR("Received service has joint name out of the scope of controller joints: %s",
                name.c_str());
    res.success = false;
    return false;
  }

  // Generate joint velocity with right order
  std::vector<float> jointVelocity(mJointName.size(), 0);
  for (size_t i = 0; i < serviceJointName.size(); ++i) {
    uint32_t index =
        std::find(mJointName.begin(), mJointName.end(), serviceJointName[i]) - mJointName.begin();
    jointVelocity[index] = req.joint_velocity[i];
  }

  // Push velocity to the queue
  uint32_t step = req.duration.toSec() / mTimestep;
  for (size_t i = 0; i < step; ++i) {
    mQueue->pushValue(jointVelocity);
  }
}
const std::vector<std::string> &robot_interface::VelocityControllerServer::getJointNames() {
  return mJointName;
}
ThreadSafeQueue *robot_interface::VelocityControllerServer::getQueue() { return mQueue.get(); }
void robot_interface::VelocityControllerServer::spin() {
  ROS_INFO("Service %s start", mServerName.c_str());
  ros::spin();
}
