//
// Created by sim on 10/2/19.
//

#include "velocity_control_service.h"
namespace sapien::robot {

JointVelocityController::JointVelocityController(ControllableArticulationWrapper *wrapper,
                                                 const std::vector<std::string> &jointName,
                                                 sensor_msgs::JointState *jointState,
                                                 const std::string &serviceName, float timestep,
                                                 ros::NodeHandle *nh, const std::string &robotName)
    : mJointName(jointName), mNodeHandle(nh),
      mServerName("/sapien/" + robotName + "/" + serviceName + "/joint_velocity"),
      mTimestep(timestep) {
  if (jointName.empty()) {
    throw std::runtime_error("Can not create joint velocity controller with no joint given");
  }
  mServer =
      mNodeHandle->advertiseService(mServerName, &robot::JointVelocityController::executeCB, this);
  mQueue = std::make_unique<ThreadSafeQueue>();
  mForceQueue = std::make_unique<ThreadSafeQueue>();

  // Create states for force equal joint controller
  mState = jointState;
  std::vector<std::string> topicJointNames = mState->name;
  for (auto &i : jointName) {
    auto index = std::find(topicJointNames.begin(), topicJointNames.end(), i);
    if (index == topicJointNames.end()) {
      ROS_ERROR("Joint name in controller not found in joint topic: %s", i.c_str());
    } else {
      jointIndex2Topic.push_back(index - topicJointNames.begin());
    }
  }

  // Register queue to controllable wrapper
  wrapper->add_velocity_controller(mJointName, mQueue.get());
  wrapper->add_position_controller(mJointName, mForceQueue.get());
}
bool JointVelocityController::executeCB(sapien_ros_utils::JointVelocityControl::Request &req,
                                        sapien_ros_utils::JointVelocityControl::Response &res) {
  // Check joint name
  auto serviceJointName = req.joint_name;
  for (const auto &name : serviceJointName) {
    if (count(mJointName.begin(), mJointName.end(), name) != 1) {
      ROS_ERROR("Received service has joint name out of the scope of controller joints: %s",
                name.c_str());
      res.success = false;
      return false;
    }
  }

  // Generate joint stepSize with right order
  std::vector<float> jointVelocity(mJointName.size(), 0);
  for (size_t i = 0; i < serviceJointName.size(); ++i) {
    uint32_t index =
        std::find(mJointName.begin(), mJointName.end(), serviceJointName[i]) - mJointName.begin();
    jointVelocity[index] = req.joint_velocity[i];
  }

  // Push stepSize to the queue
  uint32_t step = req.duration.toSec() / mTimestep;
  for (size_t i = 0; i < step; ++i) {
    mQueue->pushValue(jointVelocity);
  }
  res.success = true;
  return true;
}
void JointVelocityController::moveJoint(const std::vector<std::string> &jointName,
                                        float velocity) {
  if (forceEqual) {
    // For force equal version, no need for joint name
    auto index = jointIndex2Topic[0];
    auto current = mState->position[index] + velocity * mTimestep;
    std::vector<float> jointPosition(mJointName.size(), current);
    mForceQueue->push(jointPosition);
  }
  // Check joint name
  std::vector<float> jointVelocity(mJointName.size(), 0);
  for (const auto &i : jointName) {
    uint32_t index = std::find(mJointName.begin(), mJointName.end(), i) - mJointName.begin();
    if (index == mJointName.size()) {
      ROS_ERROR("Joint name not found in joint velocity controller: %s", i.c_str());
      return;
    }
    jointVelocity[index] = velocity;
  }
  mQueue->push(jointVelocity);
}
} // namespace sapien::robot
