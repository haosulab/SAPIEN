//
// Created by sim on 9/25/19.
//

#include "joint_pub_node.h"
#include <utility>

namespace sapien::robot {

JointPubNode::JointPubNode(ControllableArticulationWrapper *wrapper, double pubFrequency,
                           double updateFrequency, const std::string &robotName,
                           ros::NodeHandle *nh)
    : jointName(wrapper->get_drive_joint_name()), queue(wrapper->get_joint_state_queue()),
      mNodeHandle(nh), pubFrequency(pubFrequency), updateFrenquency(updateFrequency) {

  mPub = std::make_unique<ros::Publisher>(
      mNodeHandle->advertise<sensor_msgs::JointState>("/sapien/" + robotName + "/joint_states", 1));
  mStates.position.resize(jointName.size());
  mStates.name.assign(jointName.begin(), jointName.end());
  jointNum = jointName.size();

  // Multi-thread spin
  updateWorker = std::thread(&JointPubNode::updateJointStates, this);
  pubWorker = std::thread(&JointPubNode::spin, this);
}
void JointPubNode::spin() {
  ros::WallRate rate(pubFrequency);

  while (ros::ok() && !isCancel) {
    mPub->publish(mStates);
    rate.sleep();
  }
}
void JointPubNode::updateJointStates() {

  ros::WallRate rate(updateFrenquency);
  while (ros::ok() && !isCancel) {
    if (!queue->empty()) {
      std::vector<float> newJointAngles = queue->pop();
      mStates.header.stamp = ros::Time::now();
      mStates.position.assign(newJointAngles.begin(), newJointAngles.begin() + jointNum);
      mStates.velocity.assign(newJointAngles.begin() + jointNum, newJointAngles.end());
    }
    rate.sleep();
  }
}
void JointPubNode::cancel() {
  isCancel = false;
  pubWorker.join();
  updateWorker.join();
  ROS_INFO("Joint state publisher with topic %s canceled.", mPub->getTopic().c_str());
}
} // namespace sapien::robot
