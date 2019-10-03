//
// Created by sim on 9/25/19.
//

#include "joint_pub_node.h"

#include <utility>

namespace robot_interface {

JointPubNode::JointPubNode(PxKinematicsArticulationWrapper *wrapper, double pubFrequency,
                           double updateFrequency, const std::string &topicName,
                           std::shared_ptr<ros::NodeHandle> nh)
    : jointName(wrapper->get_drive_joint_name()), queue(wrapper->get_queue()),
      mNodeHandle(std::move(nh)), pubFrequency(pubFrequency), updateFrenquency(updateFrequency) {

  mPub = mNodeHandle->advertise<sensor_msgs::JointState>(topicName, 1);
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
    mPub.publish(mStates);
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
  ROS_INFO("Joint state publisher with topic %s canceled.", mPub.getTopic().c_str());
}
} // namespace robot_interface
