//
// Created by sim on 9/25/19.
//

#include "joint_pub_node.h"

#include <utility>

namespace robot_interface {

JointPubNode::JointPubNode(ThreadSafeQueue *queue,
                                         const std::vector<std::string> &jointName,
                                         double pubFrequency, double updateFrequency,
                                         const std::string &topicName,
                                         std::shared_ptr<ros::NodeHandle> nh)
    : jointName(jointName), queue(queue), mNodeHandle(std::move(nh)), pubInterval(1 / pubFrequency),
      updateInterval(1 / updateFrequency) {

  mPub = mNodeHandle->advertise<sensor_msgs::JointState>(topicName, 1);
  mStates.position.resize(jointName.size());
  mStates.name.assign(jointName.begin(), jointName.end());
}
void JointPubNode::spin() {
  ros::Time lastUpdate = ros::Time::now();
  ros::Time lastPub = ros::Time::now();
  ros::WallRate rate(1000);
  while (!ros::isShuttingDown()) {
    rate.sleep();
    ros::Time current = ros::Time::now();
    if (current.toSec() - lastUpdate.toSec() > updateInterval) {
      // Update Buffer
      updateJointStates();
      lastUpdate = current;
    }
    if (current.toSec() - lastPub.toSec() > pubInterval) {
      // Publish Topic
      mStates.header.stamp = lastUpdate;
      mPub.publish(mStates);
      lastPub = current;
    }
  }
}
void JointPubNode::updateJointStates() {
  // TODO: deal with velocity and acceleration
  if (queue->empty()) {
    return;
  }
  std::vector<float> newJointAngles = queue->pop();
  mStates.position.assign(newJointAngles.begin(), newJointAngles.end());
}
} // namespace robot_interface
