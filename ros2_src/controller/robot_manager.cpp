#include "robot_manager.h"
#include "articulation/sapien_articulation.h"
#include "articulation/sapien_joint.h"
#include "sapien_controllable_articulation.h"

namespace sapien::ros2 {

RobotManager::RobotManager(SControllableArticulationWrapper *wrapper, const std::string &nameSpace,
                           const std::string &robotName, rclcpp::Clock::SharedPtr clock)
    : mWrapper(wrapper), mClock(std::move(clock)), mNameSpace(nameSpace + "/" + robotName) {

  mNode = rclcpp::Node::make_shared(robotName, nameSpace);

  // Create Initial Joint States
  mJointStates = std::make_unique<sensor_msgs::msg::JointState>();
  auto jointName = wrapper->getDriveJointNames();
  mJointNum = jointName.size();
  mJointStates->name = jointName;
  mJointStates->position.resize(jointName.size());
  mJointStates->velocity.resize(jointName.size());
}

void RobotManager::setDriveProperty(float stiffness, float damping, float forceLimit,
                                    const std::vector<uint32_t> &jointIndex) {
  std::vector<uint32_t> index(jointIndex);
  if (jointIndex.empty()) {
    index.resize(mWrapper->mJoints.size());
    std::iota(std::begin(index), std::end(index), 0);
  }

  for (unsigned int i : index) {
    assert(i < mWrapper->mArticulation->dof());
    auto joint = mWrapper->mJoints[i];
    joint->setDriveProperty(stiffness, damping, forceLimit);
  }
}

void RobotManager::balancePassiveForce(bool gravity, bool coriolisAndCentrifugal, bool external) {
  auto balanceForce =
      mWrapper->mArticulation->computePassiveForce(gravity, coriolisAndCentrifugal, external);
  mWrapper->mArticulation->setQf(balanceForce);
}
void RobotManager::updateJointStates(const std::vector<float> &jointAngles,
                                     const std::vector<float> &jointVelocity) {
  mJointStates->position.assign(jointAngles.begin(), jointAngles.begin() + mJointNum);
  mJointStates->velocity.assign(jointVelocity.begin(), jointVelocity.begin() + mJointNum);
  mJointStates->header.stamp = mClock->now();
}
void RobotManager::step() {
  updateJointStates(mWrapper->mJointPositions.read(), mWrapper->mJointVelocities.read());
}

// Create controller and publisher
void RobotManager::createJointPublisher(double pubFrequency) {
  if (mJointPublisher) {
    RCLCPP_WARN(mNode->get_logger(),
                "Joint Pub Node has already been created for this Robot Manager");
    RCLCPP_WARN(mNode->get_logger(), "Robot Manager will use the original joint state pub node");
    return;
  }

  mJointPublisher = std::make_unique<JointPublisher>(mNameSpace, mNode->shared_from_this(), mClock,
                                                     mJointStates.get(), pubFrequency);
}

std::weak_ptr<JointVelocityController>
RobotManager::buildJointVelocityController(const std::vector<std::string> &jointNames,
                                           const std::string &serviceName) {
  auto controller = std::make_shared<JointVelocityController>(mNameSpace, mNode, mClock, mWrapper,
                                                              jointNames, serviceName);
  mJointVelocityControllers.push_back(controller);
  return std::weak_ptr<JointVelocityController>(controller);
}
} // namespace sapien::ros2
