//
// Created by sim on 9/26/19.
//

#include "kinematics_articulation_wrapper.h"
#include <cassert>
#include <iostream>
#include <limits>

// Kinematics Articulation Wrapper
KJoint *PxKinematicsArticulationWrapper::createJoint(const JointType &type, KJoint *parent,
                                                     PxRigidDynamic *link,
                                                     const PxTransform &poseFromParent,
                                                     const PxTransform &poseFromChild,
                                                     PxReal upperLimit, PxReal lowerLimit,
                                                     const std::string &name) {
  std::string qName = name;
  if (!name.empty()) {
    if (jointName2JointPtr.find(name) != jointName2JointPtr.end()) {
      std::cerr << "Duplicated link name: " << name << std::endl;
      throw std::runtime_error("Name duplication");
    }
  } else {
    qName = "undefined_name" + std::to_string(undefinedNameGeneratorId++);
  }

  std::unique_ptr<KJoint> newJoint;
  switch (type) {
  case JointType::REVOLUTE:
    newJoint = std::make_unique<RevoluteKJoint>(link, parent, poseFromChild.getInverse(),
                                                poseFromParent, upperLimit, lowerLimit);
    break;
  case JointType::CONTINUOUS:
    newJoint = std::make_unique<ContinuousKJoint>(link, parent, poseFromChild.getInverse(),
                                                  poseFromParent);
    break;
  case JointType::PRISMATIC:
    newJoint = std::make_unique<PrismaticKJoint>(link, parent, poseFromChild.getInverse(),
                                                 poseFromParent, upperLimit, lowerLimit);
    break;
  case JointType::FIXED:
    newJoint =
        std::make_unique<FixedKJoint>(link, parent, poseFromChild.getInverse(), poseFromParent);
    break;
  case JointType::UNDEFINED:
    std::cerr << "Joint type not support for " << type << std::endl;
    throw std::runtime_error("Unsupported joint");
  }
  if (!parent) {
    mRoot = newJoint.get();
  }
  newJoint->setName(qName);
  KJoint *tempJoint = newJoint.get();
  jointName2JointPtr[qName] = std::move(newJoint);
  return tempJoint;
}
uint32_t PxKinematicsArticulationWrapper::dof() const { return DOF; }
void PxKinematicsArticulationWrapper::buildCache() {
  if (cached) {
    std::cerr << "KinematicsWrapper has already been cached" << std::endl;
    return;
  }
  DOF = 0;
  std::vector<KJoint *> stack = {mRoot};
  while (!stack.empty()) {
    auto currentJoint = stack.back();
    stack.pop_back();
    for (auto const &joint : currentJoint->children) {
      stack.push_back(joint);
    }

    // Add link to buffer
    linkListPtr.push_back(currentJoint->childLink);

    // Cache DOF specific information
    uint32_t qDOF = currentJoint->getDof();
    jointStartIndex.push_back(DOF);
    DOF += qDOF;
    std::string name = currentJoint->name;
    for (size_t i = 0; i < qDOF; ++i) {
      jointNameDOF.push_back(name);
      jointLimit.push_back(currentJoint->getLimits()[i]);
    }
    jointDOF.push_back(qDOF);
    jointName.push_back(name);
    jointListPtr.push_back(jointName2JointPtr[name].get());
  }
  jointNum = jointName2JointPtr.size();
  jointStartIndex.push_back(DOF);
  driveQpos.resize(DOF, 0);
  driveQvel.resize(DOF, 0);
  qpos.resize(DOF, 0);
  qvel.resize(DOF, 0);
  qacc.resize(DOF, 0);
  qf.resize(DOF, 0);
}
std::vector<std::tuple<PxReal, PxReal>> PxKinematicsArticulationWrapper::get_joint_limits() const {
  return jointLimit;
}
std::vector<uint32_t> PxKinematicsArticulationWrapper::get_joint_dofs() const { return jointDOF; }
std::vector<std::string> PxKinematicsArticulationWrapper::get_joint_names() const {
  return jointName;
}
std::vector<PxReal> PxKinematicsArticulationWrapper::get_qpos() const { return qpos; }
std::vector<physx::PxReal> PxKinematicsArticulationWrapper::get_qvel() const { return qvel; }
std::vector<physx::PxReal> PxKinematicsArticulationWrapper::get_qacc() const { return qacc; }
std::vector<physx::PxReal> PxKinematicsArticulationWrapper::get_qf() const {
  return std::vector<physx::PxReal>();
}
void PxKinematicsArticulationWrapper::set_qpos(const std::vector<PxReal> &v) {
  assert(v.size() == DOF);
  size_t j = 0;
  while (j < jointNum) {
    auto start = v.begin() + jointStartIndex[j];
    auto end = v.begin() + jointStartIndex[j + 1];
    std::vector<PxReal> jointQpos(start, end);
    jointListPtr[j]->setQpos(jointQpos);
    j += 1;
  }
  qpos = v;
  driveQpos = v;

  updateVelocityDrive = false;
}
void PxKinematicsArticulationWrapper::set_qvel(const std::vector<PxReal> &v) {
  assert(v.size() == dof());
  updateVelocityDrive = true;
  driveQvel = v;
}
void PxKinematicsArticulationWrapper::set_qacc(const std::vector<PxReal> &v) {}
void PxKinematicsArticulationWrapper::set_qf(const std::vector<PxReal> &v) {}
void PxKinematicsArticulationWrapper::set_drive_target(const std::vector<PxReal> &v) {
  assert(v.size() == DOF);
  size_t j = 0;
  while (j < jointNum) {
    auto start = v.begin() + jointStartIndex[j];
    auto end = v.begin() + jointStartIndex[j + 1];
    std::vector<PxReal> jointQpos(start, end);
    jointListPtr[j]->driveQpos(jointQpos);
    j += 1;
  }
  updateQpos = true;
  driveQpos = v;
}
std::vector<std::string> PxKinematicsArticulationWrapper::get_drive_joint_name() const {
  return jointNameDOF;
}

// Custom function
std::vector<PxRigidDynamic *> PxKinematicsArticulationWrapper::get_links() { return linkListPtr; }

// Update function should be called in the simulation loop
void PxKinematicsArticulationWrapper::update(PxReal timestep) {
  // Update drive target based on controllers
  if (hasActuator) {
    // Update by velocity controller
    for (size_t i = 0; i < velocityControllerQueueList.size(); ++i) {
      // If no controller give the signal, continue for next one
      if (velocityControllerQueueList[i]->empty()) {
        continue;
      }
      auto controllerIndex = velocityControllerIndexList[i];
      auto queue = velocityControllerQueueList[i]->pop();
      std::cout << "Wrapper: " << queue[0] << std::endl;
      for (size_t j = 0; j < controllerIndex.size(); ++j) {
        driveQpos[controllerIndex[j]] += queue[j] * timestep;
      }
      set_drive_target(driveQpos);
    }

    // Update by position controller
    for (size_t i = 0; i < positionControllerQueueList.size(); ++i) {
      // If no controller give the signal, continue for next one
      if (positionControllerQueueList[i]->empty()) {
        continue;
      }
      auto controllerIndex = positionControllerIndexList[i];
      auto queue = positionControllerQueueList[i]->pop();
      for (size_t j = 0; j < controllerIndex.size(); ++j) {
        driveQpos[controllerIndex[j]] = queue[j];
      }
      set_drive_target(driveQpos);
    }
  } else if (updateVelocityDrive) {
    for (std::size_t i = 0; i < dof(); ++i) {
      // Update drive of next step based on the drive velocity
      PxReal newQ = driveQpos[i] + driveQvel[i] * timestep;
      auto [upperLimit, lowerLimit] = jointLimit[i];
      newQ = newQ > upperLimit ? upperLimit : newQ;
      driveQpos[i] = newQ < lowerLimit ? lowerLimit : newQ;
    }
    set_drive_target(driveQpos);
  }

  // Update velocity based on time interval if the set_drive_target function is called once in this
  // simulation step
  if (!updateVelocityDrive) {
    for (size_t i = 0; i < dof(); ++i) {
      qvel[i] = (driveQpos[i] - qpos[i]) / timestep;
    }
  }
  if (updateQpos) {
    qpos = driveQpos;
  }
  updateQpos = false;

  // Update ROS related joint publisher buffer
  // In the future, the cast between PxReal and float should be make explicitly
  std::vector<PxReal> jointState = qpos;
  jointState.insert(jointState.end(), qvel.begin(), qvel.end());
  jointStateQueue.push(jointState);
}

ThreadSafeQueue *PxKinematicsArticulationWrapper::get_queue() { return &jointStateQueue; }
void PxKinematicsArticulationWrapper::add_position_controller(const std::vector<std::string> &name,
                                                              ThreadSafeQueue *queue) {
  std::vector<uint32_t> controllerIndex = {};
  for (const auto &qName : name) {
    for (size_t j = 0; j < jointNameDOF.size(); ++j) {
      if (qName == jointNameDOF[j]) {
        controllerIndex.push_back(j);
        break;
      }
    }
  }
  assert(controllerIndex.size() == name.size());
  positionControllerIndexList.push_back(controllerIndex);
  positionControllerQueueList.push_back(queue);
  hasActuator = true;
}
void PxKinematicsArticulationWrapper::add_velocity_controller(const std::vector<std::string> &name,
                                                              ThreadSafeQueue *queue) {
  std::vector<uint32_t> controllerIndex = {};
  for (const auto &qName : name) {
    for (size_t j = 0; j < jointNameDOF.size(); ++j) {
      if (qName == jointNameDOF[j]) {
        controllerIndex.push_back(j);
        break;
      }
    }
  }

  assert(controllerIndex.size() == name.size());
  velocityControllerIndexList.push_back(controllerIndex);
  velocityControllerQueueList.push_back(queue);
  hasActuator = true;
}
