#include "sapien_controllable_articulation.h"
#include "articulation/sapien_articulation.h"
#include "articulation/sapien_joint.h"
#include <spdlog/spdlog.h>
#include <vector>
namespace sapien::ros1 {

SControllableArticulationWrapper::SControllableArticulationWrapper(
    sapien::SArticulation *articulation)
    : mArticulation(articulation), mJointPositions(articulation->dof()),
      mJointVelocities(articulation->dof()) {
  auto joints = mArticulation->getSJoints();
  for (auto &joint : joints) {
    if (joint->getDof() > 0) {
      mJoints.push_back(joint);
    }
    for (uint32_t j = 0; j < joint->getDof(); ++j) {
      mQNames.push_back(joint->getName());
    }
  }
}
bool SControllableArticulationWrapper::registerPositionCommands(
    ThreadSafeQueue<std::vector<float>> *command,
    const std::vector<std::string> &commandedJointNames,
    ThreadSafeQueue<ros::Time> *commandTimer) {
  std::vector<uint32_t> controllerIndex = {};
  for (const auto &qName : commandedJointNames) {
    auto iterator = std::find(mQNames.begin(), mQNames.end(), qName);
    if (iterator == mQNames.end()) {
      auto logger = spdlog::get("SAPIEN_ROS1");
      logger->warn("Joint name given not found in articulation %s \n", qName.c_str());
      return false;
    }
    auto index = iterator - mQNames.begin();
    controllerIndex.push_back(index);
  }

  // Register position based command
  mPositionCommandsIndex.push_back(controllerIndex);
  mPositionCommands.push_back(command);
  mPositionCommandsTimer.push_back(commandTimer);
  return true;
}
bool SControllableArticulationWrapper::registerVelocityCommands(
    ThreadSafeQueue<std::vector<float>> *command,
    const std::vector<std::string> &commandedJointNames,
    ThreadSafeQueue<ros::Time> *commandTimer) {
  std::vector<uint32_t> controllerIndex = {};
  for (const auto &qName : commandedJointNames) {
    auto iterator = std::find(mQNames.begin(), mQNames.end(), qName);
    if (iterator == mQNames.end()) {
      auto logger = spdlog::get("SAPIEN_ROS1");
      logger->warn("Joint name given not found in articulation %s \n", qName.c_str());
      return false;
    }
    auto index = iterator - mQNames.begin();
    controllerIndex.push_back(index);
  }

  // Register velocity based command
  mVelocityCommandsIndex.push_back(controllerIndex);
  mVelocityCommands.push_back(command);
  mVelocityCommandsTimer.push_back(commandTimer);
  return true;
}
bool SControllableArticulationWrapper::registerContinuousVelocityCommands(
    ThreadSafeVector<float> *command, const std::vector<std::string> &commandedJointNames,
    ThreadSafeQueue<ros::Time> *commandTimer) {
  std::vector<uint32_t> controllerIndex = {};
  for (const auto &qName : commandedJointNames) {
    auto iterator = std::find(mQNames.begin(), mQNames.end(), qName);
    if (iterator == mQNames.end()) {
      auto logger = spdlog::get("SAPIEN_ROS1");
      logger->warn("Joint name given not found in articulation %s \n", qName.c_str());
      return false;
    }
    auto index = iterator - mQNames.begin();
    controllerIndex.push_back(index);
  }

  // Register velocity based command
  mContinuousVelocityCommandsIndex.push_back(controllerIndex);
  mContinuousVelocityCommands.push_back(command);
  mContinuousVelocityCommandsTimer.push_back(commandTimer);
  return true;
}

void SControllableArticulationWrapper::onEvent(sapien::EventStep &event) {
  mJointPositions.write(mArticulation->getQpos());
  mJointVelocities.write(mArticulation->getQvel());
  mTimeStep = event.timeStep;
  auto current = ros::Time::now();

  // Aggregate position commands information
  std::vector<float> currentPositionCommands(mArticulation->dof(), -2000);
  for (size_t k = 0; k < mPositionCommands.size(); ++k) {
    if (mPositionCommands[k]->empty() || mPositionCommandsTimer[k]->front() > current)
      continue;

    auto command = mPositionCommands[k]->pop();
    auto index = mPositionCommandsIndex[k];
    while (!mPositionCommands[k]->empty() && mPositionCommandsTimer[k]->front() < current) {
      mPositionCommands[k]->pop();
      mPositionCommandsTimer[k]->pop();
    }
    for (size_t i = 0; i < index.size(); ++i) {
      currentPositionCommands[index[i]] = command[i];
    }
  }

  // Aggregate velocity commands information
  std::vector<float> currentVelocityCommands(mArticulation->dof(), 0);
  for (size_t k = 0; k < mVelocityCommands.size(); ++k) {
    if (mVelocityCommands[k]->empty() || mVelocityCommandsTimer[k]->front() > current)
      continue;

    auto index = mVelocityCommandsIndex[k];
    auto command = mVelocityCommands[k]->front();
    while (!mVelocityCommands[k]->empty() && mVelocityCommandsTimer[k]->front() < current) {
      mVelocityCommands[k]->pop();
      mVelocityCommandsTimer[k]->pop();
    }
    for (size_t i = 0; i < index.size(); ++i) {
      currentVelocityCommands[index[i]] += command[i];
    }
  }

  // Integrate Position and Velocity Commands
  auto lastDriveTarget = mArticulation->getDriveTarget();
  for (size_t l = 0; l < currentPositionCommands.size(); ++l) {
    if (currentPositionCommands[l] > -999) {
      currentPositionCommands[l] += currentVelocityCommands[l] * mTimeStep;
    } else {
      currentPositionCommands[l] = lastDriveTarget[l] + currentVelocityCommands[l] * mTimeStep;
    }
  }

  // Execute position command
  if (!mPositionCommands.empty() || !mVelocityCommands.empty()) {
    uint32_t i = 0;
    for (auto &j : mJoints) {
      for (auto axis : j->getAxes()) {
        if (currentPositionCommands[i] > -999) {
          j->getPxJoint()->setDriveTarget(axis, currentPositionCommands[i]);
        }
        i += 1;
      }
    }
  }

  // Aggregate continuous velocity information to velocity command
  // Continuous command should be added to the normal velocity command before drive velocity
  if (!mContinuousVelocityCommands.empty()) {
    for (size_t i = 0; i < mContinuousVelocityCommands.size(); ++i) {
      if (mContinuousVelocityCommandsTimer[i]->empty() ||
          mContinuousVelocityCommandsTimer[i]->front() > current)
        continue;
      auto continuousVelocityCommands = mContinuousVelocityCommands[i]->read();
      auto index = mContinuousVelocityCommandsIndex[i];
      for (size_t j = 0; j < index.size(); ++j) {
        currentVelocityCommands[index[j]] += continuousVelocityCommands[j];
      }
      mContinuousVelocityCommandsTimer[i]->pop();
    }
  }

  // Execute velocity command
  if (!mVelocityCommands.empty() || !mContinuousVelocityCommands.empty()) {
    uint32_t i = 0;
    for (auto &j : mJoints) {
      for (auto axis : j->getAxes()) {
        j->getPxJoint()->setDriveVelocity(axis, currentVelocityCommands[i]);
        i += 1;
      }
    }
  }
}
} // namespace sapien::ros1
