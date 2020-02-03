#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sapien_scene.h"

namespace sapien {
class ControllableArticulationWrapper;
class SScene;
namespace ros2 {

class RobotManager;

class ROSSceneManager {
protected:
  SScene *mScene;
  rclcpp::Node mNode;
  rclcpp::Clock::SharedPtr mClock;
  std::string nameSpace;

  // Cache
  std::vector<std::unique_ptr<RobotManager>> mRobotManagers;
  std::vector<std::unique_ptr<ControllableArticulationWrapper>> mArticulationWrappers;

protected:
public:
  ROSSceneManager(SScene *scene) : mScene(scene), mNode(scene->getName(), "/" + scene->getName()) {
    nameSpace = "/" + scene->getName();

    // ROS Time Clock which is different wall time and steady time, given by /clock topic
    mClock = rclcpp::Clock::make_shared(RCL_ROS_TIME);
  };

  RobotManager *buildRobotManager(SArticulation *articulation, const std::string robotName) {
    auto robotNameSpace = nameSpace + "/" + robotName;
    auto wrapper = std::make_unique<ControllableArticulationWrapper>(articulation);
    auto robotManager = std::make_unique<RobotManager>(wrapper.get(), robotNameSpace, mClock);
    mRobotManagers.push_back(std::move(robotManager));
    mArticulationWrappers.push_back(std::move(wrapper));
  };
};

} // namespace ros2
} // namespace sapien
