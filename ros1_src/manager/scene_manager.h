#pragma once

#include <ros/ros.h>
#include <ros/timer.h>
#include <rosgraph_msgs/Clock.h>
#include <utility>

#include "event_system/event_system.h"
#include "robot_loader.h"

namespace sapien {
class SScene;
class SArticulation;

namespace ros1 {

class RobotManager;
class RobotLoader;
class SControllableArticulationWrapper;
class RobotDescriptor;

class SceneManager : public IEventListener<EventStep> {
  friend RobotManager;
  friend RobotLoader;

protected:
  // Main handle
  SScene *mScene;
  std::string mNameSpace;
  ros::NodeHandlePtr mNode;

  // Cache
  std::vector<std::unique_ptr<RobotManager>> mRobotManagers;
  std::vector<std::unique_ptr<SControllableArticulationWrapper>> mArticulationWrappers;

  // Clock and Time Manage
  ros::Publisher mClockPub;
  ros::Time mTime;
  float mTimeStep = 0;

  // Thread and spin
  ros::AsyncSpinner mSpinner;

protected:
  RobotManager *buildRobotManager(SArticulation *articulation, const std::string &robotName,
                                  double frequency);
  void onEvent(EventStep &event) override;

public:
  explicit SceneManager(SScene *scene, const std::string &name, uint8_t numThread = 1);
  ~SceneManager();

  inline std::unique_ptr<RobotLoader> createRobotLoader() {
    return std::make_unique<RobotLoader>(this);
  };

  /* ROS1 communication spin loop*/
  inline void start() { mSpinner.start(); };
  inline void stop() { mSpinner.stop(); };
};

} // namespace ros1
} // namespace sapien
