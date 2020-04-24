#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time_source.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <utility>

#include "event_system/event_system.h"
#include "robot_loader.h"

namespace sapien {
class SScene;
class SArticulation;

namespace ros2 {

class RobotManager;
class RobotLoader;
class PS3Publisher;
class SControllableArticulationWrapper;
class RobotDescriptor;

class SceneManager : public IEventListener<EventStep> {
  friend RobotManager;
  friend RobotLoader;

protected:
  // Main handle
  SScene *mScene;
  rclcpp::Node::SharedPtr mNode;
  std::string mNameSpace;

  // Cache
  std::vector<std::unique_ptr<RobotManager>> mRobotManagers;
  std::vector<std::unique_ptr<SControllableArticulationWrapper>> mArticulationWrappers;

  // Clock and Time Manage
  rclcpp::TimeSource mTS;
  rclcpp::Clock::SharedPtr mClock;
  rclcpp::Time mTime;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr mClockPub;
  float mTimeStep = 0;

  // Thread and spin
  std::thread mThread;
  rclcpp::executors::SingleThreadedExecutor mExecutor;

  // PS3
  std::unique_ptr<PS3Publisher> ps3Publisher;

protected:
  RobotManager *buildRobotManager(SArticulation *articulation, const std::string &robotName);

public:
  explicit SceneManager(SScene *scene, const std::string &name);
  ~SceneManager();

  /* Start the ROS2 communication spin loop*/
  void start();

  /* Scene Level Construction */
  void createPS3Publisher(double pubFrequency);
  inline std::unique_ptr<RobotLoader> createRobotLoader() {
    return std::make_unique<RobotLoader>(this);
  };

  /* Utils */
  inline rclcpp::Time now() { return mClock->now(); };

protected:
  void onEvent(EventStep &event) override;
};

} // namespace ros2
} // namespace sapien
