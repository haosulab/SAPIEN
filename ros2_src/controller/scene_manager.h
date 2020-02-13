#pragma once

#include "event_system/event_system.h"
#include "rclcpp/rclcpp.hpp"
#include "robot_manager.h"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sapien_scene.h"

namespace sapien {
class SControllableArticulation;
class SScene;
namespace ros2 {

class RobotManager;

class SceneManager : public IEventListener<EventStep> {
protected:
  SScene *mScene;
  rclcpp::Node::SharedPtr mNode;
  std::string mNameSpace;

  // Cache
  std::vector<std::unique_ptr<RobotManager>> mRobotManagers;
  std::vector<std::unique_ptr<SControllableArticulation>> mArticulationWrappers;

  // Clock and Time Manage
  rclcpp::Clock::SharedPtr mClock;
  rclcpp::Time mTime;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr mClockPub;

protected:
public:
  explicit SceneManager(SScene *scene, const std::string &name)
      : mScene(scene){
    // Namespace of scene manager should be the same as the scene
    mNameSpace = "/" + name;
    scene->setName(name);
    mNode = rclcpp::Node::make_shared(name, mNameSpace);
    // Set the use_sim_time
    auto parameters_client = rclcpp::SyncParametersClient(mNode.get());
    using namespace std::chrono_literals;
    parameters_client.wait_for_service(0.05s);

    auto set_parameters_results =
        parameters_client.set_parameters({rclcpp::Parameter("use_sim_time", true)});
    for (auto &result : set_parameters_results) {
      assert(result.successful);
    }
    rclcpp::spin_some(mNode->shared_from_this());

    // ROS Time Clock which is different wall time and steady time, given by /clock topic
    // TODO: add namespace support
    mTime = rclcpp::Time(0, 0, RCL_ROS_TIME);
    mClock = rclcpp::Clock::make_shared(RCL_ROS_TIME);
    mClockPub =
        mNode->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rmw_qos_profile_default);
    if (!mClock->ros_time_is_active()) {
      RCLCPP_WARN(mNode->get_logger(), "ROS Time is not active for scene with name %s",
                  scene->getName().c_str());
    } else {
      RCLCPP_INFO(mNode->get_logger(), "Current time of scene %s is %ulld \n",
                  mScene->getName().c_str(), mClock->now().nanoseconds());
    }

    // Register Simulation Callback
    mScene->registerListener(*this);
  };

  ~SceneManager() { mScene->unregisterListener(*this); };

  RobotManager *buildRobotManager(SArticulation *articulation, const std::string &robotName) {
    auto robotNameSpace = mNameSpace + "/" + robotName;
    auto wrapper = std::make_unique<SControllableArticulation>(articulation);
    auto robotManager = std::make_unique<RobotManager>(wrapper.get(), robotNameSpace, mClock);
    auto robotMangerWeakPtr = robotManager.get();
    mRobotManagers.push_back(std::move(robotManager));
    mArticulationWrappers.push_back(std::move(wrapper));
    return robotMangerWeakPtr;
  };

protected:
  void onEvent(EventStep &event) override {
    // Update time step and publish the /clock message
    float currentTimeStep = mScene->getTimestep();
    mTime = mTime + rclcpp::Duration(rcl_duration_value_t(currentTimeStep * 10e9));
    rosgraph_msgs::msg::Clock clockMessage;
    clockMessage.set__clock(mTime);
    mClockPub->publish(clockMessage);

    // Fetch current information and add control signal to system for each robot
    for (int i = 0; i < mRobotManagers.size(); ++i) {
      mRobotManagers[i]->step();
    }
  };
};

} // namespace ros2
} // namespace sapien
