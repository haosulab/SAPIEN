#pragma once

#include <utility>

#include "event_system/event_system.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "robot_manager.h"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sapien_scene.h"
#include "device/PS3_publisher.h"

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
  rclcpp::TimeSource mTS;
  rclcpp::Clock::SharedPtr mClock;
  rclcpp::Time mTime;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr mClockPub;

  // Thread and spin
  std::thread mThread;
  rclcpp::executors::SingleThreadedExecutor mExecutor;

  // PS3
  std::unique_ptr<PS3Publisher> ps3Publisher = nullptr;

protected:
public:
  explicit SceneManager(SScene *scene, const std::string &name) : mScene(scene), mExecutor() {
    // Namespace of scene manager should be the same as the scene
    mNameSpace = "/" + name;
    scene->setName(name);
    mNode = rclcpp::Node::make_shared(name);
    // Set the use_sim_time
    auto parameters_client = rclcpp::SyncParametersClient(mNode.get());
    using namespace std::chrono_literals;
    parameters_client.wait_for_service(0.2s);

    auto set_parameters_results =
        parameters_client.set_parameters({rclcpp::Parameter("use_sim_time", true)});
    for (auto &result : set_parameters_results) {
      assert(result.successful);
    }
    rclcpp::spin_some(mNode->shared_from_this());

    // ROS Time Clock which is different wall time and steady time, given by /clock topic
    // TODO: add namespace support
    mTS.attachNode(mNode);
    mTime = rclcpp::Time(0, 0, RCL_ROS_TIME);
    mClock = rclcpp::Clock::make_shared(RCL_ROS_TIME);
    mTS.attachClock(mClock);

    mClockPub =
        mNode->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rmw_qos_profile_default);
    if (!mClock->ros_time_is_active()) {
      RCLCPP_WARN(mNode->get_logger(), "ROS Time is not active for scene with name %s",
                  scene->getName().c_str());
    } else {
      RCLCPP_INFO(mNode->get_logger(), "Current time of scene %s is %u \n",
                  mScene->getName().c_str(), mClock->now().nanoseconds());
    }

    // Register Simulation Callback
    mScene->registerListener(*this);

    // Add it to executor
    mExecutor.add_node(mNode);
  };

  ~SceneManager() { mScene->unregisterListener(*this); };

  RobotManager *buildRobotManager(SArticulation *articulation, const std::string &robotName) {
    auto robotNameSpace = mNameSpace;
    auto wrapper = std::make_unique<SControllableArticulation>(articulation);
    auto robotManager =
        std::make_unique<RobotManager>(wrapper.get(), robotNameSpace, robotName, mClock);
    auto robotMangerWeakPtr = robotManager.get();

    // Register Scene Call Back
    mScene->registerListener(*wrapper);

    // Maintain Cache and add to executor
    mExecutor.add_node(robotManager->mNode->shared_from_this());
    mRobotManagers.push_back(std::move(robotManager));
    mArticulationWrappers.push_back(std::move(wrapper));

    return robotMangerWeakPtr;
  };

  void start() {
    for (auto &mRobotManager : mRobotManagers) {
      mRobotManager->start();
    }
    mThread = std::thread(&rclcpp::executors::SingleThreadedExecutor::spin, &mExecutor);
  }


  void createPS3Publisher(double pubFrequency) {
    if (ps3Publisher) {
      RCLCPP_WARN(mNode->get_logger(),
                  "PS3 publisher Node has already been created for this Scene Manager");
      RCLCPP_WARN(mNode->get_logger(), "Scene Manager will use the original PS3 pub node");
      return;
    }

    ps3Publisher = std::make_unique<PS3Publisher>(mNameSpace, mNode.get(), mClock, pubFrequency);
  }

  rclcpp::Time now() { return mClock->now(); };

protected:
  void onEvent(EventStep &event) override {
    // Update time step and publish the /clock message
    float currentTimeStep = mScene->getTimestep();
    mTime = mTime + rclcpp::Duration(rcl_duration_value_t(currentTimeStep * 1e9));
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
