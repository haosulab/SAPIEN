#include "scene_manager.h"

#include "controller/sapien_controllable_articulation.h"
#include "device/PS3_publisher.h"
#include "robot_manager.h"
#include "sapien_scene.h"

sapien::ros2::SceneManager::SceneManager(sapien::SScene *scene, const std::string &name)
    : mScene(scene), mExecutor() {
  // Namespace of scene manager should be the same as the scene
  if (!scene->getName().empty() && scene->getName() != name)
    RCLCPP_WARN(mNode->get_logger(), "Name of scene will be updated, original: %s, now: %s",
                scene->getName().c_str(), name.c_str());
  mNameSpace = "/" + name;
  scene->setName(name);
  mNode = rclcpp::Node::make_shared(name);
  // Set the use_sim_time
  auto parametersClient = rclcpp::SyncParametersClient(mNode.get());
  using namespace std::chrono_literals;
  parametersClient.wait_for_service(0.1s);

  auto set_parameters_results =
      parametersClient.set_parameters({rclcpp::Parameter("use_sim_time", true)});
  for (auto &result : set_parameters_results) {
    assert(result.successful);
  }
  rclcpp::spin_some(mNode->shared_from_this());

  // ROS Time Clock which is different wall time and steady time, given by /clock topic
  mTS.attachNode(mNode);
  mTime = rclcpp::Time(0, 0, RCL_ROS_TIME);
  mClock = rclcpp::Clock::make_shared(RCL_ROS_TIME);
  mTS.attachClock(mClock);

  mClockPub = mNode->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
  if (!mClock->ros_time_is_active()) {
    RCLCPP_WARN(mNode->get_logger(), "ROS Time is not active for scene with name %s",
                scene->getName().c_str());
  } else {
    RCLCPP_INFO(mNode->get_logger(), "Current time of scene %s is %u \n",
                mScene->getName().c_str(), mClock->now().nanoseconds());
  }

  // Register Simulation Callback
  mScene->registerListener(*this);
  mTimeStep = scene->getTimestep();

  // Add it to executor
  mExecutor.add_node(mNode);
}
sapien::ros2::SceneManager::~SceneManager() { mScene->unregisterListener(*this); }
sapien::ros2::RobotManager *
sapien::ros2::SceneManager::buildRobotManager(sapien::SArticulation *articulation,
                                              const std::string &robotName) {
  auto robotNameSpace = mNameSpace;
  auto wrapper = std::make_unique<SControllableArticulationWrapper>(articulation, mClock);
  auto robotManager =
      std::make_unique<RobotManager>(wrapper.get(), robotNameSpace, robotName, mClock);
  auto robotMangerWeakPtr = robotManager.get();
  robotManager->mSceneManager = this;

  // Register Scene Call Back
  mScene->registerListener(*wrapper);

  // Maintain Cache and add to executor
  mExecutor.add_node(robotManager->mNode->shared_from_this());
  mRobotManagers.push_back(std::move(robotManager));
  mArticulationWrappers.push_back(std::move(wrapper));

  return robotMangerWeakPtr;
}
void sapien::ros2::SceneManager::start() {
  for (auto &mRobotManager : mRobotManagers) {
    mRobotManager->start();
  }
  for (auto &mRobotManager : mRobotManagers) {
    mRobotManager->step(true, mTimeStep);
  }
  mThread = std::thread(&rclcpp::executors::SingleThreadedExecutor::spin, &mExecutor);
}
void sapien::ros2::SceneManager::createPS3Publisher(double pubFrequency) {
  if (ps3Publisher) {
    RCLCPP_WARN(mNode->get_logger(),
                "PS3 publisher Node has already been created for this Scene Manager");
    RCLCPP_WARN(mNode->get_logger(), "Scene Manager will use the original PS3 pub node");
    return;
  }

  ps3Publisher =
      std::make_unique<PS3Publisher>(mNameSpace, mNode->shared_from_this(), mClock, pubFrequency);
}
void sapien::ros2::SceneManager::onEvent(sapien::EventStep &event) {
  // Update time step and publish the /clock message
  float currentTimeStep = event.timeStep;
  mTime = mTime + rclcpp::Duration(rcl_duration_value_t(currentTimeStep * 1e9));
  rosgraph_msgs::msg::Clock clockMessage;
  clockMessage.set__clock(mTime);
  mClockPub->publish(clockMessage);

  // Fetch current information and add control signal to system for each robot
  // Update each robot if time step change
  bool changeTimeStep = false;
  if (abs(currentTimeStep - mTimeStep) > 1e-7) {
    changeTimeStep = true;
    mTimeStep = currentTimeStep;
  }
  for (auto &mRobotManager : mRobotManagers) {
    mRobotManager->step(changeTimeStep, currentTimeStep);
  }
}
