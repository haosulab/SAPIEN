#include "scene_manager.h"

#include "camera_publisher.h"
#include "robot_manager.h"
#include "sapien_actor_base.h"
#include "sapien_controllable_articulation.h"
#include "sapien_scene.h"
#include <spdlog/spdlog.h>

namespace sapien::ros1 {

SceneManager::SceneManager(sapien::SScene *scene, const std::string &name, uint8_t numThread)
    : mScene(scene), mNameSpace("/" + name), mNode(new ros::NodeHandle(name)),
      mSpinner(numThread) {
  // Namespace of scene manager should be the same as the scene
  auto logger = spdlog::get("SAPIEN_ROS1");
  if (!scene->getName().empty() && scene->getName() != name)
    logger->warn("Name of scene will be updated, original: {}, now: {}", scene->getName(), name);
  scene->setName(name);

  // Set the use_sim_time
  if (!ros::Time::isSimTime()) {
    logger->error("/use_sim_time parameter is not set to true before ROS init");
    logger->error("Set this parameter in launch file or using command line tools");
    assert(ros::Time::isSimTime());
  }
  double currentTime = ros::Time::now().toSec();
  logger->info("Current time of the scene is {:03.3f} s", currentTime);
  mClockPub = mNode->advertise<rosgraph_msgs::Clock>("/clock", 10);

  // Register Simulation Callback
  mScene->registerListener(*this);
}
SceneManager::~SceneManager() {
  mScene->unregisterListener(*this);
  stop();
}

// RobotManager *SceneManager::buildRobotManager(sapien::SArticulation *articulation,
//                                              const std::string &robotName) {
//  auto robotNameSpace = mNameSpace;
//  auto wrapper = std::make_unique<SControllableArticulationWrapper>(articulation, mClock);
//  auto robotManager =
//      std::make_unique<RobotManager>(wrapper.get(), robotNameSpace, robotName, mClock);
//  auto robotMangerWeakPtr = robotManager.get();
//  robotManager->mSceneManager = this;
//
//  // Register Scene Call Back
//  mScene->registerListener(*wrapper);
//
//  // Maintain Cache and add to executor
//  mExecutor.add_node(robotManager->mNode->shared_from_this());
//  mRobotManagers.push_back(std::move(robotManager));
//  mArticulationWrappers.push_back(std::move(wrapper));
//
//  return robotMangerWeakPtr;
//}
void SceneManager::onEvent(sapien::EventStep &event) {
  // Update time step and publish the /clock message
  float currentTimeStep = event.timeStep;
  mTime += ros::Duration(currentTimeStep);
  rosgraph_msgs::Clock clockMessage;
  clockMessage.clock = mTime;
  mClockPub.publish(clockMessage);

  // Update camera publisher
  for (auto & j : mCameraPub) {
    j->update();
  }

  // Fetch current information and add control signal to system for each robot
  // Update each robot if time step change
  //  bool changeTimeStep = false;
  //  if (abs(currentTimeStep - mTimeStep) > 1e-7) {
  //    changeTimeStep = true;
  //    mTimeStep = currentTimeStep;
  //  }
  //  for (auto &mRobotManager : mRobotManagers) {
  //    mRobotManager->step(changeTimeStep, currentTimeStep);
  //  }
}
RobotManager *SceneManager::buildRobotManager(SArticulation *articulation, ros::NodeHandlePtr node,
                                              const std::string &robotName, double frequency) {
  auto wrapper = std::make_unique<SControllableArticulationWrapper>(articulation);
  auto robotManager = std::make_unique<RobotManager>(wrapper.get(), robotName, node, frequency);
  auto robotMangerWeakPtr = robotManager.get();
  robotManager->mSceneManager = this;

  // Register Scene Call Back
  mScene->registerListener(*wrapper);

  mRobotManagers.push_back(std::move(robotManager));
  mArticulationWrappers.push_back(std::move(wrapper));
  return robotMangerWeakPtr;
}
void SceneManager::startAllCamera(double frequency) {
  auto logger = spdlog::get("SAPIEN_ROS1");
  for (int j = 0; j < mScene->getMountedCameras().size(); ++j) {
    auto cam = mScene->getMountedCameras()[j];
    auto actor = mScene->getMountedActors()[j];
    auto frameName = actor->getName();
    auto cameraPub = std::make_unique<CameraPublisher>(cam, mNode, frameName, frequency);
    mCameraPub.push_back(std::move(cameraPub));
    cam->takePicture();
  }
}
} // namespace sapien::ros1
