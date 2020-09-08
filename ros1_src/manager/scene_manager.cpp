#include "scene_manager.h"

#include "camera_publisher.h"
#include "robot_manager.h"
#include "sapien_actor_base.h"
#include "sapien_controllable_articulation.h"
#include "sapien_scene.h"
#include <sapien_actor.h>
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

void SceneManager::onEvent(sapien::EventStep &event) {
  // Update time step and publish the /clock message
  float currentTimeStep = event.timeStep;
  mTime += ros::Duration(currentTimeStep);
  rosgraph_msgs::Clock clockMessage;
  clockMessage.clock = mTime;
  mClockPub.publish(clockMessage);

  // Update camera publisher
  for (auto &j : mCameraPub) {
    j->update();
  }
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
void SceneManager::startAllCamera(double frequency, float mean, float std, float cloudCutOff,
                                  float cloudCutOffMax) {
  auto logger = spdlog::get("SAPIEN_ROS1");
  for (int j = 0; j < mScene->getMountedCameras().size(); ++j) {
    auto cam = mScene->getMountedCameras()[j];
    auto actor = mScene->getMountedActors()[j];
    auto frameName = actor->getName();
    auto cameraPub = std::make_unique<CameraPublisher>(cam, mNode, frameName, frequency);
    cameraPub->mPointCloudCutoff = cloudCutOff;
    cameraPub->mPointCloudCutoffMax = cloudCutOffMax;
    cameraPub->mMean = mean;
    cameraPub->mStd = std;
    mCameraPub.push_back(std::move(cameraPub));
    cam->takePicture();
  }
}
void SceneManager::start() {
  for (auto &mRobotManager : mRobotManagers) {
    mRobotManager->start();
  }
  mThread = std::thread(&ros::MultiThreadedSpinner::spin, mSpinner, nullptr);
}
bool SceneManager::onGetModelService(gazebo_msgs::GetModelStateRequest &req,
                                     gazebo_msgs::GetModelStateResponse &res) {
  auto logger = spdlog::get("SAPIEN_ROS1");
  logger->warn("receive {}", req.model_name);
  std::string name = req.model_name;
  auto index = std::find(mModelNames.begin(), mModelNames.end(), name);
  if (index == mModelNames.end()) {
    logger->error("Model name [{}] not found", name);
    res.success = false;
    return false;
  }

  uint32_t actorIndex = index - mModelNames.begin();
  auto sapienPose = mModelActors[actorIndex]->getPose();
  geometry_msgs::Pose pose;
  pose.position.x = sapienPose.p.x;
  pose.position.y = sapienPose.p.y;
  pose.position.z = sapienPose.p.z;
  pose.orientation.w = sapienPose.q.w;
  pose.orientation.x = sapienPose.q.x;
  pose.orientation.y = sapienPose.q.y;
  pose.orientation.z = sapienPose.q.z;
  res.pose = pose;
  res.success = true;
  res.status_message = "GetModelState: got properties";
  return true;
}

void SceneManager::startGetModelService(const std::string &serviceName,
                                        std::vector<SActorBase *> &actors) {
  mModelActors.clear();
  mModelNames.clear();
  for (auto &k : actors) {
    mModelActors.push_back(k);
    mModelNames.push_back(k->getName());
  }

  mGetModelService = std::make_unique<ros::ServiceServer>(
      mNode->advertiseService(serviceName, &SceneManager::onGetModelService, this));
}
} // namespace sapien::ros1
