#include "robot_loader.h"
#include "articulation/sapien_articulation.h"
#include "robot_descriptor.h"
#include "robot_manager.h"
#include "sapien_scene.h"
#include "scene_manager.h"

#include <experimental/filesystem>
#include <spdlog/spdlog.h>

using namespace tinyxml2;
using namespace sapien::ros2;

sapien::ros2::RobotLoader::RobotLoader(sapien::ros2::SceneManager *manager)
    : mNode(manager->mNode), mManager(manager), mLoader(mManager->mScene->createURDFLoader()),
      fixRootLink(mLoader->fixRootLink), defaultDensity(mLoader->defaultDensity),
      collisionIsVisual(mLoader->collisionIsVisual) {
  // The general standard: if no srdf, srdf path should be empty
  // If no urdf, directly raise error and exit;
  auto logger = spdlog::get("SAPIEN_ROS2");
}

void RobotLoader::publishRobotDescription(rclcpp::Node::SharedPtr &node,
                                          const std::string &URDFString,
                                          const std::string &SRDFString) {
  auto logger = spdlog::get("SAPIEN_ROS2");
  node->declare_parameter(ROBOT_PARAM_NAME, rclcpp::ParameterValue(URDFString));
  node->declare_parameter(SEMANTIC_PARAM_NAME, rclcpp::ParameterValue(SRDFString));
  logger->info("Publishing robot description for ROS [{}]", node->get_name());
}

std::tuple<sapien::SArticulation *, RobotManager *>
RobotLoader::loadRobotAndManager(RobotDescriptor &descriptor, const std::string &name,
                                 physx::PxMaterial *material) {
  auto logger = spdlog::get("SAPIEN_ROS2");
  const std::string &urdf = descriptor.getURDF();
  const std::string &srdf = descriptor.getSRDF();

  // Load robot
  auto robot = mLoader->loadFromXML(urdf, srdf, material);
  if (not robot) {
    logger->error("Robot {} loading fail", name);
    return {nullptr, nullptr};
  }

  // Load robot manager
  // Robot Manager should be init after the parameters are loaded
  auto robotManager = mManager->buildRobotManager(robot, name);
  publishRobotDescription(robotManager->mNode, descriptor.getStandardURDF(), srdf);
  descriptor.mArticulation = robot;
  robotManager->init();

  return {robot, robotManager};
}
