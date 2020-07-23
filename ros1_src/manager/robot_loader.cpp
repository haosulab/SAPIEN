#include "robot_loader.h"
#include "articulation/sapien_articulation.h"
#include "robot_descriptor.h"
#include "robot_manager.h"
#include "sapien_scene.h"
#include "scene_manager.h"

#include <experimental/filesystem>
#include <spdlog/spdlog.h>

using namespace tinyxml2;
using namespace sapien::ros1;

RobotLoader::RobotLoader(SceneManager *manager)
    : mNode(manager->mNode), mManager(manager), mLoader(mManager->mScene->createURDFLoader()),
      fixRootLink(mLoader->fixRootLink), collisionIsVisual(mLoader->collisionIsVisual) {
  auto logger = spdlog::get("SAPIEN_ROS1");
}

// void RobotLoader::publishRobotDescription(rclcpp::Node::SharedPtr &node,
//                                          const std::string &URDFString,
//                                          const std::string &SRDFString) {
//  auto logger = spdlog::get("SAPIEN_ROS2");
//  node->declare_parameter(ROBOT_PARAM_NAME, rclcpp::ParameterValue(URDFString));
//  node->declare_parameter(SEMANTIC_PARAM_NAME, rclcpp::ParameterValue(SRDFString));
//  logger->info("Publishing robot description for ROS [{}]", node->get_name());
//}

std::tuple<sapien::SArticulation *, RobotManager *> RobotLoader::loadFromParameterServer(
    const std::string &robotName, const sapien::URDF::URDFConfig &config, double frequency,
    const std::string &URDFParamName, const std::string &SRDFName) {
  auto logger = spdlog::get("SAPIEN_ROS1");
  std::string urdfName = URDFParamName.empty() ? ROBOT_PARAM_NAME : URDFParamName;
  std::string srdfName = SRDFName.empty() ? SEMANTIC_PARAM_NAME : SRDFName;
  RobotDescriptor descriptor = RobotDescriptor::fromParamterServer(urdfName, srdfName);

  const std::string &urdf = descriptor.getURDF();
  const std::string &srdf = descriptor.getSRDF();

  // Load robot
  auto robot = mLoader->loadFromXML(urdf, srdf, config);
  if (not robot) {
    logger->error("Robot {} loading fail", robotName);
    assert(robot);
  }

  // Load robot manager
  // Robot Manager should be init after the parameters are loaded
  auto robotManager = mManager->buildRobotManager(robot, robotName, frequency);
  return {robot, robotManager};
}
