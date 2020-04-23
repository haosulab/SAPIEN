#include "robot_loader.h"
#include "scene_manager.h"
#include <experimental/filesystem>
#include <fstream>
#include <tinyxml2.h>
using namespace tinyxml2;
using namespace sapien;
using namespace sapien::ros2;

std::array<std::string, 2>
sapien::ros2::RobotLoader::getFilePath(const std::string &packageName,
                                       const std::string &robotRelativePath,
                                       const std::string &semanticRelativePath) {
  auto packagePath = ament_index_cpp::get_package_share_directory(packageName);
  auto URDFPath = packagePath + "/" + robotRelativePath;
  if (!std::experimental::filesystem::exists(URDFPath)) {
    throw std::runtime_error("URDF path does not exist!");
  }

  //  If no srdf relative path is given, assume it locate near the urdf file
  std::string SRDFPath(URDFPath);
  SRDFPath = semanticRelativePath.empty()
                 ? SRDFPath.replace(SRDFPath.end() - 4, SRDFPath.end(), "srdf")
                 : packagePath + "/" + semanticRelativePath;
  SRDFPath = std::experimental::filesystem::exists(SRDFPath) ? SRDFPath : "";

  return {URDFPath, SRDFPath};
}

void RobotLoader::publishRobotDescription(rclcpp::Node::SharedPtr &node,
                                          const std::string &URDFString,
                                          const std::string &SRDFString) {
  node->declare_parameter(ROBOT_PARAM_NAME,
                          rclcpp::ParameterValue(modifyURDFPath2ROSConvention(URDFString)));
  node->declare_parameter(SEMANTIC_PARAM_NAME, rclcpp::ParameterValue(SRDFString));
  //  RCLCPP_INFO(node->get_logger(), "Successfully parsing URDF for ROS [%s]", node->get_name());
}

std::array<std::string, 2> sapien::ros2::RobotLoader::parseURDFFile(const std::string &URDFPath) {
  // read file
  std::ifstream file(URDFPath);
  if (!file) {
    throw std::runtime_error("Can not load URDF path: " + URDFPath);
  }
  //  RCLCPP_INFO(mNode->get_logger(), "Find URDF file %s [%s]", URDFPath.c_str(),
  //  mNode->get_name());
  std::string originURDFString;
  std::ifstream t(URDFPath);
  t.seekg(0, std::ios::end);
  originURDFString.reserve(t.tellg());
  t.seekg(0, std::ios::beg);
  originURDFString.assign((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());

  return {modifyURDFPath2SapienConvention(originURDFString, URDFPath), originURDFString};
}

std::tuple<SArticulation *, RobotManager *>
sapien::ros2::RobotLoader::loadRobot(const std::string &name, const std::string &URDFPath,
                                     const std::string &SRDFPath, physx::PxMaterial *material) {
  auto URDFInfo = parseURDFFile(URDFPath);
  std::string SRDFString;
  if (URDFInfo[0].empty()) {
    //    RCLCPP_ERROR(mNode->get_logger(), "Can not open URDF path %s", URDFPath.c_str());
    return {nullptr, nullptr};
  }
  //
  // Read srdf string from file
  if (!SRDFPath.empty()) {
    std::ifstream t(SRDFPath);
    t.seekg(0, std::ios::end);
    SRDFString.reserve(t.tellg());
    t.seekg(0, std::ios::beg);
    SRDFString.assign((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
  }

  // Load articulation and manager
  auto robot = mLoader->loadFromXML(URDFInfo[0], SRDFString, nullptr, false);
  if (not robot) {
    //    RCLCPP_ERROR(mNode->get_logger(), "Robot URDF parse fail: %s", URDFPath.c_str());
    return {nullptr, nullptr};
  }

  // Both original ROS URDF specify path with package:// or
  // SAPIEN URDF with absolute path can both be recognized by ROS.
  // Thus both URDFInfo[0] or URDFInfo[1] can be used for ROS applications
  auto robotManager = mManager->buildRobotManager(robot, name);
  publishRobotDescription(robotManager->mNode, URDFInfo[1], "");

  // Robot Manager should be init after the parameters are loaded
  robotManager->init();
  return {robot, (RobotManager *)nullptr};
}

sapien::ros2::RobotLoader::RobotLoader(sapien::ros2::SceneManager *manager)
    : mNode(manager->mNode), mManager(manager), mLoader(mManager->mScene->createURDFLoader()),
      fixRootLink(mLoader->fixRootLink), defaultDensity(mLoader->defaultDensity),
      collisionIsVisual(mLoader->collisionIsVisual) {
  // The general standard: if no srdf, srdf path should be empty
  // If no urdf, directly raise error and exit;
}

std::tuple<SArticulation *, RobotManager *>
sapien::ros2::RobotLoader::loadFromROS(const std::string &ROSPackageName,
                                       const std::string &robotRelativePath,
                                       const std::string &semanticRelativePath,
                                       const std::string &name, physx::PxMaterial *material) {
  //  auto paths = getFilePath(ROSPackageName, robotRelativePath, semanticRelativePath);
  //  std::string robotName = name.empty() ? ROSPackageName : name;
  return loadRobot("", "", "", material);
}

std::tuple<SArticulation *, RobotManager *> RobotLoader::load(const std::string &robotPath,
                                                              const std::string &semanticPath,
                                                              const std::string &name,
                                                              physx::PxMaterial *material) {
  std::string SRDFPath(robotPath);
  SRDFPath = semanticPath.empty() ? SRDFPath.replace(SRDFPath.end() - 4, SRDFPath.end(), "srdf")
                                  : semanticPath;
  SRDFPath = std::experimental::filesystem::exists(SRDFPath) ? SRDFPath : "";

  std::string robotName = name.empty() ? getRobotNameFromPath(robotPath) : name;
  return loadRobot(robotName, robotPath, SRDFPath, material);
}

std::tuple<SArticulation *, RobotManager *>
RobotLoader::loadFromString(const std::string &URDFString, const std::string &SRDFString,
                            const std::string &name, physx::PxMaterial *material) {

  auto parsedURDFString = modifyURDFPath2SapienConvention(URDFString);
  auto robot = mLoader->loadFromXML(parsedURDFString, SRDFString, material, false);
  if (not robot) {
    //    RCLCPP_ERROR(mNode->get_logger(), "Fail to load robot");
    return {nullptr, nullptr};
  }
  auto robotManager = mManager->buildRobotManager1(robot, name);
  publishRobotDescription(robotManager->mNode, URDFString, SRDFString);

  // Robot Manager should be init after the parameters are loaded
  robotManager->init();
  return {robot, robotManager};
}
std::string RobotLoader::modifyURDFPath2ROSConvention(const std::string &URDFString) {
  // Add fill:// before publish absolute path, otherwise ROS xml path can not recognize it
  auto legalURDFString = URDFString;
  const std::string searchString = "filename=\"/";
  const std::string replaceString = "filename=\"file:///";
  auto pos = legalURDFString.find(searchString);
  while (pos != std::string::npos) {
    legalURDFString.replace(pos, searchString.size(), replaceString);
    pos = legalURDFString.find(searchString, pos);
  }
  return legalURDFString;
}
std::string RobotLoader::getRobotNameFromPath(const std::string &URDFPath) {
  auto pos = URDFPath.find('/');
  auto lastPos = 0;
  while (pos != std::string::npos) {
    lastPos = pos;
    pos = URDFPath.find('/', pos + 1);
  }
  auto dotPos = URDFPath.find('.', lastPos);
  auto result = URDFPath.substr(lastPos + 1, dotPos - lastPos - 1);
  std::cout << result;
  return result;
}
std::string RobotLoader::modifyURDFPath2SapienConvention(const std::string &URDFString,
                                                         const std::string &URDFPath) {
  // Replace ROS package signature
  auto modifiedURDFString = URDFString;
  const std::string replace = "package:/";
  auto pos = modifiedURDFString.find(replace);
  while (pos != std::string::npos) {
    size_t packageEndPos = modifiedURDFString.find('/', pos + 10);
    auto packageName = modifiedURDFString.substr(pos + 10, packageEndPos - pos - 10);
    auto packagePath = ament_index_cpp::get_package_share_directory(packageName);
    modifiedURDFString.replace(pos, replace.size() + packageName.size() + 1, packagePath);
    pos = modifiedURDFString.find(replace, packageEndPos);
  }

  // Replace relative path
  namespace fs = std::experimental::filesystem;
  const std::string searchString = "filename=\"";
  pos = modifiedURDFString.find(searchString);
  auto rootPath = fs::canonical(fs::path(URDFPath)).remove_filename().string();
  while (pos != std::string::npos) {
    if (modifiedURDFString[pos + searchString.size()] != '/') {
      modifiedURDFString.insert(pos + searchString.size(), rootPath);
    }
    pos = modifiedURDFString.find(searchString, pos + searchString.size());
  }

  return modifiedURDFString;
}
