#include "articulation/urdf_loader.h"
#include "rclcpp/rclcpp.hpp"
#include "scene_manager.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <experimental/filesystem>
#include <fstream>
#include <tinyxml2.h>

namespace sapien::ros2 {
using namespace tinyxml2;

class ROSURDFLoader {
protected:
  /* Global Definition */
  const std::string ROBOT_PARAM_NAME = "robot_description";
  const std::string SEMANTIC_PARAM_NAME = "robot_description_semantic";

  /* ROS side variable */
  rclcpp::Node::SharedPtr mNode;
  SceneManager *mManager;

  /* URDF loader side variable */
  std::unique_ptr<URDF::URDFLoader> mLoader = nullptr;

public:
protected:
  static std::array<std::string, 2> getFilePath(const std::string &packageName,
                                                const std::string &robotRelativePath,
                                                const std::string &semanticRelativePath = "") {
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

  std::array<std::string, 2> parseURDFFile(const std::string &URDFPath) {
    // read file
    std::ifstream file(URDFPath);
    if (!file) {
      throw std::runtime_error("Can not load URDF path: " + URDFPath);
    }
    RCLCPP_INFO(mNode->get_logger(), "Find URDF file %s [%s]", URDFPath.c_str(),
                mNode->get_name());
    std::string originURDFString;
    std::ifstream t(URDFPath);
    t.seekg(0, std::ios::end);
    originURDFString.reserve(t.tellg());
    t.seekg(0, std::ios::beg);
    originURDFString.assign((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    std::string modifiedURDFString = originURDFString;

    // Change to absolute path
    const std::string replace = "package:/";
    size_t pos = modifiedURDFString.find(replace);
    while (pos != std::string::npos) {
      size_t packageEndPos = modifiedURDFString.find('/', pos + 10);
      auto packageName = modifiedURDFString.substr(pos + 10, packageEndPos - pos - 10);
      auto packagePath = ament_index_cpp::get_package_share_directory(packageName);
      modifiedURDFString.replace(pos, replace.size() + packageName.size() + 1, packagePath);
      pos = modifiedURDFString.find(replace, packageEndPos);
    }

    return {modifiedURDFString, originURDFString};
  }

  std::tuple<SArticulation *, RobotManager *> loadRobot(const std::string &name,
                                                        const std::string &URDFPath,
                                                        const std::string &SRDFPath,
                                                        physx::PxMaterial *material) {
    auto URDFInfo = parseURDFFile(URDFPath);
    std::string SRDFString;
    if (URDFInfo[0].empty()) {
      RCLCPP_ERROR(mNode->get_logger(), "Can not open URDF path %s", URDFPath.c_str());
      return {nullptr, nullptr};
    }

    // Read srdf string from file
    if (!SRDFPath.empty()) {
      std::ifstream t(SRDFPath);
      t.seekg(0, std::ios::end);
      SRDFString.reserve(t.tellg());
      t.seekg(0, std::ios::beg);
      SRDFString.assign((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    }

    // Load articulation and manager
    auto robot = mLoader->loadFromXML(URDFInfo[0], SRDFString, material, false);
    if (not robot) {
      RCLCPP_ERROR(mNode->get_logger(), "Robot URDF parse fail: %s", URDFPath.c_str());
      return {nullptr, nullptr};
    }
    auto robotManager = mManager->buildRobotManager(robot, name);
    auto robotNode = robotManager->mNode;

    // Set ROS parameter for robot description
    robotNode->declare_parameter(ROBOT_PARAM_NAME, rclcpp::ParameterValue(URDFInfo[1]));
    robotNode->declare_parameter(SEMANTIC_PARAM_NAME, rclcpp::ParameterValue(SRDFString));

    // Robot Manager should be init after the parameters are loaded
    robotManager->init();
    RCLCPP_INFO(robotNode->get_logger(), "Successfully parsing URDF for ROS [%s]",
                robotNode->get_name());
    return {robot, robotManager};
  };

public:
  std::tuple<SArticulation *, RobotManager *> load(const std::string &packageName,
                                                   const std::string &robotRelativePath,
                                                   const std::string &semanticRelativePath = "",
                                                   const std::string &name = "",
                                                   physx::PxMaterial *material = nullptr) {
    auto paths = getFilePath(packageName, robotRelativePath, semanticRelativePath);
    std::string robotName = name.empty() ? packageName : name;
    return loadRobot(robotName, paths.at(0), paths.at(1), material);
  };

  explicit ROSURDFLoader(SceneManager *manager) : mNode(manager->mNode), mManager(manager) {
    // The general standard should be, if no srdf: srdf path should be empty
    // If no urdf, directly raise error and exit;
    mLoader = mManager->mScene->createURDFLoader();
  };

}; // end class ROS_urdf_Loader
} // end namespace sapien::ros2