#include <fstream>
#include <cstdlib>
#include <tinyxml2.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/rclcpp.hpp"
#include "articulation/urdf_loader.h"
#include "articulation/sapien_kinematic_articulation.h"
#include "sapien_scene.h"

namespace sapien::ros2 {
using namespace tinyxml2;

static const std::string ROBO_DESC_PARAM_NAME = "robot_description";

class ROS_urdf_Loader{
protected:
  const rclcpp::Node::SharedPtr mNode;
  SScene *mScene;
  std::shared_ptr<XMLDocument> doc;
  bool successfulParsed = false;
  std::string fileFullPath;

public:
  SKArticulation *loadArticulation(physx::PxMaterial *material) {
    if (!successfulParsed) {
      RCLCPP_ERROR(mNode->get_logger(), "Did not load XML successfully %s [%s]", mNode->get_name());
      return nullptr;
    }

    auto loader = mScene->createURDFLoader();
    return loader->loadKinematic(fileFullPath, doc, material);
  }

  ROS_urdf_Loader(
      const std::shared_ptr<rclcpp::Node>& node,
      SScene *scene,
      const std::string& packageName,
      const std::string& filePath): mNode(node), mScene(scene){
    // find paths
    std::string packagePath = ament_index_cpp::get_package_share_directory(packageName);
    fileFullPath = packagePath + "/" + filePath;

    // read file
    std::ifstream file(fileFullPath);
    if(!file) {
      RCLCPP_ERROR(mNode->get_logger(), "Failed finding URDF file %s [%s]", fileFullPath.c_str(), mNode->get_name());
      return;
    }
    RCLCPP_INFO(mNode->get_logger(), "Successfull finding URDF file %s [%s]", fileFullPath.c_str(), mNode->get_name());
    file.seekg(0, std::ios::end);
    size_t fileLen = file.tellg();
    file.seekg(0);
    std::string origFileContent(fileLen+1, '\0');
    file.read(&origFileContent[0], fileLen);
    std::string fileContent = origFileContent;

    // make absolute path
    std::string replace = "package:/";
    size_t pos = fileContent.find(replace);
    while(pos != std::string::npos) {
      fileContent.replace(pos, replace.size(), packagePath);
      pos = fileContent.find(replace, pos+packagePath.size());
    }

    // parse file
    doc = std::make_shared<XMLDocument>();
    if(doc->Parse(fileContent.c_str(), fileContent.length())) {
      RCLCPP_ERROR(mNode->get_logger(), "Failed parsing URDF file [%s]", mNode->get_name());
      return;
    }
    RCLCPP_INFO(mNode->get_logger(), "Successfull parsing URDF file [%s]", mNode->get_name());

    successfulParsed = true;

    // publish file
    node->declare_parameter(ROBO_DESC_PARAM_NAME);
    auto set_param_result = node->set_parameter({
      rclcpp::Parameter(ROBO_DESC_PARAM_NAME, origFileContent)
    });

    if(!set_param_result.successful) {
      RCLCPP_ERROR(mNode->get_logger(), "Node %s failed publishing parameter %s", node->get_name(), ROBO_DESC_PARAM_NAME.c_str());
    }
    RCLCPP_INFO(mNode->get_logger(), "Successfull published %s with file %s [%s]", ROBO_DESC_PARAM_NAME.c_str(), fileFullPath.c_str(), mNode->get_name());
  }
}; // end class ROS_urdf_Loader
} // end namespace sapien::ros2