#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <stdlib.h>
#include <tinyxml2.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace sapien::ros2 {
using namespace tinyxml2;

static const std::string ROBO_DESC_PARAM_NAME = "robot_description";

class ROS_urdf_Loaer{
protected:
  rclcpp::Node::SharedPtr mNode;

public:
  ROS_urdf_Loaer(std::shared_ptr<rclcpp::Node> node, const char *fileName): mNode(node) {
    // read file
    std::ifstream file(fileName);
    if(!file) {
      RCLCPP_ERROR(mNode->get_logger(), "Failed loading URDF file %s [%s]", fileName, mNode->get_name());
      return;
    }
    RCLCPP_INFO(mNode->get_logger(), "Successful loading file %s [%s]", fileName, mNode->get_name());
    file.seekg(0, std::ios::end);
    size_t fileLen = file.tellg();
    file.seekg(0);
    std::string origFileContent(fileLen+1, '\0');
    file.read(&origFileContent[0], fileLen);
    std::string fileContent = origFileContent;


    // make absolute path
    std::string packagePath = ament_index_cpp::get_package_share_directory("sapien_resources");

    std::string replace = "package:/";
    size_t pos = fileContent.find(replace);

    while(pos != std::string::npos) {
      fileContent.replace(pos, replace.size(), packagePath);
      pos = fileContent.find(replace, pos+packagePath.size());
    }

    std::cout << fileContent << std::endl;

    // parse file
    XMLDocument doc;
    doc.Parse(fileContent.c_str(), fileContent.length());


    // publish file
    node->declare_parameter(ROBO_DESC_PARAM_NAME);
    auto set_param_result = node->set_parameter({
      rclcpp::Parameter(ROBO_DESC_PARAM_NAME, origFileContent)
    });

    if(!set_param_result.successful) {
      RCLCPP_ERROR(mNode->get_logger(), "Node %s failed publishing parameter %s", node->get_name(), ROBO_DESC_PARAM_NAME.c_str());
    }

    // process and call sapien urder loader
  }
};
}