#pragma once

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "articulation/sapien_articulation.h"
#include "articulation/urdf_loader.h"
#include "rclcpp/rclcpp.hpp"

namespace sapien::ros2 {

class SceneManager;
class RobotManager;

class RobotLoader {
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
  /* Variables from URDF Loader */
  bool &fixRootLink;
  float &defaultDensity;
  bool &collisionIsVisual;

protected:
  /* util function for path and string operation */
  static std::array<std::string, 2> getFilePath(const std::string &packageName,
                                                const std::string &robotRelativePath,
                                                const std::string &semanticRelativePath = "");
  static std::string modifyURDFPath2ROSConvention(const std::string &URDFString);
  static std::string modifyURDFPath2SapienConvention(const std::string &URDFString,
                                                     const std::string &URDFPath = "");
  std::array<std::string, 2> parseURDFFile(const std::string &URDFPath);
  static std::string getRobotNameFromPath(const std::string &URDFPath);

  std::tuple<SArticulation *, RobotManager *> loadRobot(const std::string &name,
                                                        const std::string &URDFPath,
                                                        const std::string &SRDFPath,
                                                        physx::PxMaterial *material);

  void publishRobotDescription(rclcpp::Node::SharedPtr &node, const std::string &URDFString,
                               const std::string &SRDFString);

public:
  explicit RobotLoader(SceneManager *manager);
  /* Load robot using ROS convention by ament package list */
  std::tuple<SArticulation *, RobotManager *> loadROS(const std::string &ROSPackageName,
                                                      const std::string &robotRelativePath,
                                                      const std::string &semanticRelativePath = "",
                                                      const std::string &name = "",
                                                      physx::PxMaterial *material = nullptr);

  /* Load pure robot urdf, does not rely on ROS package indexing */
  std::tuple<SArticulation *, RobotManager *> load(const std::string &robotPath,
                                                   const std::string &semanticPath = "",
                                                   const std::string &name = "",
                                                   physx::PxMaterial *material = nullptr);

  /* Load pure string, path can be ROS convention or not */
  std::tuple<SArticulation *, RobotManager *>
  loadFromString(const std::string &URDFString, const std::string &SRDFString,
                 const std::string &name, physx::PxMaterial *material = nullptr);

  /* Setter and getter */
  inline void setFixRootLink(bool value) { fixRootLink = value; }
  inline bool getFixRootLink() { return fixRootLink; }

  inline void setDefaultDensity(float value) { defaultDensity = value; }
  inline bool getDefaultDensity() { return defaultDensity; }

  inline void setCollisionIsVisual(bool value) { collisionIsVisual = value; }
  inline bool getCollisionIsVisual() { return collisionIsVisual; }

}; // end class ROS_urdf_Loader
// namespace ros2
} // namespace sapien::ros2