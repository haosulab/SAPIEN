#pragma once

#include "articulation/sapien_articulation.h"
#include "articulation/urdf_loader.h"
#include "rclcpp/rclcpp.hpp"

#include <tuple>

namespace sapien {
class SArticulation;

namespace ros2 {
class SceneManager;
class RobotManager;
class RobotDescriptor;

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
  bool &fixRootLink;
  float &defaultDensity;
  bool &collisionIsVisual;

public:
protected:
  void publishRobotDescription(rclcpp::Node::SharedPtr &node, const std::string &URDFString,
                               const std::string &SRDFString);

public:
  explicit RobotLoader(SceneManager *manager);

  std::tuple<SArticulation *, RobotManager *>
  loadRobotAndManager(RobotDescriptor &descriptor, const std::string &name,
                      physx::PxMaterial *material = nullptr);

  /* Setter and getter */
  inline void setFixRootLink(bool value) { fixRootLink = value; }
  inline bool getFixRootLink() { return fixRootLink; }

  inline void setDefaultDensity(float value) { defaultDensity = value; }
  inline float getDefaultDensity() { return defaultDensity; }

  inline void setCollisionIsVisual(bool value) { collisionIsVisual = value; }
  inline bool getCollisionIsVisual() { return collisionIsVisual; }

}; // end class ROS_urdf_Loader
// namespace ros2
} // namespace ros2
} // namespace sapien
