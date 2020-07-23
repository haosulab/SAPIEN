#pragma once

#include "articulation/sapien_articulation.h"
#include "articulation/urdf_loader.h"
#include <ros/ros.h>

#include <tuple>

namespace sapien {
class SArticulation;

namespace ros1 {
class SceneManager;
class RobotManager;
class RobotDescriptor;

class RobotLoader {
protected:
  /* Global Definition */
  const std::string ROBOT_PARAM_NAME = "/robot_description";
  const std::string SEMANTIC_PARAM_NAME = "/robot_description_semantic";

  // Basic handle
  ros::NodeHandlePtr mNode;
  SceneManager *mManager;

  // URDF Loader variable
  std::unique_ptr<URDF::URDFLoader> mLoader = nullptr;
  bool &fixRootLink;
  bool &collisionIsVisual;

public:
protected:
  //  void publishRobotDescription(const std::string &URDFString, const std::string &SRDFString);

public:
  explicit RobotLoader(SceneManager *manager);

  std::tuple<SArticulation *, RobotManager *>
  loadFromParameterServer(const std::string &robotName, const URDF::URDFConfig &config,
                          double frequency, const std::string &URDFParamName = "",
                          const std::string &SRDFName = "");

  /* Setter and getter */
  inline void setFixRootLink(bool value) { fixRootLink = value; }
  inline bool getFixRootLink() { return fixRootLink; }

  inline void setCollisionIsVisual(bool value) { collisionIsVisual = value; }
  inline bool getCollisionIsVisual() { return collisionIsVisual; }

}; // end class ROS_urdf_Loader
} // namespace ros1
} // namespace sapien
