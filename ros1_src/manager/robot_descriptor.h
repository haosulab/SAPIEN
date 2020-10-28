#pragma once

#include <PxMaterial.h>
#include <moveit/robot_model/robot_model.h>
#include <ros/package.h>
#include <string>

namespace sapien {
class SArticulation;
class SScene;

namespace URDF {
class URDFLoader;
}

namespace ros1 {
// class SceneManager;
// class RobotLoader;

class RobotDescriptor {
  //  friend SceneManager;
  //  friend RobotLoader;

protected:
  /* Robot Description String, URDF in SAPIEN convention */
  std::string mURDFString;
  std::string mSRDFString;

  /* tag from urdf loader*/
  bool fixRootLink = true;
  float defaultDensity = 1000.f;
  bool collisionIsVisual = false;

  /* ROS Param Setting */
  std::string ROBOT_PARAM_NAME = "robot_description";
  std::string SEMANTIC_PARAM_NAME = "robot_description_semantic";

public:
protected:
  /* util function for reading string from path */
  static std::string readFile(const std::string &path);

  /* util function for path operation */
  static std::array<std::string, 2> getFilePath(const std::string &packageName,
                                                const std::string &URDFRelativePath,
                                                const std::string &SRDFRelativePath = "");

public:
  RobotDescriptor(const std::string &URDF, const std::string &SRDF = "",
                  const std::string &substitutePath = "");

  static RobotDescriptor fromROSPackage(const std::string &ROSPackageName,
                                        const std::string &URDFRelativePath,
                                        const std::string &SRDFRelativePath);
  static RobotDescriptor fromPath(const std::string &URDF, const std::string &SRDF = "",
                                  const std::string &substitutePath = "");
  static RobotDescriptor
  fromParameterServer(ros::NodeHandlePtr node,
                     const std::string &URDFParamName = "robot_description",
                     const std::string &SRDFParamName = "robot_description_semantic");

  /* util function for urdf modification */
  static std::string fromSAPIENConvention(const std::string &URDFString);
  static std::string substituteROSPath(const std::string &URDFString);
  static std::string substituteRelativePath(const std::string &URDFString,
                                            const std::string &URDFPath);
  /* inline function to get string */
  inline std::string getURDF() { return mURDFString; };
  inline std::string getStandardURDF() { return fromSAPIENConvention(mURDFString); }
  inline std::string getSRDF() { return mSRDFString; };
};
} // namespace ros1
} // namespace sapien
