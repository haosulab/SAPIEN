#pragma once

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <PxMaterial.h>

namespace sapien {
class SArticulation;
class SScene;

namespace URDF {
class URDFLoader;
}

namespace ros2 {
class SceneManager;

class RobotDescriptor {
  friend SceneManager;

protected:
  /* Robot Description String, URDF in SAPIEN convention */
  std::string mURDFString;
  std::string mSRDFString;

  /* tag from urdf loader*/
  bool fixRootLink = true;
  float defaultDensity = 1000.f;
  bool collisionIsVisual = false;

  /* Articulation Reference */
  SArticulation *mArticulation = nullptr;
  bool mBuilt = false;

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

  /* util function for urdf modification */
  static std::string fromSAPIENConvention(const std::string &URDFString);
  static std::string substituteROSPath(const std::string &URDFString);
  static std::string substituteRelativePath(const std::string &URDFString,
                                            const std::string &URDFPath);

public:
  RobotDescriptor(bool isPath, const std::string &URDF, const std::string &SRDF,
                  const std::string &substitutePath = "");

  RobotDescriptor(const std::string &ROSPackageName, const std::string &URDFRelativePath,
                  const std::string &SRDFRelativePath, bool useSharedDirectory);

  SArticulation *build(URDF::URDFLoader &loader, physx::PxMaterial *material = nullptr);

  /* inline function to get string */
  inline std::string getURDF() { return mURDFString; };
  inline std::string getSRDF() { return mSRDFString; };

};
} // namespace ros2
} // namespace sapien
