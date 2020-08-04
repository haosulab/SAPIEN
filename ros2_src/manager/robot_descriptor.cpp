#include <experimental/filesystem>
#include <fstream>
#include <tinyxml2.h>

#include "articulation/sapien_articulation.h"
#include "articulation/urdf_loader.h"
#include "robot_descriptor.h"
#include "sapien_scene.h"

namespace fs = std::experimental::filesystem;

namespace sapien::ros2 {

RobotDescriptor::RobotDescriptor(const std::string &URDF, const std::string &SRDF,
                                 const std::string &substitutePath) {
  auto logger = spdlog::get("SAPIEN_ROS2");
  std::string urdf(URDF), srdf(SRDF);
  if (urdf.empty()) {
    logger->error("URDF String is empty, urdf parsing fail");
    throw std::runtime_error("URDF String Empty");
  }
  logger->info("Get robot descriptor URDF string");
  urdf = substituteROSPath(urdf);
  urdf = substitutePath.empty() ? urdf : substituteRelativePath(urdf, substitutePath);
  mURDFString = urdf;
  mSRDFString = srdf;
}

std::unique_ptr<RobotDescriptor> RobotDescriptor::fromPath(const std::string &URDF,
                                                           const std::string &SRDF) {
  auto logger = spdlog::get("SAPIEN_ROS2");
  std::string urdf, srdf(SRDF);
  std::string srdfPath = URDF;
  srdfPath = SRDF.empty() ? srdfPath.replace(srdfPath.end() - 4, srdfPath.end(), "srdf") : SRDF;
  srdf = fs::is_regular_file(srdfPath) ? readFile(srdfPath) : "";

  if (!fs::is_regular_file(URDF)) {
    logger->error("Path {} do not exist, urdf parsing fail", URDF);
    throw std::runtime_error("URDF Path Not Exist");
  }
  if (URDF.compare(URDF.size() - 4, 4, "urdf")) {
    logger->error("Path {} is not a valid URDF file", URDF);
    throw std::runtime_error("URDF Path Not Valid");
  }
  logger->info("Get robot descriptor from local path");
  urdf = readFile(URDF);
  urdf = substituteROSPath(urdf);

  return std::make_unique<RobotDescriptor>(urdf, srdf, URDF);
};

std::unique_ptr<RobotDescriptor> RobotDescriptor::fromROS(const std::string &ROSPackageName,
                                 const std::string &URDFRelativePath,
                                 const std::string &SRDFRelativePath) {
  auto paths = getFilePath(ROSPackageName, URDFRelativePath, SRDFRelativePath);
  auto logger = spdlog::get("SAPIEN_ROS2");
  logger->info("Get robot descriptor from ROS package");
  return fromPath(paths[0], paths[1]);
}

std::array<std::string, 2> RobotDescriptor::getFilePath(const std::string &packageName,
                                                        const std::string &URDFRelativePath,
                                                        const std::string &SRDFRelativePath) {
  auto packagePath = ament_index_cpp::get_package_share_directory(packageName);
  auto URDFPath = packagePath + "/" + URDFRelativePath;
  if (!std::experimental::filesystem::exists(URDFPath)) {
    throw std::runtime_error("URDF path does not exist!");
  }

  //  If no srdf relative path is given, assume it locate near the urdf file
  std::string SRDFPath(URDFPath);
  SRDFPath = SRDFRelativePath.empty()
                 ? SRDFPath.replace(SRDFPath.end() - 4, SRDFPath.end(), "srdf")
                 : packagePath + "/" + SRDFRelativePath;
  SRDFPath = std::experimental::filesystem::exists(SRDFPath) ? SRDFPath : "";

  return {URDFPath, SRDFPath};
}

std::string RobotDescriptor::fromSAPIENConvention(const std::string &URDFString) {
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

std::string RobotDescriptor::substituteROSPath(const std::string &URDFString) {
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
  return modifiedURDFString;
}
std::string RobotDescriptor::readFile(const std::string &path) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("Can not read file from path: " + path);
  }

  std::string fileString;
  std::ifstream t(path);
  t.seekg(0, std::ios::end);
  fileString.reserve(t.tellg());
  t.seekg(0, std::ios::beg);
  fileString.assign((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());

  return fileString;
}

std::string RobotDescriptor::substituteRelativePath(const std::string &URDFString,
                                                    const std::string &URDFPath) {
  std::string modifiedURDFString(URDFString);
  const std::string searchString = "filename=\"";
  auto pos = modifiedURDFString.find(searchString);
  auto rootPath = fs::canonical(fs::path(URDFPath)).remove_filename().string();

  while (pos != std::string::npos) {
    if (modifiedURDFString[pos + searchString.size()] != '/') {
      modifiedURDFString.insert(pos + searchString.size(), rootPath);
    }
    pos = modifiedURDFString.find(searchString, pos + searchString.size());
  }

  return modifiedURDFString;
}

} // namespace sapien::ros2

// sapien::SArticulation* sapien::URDF::URDFLoader::loadFromRobotDescription(
//    ros2::RobotDescriptor *descriptor, physx::PxMaterial *material) {
//  loadFromXML(descriptor->getURDF(), descriptor->getSRDF(), material, false);
//}
