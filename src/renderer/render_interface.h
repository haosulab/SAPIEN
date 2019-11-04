#pragma once
#include <PxPhysicsAPI.h>
#include <array>
#include <foundation/PxTransform.h>
#include <functional>
#include <string>
#include <vector>

namespace sapien {
namespace Renderer {

struct JointGuiInfo {
  std::string name;
  std::array<float, 2> limits;
  float value;
};

struct ArticulationGuiInfo {
  std::string name = "";
  std::vector<JointGuiInfo> jointInfo;
};

struct LinkGuiInfo {
  std::string name = "";
  physx::PxTransform transform;
};

struct GuiInfo {
  ArticulationGuiInfo articulationInfo;
  LinkGuiInfo linkInfo;
};

struct SensorPose {
  std::array<float, 3> positionXYZ;
  std::array<float, 4> rotationWXYZ;
};

class ISensor {
public:
  virtual SensorPose getSensorPose() const = 0;
  virtual void setSensorPose(const SensorPose &pose) = 0;
  virtual ~ISensor() {}
};

class ICamera : public ISensor {
public:
  virtual const std::string &getName() const = 0;
  virtual uint32_t getWidth() const = 0;
  virtual uint32_t getHeight() const = 0;
  virtual float getFovy() const = 0;

  virtual void takePicture() = 0;
  virtual std::vector<float> getColorRGBA() = 0;
  virtual std::vector<float> getAlbedoRGBA() = 0;
  virtual std::vector<float> getNormalRGBA() = 0;
  virtual std::vector<float> getDepth() = 0;
  virtual std::vector<int> getSegmentation() = 0;
  virtual std::vector<int> getObjSegmentation() = 0;
};

class ICameraManager {
public:
  virtual std::vector<ICamera *> getCameras() = 0;
  virtual void addCamera(uint32_t uniqueId, std::string const &name, uint32_t width,
                         uint32_t height, float fovx, float fovy, float near, float far) = 0;
  virtual void updateCamera(uint32_t uniqueId, physx::PxTransform const &transform) = 0;
  virtual ~ICameraManager() {}
};

class IPhysxRenderer : public ICameraManager {
public:
  /* This function is called when a rigid body is added to a scene */
  virtual void addRigidbody(uint32_t uniqueId, const std::string &meshFile,
                            const physx::PxVec3 &scale) = 0;
  virtual void addRigidbody(uint32_t uniqueId, physx::PxGeometryType::Enum type,
                            const physx::PxVec3 &scale, const physx::PxVec3 &color) = 0;
  virtual void setSegmentationId(uint32_t uniqueId, uint32_t segmentationId) = 0;
  virtual void setSegmentationCustomData(uint32_t segmentationId,
                                         std::vector<float> const &customData) = 0;

  /* This function is called when a rigid body is removed from a scene */
  virtual void removeRigidbody(uint32_t uniqueId) = 0;

  /* This function is called when a rigid body is updated */
  virtual void updateRigidbody(uint32_t uniqueId, const physx::PxTransform &transform) = 0;

  virtual void bindQueryCallback(std::function<GuiInfo(uint32_t)>) = 0;
  virtual void bindSyncCallback(std::function<void(uint32_t, const GuiInfo &info)>) = 0;
};

} // namespace Renderer

} // namespace sapien
