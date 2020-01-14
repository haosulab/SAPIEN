#pragma once
#include <PxPhysicsAPI.h>
#include <array>
#include <foundation/PxTransform.h>
#include <functional>
#include <string>
#include <vector>

namespace sapien {
namespace Renderer {

class ISensor;
class ICamera;
class IPxrScene;
class IPxrRigidbody;
class IPxrRenderer;

class ISensor {
public:
  virtual void setInitialPose(physx::PxTransform const &pose) = 0;
  virtual physx::PxTransform getPose() const = 0;
  virtual void setPose(physx::PxTransform const &pose) = 0;
  virtual IPxrScene *getScene() = 0;
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

class IPxrRigidbody {
public:
  virtual void setUniqueId(uint32_t uniqueId) = 0;
  virtual uint32_t getUniqueId() const = 0;
  virtual void setSegmentationId(uint32_t segmentationId) = 0;
  virtual uint32_t getSegmentationId() const = 0;
  virtual void setSegmentationCustomData(std::vector<float> const &customData) = 0;
  virtual void setInitialPose(const physx::PxTransform &transform) = 0;
  virtual void update(const physx::PxTransform &transform) = 0;

  virtual void destroy() = 0;
};

class IPxrScene {

public:
  virtual IPxrRigidbody *addRigidbody(const std::string &meshFile, const physx::PxVec3 &scale) = 0;

  virtual IPxrRigidbody *addRigidbody(physx::PxGeometryType::Enum type, const physx::PxVec3 &scale,
                                      const physx::PxVec3 &color) = 0;

  virtual void removeRigidbody(IPxrRigidbody *body) {}

  virtual ICamera *addCamera(std::string const &name, uint32_t width, uint32_t height, float fovx,
                             float fovy, float near, float far,
                             std::string const &shaderDir = "glsl_shader/130") = 0;
  virtual void removeCamera(ICamera *camera) = 0;

  virtual std::vector<ICamera *> getCameras() = 0;

  virtual void destroy() = 0;
};

class IPxrRenderer {
public:
  virtual IPxrScene *createScene(std::string const &name = "") = 0;
  virtual void removeScene(IPxrScene *scene) = 0;
};

// class IPhysxRenderer : public ICameraManager {
// public:
// public:
//   /* call this function to initialize a scene */
//   virtual void resetScene(uint32_t sceneId) = 0;

//   /* This function is called when a rigid body is added to a scene */
//   virtual void addRigidbody(uint32_t sceneId, uint32_t uniqueId, const std::string &meshFile,
//                             const physx::PxVec3 &scale) = 0;
//   virtual void addRigidbody(uint32_t sceneId, uint32_t uniqueId, physx::PxGeometryType::Enum
//   type,
//                             const physx::PxVec3 &scale, const physx::PxVec3 &color) = 0;
//   virtual void setSegmentationId(uint32_t sceneId, uint32_t uniqueId, uint32_t segmentationId)
//   = 0; virtual void setSegmentationCustomData(uint32_t sceneId, uint32_t segmentationId,
//                                          std::vector<float> const &customData) = 0;

//   /* This function is called when a rigid body is removed from a scene */
//   virtual void removeRigidbody(uint32_t sceneId, uint32_t uniqueId) = 0;

//   /* This function is called when a rigid body is updated */
//   virtual void updateRigidbody(uint32_t sceneId, uint32_t uniqueId,
//                                const physx::PxTransform &transform) = 0;

//   virtual void bindQueryCallback(std::function<GuiInfo(uint32_t)>) = 0;
//   virtual void bindSyncCallback(std::function<void(uint32_t, const GuiInfo &info)>) = 0;

//   /* Save and load */
//   virtual void setSaveNames(std::vector<std::string> const &names) = 0;
//   virtual void bindSaveActionCallback(std::function<void(uint32_t index, uint32_t action)>) =
//   0; virtual void bindSaveCallback(std::function<void(uint32_t index, std::string const
//   &name)>) = 0;
// };

} // namespace Renderer
} // namespace sapien
