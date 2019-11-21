#pragma once
#include "camera.h"
#include "render_interface.h"
#include <camera_spec.h>
#include <memory>
#include <optifuser.h>

namespace sapien {
namespace Renderer {

class OptifuserRenderer : public IPhysxRenderer {
public:
  OptifuserRenderer();

  std::map<uint32_t, std::vector<Optifuser::Object *>> mObjectRegistry;
  std::map<uint32_t, std::vector<uint32_t>> mSegId2RenderId;
  std::shared_ptr<Optifuser::Scene> mScene;
  Optifuser::GLFWRenderContext *mContext = nullptr;
  Optifuser::FPSCameraSpec cam;

  std::function<GuiInfo(uint32_t)> queryCallback = {};
  std::function<void(uint32_t, const GuiInfo &info)> syncCallback = {};

  std::function<void(uint32_t index, uint32_t action)> saveActionCallback = {};
  std::function<void(uint32_t index, std::string const & name)> saveCallback = {};

  // IPhysxRenderer
  virtual void addRigidbody(uint32_t uniqueId, const std::string &meshFile,
                            const physx::PxVec3 &scale) override;
  virtual void addRigidbody(uint32_t uniqueId, physx::PxGeometryType::Enum type,
                            const physx::PxVec3 &scale, const physx::PxVec3 &color) override;
  virtual void setSegmentationId(uint32_t uniqueId, uint32_t segmentationId) override;
  virtual void setSegmentationCustomData(uint32_t segmentationId,
                                         std::vector<float> const &customData) override;
  virtual void removeRigidbody(uint32_t uniqueId) override;
  virtual void updateRigidbody(uint32_t uniqueId, const physx::PxTransform &transform) override;

  virtual void bindQueryCallback(std::function<GuiInfo(uint32_t)>) override;
  virtual void bindSyncCallback(std::function<void(uint32_t, const GuiInfo &info)>) override;

  /* For OpenGL only */
  void setAmbientLight(std::array<float, 3> color);

  /* For OpenGL only */
  void setShadowLight(std::array<float, 3> direction, std::array<float, 3> color);

  /* For both OpenGL and OptiX */
  void addPointLight(std::array<float, 3> position, std::array<float, 3> color);

  /* For OptiX Only */
  void addDirectionalLight(std::array<float, 3> position, std::array<float, 3> color);

private:
  void init();
  void destroy();

public:
  void render();

private:
  std::map<uint32_t, std::unique_ptr<MountedCamera>> mMountedCameras;

public:
  // ICameraManager
  virtual std::vector<ICamera *> getCameras() override;
  virtual void addCamera(uint32_t uniqueId, std::string const &name, uint32_t width,
                         uint32_t height, float fovx, float fovy, float near, float far) override;
  virtual void updateCamera(uint32_t uniqueId, physx::PxTransform const &transform) override;

  void showWindow();
  void hideWindow();

  // save and load
  virtual void setSaveNames(std::vector<std::string> const &names) override;
  virtual void
  bindSaveActionCallback(std::function<void(uint32_t index, uint32_t action)>) override;
  virtual void
  bindSaveCallback(std::function<void(uint32_t index, std::string const &name)>) override;
};
} // namespace Renderer
} // namespace sapien
