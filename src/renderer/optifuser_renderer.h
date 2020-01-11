#pragma once
#include "camera.h"
#include "render_interface.h"
#include <camera_spec.h>
#include <memory>
#include <optifuser.h>

namespace sapien {
namespace Renderer {

class OptifuserRenderer;
class OptifuserScene;

class OptifuserRigidbody : public IPxrRigidbody {
  OptifuserScene *mParentScene = nullptr;
  physx::PxTransform mInitialPose = {{0, 0, 0}, physx::PxIdentity};
  std::vector<Optifuser::Object *> mObjects;

public:
  OptifuserRigidbody(OptifuserScene *scene, std::vector<Optifuser::Object *> const &objects);
  OptifuserRigidbody(OptifuserRigidbody const &other) = delete;

  OptifuserRigidbody &operator=(OptifuserRigidbody const &other) = delete;

  void setUniqueId(uint32_t uniqueId) override;
  void setSegmentationId(uint32_t segmentationId) override;
  void setSegmentationCustomData(const std::vector<float> &customData) override;
  void setInitialPose(const physx::PxTransform &transform) override;
  void update(const physx::PxTransform &transform) override;

  void destroy() override;
};

class OptifuserScene : public IPxrScene {
  OptifuserRenderer *mParentRenderer;
  std::unique_ptr<Optifuser::Scene> mScene;
  std::vector<std::unique_ptr<OptifuserRigidbody>> mBodies;
  std::vector<std::unique_ptr<OptifuserCamera>> mCameras;
  std::string mName;

public:
  OptifuserScene(OptifuserRenderer *renderer, std::string const &name = "");

  inline std::string getName() { return mName; }

  Optifuser::Scene *getScene();

  // IPxrScene
  virtual IPxrRigidbody *addRigidbody(const std::string &meshFile,
                                      const physx::PxVec3 &scale) override;
  virtual IPxrRigidbody *addRigidbody(physx::PxGeometryType::Enum type, const physx::PxVec3 &scale,
                                      const physx::PxVec3 &color) override;
  virtual void removeRigidbody(IPxrRigidbody *body) override;

  virtual ICamera *addCamera(std::string const &name, uint32_t width, uint32_t height, float fovx,
                             float fovy, float near, float far,
                             std::string const &shaderDir = "glsl_shader/130") override;
  virtual void removeCamera(ICamera *camera) override;
  virtual std::vector<ICamera *> getCameras() override;

  void destroy() override;
  //======== Lights ========//

  /* For OpenGL only */
  void setAmbientLight(std::array<float, 3> color);

  /* For OpenGL only */
  void setShadowLight(std::array<float, 3> direction, std::array<float, 3> color);

  /* For both OpenGL and OptiX */
  void addPointLight(std::array<float, 3> position, std::array<float, 3> color);

  /* For OptiX Only */
  void addDirectionalLight(std::array<float, 3> direction, std::array<float, 3> color);
};

class OptifuserRenderer : public IPxrRenderer {

  std::vector<std::unique_ptr<OptifuserScene>> mScenes;

public:
  const std::string mGlslDir;

  Optifuser::GLFWRenderContext *mContext = nullptr;
  OptifuserRenderer(std::string const &glslDir = "glsl_shader/130",
                    std::string const &glslVersion = "130");

  // IPxrRenderer
  IPxrScene *createScene(std::string const &name) override;
  void removeScene(IPxrScene *scene) override;

public:
  // void showWindow();
  // void hideWindow();
  // void render();
};

// class OptifuserRenderer : public IPhysxRenderer {
//   const std::string glslVersion = "130";

//   struct SceneData {
//     std::map<uint32_t, std::vector<Optifuser::Object *>> renderId2Objects = {};
//     std::map<uint32_t, std::vector<uint32_t>> segId2RenderId = {};
//     std::unique_ptr<Optifuser::Scene> scene = nullptr;
//     std::map<uint32_t, std::unique_ptr<MountedCamera>> mountedCameras;
//   };

//   uint32_t mSelectedSceneId = 0;

//   int pickedId = 0, pickedRenderId = 0;

// public:
//   OptifuserRenderer(const std::string &version = "130");

//   // std::map<uint32_t, std::vector<Optifuser::Object *>> mObjectRegistry;
//   // std::map<uint32_t, std::vector<uint32_t>> mSegId2RenderId;

//   // std::shared_ptr<Optifuser::Scene> mScene;
//   // std::vector<std::unique_ptr<Optifuser::Scene>> mScenes;
//   std::vector<SceneData> mSceneData;

//   Optifuser::GLFWRenderContext *mContext = nullptr;
//   Optifuser::FPSCameraSpec cam;

//   std::function<GuiInfo(uint32_t)> queryCallback = {};
//   std::function<void(uint32_t, const GuiInfo &info)> syncCallback = {};

//   std::function<void(uint32_t index, uint32_t action)> saveActionCallback = {};
//   std::function<void(uint32_t index, std::string const &name)> saveCallback = {};

//   // IPhysxRenderer
//   virtual void resetScene(uint32_t sceneId) override;
//   virtual void addRigidbody(uint32_t sceneId, uint32_t uniqueId, const std::string &meshFile,
//                             const physx::PxVec3 &scale) override;
//   virtual void addRigidbody(uint32_t sceneId, uint32_t uniqueId, physx::PxGeometryType::Enum
//   type,
//                             const physx::PxVec3 &scale, const physx::PxVec3 &color) override;
//   virtual void setSegmentationId(uint32_t sceneId, uint32_t uniqueId,
//                                  uint32_t segmentationId) override;
//   virtual void setSegmentationCustomData(uint32_t sceneId, uint32_t segmentationId,
//                                          std::vector<float> const &customData) override;
//   virtual void removeRigidbody(uint32_t sceneId, uint32_t uniqueId) override;
//   virtual void updateRigidbody(uint32_t sceneId, uint32_t uniqueId,
//                                const physx::PxTransform &transform) override;

//   virtual void bindQueryCallback(std::function<GuiInfo(uint32_t)>) override;
//   virtual void bindSyncCallback(std::function<void(uint32_t, const GuiInfo &info)>) override;

//   /* For OpenGL only */
//   void setAmbientLight(uint32_t sceneId, std::array<float, 3> color);

//   /* For OpenGL only */
//   void setShadowLight(uint32_t sceneId, std::array<float, 3> direction,
//                       std::array<float, 3> color);

//   /* For both OpenGL and OptiX */
//   void addPointLight(uint32_t sceneId, std::array<float, 3> position, std::array<float, 3>
//   color);

//   /* For OptiX Only */
//   void addDirectionalLight(uint32_t sceneId, std::array<float, 3> position,
//                            std::array<float, 3> color);

// private:
//   void init();
//   void destroy();

//   Optifuser::Scene *getScene(uint32_t sceneId);

// public:
//   void render();

// public:
//   // ICameraManager
//   virtual std::vector<ICamera *> getCameras(uint32_t sceneId) override;
//   virtual void addCamera(uint32_t sceneId, uint32_t uniqueId, std::string const &name,
//                          uint32_t width, uint32_t height, float fovx, float fovy, float near,
//                          float far) override;
//   virtual void updateCamera(uint32_t sceneId, uint32_t uniqueId,
//                             physx::PxTransform const &transform) override;

//   void selectScene(uint32_t sceneId);
//   void showWindow();
//   void hideWindow();

//   // save and load
//   virtual void setSaveNames(std::vector<std::string> const &names) override;
//   virtual void
//   bindSaveActionCallback(std::function<void(uint32_t index, uint32_t action)>) override;
//   virtual void
//   bindSaveCallback(std::function<void(uint32_t index, std::string const &name)>) override;
// };

} // namespace Renderer
} // namespace sapien
