#pragma once
#include "camera.h"
#include "render_interface.h"
#include <camera_spec.h>
#include <memory>
#include <optifuser.h>
#include "config.h"

namespace sapien {
namespace Renderer {

class OptifuserRenderer;
class OptifuserScene;


class OptifuserRigidbody : public IPxrRigidbody {
  OptifuserScene *mParentScene = nullptr;
  physx::PxTransform mInitialPose = {{0, 0, 0}, physx::PxIdentity};
  std::vector<Optifuser::Object *> mObjects;

  uint32_t mUniqueId = 0;
  uint32_t mSegmentationId = 0;

public:
  OptifuserRigidbody(OptifuserScene *scene, std::vector<Optifuser::Object *> const &objects);
  OptifuserRigidbody(OptifuserRigidbody const &other) = delete;

  OptifuserRigidbody &operator=(OptifuserRigidbody const &other) = delete;

  void setUniqueId(uint32_t uniqueId) override;
  uint32_t getUniqueId() const override;
  void setSegmentationId(uint32_t segmentationId) override;
  uint32_t getSegmentationId() const override;
  void setSegmentationCustomData(const std::vector<float> &customData) override;
  void setInitialPose(const physx::PxTransform &transform) override;
  void update(const physx::PxTransform &transform) override;

  void setVisible(bool visible) override;
  void setRenderMode(uint32_t mode) override;

  void destroy() override;

  void destroyVisualObjects();
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
                                      const PxrMaterial &material) override;

  virtual IPxrRigidbody *addRigidbody(std::vector<physx::PxVec3> const &vertices,
                                      std::vector<physx::PxVec3> const &normals,
                                      std::vector<uint32_t> const &indices,
                                      const physx::PxVec3 &scale,
                                      const PxrMaterial &material) override;

  virtual void removeRigidbody(IPxrRigidbody *body) override;

  virtual ICamera *addCamera(std::string const &name, uint32_t width, uint32_t height, float fovx,
                             float fovy, float near, float far,
                             std::string const &shaderDir = "") override;
  virtual void removeCamera(ICamera *camera) override;
  virtual std::vector<ICamera *> getCameras() override;

  void destroy() override;
  //======== Lights ========//

  /* For OpenGL only */
  void setAmbientLight(std::array<float, 3> const &color) override;

  /* For OpenGL only */
  void setShadowLight(std::array<float, 3> const &direction,
                      std::array<float, 3> const &color) override;

  /* For both OpenGL and OptiX */
  void addPointLight(std::array<float, 3> const &position,
                     std::array<float, 3> const &color) override;

  /* For OptiX Only */
  void addDirectionalLight(std::array<float, 3> const &direction,
                           std::array<float, 3> const &color) override;
};

class OptifuserRenderer : public IPxrRenderer {

  static std::string gDefaultGlslDir;
  static std::string gDefaultGlslVersion;

  std::vector<std::unique_ptr<OptifuserScene>> mScenes;

public:
  std::string mGlslDir;
  OptifuserConfig mConfig;

  Optifuser::GLFWRenderContext *mContext = nullptr;
  OptifuserRenderer(std::string const &glslDir = "", std::string const &glslVersion = "",
                    OptifuserConfig const &config = {});

  // IPxrRenderer
  IPxrScene *createScene(std::string const &name) override;
  void removeScene(IPxrScene *scene) override;

  static void setDefaultShaderConfig(std::string const &glslDir, std::string const &glslVersion);

#ifdef _USE_OPTIX
  static std::string gPtxDir;
  static void setOptixConfig(std::string const &ptxDir);
#endif

  void enableGlobalAxes(bool enable = true);
};

} // namespace Renderer
} // namespace sapien
