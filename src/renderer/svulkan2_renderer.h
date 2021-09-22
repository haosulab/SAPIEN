#pragma once
#include "renderer/render_interface.h"
#include "svulkan2_light.h"
#include "svulkan2_material.h"
#include <memory>
#include <svulkan2/core/context.h>
#include <svulkan2/renderer/renderer.h>
#include <svulkan2/scene/scene.h>

namespace sapien {
namespace Renderer {
class SVulkan2Renderer;
class SVulkan2Rigidbody;
class SVulkan2Camera;
class SVulkan2Scene;

extern std::string gDefaultViewerShaderDirectory;
void setDefaultViewerShaderDirectory(std::string const &dir);

extern std::string gDefaultCameraShaderDirectory;
void setDefaultCameraShaderDirectory(std::string const &dir);

class SVulkan2Rigidbody : public IPxrRigidbody {
  std::string mName{};
  SVulkan2Scene *mParentScene = nullptr;
  physx::PxTransform mInitialPose = {{0, 0, 0}, physx::PxIdentity};
  std::vector<svulkan2::scene::Object *> mObjects;

  uint32_t mUniqueId{0};
  uint32_t mSegmentationId{0};

  physx::PxGeometryType::Enum mType;
  physx::PxVec3 mScale;

public:
  SVulkan2Rigidbody(SVulkan2Scene *scene, std::vector<svulkan2::scene::Object *> const &objects,
                    physx::PxGeometryType::Enum type, physx::PxVec3 scale);
  SVulkan2Rigidbody(SVulkan2Rigidbody const &other) = delete;

  SVulkan2Rigidbody &operator=(SVulkan2Rigidbody const &other) = delete;

  inline void setName(std::string const &name) override { mName = name; };
  std::string getName() const override { return mName; };

  void setUniqueId(uint32_t uniqueId) override;
  uint32_t getUniqueId() const override;
  void setSegmentationId(uint32_t segmentationId) override;
  uint32_t getSegmentationId() const override;
  void setSegmentationCustomData(const std::vector<float> &customData) override;
  void setInitialPose(const physx::PxTransform &transform) override;
  inline physx::PxTransform getInitialPose() const override { return mInitialPose; };
  void update(const physx::PxTransform &transform) override;

  void setVisibility(float visibility) override;
  void setVisible(bool visible) override;
  void setRenderMode(uint32_t mode) override;

  void destroy() override;

  physx::PxGeometryType::Enum getType() const override { return mType; }
  physx::PxVec3 getScale() const override;

  /** internal use only */
  void destroyVisualObjects();
  /** internal use only */
  inline std::vector<svulkan2::scene::Object *> const &getVisualObjects() const {
    return mObjects;
  }

  std::vector<std::shared_ptr<IPxrRenderShape>> getRenderShapes() const override;
};

class SVulkan2Scene : public IPxrScene {
  SVulkan2Renderer *mParentRenderer;
  std::unique_ptr<svulkan2::scene::Scene> mScene;
  std::vector<std::unique_ptr<SVulkan2Rigidbody>> mBodies;
  std::vector<std::unique_ptr<SVulkan2Camera>> mCameras;
  std::vector<std::unique_ptr<ILight>> mLights;
  std::string mName;

  std::shared_ptr<svulkan2::resource::SVMesh> mCubeMesh{};
  std::shared_ptr<svulkan2::resource::SVMesh> mSphereMesh{};
  std::shared_ptr<svulkan2::resource::SVMesh> mPlaneMesh{};

public:
  SVulkan2Scene(SVulkan2Renderer *parent, std::string const &name);

  inline std::string getName() { return mName; }

  inline svulkan2::scene::Scene *getScene() { return mScene.get(); };

  // IPxrScene
  IPxrRigidbody *addRigidbody(const std::string &meshFile, const physx::PxVec3 &scale) override;

  IPxrRigidbody *addRigidbody(physx::PxGeometryType::Enum type, const physx::PxVec3 &scale,
                              std::shared_ptr<IPxrMaterial> material) override;

  IPxrRigidbody *addRigidbody(physx::PxGeometryType::Enum type, const physx::PxVec3 &scale,
                              const physx::PxVec3 &color) override;

  IPxrRigidbody *addRigidbody(std::vector<physx::PxVec3> const &vertices,
                              std::vector<physx::PxVec3> const &normals,
                              std::vector<uint32_t> const &indices, const physx::PxVec3 &scale,
                              std::shared_ptr<IPxrMaterial> material) override;

  IPxrRigidbody *addRigidbody(std::vector<physx::PxVec3> const &vertices,
                              std::vector<physx::PxVec3> const &normals,
                              std::vector<uint32_t> const &indices, const physx::PxVec3 &scale,
                              const physx::PxVec3 &color) override;

  IPxrRigidbody *cloneRigidbody(SVulkan2Rigidbody *other);

  void removeRigidbody(IPxrRigidbody *body) override;

  ICamera *addCamera(uint32_t width, uint32_t height, float fovy, float near, float far,
                     std::string const &shaderDir = "") override;

  void removeCamera(ICamera *camera) override;
  std::vector<ICamera *> getCameras() override;
  void updateRender() override;

  void destroy() override;

  void setAmbientLight(std::array<float, 3> const &color) override;
  std::array<float, 3> getAmbientLight() const override;
  SVulkan2PointLight *addPointLight(std::array<float, 3> const &position,
                                    std::array<float, 3> const &color, bool enableShadow,
                                    float shadowNear = 0.1f, float shadowFar = 5.f) override;
  SVulkan2DirectionalLight *
  addDirectionalLight(std::array<float, 3> const &direction, std::array<float, 3> const &color,
                      bool enableShadow, std::array<float, 3> const &position = {0.f, 0.f, 0.f},
                      float shadowScale = 10.f, float shadowNear = -5.f,
                      float shadowFar = 5.f) override;
  SVulkan2SpotLight *addSpotLight(std::array<float, 3> const &position,
                                  std::array<float, 3> const &direction, float fovInner,
                                  float fovOuter, std::array<float, 3> const &color,
                                  bool enableShadow, float shadowNear, float shadowFar) override;
  void removeLight(ILight *light) override;

  void setEnvironmentMap(std::string_view path) override;
  void setEnvironmentMap(std::array<std::string_view, 6> paths) override;

  inline SVulkan2Renderer *getParentRenderer() const { return mParentRenderer; }
};

class SVulkan2Renderer : public IPxrRenderer {
public:
  std::shared_ptr<svulkan2::core::Context> mContext{};
  std::shared_ptr<svulkan2::resource::SVResourceManager> mResourceManager{};
  vk::CullModeFlagBits mCullMode;

private:
  static std::string gDefaultSpvDir;
  std::vector<std::unique_ptr<SVulkan2Scene>> mScenes;

public:
  static void setLogLevel(std::string const &level);

  SVulkan2Renderer(bool offscreenOnly, uint32_t maxNumMaterials, uint32_t maxNumTextures,
                   uint32_t defaultMipLevels, std::string const &device,
                   std::string const &culling);
  SVulkan2Scene *createScene(std::string const &name) override;
  void removeScene(IPxrScene *scene) override;

  std::shared_ptr<IPxrMaterial> createMaterial() override;
  std::shared_ptr<IPxrTexture>
  createTexture(std::string_view filename, uint32_t mipLevels = 1,
                IPxrTexture::FilterMode::Enum filterMode = {},
                IPxrTexture::AddressMode::Enum addressMode = {}) override;
};

class SVulkan2Camera : public ICamera {
  uint32_t mWidth, mHeight;
  SVulkan2Scene *mScene;
  std::unique_ptr<svulkan2::renderer::Renderer> mRenderer;
  // physx::PxTransform mInitialPose;
  svulkan2::scene::Camera *mCamera;

  vk::UniqueCommandBuffer mCommandBuffer;
  vk::UniqueFence mFence;

  void waitForFence();

public:
  inline uint32_t getWidth() const override { return mWidth; };
  inline uint32_t getHeight() const override { return mHeight; };

  [[nodiscard]] float getPrincipalPointX() const override;
  [[nodiscard]] float getPrincipalPointY() const override;
  [[nodiscard]] float getFocalX() const override;
  [[nodiscard]] float getFocalY() const override;
  [[nodiscard]] float getFovX() const override;
  [[nodiscard]] float getFovY() const override;
  [[nodiscard]] float getNear() const override;
  [[nodiscard]] float getFar() const override;
  [[nodiscard]] float getSkew() const override;

  void setPerspectiveCameraParameters(float near, float far, float fx, float fy, float cx,
                                      float cy, float skew) override;

  SVulkan2Camera(uint32_t width, uint32_t height, float fovy, float near, float far,
                 SVulkan2Scene *scene, std::string const &shaderDir);

  void takePicture() override;

  std::vector<std::string> getRenderTargetNames();

  std::vector<float> getFloatImage(std::string const &name) override;
  std::vector<uint32_t> getUintImage(std::string const &name) override;
  DLManagedTensor *getDLImage(std::string const &name) override;

  inline IPxrScene *getScene() override { return mScene; }

  glm::mat4 getModelMatrix() const;
  glm::mat4 getProjectionMatrix() const;
  glm::mat4 getCameraMatrix() const;

  // ISensor
  physx::PxTransform getPose() const override;
  void setPose(physx::PxTransform const &pose) override;

  void setOrthographicParameters(float near, float far, float scaling, float width, float height);

  svulkan2::scene::Camera *getCamera() const { return mCamera; }
  std::string getMode() const;

  inline svulkan2::renderer::Renderer *getInternalRenderer() const { return mRenderer.get(); }
};

} // namespace Renderer
} // namespace sapien
