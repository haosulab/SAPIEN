#ifdef _USE_VULKAN
#pragma once
#include "renderer/render_interface.h"

#include "sapien_vulkan/internal/vulkan_context.h"
#include "sapien_vulkan/internal/vulkan_renderer.h"
#include "sapien_vulkan/object.h"
#include "sapien_vulkan/scene.h"
#include "sapien_vulkan/camera.h"

namespace sapien {
namespace Renderer {

class SapienVulkanRenderer;
class SapienVulkanScene;
class SapienVulkanCamera;

class SapienVulkanRigidbody : public IPxrRigidbody {
  SapienVulkanScene *mParentScene = nullptr;
  physx::PxTransform mInitialPose = {{0, 0, 0}, physx::PxIdentity};
  std::vector<svulkan::Object *> mObjects;

  uint32_t mUniqueId = 0;
  uint32_t mSegmentationId = 0;

public:
  SapienVulkanRigidbody(SapienVulkanScene *scene, std::vector<svulkan::Object *> const &objects);
  SapienVulkanRigidbody(SapienVulkanRigidbody const &other) = delete;

  SapienVulkanRigidbody &operator=(SapienVulkanRigidbody const &other) = delete;

  void setUniqueId(uint32_t uniqueId) override;
  uint32_t getUniqueId() const override;
  void setSegmentationId(uint32_t segmentationId) override;
  uint32_t getSegmentationId() const override;
  void setSegmentationCustomData(const std::vector<float> &customData) override;
  void setInitialPose(const physx::PxTransform &transform) override;
  inline physx::PxTransform getInitialPose() const { return mInitialPose; };
  void update(const physx::PxTransform &transform) override;

  void setVisible(bool visible) override;
  void setRenderMode(uint32_t mode) override;

  void destroy() override;

  void destroyVisualObjects();
  inline std::vector<svulkan::Object *> const &getVisualObjects() const { return mObjects; }
};

class SapienVulkanScene : public IPxrScene {
  SapienVulkanRenderer *mParentRenderer;
  std::unique_ptr<svulkan::Scene> mScene;
  std::vector<std::unique_ptr<SapienVulkanRigidbody>> mBodies;
  std::vector<std::unique_ptr<SapienVulkanCamera>> mCameras;
  std::string mName;

public:
  SapienVulkanScene(SapienVulkanRenderer *renderer, std::string const &name = "");

  inline std::string getName() { return mName; }

  inline svulkan::Scene *getScene() { return mScene.get(); };

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

  // virtual IPxrRigidbody *cloneRigidbody(SapienVulkanRigidbody *other);

  virtual void removeRigidbody(IPxrRigidbody *body) override;

  virtual ICamera *addCamera(std::string const &name, uint32_t width, uint32_t height, float fovx,
                             float fovy, float near, float far,
                             std::string const &shaderDir = "") override;
  virtual void removeCamera(ICamera *camera) override;
  virtual std::vector<ICamera *> getCameras() override;

  void destroy() override;

  //======== Lights ========//
  void setAmbientLight(std::array<float, 3> const &color) override;
  void addPointLight(std::array<float, 3> const &position,
                     std::array<float, 3> const &color) override;
  void addDirectionalLight(std::array<float, 3> const &direction,
                           std::array<float, 3> const &color) override;
  void setShadowLight(std::array<float, 3> const &direction,
                      std::array<float, 3> const &color) override;

  inline SapienVulkanRenderer* getParentRenderer() const { return mParentRenderer; }
};

class SapienVulkanRenderer : public IPxrRenderer {
  static std::string gDefaultSpvDir;
  std::vector<std::unique_ptr<SapienVulkanScene>> mScenes;

public:
  std::unique_ptr<svulkan::VulkanContext> mContext = nullptr;
  SapienVulkanRenderer();

  // IPxrRenderer
  SapienVulkanScene *createScene(std::string const &name) override;
  void removeScene(IPxrScene *scene) override;
  void setLogLevel(std::string const &level);
};

class SapienVulkanCamera : public ICamera {
  std::string mName;
  uint32_t mWidth, mHeight;
  SapienVulkanScene *mScene;
  std::unique_ptr<svulkan::VulkanRenderer> mRenderer;
  physx::PxTransform mInitialPose;
  std::unique_ptr<svulkan::Camera> mCamera;

  vk::UniqueCommandBuffer mCommandBuffer;
  vk::UniqueFence mFence;

  void waitForFence();
public:
  inline std::string const &getName() const override { return mName; }
  inline uint32_t getWidth() const override { return mWidth; };
  inline uint32_t getHeight() const override{return mHeight; };

  inline float getFovy() const override { return mCamera->fovy; }
  inline float getNear() const override { return mCamera->near; }
  inline float getFar() const override { return mCamera->far; }

  SapienVulkanCamera(std::string const &name, uint32_t width, uint32_t height, float fovy,
                     float near, float far, SapienVulkanScene *scene, std::string const &shaderDir);
  void takePicture() override;
  std::vector<float> getColorRGBA() override;
  std::vector<float> getAlbedoRGBA() override;
  std::vector<float> getNormalRGBA() override;
  std::vector<float> getDepth() override;
  std::vector<int> getSegmentation() override;
  std::vector<int> getObjSegmentation() override;

  inline IPxrScene *getScene() override { return mScene; }


  // ISensor
  physx::PxTransform getPose() const override;
  void setInitialPose(physx::PxTransform const &pose) override;
  void setPose(physx::PxTransform const &pose) override;

  void changeModeToOrthographic(float scaling);
  void changeModeToPerspective(float fovy);

  bool isOrthographic() const;
};



} // namespace Renderer
} // namespace sapien
#endif
