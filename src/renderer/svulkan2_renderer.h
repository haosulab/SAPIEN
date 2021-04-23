#pragma once
#include "renderer/render_interface.h"
#include "svulkan2_light.h"
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

class SVulkan2Material : public IPxrMaterial {
  std::shared_ptr<svulkan2::resource::SVMetallicMaterial> mMaterial;

public:
  SVulkan2Material(std::shared_ptr<svulkan2::resource::SVMetallicMaterial> material);
  void setBaseColor(std::array<float, 4> color) override;
  void setRoughness(float roughness) override;
  void setSpecular(float specular) override;
  void setMetallic(float metallic) override;
  std::shared_ptr<svulkan2::resource::SVMetallicMaterial> getMaterial() const { return mMaterial; }
};

class SVulkan2Rigidbody : public IPxrRigidbody {
  std::string mName{};
  SVulkan2Scene *mParentScene = nullptr;
  physx::PxTransform mInitialPose = {{0, 0, 0}, physx::PxIdentity};
  std::vector<svulkan2::scene::Object *> mObjects;

  uint32_t mUniqueId{0};
  uint32_t mSegmentationId{0};

public:
  SVulkan2Rigidbody(SVulkan2Scene *scene, std::vector<svulkan2::scene::Object *> const &objects);
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
  inline physx::PxTransform getInitialPose() const { return mInitialPose; };
  void update(const physx::PxTransform &transform) override;

  void setVisibility(float visibility) override;
  void setVisible(bool visible) override;
  void setRenderMode(uint32_t mode) override;

  void destroy() override;

  /** internal use only */
  void destroyVisualObjects();
  /** internal use only */
  inline std::vector<svulkan2::scene::Object *> const &getVisualObjects() const {
    return mObjects;
  }

  std::vector<std::unique_ptr<RenderShape>> getRenderShapes() const override;
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

  ICamera *addCamera(std::string const &name, uint32_t width, uint32_t height, float fovx,
                     float fovy, float near, float far,
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
                                  std::array<float, 3> const &direction, float fov,
                                  std::array<float, 3> const &color, bool enableShadow,
                                  float shadowNear = 0.1f, float shadowFar = 5.f) override;

  inline SVulkan2Renderer *getParentRenderer() const { return mParentRenderer; }
};

class SVulkan2Renderer : public IPxrRenderer {
public:
  std::shared_ptr<svulkan2::core::Context> mContext{};
  std::shared_ptr<svulkan2::resource::SVResourceManager> mResourceManager{};

private:
  static std::string gDefaultSpvDir;
  std::vector<std::unique_ptr<SVulkan2Scene>> mScenes;

public:
  static void setLogLevel(std::string const &level);

  SVulkan2Renderer(bool offscreenOnly, uint32_t maxNumMaterials, uint32_t maxNumTextures,
                   uint32_t defaultMipLevels);
  SVulkan2Scene *createScene(std::string const &name) override;
  void removeScene(IPxrScene *scene) override;

  std::shared_ptr<IPxrMaterial> createMaterial() override;
};

class SVulkan2Camera : public ICamera {
  std::string mName;
  uint32_t mWidth, mHeight;
  SVulkan2Scene *mScene;
  std::unique_ptr<svulkan2::renderer::Renderer> mRenderer;
  physx::PxTransform mInitialPose;
  svulkan2::scene::Camera *mCamera;

  vk::UniqueCommandBuffer mCommandBuffer;
  vk::UniqueFence mFence;

  void waitForFence();

public:
  inline std::string const &getName() const override { return mName; }
  inline uint32_t getWidth() const override { return mWidth; };
  inline uint32_t getHeight() const override { return mHeight; };

  inline float getFovy() const override { return mCamera->getFovy(); }
  inline float getNear() const override { return mCamera->getNear(); }
  inline float getFar() const override { return mCamera->getFar(); }

  SVulkan2Camera(std::string const &name, uint32_t width, uint32_t height, float fovy, float near,
                 float far, SVulkan2Scene *scene, std::string const &shaderDir);

  void takePicture() override;
  std::vector<float> getColorRGBA() override;
  std::vector<float> getAlbedoRGBA() override;
  std::vector<float> getNormalRGBA() override;
  std::vector<float> getDepth() override;
  std::vector<int> getSegmentation() override;
  std::vector<int> getObjSegmentation() override;

  std::tuple<std::vector<float>, std::array<uint32_t, 3>>
  getFloatTexture(std::string const &textureName);

  std::tuple<std::vector<uint32_t>, std::array<uint32_t, 3>>
  getUint32Texture(std::string const &textureName);

  std::tuple<std::vector<uint8_t>, std::array<uint32_t, 3>>
  getUint8Texture(std::string const &textureName);

#ifdef SAPIEN_DLPACK_INTEROP
  std::tuple<std::shared_ptr<svulkan2::core::CudaBuffer>, std::array<uint32_t, 2>, vk::Format>
  getCudaBuffer(std::string const &textureName);
#endif

  inline IPxrScene *getScene() override { return mScene; }

  glm::mat4 getModelMatrix() const;
  glm::mat4 getProjectionMatrix() const;
  glm::mat4 getCameraMatrix() const;

  // ISensor
  physx::PxTransform getPose() const override;
  void setInitialPose(physx::PxTransform const &pose) override;
  void setPose(physx::PxTransform const &pose) override;

  void setPerspectiveParameters(float near, float far, float fovy, float aspect);
  void setFullPerspectiveParameters(float near, float far, float fx, float fy, float cx, float cy,
                                    float width, float height, float skew);
  void setOrthographicParameters(float near, float far, float aspect, float scaling);

  svulkan2::scene::Camera *getCamera() const { return mCamera; }
  std::string getMode() const;

  inline svulkan2::renderer::Renderer *getInternalRenderer() const { return mRenderer.get(); }
};

} // namespace Renderer
} // namespace sapien
