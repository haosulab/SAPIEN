//
// Created by jet on 7/18/21.
//

#pragma once
#include "render_interface.h"
#include "kuafu.hpp"
#include <spdlog/spdlog.h>

#include <utility>

namespace sapien::Renderer {

class KuafuScene;

class KCamera : public kuafu::Camera {
  std::string mName;
  void processKeyboard() override;

public:
  KCamera() = delete;
  KCamera(std::string name, int width, int height)
      : kuafu::Camera(width, height), mName(std::move(name)) {}
  [[nodiscard]] const std::string& getName() const { return mName; }
};

class KuafuCamera : public ICamera {
  friend class KuafuScene;

  physx::PxTransform mInitialPose = {{0, 0, 0}, physx::PxIdentity};
  KuafuScene *pParentScene = nullptr;
  std::shared_ptr<KCamera> mKCamera;

  void setPxPose(const physx::PxTransform &pose);

public:
  // ICamera

  KuafuCamera(std::string const& name, int width, int height,
              float fovy, KuafuScene *scene){
    pParentScene = scene;
    mKCamera = std::make_shared<KCamera>(name, width, height);
    mKCamera->reset();
    mKCamera->setSize(width, height);
    mKCamera->setFov(glm::degrees(fovy));
  }

  [[nodiscard]] inline const std::string &getName() const override { return mKCamera->getName(); };
  [[nodiscard]] inline uint32_t getWidth() const override { return mKCamera->getWidth(); };
  [[nodiscard]] inline uint32_t getHeight() const override { return mKCamera->getHeight(); };
  [[nodiscard]] inline float getFovy() const override { return glm::radians(mKCamera->getFov()); };
  [[nodiscard]] inline float getNear() const override { return 0.1; };
  [[nodiscard]] inline float getFar() const override { return 100.0; };

  void takePicture() override;
  std::vector<float> getColorRGBA() override;
  inline std::vector<float> getAlbedoRGBA() override {
    spdlog::get("SAPIEN")->warn("getAlbedoRGBA not implemented yet");
    return {};
  };
  inline std::vector<float> getNormalRGBA() override {
    spdlog::get("SAPIEN")->warn("getNormalRGBA not implemented yet");
    return {};
  };
  inline std::vector<float> getDepth() override {
    spdlog::get("SAPIEN")->warn("getDepth not implemented yet");
    return {};
  };
  inline std::vector<int> getSegmentation() override {
    spdlog::get("SAPIEN")->warn("getSegmentation not implemented yet");
    return {};
  };
  inline std::vector<int> getObjSegmentation() override {
    spdlog::get("SAPIEN")->warn("getObjSegmentation not implemented yet");
    return {};
  };

  // ISensor

  void setInitialPose(physx::PxTransform const &pose) override;
  physx::PxTransform getPose() const override;
  void setPose(physx::PxTransform const &pose) override;
  IPxrScene *getScene() override;

//  virtual ~ISensor() = default;
};

class KuafuRigidBody : public IPxrRigidbody {
  std::string mName{};
  KuafuScene *mParentScene = nullptr;
  physx::PxVec3 mScale = {1.0, 1.0, 1.0};
  physx::PxTransform mInitialPose = {{0, 0, 0}, physx::PxIdentity};
//  kuafu::GeometryInstance *mKObject;

  std::vector<size_t> mKGeometryInstanceIndices;

  uint32_t mUniqueId = 0;
  uint32_t mSegmentationId = 0;

  bool mHaveSetInvisible = false;

public:
  KuafuRigidBody(KuafuScene *scene, std::vector<size_t> indices, physx::PxVec3 scale = {1.0, 1.0, 1.0});
  KuafuRigidBody(KuafuRigidBody const &other) = delete;
  KuafuRigidBody &operator=(KuafuRigidBody const &other) = delete;

  inline void setName(std::string const &name) override { mName = name; };
  inline std::string getName() const override { return mName; };
  inline auto getKGeometryInstanceIndices() const { return mKGeometryInstanceIndices; };

  inline void setUniqueId(uint32_t uniqueId) override { /* TODO:kuafu_urgent */ };
  inline uint32_t getUniqueId() const override { return mUniqueId; };
  inline void setSegmentationId(uint32_t segmentationId) override { /* TODO:kuafu_urgent */ };
  inline uint32_t getSegmentationId() const override { return mSegmentationId; };
  inline void setSegmentationCustomData(std::vector<float> const &customData) override { /* TODO:kuafu_urgent */ };
  void setInitialPose(const physx::PxTransform &transform) override;
  inline physx::PxTransform getInitialPose() const { return mInitialPose; };
  void update(const physx::PxTransform &transform) override;

  void setVisibility(float visibility) override;
  void setVisible(bool visible) override;
  void setRenderMode(uint32_t mode) override;

  void destroy() override;

  std::vector<std::unique_ptr<RenderShape> > getRenderShapes() const {
    spdlog::get("SAPIEN")->error("getRenderShapes not implemented yet");
    return {};
  };
};

class KuafuScene : public IPxrScene {
  friend class KuafuCamera;
  friend class KuafuRenderer;
  friend class KuafuRigidBody;

  kuafu::Kuafu *pKRenderer;
  std::vector<std::unique_ptr<KuafuRigidBody>> mBodies;
  std::vector<std::unique_ptr<KuafuCamera>> mCameras;

  bool mUseViewer = false;

public:
  IPxrRigidbody *addRigidbodyWithNewMaterial(
      const std::string &meshFile, const physx::PxVec3 &scale, std::shared_ptr<IPxrMaterial> material = nullptr);

  IPxrRigidbody *addRigidbody(const std::string &meshFile, const physx::PxVec3 &scale) override;
  IPxrRigidbody *addRigidbody(physx::PxGeometryType::Enum type, const physx::PxVec3 &scale,
                                      std::shared_ptr<IPxrMaterial> material) override;
  IPxrRigidbody *addRigidbody(physx::PxGeometryType::Enum type,
                                             const physx::PxVec3 &scale,
                                             const physx::PxVec3 &color) override;
  IPxrRigidbody *addRigidbody(std::vector<physx::PxVec3> const &vertices,
                                      std::vector<physx::PxVec3> const &normals,
                                      std::vector<uint32_t> const &indices,
                                      const physx::PxVec3 &scale,
                                      std::shared_ptr<IPxrMaterial> material) override;
  IPxrRigidbody *addRigidbody(std::vector<physx::PxVec3> const &vertices,
                                             std::vector<physx::PxVec3> const &normals,
                                             std::vector<uint32_t> const &indices,
                                             const physx::PxVec3 &scale,
                                             const physx::PxVec3 &color) override;

  void removeRigidbody(IPxrRigidbody *body) override;

  ICamera *addCamera(std::string const &name, uint32_t width, uint32_t height, float fovx,
                             float fovy, float near, float far,
                             std::string const &shaderDir = "") override;
  void removeCamera(ICamera *camera) override;

  std::vector<ICamera *> getCameras() override;

  void setAmbientLight(std::array<float, 3> const &color) override;
  std::array<float, 3> getAmbientLight() const override;

  IPointLight *addPointLight(std::array<float, 3> const &position,
                                     std::array<float, 3> const &color, bool enableShadow,
                                     float shadowNear, float shadowFar) override;

  IDirectionalLight *
  addDirectionalLight(std::array<float, 3> const &direction, std::array<float, 3> const &color,
                      bool enableShadow, std::array<float, 3> const &position, float shadowScale,
                      float shadowNear, float shadowFar) override;

  ISpotLight *addSpotLight(std::array<float, 3> const &position,
                                   std::array<float, 3> const &direction, float fovInner,
                                   float fovOuter, std::array<float, 3> const &color,
                                   bool enableShadow, float shadowNear, float shadowFar) override;

  void removeLight(ILight *light) override;

  /** call this function before every rendering time frame */
  inline void updateRender() {
//    spdlog::get("SAPIEN")->error("updateRender not implemented yet");
  };

  inline auto& getKScene() { return pKRenderer->getScene(); }

  void destroy() override;
};
}