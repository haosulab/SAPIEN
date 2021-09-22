//
// Created by jet on 7/18/21.
//

#pragma once
#include "kuafu.hpp"
#include "kuafu_light.hpp"
#include "render_interface.h"
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
  [[nodiscard]] const std::string &getName() const { return mName; }
};

class KuafuCamera : public ICamera {
  friend class KuafuScene;

  KuafuScene *pParentScene = nullptr;
  std::shared_ptr<KCamera> pKCamera;

  void setPxPose(const physx::PxTransform &pose);

public:
  // ICamera

  KuafuCamera(int width, int height, float fovy, KuafuScene *scene) {
    pParentScene = scene;
    pKCamera = std::make_shared<KCamera>("", width, height);
    pKCamera->reset();
    pKCamera->setSize(width, height);
    pKCamera->setFov(glm::degrees(fovy));
  }

  inline uint32_t getWidth() const override { return pKCamera->getWidth(); }
  inline uint32_t getHeight() const override { return pKCamera->getHeight(); }

  [[nodiscard]] virtual float getPrincipalPointX() const override;
  [[nodiscard]] virtual float getPrincipalPointY() const override;
  [[nodiscard]] virtual float getFocalX() const override;
  [[nodiscard]] virtual float getFocalY() const override;
  [[nodiscard]] virtual float getFovX() const override;
  [[nodiscard]] virtual float getFovY() const override;
  [[nodiscard]] virtual float getNear() const override;
  [[nodiscard]] virtual float getFar() const override;
  [[nodiscard]] virtual float getSkew() const override;

  void setPerspectiveCameraParameters(float near, float far, float fx, float fy, float cx,
                                      float cy, float skew) override;

  void takePicture() override;

  std::vector<float> getFloatImage(std::string const &name) override;
  std::vector<uint32_t> getUintImage(std::string const &name) override;

  // ISensor

  [[nodiscard]] physx::PxTransform getPose() const override;
  void setPose(physx::PxTransform const &pose) override;
  IPxrScene *getScene() override;

  //  virtual ~ISensor() = default;

  // Non-override
  inline void setFullPerspective(float fx, float fy, float cx, float cy, float width, float height,
                                 float skew) {
    pKCamera->setFullPerspective(fx, fy, cx, cy, width, height, skew);
  }
};

class KuafuRigidBody : public IPxrRigidbody {
  std::string mName{};
  KuafuScene *mParentScene = nullptr;
  physx::PxVec3 mScale = {1.0, 1.0, 1.0};
  physx::PxTransform mInitialPose = {{0, 0, 0}, physx::PxIdentity};

  std::vector<size_t> mKGeometryInstanceIndices;

  uint32_t mUniqueId = 0;
  uint32_t mSegmentationId = 0;

public:
  KuafuRigidBody(KuafuScene *scene, std::vector<size_t> indices,
                 const physx::PxVec3 &scale = {1.0, 1.0, 1.0});
  KuafuRigidBody(KuafuRigidBody const &other) = delete;
  KuafuRigidBody &operator=(KuafuRigidBody const &other) = delete;

  inline void setName(std::string const &name) override { mName = name; };
  [[nodiscard]] inline std::string getName() const override { return mName; };
  [[maybe_unused]] [[nodiscard]] inline auto getKGeometryInstanceIndices() const {
    return mKGeometryInstanceIndices;
  };

  inline void setUniqueId(uint32_t uniqueId) override{/* TODO:kuafu_urgent */};
  [[nodiscard]] inline uint32_t getUniqueId() const override { return mUniqueId; };
  inline void setSegmentationId(uint32_t segmentationId) override{/* TODO:kuafu_urgent */};
  [[nodiscard]] inline uint32_t getSegmentationId() const override { return mSegmentationId; };
  inline void setSegmentationCustomData(std::vector<float> const &customData) override{
      /* TODO:kuafu_urgent */};
  void setInitialPose(const physx::PxTransform &transform) override;
  [[nodiscard]] inline physx::PxTransform getInitialPose() const { return mInitialPose; };
  void update(const physx::PxTransform &transform) override;

  void setVisibility(float visibility) override;
  void setVisible(bool visible) override;
  void setRenderMode(uint32_t mode) override;

  void destroy() override;
};

class KuafuScene : public IPxrScene {
  friend class KuafuCamera;
  friend class KuafuRenderer;
  friend class KuafuRigidBody;

  std::shared_ptr<kuafu::Kuafu> pKRenderer;
  std::vector<std::unique_ptr<KuafuRigidBody>> mBodies;
  std::vector<std::unique_ptr<KuafuCamera>> mCameras;

  std::vector<std::shared_ptr<KuafuPointLight>> mPointLights;
  std::vector<std::shared_ptr<KuafuDirectionalLight>> mDirectionalLights;
  std::vector<std::shared_ptr<KuafuSpotLight>> mSpotLights;
  std::vector<std::shared_ptr<KuafuActiveLight>> mActiveLights;

public:
  IPxrRigidbody *addRigidbody(const std::string &meshFile, const physx::PxVec3 &scale) override;
  IPxrRigidbody *addRigidbody(const std::string &meshFile, const physx::PxVec3 &scale,
                              std::shared_ptr<IPxrMaterial> material) override;
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

  void removeRigidbody(IPxrRigidbody *body) override;

  ICamera *addCamera(uint32_t width, uint32_t height, float fovy, float near, float far,
                     std::string const &shaderDir) override;

  void removeCamera(ICamera *camera) override;

  std::vector<ICamera *> getCameras() override;

  void setAmbientLight(std::array<float, 3> const &color) override;
  [[nodiscard]] std::array<float, 3> getAmbientLight() const override;

  IPointLight *addPointLight(std::array<float, 3> const &position,
                             std::array<float, 3> const &color, bool enableShadow,
                             float shadowNear, float shadowFar) override;

  IDirectionalLight *addDirectionalLight(std::array<float, 3> const &direction,
                                         std::array<float, 3> const &color, bool enableShadow,
                                         std::array<float, 3> const &position, float shadowScale,
                                         float shadowNear, float shadowFar) override;

  ISpotLight *addSpotLight(std::array<float, 3> const &position,
                           std::array<float, 3> const &direction, float fovInner, float fovOuter,
                           std::array<float, 3> const &color, bool enableShadow, float shadowNear,
                           float shadowFar) override;

  IActiveLight *addActiveLight(physx::PxTransform const &pose, std::array<float, 3> const &color,
                               float fov, std::string_view texPath) override;

  void removeLight(ILight *light) override;

  /** call this function before every rendering time frame */
  inline void updateRender() override{
      //    spdlog::get("SAPIEN")->warn("KF: updateRender not implemented yet");
  };

  inline auto &getKScene() { return pKRenderer->getScene(); }

  void destroy() override;

  inline void setEnvironmentMap(std::string_view path) override {
    pKRenderer->getScene().setEnvironmentMap(path);
  };
};
} // namespace sapien::Renderer
