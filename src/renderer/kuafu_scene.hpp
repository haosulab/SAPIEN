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

typedef std::vector<std::shared_ptr<kuafu::GeometryInstance>> KuafuGeometryInstances;

class KuafuScene;

class KuafuCamera : public ICamera {
  friend class KuafuScene;

  KuafuScene *pParentScene = nullptr;
  kuafu::Camera *pKCamera = nullptr;
  size_t mIdx;

  void setPxPose(const physx::PxTransform &pose);

public:
  // ICamera
  KuafuCamera(int width, int height, float fovy, KuafuScene *scene, size_t idx);

  [[nodiscard]] inline uint32_t getWidth() const override { return pKCamera->getWidth(); }
  [[nodiscard]] inline uint32_t getHeight() const override { return pKCamera->getHeight(); }

  [[nodiscard]] float getPrincipalPointX() const override;
  [[nodiscard]] float getPrincipalPointY() const override;
  [[nodiscard]] float getFocalX() const override;
  [[nodiscard]] float getFocalY() const override;
  [[nodiscard]] float getNear() const override;
  [[nodiscard]] float getFar() const override;
  [[nodiscard]] float getSkew() const override;

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
    pKCamera->setFullPerspective(width, height, fx, fy, cx, cy, skew);
  }
};

class KuafuRenderShape : public IPxrRenderShape {
  std::shared_ptr<kuafu::GeometryInstance> pKInstance;
  KuafuScene *pParentScene = nullptr;

public:
  KuafuRenderShape(const std::shared_ptr<kuafu::GeometryInstance> &ins, KuafuScene *p)
      : pKInstance(ins), pParentScene(p) {}
  [[nodiscard]] inline std::shared_ptr<IRenderMesh> getGeometry() const override {
    spdlog::get("SAPIEN")->error("KuafuRenderShape::getGeometry not implemented");
    return {};
  }
  [[nodiscard]] std::shared_ptr<IPxrMaterial> getMaterial() const override;
  void setMaterial(std::shared_ptr<IPxrMaterial> m) override;
  //  virtual ~KuafuRenderShape() = default;
};

class KuafuRigidBody : public IPxrRigidbody {
  std::string mName{};
  KuafuScene *pParentScene = nullptr;
  physx::PxVec3 mScale = {1.0, 1.0, 1.0};
  physx::PxTransform mInitialPose = {{0, 0, 0}, physx::PxIdentity};

  KuafuGeometryInstances pKGeometryInstances;

  uint32_t mUniqueId = 0;
  uint32_t mSegmentationId = 0;

public:
  KuafuRigidBody(KuafuScene *scene, KuafuGeometryInstances instances,
                 const physx::PxVec3 &scale = {1.0, 1.0, 1.0});
  KuafuRigidBody(KuafuRigidBody const &other) = delete;
  KuafuRigidBody &operator=(KuafuRigidBody const &other) = delete;

  inline void setName(std::string const &name) override { mName = name; };
  [[nodiscard]] inline std::string getName() const override { return mName; };

  inline void setUniqueId(uint32_t uniqueId) override{
      /* TODO:kuafu_urgent */
  };
  [[nodiscard]] inline uint32_t getUniqueId() const override { return mUniqueId; };
  inline void setSegmentationId(uint32_t segmentationId) override{
      /* TODO:kuafu_urgent */
  };
  [[nodiscard]] inline uint32_t getSegmentationId() const override { return mSegmentationId; };
  inline void setSegmentationCustomData(std::vector<float> const &customData) override{
      /* TODO:kuafu_urgent */
  };
  void setInitialPose(const physx::PxTransform &transform) override;
  [[nodiscard]] inline physx::PxTransform getInitialPose() const override { return mInitialPose; };
  void update(const physx::PxTransform &transform) override;

  void setVisibility(float visibility) override;
  void setVisible(bool visible) override;
  void setRenderMode(uint32_t mode) override;
  void setShadeFlat(bool shadeFlat) override;
  bool getShadeFlat() override;

  std::vector<std::shared_ptr<IPxrRenderShape>> getRenderShapes() override {
    std::vector<std::shared_ptr<IPxrRenderShape>> ret;
    for (const auto &p : pKGeometryInstances)
      ret.push_back(std::make_shared<KuafuRenderShape>(p, pParentScene));
    return ret;
  }

  void destroy() override;
};

class KuafuScene : public IPxrScene {
  friend class KuafuCamera;
  friend class KuafuRenderer;
  friend class KuafuRigidBody;

  kuafu::Scene *pKScene;
  std::shared_ptr<kuafu::Kuafu> pKRenderer;
  std::vector<std::unique_ptr<KuafuRigidBody>> mBodies;
  std::vector<std::unique_ptr<KuafuCamera>> mCameras;
  std::vector<std::unique_ptr<IKuafuLight>> mLights;

  size_t nextCameraIdx = 1;
  size_t lastCameraIdx = 0;

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
   IPxrRigidbody *addRigidbody(std::shared_ptr<IRenderMesh> mesh,
                                      const physx::PxVec3 &scale,
                               std::shared_ptr<IPxrMaterial> material) override {
     throw std::runtime_error("KuafuScene::addRigidbody from mesh is not implemented");
   }


  void removeRigidbody(IPxrRigidbody *body) override;

  ICamera *addCamera(uint32_t width, uint32_t height, float fovy, float near, float far,
                     std::string const &shaderDir) override;

  void removeCamera(ICamera *camera) override;

  std::vector<ICamera *> getCameras() override;

  void setAmbientLight(std::array<float, 3> const &color) override;
  [[nodiscard]] std::array<float, 3> getAmbientLight() const override;

  IPointLight *addPointLight(std::array<float, 3> const &position,
                             std::array<float, 3> const &color, bool enableShadow,
                             float shadowNear, float shadowFar, uint32_t shadowMapSize) override;

  IDirectionalLight *addDirectionalLight(std::array<float, 3> const &direction,
                                         std::array<float, 3> const &color, bool enableShadow,
                                         std::array<float, 3> const &position, float shadowScale,
                                         float shadowNear, float shadowFar,
                                         uint32_t shadowMapSize) override;

  ISpotLight *addSpotLight(std::array<float, 3> const &position,
                           std::array<float, 3> const &direction, float fovInner, float fovOuter,
                           std::array<float, 3> const &color, bool enableShadow, float shadowNear,
                           float shadowFar, uint32_t shadowMapSize) override;

  IActiveLight *addActiveLight(physx::PxTransform const &pose, std::array<float, 3> const &color,
                               float fov, std::string_view texPath, float shadowNear,
                               float shadowFar, uint32_t shadowMapSize) override;

  void removeLight(ILight *light) override;

  /** call this function before every rendering time frame */
  inline void updateRender() override{};

  void destroy() override;

  // non override
  inline auto getKScene() const { return pKScene; };

  inline void setEnvironmentMap(std::string_view path) override {
    getKScene()->setEnvironmentMap(path);
  };
};
} // namespace sapien::Renderer
