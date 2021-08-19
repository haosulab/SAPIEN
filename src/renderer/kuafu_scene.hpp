//
// Created by jet on 7/18/21.
//

#pragma once
#include "render_interface.h"
#include "kuafu.hpp"


namespace sapien::Renderer {

class KuafuScene;

class KuafuRigidBody : public IPxrRigidbody {
  std::string mName{};
  KuafuScene *mParentScene = nullptr;
  physx::PxTransform mInitialPose = {{0, 0, 0}, physx::PxIdentity};
//  kuafu::GeometryInstance *mKObject;

  size_t mKGeometryInstanceIdx;

  uint32_t mUniqueId = 0;
  uint32_t mSegmentationId = 0;
public:
  KuafuRigidBody(KuafuScene *scene, size_t kGeometryInstanceIdx);
  KuafuRigidBody(KuafuRigidBody const &other) = delete;
  KuafuRigidBody &operator=(KuafuRigidBody const &other) = delete;

  inline void setName(std::string const &name) override { mName = name; };
  inline std::string getName() const override { return mName; };
  inline auto getKGeometryInstanceIdx() const { return mKGeometryInstanceIdx; };

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

  std::vector<std::unique_ptr<RenderShape>> getRenderShapes() const { return {}; };
};

class KuafuScene : public IPxrScene {
  friend class KuafuRenderer;
  friend class KuafuRigidBody;

  kuafu::Kuafu* mKRenderer;
  std::vector<std::unique_ptr<KuafuRigidBody>> mBodies;

public:
  IPxrRigidbody *addRigidbody(const std::string &meshFile, const physx::PxVec3 &scale) override;
  IPxrRigidbody *addRigidbody(physx::PxGeometryType::Enum type, const physx::PxVec3 &scale,
                                      std::shared_ptr<IPxrMaterial> material) override;
  inline IPxrRigidbody *addRigidbody(physx::PxGeometryType::Enum type,
                                             const physx::PxVec3 &scale,
                                             const physx::PxVec3 &color) override {
      assert(false); // not implemented yet
  };
  IPxrRigidbody *addRigidbody(std::vector<physx::PxVec3> const &vertices,
                                      std::vector<physx::PxVec3> const &normals,
                                      std::vector<uint32_t> const &indices,
                                      const physx::PxVec3 &scale,
                                      std::shared_ptr<IPxrMaterial> material) override;
  inline IPxrRigidbody *addRigidbody(std::vector<physx::PxVec3> const &vertices,
                                             std::vector<physx::PxVec3> const &normals,
                                             std::vector<uint32_t> const &indices,
                                             const physx::PxVec3 &scale,
                                             const physx::PxVec3 &color) override {
    assert(false); // not implemented yet
  }

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
  inline void updateRender() { assert(false); /* not implemented yet */ };

  void destroy() override;
};
}