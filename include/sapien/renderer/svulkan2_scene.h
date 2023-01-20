#pragma once
#include "render_interface.h"
#include "svulkan2_light.h"
#include "svulkan2_pointbody.h"
#include <memory>
#include <svulkan2/scene/scene.h>

namespace sapien {
namespace Renderer {
class SVulkan2Renderer;
class SVulkan2Rigidbody;
class SVulkan2Camera;

class SVulkan2Scene : public IPxrScene {
  SVulkan2Renderer *mParentRenderer;
  std::shared_ptr<svulkan2::scene::Scene> mScene;
  std::vector<std::unique_ptr<SVulkan2Rigidbody>> mBodies;
  std::vector<std::unique_ptr<SVulkan2PointBody>> mPointBodies;
  std::vector<std::unique_ptr<SVulkan2Camera>> mCameras;
  std::vector<std::unique_ptr<ILight>> mLights;
  std::string mName;

  std::shared_ptr<svulkan2::resource::SVMesh> mCubeMesh{};
  std::shared_ptr<svulkan2::resource::SVMesh> mSphereMesh{};
  std::shared_ptr<svulkan2::resource::SVMesh> mPlaneMesh{};

public:
  SVulkan2Scene(SVulkan2Renderer *parent, std::string const &name);

  inline std::string getName() { return mName; }

  inline std::shared_ptr<svulkan2::scene::Scene> getScene() { return mScene; };

  // IPxrScene
  IPxrRigidbody *addRigidbody(const std::string &meshFile, const physx::PxVec3 &scale) override;

  IPxrRigidbody *addRigidbody(const std::string &meshFile, const physx::PxVec3 &scale,
                              std::shared_ptr<IPxrMaterial> material) override;

  IPxrRigidbody *addRigidbody(std::shared_ptr<IRenderMesh> mesh, const physx::PxVec3 &scale,
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

  IPxrRigidbody *cloneRigidbody(SVulkan2Rigidbody *other);

  IPxrPointBody *addPointBody(
      Eigen::Ref<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>> positions) override;

  void removeRigidbody(IPxrRigidbody *body) override;
  void removePointBody(IPxrPointBody *body) override;

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
                                    float shadowNear, float shadowFar,
                                    uint32_t shadowMapSize) override;
  SVulkan2DirectionalLight *
  addDirectionalLight(std::array<float, 3> const &direction, std::array<float, 3> const &color,
                      bool enableShadow, std::array<float, 3> const &position, float shadowScale,
                      float shadowNear, float shadowFar, uint32_t shadowMapSize) override;
  SVulkan2SpotLight *addSpotLight(std::array<float, 3> const &position,
                                  std::array<float, 3> const &direction, float fovInner,
                                  float fovOuter, std::array<float, 3> const &color,
                                  bool enableShadow, float shadowNear, float shadowFar,
                                  uint32_t shadowMapSize) override;
  IActiveLight *addActiveLight(physx::PxTransform const &pose, std::array<float, 3> const &color,
                               float fov, std::string_view texPath, float shadowNear,
                               float shadowFar, uint32_t shadowMapSize) override;

  void removeLight(ILight *light) override;

  void setEnvironmentMap(std::string_view path) override;
  void setEnvironmentMap(std::array<std::string_view, 6> paths) override;

  inline SVulkan2Renderer *getParentRenderer() const { return mParentRenderer; }
};

} // namespace Renderer
} // namespace sapien
