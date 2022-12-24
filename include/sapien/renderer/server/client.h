#pragma once

#include "common.h"
#include "renderer/server/protos/render_server.grpc.pb.h"
#include "sapien/renderer/render_interface.h"
#include <grpc/grpc.h>
#include <grpcpp/grpcpp.h>

namespace sapien {
namespace Renderer {
namespace server {

using ::sapien::Renderer::ICamera;
using ::sapien::Renderer::IPxrMaterial;
using ::sapien::Renderer::IPxrRenderer;
using ::sapien::Renderer::IPxrRigidbody;
using ::sapien::Renderer::IPxrScene;
using ::sapien::Renderer::IPxrTexture;

class ClientRenderer;
class ClientScene;
class ClientShape;
class ClientRigidbody;

class ClientMaterial : public IPxrMaterial {
public:
  ClientMaterial(std::shared_ptr<ClientRenderer> renderer, rs_id_t id);

  void setBaseColor(std::array<float, 4> color) override;
  [[nodiscard]] std::array<float, 4> getBaseColor() const override {
    throw std::runtime_error("get material not implemented for rendering client");
  };
  void setRoughness(float roughness) override;
  [[nodiscard]] float getRoughness() const override {
    throw std::runtime_error("get material not implemented for rendering client");
  };
  void setSpecular(float specular) override;
  [[nodiscard]] float getSpecular() const override {
    throw std::runtime_error("get material not implemented for rendering client");
  };
  void setMetallic(float metallic) override;
  [[nodiscard]] float getMetallic() const override {
    throw std::runtime_error("get material not implemented for rendering client");
  };

  virtual ~ClientMaterial();

  rs_id_t getId() const { return mId; }

private:
  std::shared_ptr<ClientRenderer> mRenderer;
  rs_id_t mId;
};

class ClientCamera : public ICamera {
public:
  ClientCamera(ClientScene *scene, rs_id_t id, uint32_t width, uint32_t height, float cx, float cy,
               float fx, float fy, float near, float far, float skew);

  [[nodiscard]] inline physx::PxTransform getPose() const { return mPose; };
  inline void setPose(physx::PxTransform const &pose) override { mPose = pose; }
  IPxrScene *getScene() override;

  uint32_t getWidth() const override { return mWidth; }
  uint32_t getHeight() const override { return mHeight; };

  [[nodiscard]] float getPrincipalPointX() const override { return mCx; }
  [[nodiscard]] float getPrincipalPointY() const override { return mCy; }
  [[nodiscard]] float getFocalX() const override { return mFx; }
  [[nodiscard]] float getFocalY() const override { return mFy; }
  [[nodiscard]] float getNear() const override { return mNear; }
  [[nodiscard]] float getFar() const override { return mFar; }
  [[nodiscard]] float getSkew() const override { return mSkew; }

  std::vector<float> getFloatImage(std::string const &name) override {
    throw std::runtime_error("get image for cameras are not supported for rendering client");
  };
  std::vector<uint32_t> getUintImage(std::string const &name) override {
    throw std::runtime_error("get image for cameras are not supported for rendering client");
  };

  void setPerspectiveCameraParameters(float near, float far, float fx, float fy, float cx,
                                      float cy, float skew) override;

  void takePicture() override;

  rs_id_t getId() const { return mId; }

private:
  ClientScene *mScene;
  rs_id_t mId;
  physx::PxTransform mPose;
  uint32_t mWidth;
  uint32_t mHeight;
  float mCx;
  float mCy;
  float mFx;
  float mFy;
  float mNear;
  float mFar;
  float mSkew;
};

class ClientPointLight : public IPointLight {
public:
  ClientPointLight(rs_id_t id) : mId(id){};

  physx::PxTransform getPose() const override {
    throw std::runtime_error("light cannot be accessed in rendering client");
  }
  void setPose(physx::PxTransform const &transform) override {
    throw std::runtime_error("light cannot be accessed in rendering client");
  }
  physx::PxVec3 getColor() const override {
    throw std::runtime_error("light cannot be accessed in rendering client");
  }
  void setColor(physx::PxVec3 color) override {
    throw std::runtime_error("light cannot be accessed in rendering client");
  }
  bool getShadowEnabled() const override {
    throw std::runtime_error("light cannot be accessed in rendering client");
  }
  void setShadowEnabled(bool enabled) override {
    throw std::runtime_error("light cannot be accessed in rendering client");
  }
  physx::PxVec3 getPosition() const override {
    throw std::runtime_error("light cannot be accessed in rendering client");
  }
  void setPosition(physx::PxVec3 position) override {
    throw std::runtime_error("light cannot be accessed in rendering client");
  }
  void setShadowParameters(float near, float far) override {
    throw std::runtime_error("light cannot be accessed in rendering client");
  }
  float getShadowNear() const override {
    throw std::runtime_error("light cannot be accessed in rendering client");
  }
  float getShadowFar() const override {
    throw std::runtime_error("light cannot be accessed in rendering client");
  }

private:
  rs_id_t mId;
};

class ClientDirectionalLight : public IDirectionalLight {
public:
  ClientDirectionalLight(rs_id_t id) : mId(id) {}

  physx::PxTransform getPose() const override {
    throw std::runtime_error("light cannot be accessed in rendering client");
  }
  void setPose(physx::PxTransform const &transform) override {
    throw std::runtime_error("light cannot be accessed in rendering client");
  }
  physx::PxVec3 getColor() const override {
    throw std::runtime_error("light cannot be accessed in rendering client");
  }
  void setColor(physx::PxVec3 color) override {
    throw std::runtime_error("light cannot be accessed in rendering client");
  }
  bool getShadowEnabled() const override {
    throw std::runtime_error("light cannot be accessed in rendering client");
  }
  void setShadowEnabled(bool enabled) override {
    throw std::runtime_error("light cannot be accessed in rendering client");
  }
  physx::PxVec3 getDirection() const override {
    throw std::runtime_error("light cannot be accessed in rendering client");
  }
  void setDirection(physx::PxVec3 direction) override {
    throw std::runtime_error("light cannot be accessed in rendering client");
  }
  void setShadowParameters(float halfSize, float near, float far) override {
    throw std::runtime_error("light cannot be accessed in rendering client");
  }
  float getShadowHalfSize() const override {
    throw std::runtime_error("light cannot be accessed in rendering client");
  }
  float getShadowNear() const override {
    throw std::runtime_error("light cannot be accessed in rendering client");
  }
  float getShadowFar() const override {
    throw std::runtime_error("light cannot be accessed in rendering client");
  }

private:
  rs_id_t mId;
};

class ClientShape : public IPxrRenderShape {
public:
  ClientShape(ClientRigidbody *body, uint32_t index);
  [[nodiscard]] std::shared_ptr<IPxrMaterial> getMaterial() const override;

private:
  ClientRigidbody *mBody;
  uint32_t mIndex;
};

class ClientRigidbody : public IPxrRigidbody {

public:
  ClientRigidbody(ClientScene *scene, rs_id_t id);

  inline void setName(std::string const &name) override { mName = name; }
  inline std::string getName() const override { return mName; }

  void setUniqueId(uint32_t uniqueId) override;
  uint32_t getUniqueId() const override;

  void setSegmentationId(uint32_t segmentationId) override;
  uint32_t getSegmentationId() const override;

  void setInitialPose(const physx::PxTransform &transform) override;

  void setSegmentationCustomData(std::vector<float> const &customData) override {
    throw std::runtime_error("custom data is not valid for render client");
  }
  void update(const physx::PxTransform &transform) override;
  void setVisibility(float visibility) override;
  void setVisible(bool visible) override { setVisibility(visible); }
  void setRenderMode(uint32_t mode) override {
    throw std::runtime_error("render mode is not supported for render client");
  }
  void setShadeFlat(bool shadeFlat) override {
    throw std::runtime_error("shade flat is not supported for render client");
  }
  bool getShadeFlat() override {
    throw std::runtime_error("shade flat is not supported for render client");
  }

  std::vector<std::shared_ptr<IPxrRenderShape>> getRenderShapes() override;

  void destroy() override;

  inline rs_id_t getId() const { return mId; };

  inline physx::PxTransform const &getCurrentPose() const { return mCurrentPose; }

  inline ClientScene *getScene() const { return mScene; }

public:
  ClientScene *mScene;
  rs_id_t mId;
  std::string mName;

  uint32_t mUniqueId;
  uint32_t mSegmentationId;

  physx::PxTransform mInitialPose;
  physx::PxTransform mCurrentPose;
};

class ClientScene : public IPxrScene {
public:
  ClientScene(ClientRenderer *renderer, rs_id_t id, std::string const &name);

  //========== Body ==========//
  IPxrRigidbody *addRigidbody(const std::string &meshFile, const physx::PxVec3 &scale) override;
  IPxrRigidbody *addRigidbody(physx::PxGeometryType::Enum type, const physx::PxVec3 &scale,
                              std::shared_ptr<IPxrMaterial> material) override;

  IPxrRigidbody *addRigidbody(std::shared_ptr<IRenderMesh> mesh, const physx::PxVec3 &scale,
                              std::shared_ptr<IPxrMaterial> material) override {
    throw std::runtime_error("Body creation from mesh is not supported for rendering client");
  };
  IPxrRigidbody *addRigidbody(std::vector<physx::PxVec3> const &vertices,
                              std::vector<physx::PxVec3> const &normals,
                              std::vector<uint32_t> const &indices, const physx::PxVec3 &scale,
                              std::shared_ptr<IPxrMaterial> material) override {
    spdlog::get("SAPIEN")->warn("Body creation from vertices (e.g. debug visuals for collisions) "
                                "is not supported for rendering client");
    return nullptr;
  };

  IPxrRigidbody *addRigidbody(physx::PxGeometryType::Enum type, const physx::PxVec3 &scale,
                              const physx::PxVec3 &color);
  IPxrRigidbody *addRigidbody(std::vector<physx::PxVec3> const &vertices,
                              std::vector<physx::PxVec3> const &normals,
                              std::vector<uint32_t> const &indices, const physx::PxVec3 &scale,
                              const physx::PxVec3 &color) {
    spdlog::get("SAPIEN")->warn("Body creation from vertices (e.g. debug visuals for collisions) "
                                "is not supported for rendering client");
    return nullptr;
  }

  void removeRigidbody(IPxrRigidbody *body) override;

  //========== Camera ==========//
  ClientCamera *addCamera(uint32_t width, uint32_t height, float fovy, float near, float far,
                          std::string const &shaderDir = "") override;
  void removeCamera(ICamera *camera) override {
    throw std::runtime_error("remove camera is not supported for rendering client");
  };
  std::vector<ICamera *> getCameras() override;

  void setAmbientLight(std::array<float, 3> const &color) override;
  std::array<float, 3> getAmbientLight() const override {
    throw std::runtime_error("get ambient light is not supported for rendering client");
  };

  //========== Light ==========//
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
                           float shadowFar, uint32_t shadowMapSize) override {
    throw std::runtime_error("Spot light is not supported for rendering client");
  }
  IActiveLight *addActiveLight(physx::PxTransform const &pose, std::array<float, 3> const &color,
                               float fov, std::string_view texPath, float shadowNear,
                               float shadowFar, uint32_t shadowMapSize) override {
    throw std::runtime_error("Active light is not supported for rendering client");
  };

  void removeLight(ILight *light) override {
    throw std::runtime_error("remove light is not supported for rendering client");
  };

  void updateRender() override;

  void updateRenderAndTakePictures(std::vector<ICamera *> const &cameras) override;

  void destroy() override;

  inline rs_id_t getId() const { return mId; }

  ClientRenderer *getRenderer() { return mRenderer; }

private:
  void syncId();

  ClientRenderer *mRenderer;
  rs_id_t mId;
  std::string mName;
  std::vector<std::unique_ptr<ClientRigidbody>> mBodies;
  std::vector<std::unique_ptr<ClientCamera>> mCameras;
  std::vector<std::unique_ptr<ILight>> mLights;
  bool mIdSynced{false};
};

class ClientRenderer : public IPxrRenderer, public std::enable_shared_from_this<ClientRenderer> {
public:
  ClientRenderer(std::string const &address, uint64_t processIndex);

  ClientScene *createScene(std::string const &name) override;
  void removeScene(IPxrScene *scene) override;
  std::shared_ptr<IPxrMaterial> createMaterial() override;

  virtual std::shared_ptr<IRenderMesh> createMesh(std::vector<float> const &vertices,
                                                  std::vector<uint32_t> const &indices) {
    throw std::runtime_error("Mesh creation is not supported for rendering client");
  };

  proto::RenderService::Stub &getStub() const { return *mStub; }

  inline uint64_t getProcessIndex() const { return mProcessIndex; }

private:
  uint64_t mProcessIndex;
  std::shared_ptr<grpc::Channel> mChannel;
  std::unique_ptr<proto::RenderService::Stub> mStub;

  std::vector<std::unique_ptr<ClientScene>> mScenes;
};

} // namespace server
} // namespace Renderer
} // namespace sapien
