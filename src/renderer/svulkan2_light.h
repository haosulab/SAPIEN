#pragma once

#include "render_interface.h"
#include <svulkan2/scene/light.h>
namespace sapien {
namespace Renderer {
class SVulkan2PointLight : public IPointLight {
  svulkan2::scene::PointLight *mLight;

public:
  SVulkan2PointLight(svulkan2::scene::PointLight &light);

  physx::PxTransform getPose() const override;
  void setPose(physx::PxTransform const &transform) override;
  physx::PxVec3 getColor() const override;
  void setColor(physx::PxVec3 color) override;
  bool getShadowEnabled() const override;
  void setShadowEnabled(bool enabled) override;
  physx::PxVec3 getPosition() const override;
  void setPosition(physx::PxVec3 position) override;
  void setShadowParameters(float near, float far) override;

  float getShadowNear() const override;
  float getShadowFar() const override;

  inline svulkan2::scene::PointLight *getInternalLight() const { return mLight; }
};

class SVulkan2DirectionalLight : public IDirectionalLight {
  svulkan2::scene::DirectionalLight *mLight;

public:
  SVulkan2DirectionalLight(svulkan2::scene::DirectionalLight &light);

  physx::PxTransform getPose() const override;
  void setPose(physx::PxTransform const &transform) override;
  physx::PxVec3 getColor() const override;
  void setColor(physx::PxVec3 color) override;
  bool getShadowEnabled() const override;
  void setShadowEnabled(bool enabled) override;
  physx::PxVec3 getDirection() const override;
  void setDirection(physx::PxVec3 position) override;
  void setShadowParameters(float halfSize, float near, float far) override;

  float getShadowHalfSize() const override;
  float getShadowNear() const override;
  float getShadowFar() const override;

  inline svulkan2::scene::DirectionalLight *getInternalLight() const { return mLight; }
};

class SVulkan2SpotLight : public ISpotLight {
  svulkan2::scene::SpotLight *mLight;

public:
  SVulkan2SpotLight(svulkan2::scene::SpotLight &light);

  physx::PxTransform getPose() const override;
  void setPose(physx::PxTransform const &transform) override;
  physx::PxVec3 getColor() const override;
  void setColor(physx::PxVec3 color) override;
  bool getShadowEnabled() const override;
  void setShadowEnabled(bool enabled) override;
  physx::PxVec3 getPosition() const override;
  void setPosition(physx::PxVec3 position) override;
  physx::PxVec3 getDirection() const override;
  void setDirection(physx::PxVec3 position) override;
  void setShadowParameters(float near, float far) override;

  void setFov(float fov) override;
  float getFov() const override;

  float getShadowNear() const override;
  float getShadowFar() const override;

  inline svulkan2::scene::SpotLight *getInternalLight() const { return mLight; }
};

class SVulkan2ActiveLight : public IActiveLight {
  svulkan2::scene::TexturedLight *mLight;

public:
  SVulkan2ActiveLight(svulkan2::scene::TexturedLight &light);

  physx::PxTransform getPose() const override;
  void setPose(physx::PxTransform const &transform) override;
  physx::PxVec3 getColor() const override;
  void setColor(physx::PxVec3 color) override;
  bool getShadowEnabled() const override;
  void setShadowEnabled(bool enabled) override;
  physx::PxVec3 getPosition() const override;
  void setPosition(physx::PxVec3 position) override;
  void setShadowParameters(float near, float far) override;

  void setFov(float fov) override;
  float getFov() const override;

  float getShadowNear() const override;
  float getShadowFar() const override;

  void setTexture(std::string_view path) override;
  std::string_view getTexture() override;

  inline svulkan2::scene::SpotLight *getInternalLight() const { return mLight; }
};

} // namespace Renderer
} // namespace sapien
