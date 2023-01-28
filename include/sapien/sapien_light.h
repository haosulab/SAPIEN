#pragma once
#include "renderer/render_interface.h"
#include "sapien_actor_base.h"
#include "sapien_entity.h"

namespace sapien {

class SLight : public SEntity {
public:
  physx::PxTransform getPose() const override;
  void setPose(physx::PxTransform const &transform);
  inline physx::PxVec3 getColor() const { return getRendererLight()->getColor(); }
  inline void setColor(physx::PxVec3 color) { getRendererLight()->setColor(color); }
  inline bool getShadowEnabled() const { return getRendererLight()->getShadowEnabled(); }
  inline void setShadowEnabled(bool enabled) {
    return getRendererLight()->setShadowEnabled(enabled);
  }

  inline physx::PxVec3 getPosition() const { return getPose().p; }
  physx::PxVec3 getDirection() const { return getPose().q.rotate({1.f, 0.f, 0.f}); }
  void setPosition(physx::PxVec3 position);
  void setDirection(physx::PxVec3 direction);

  using SEntity::SEntity;
  virtual Renderer::ILight *getRendererLight() const = 0;

  inline physx::PxTransform getLocalPose(PxTransform const &pose) { return mLocalPose; }
  void setLocalPose(PxTransform const &pose);
  inline SActorBase *getParent() const { return mParent; };
  void setParent(SActorBase *actor, bool keepPose = false);

  /** call update to sync light pose to renderer */
  void update();

private:
  PxTransform getParentPose() const;

  PxTransform mLocalPose{PxIdentity};
  SActorBase *mParent{};
};

class SPointLight : public SLight {
public:
  // inline physx::PxVec3 getPosition() const { return getRendererLight()->getPosition(); }
  // inline void setPosition(physx::PxVec3 position) { getRendererLight()->setPosition(position); }
  inline void setShadowParameters(float near, float far) {
    getRendererLight()->setShadowParameters(near, far);
  }
  inline float getShadowNear() const { return getRendererLight()->getShadowNear(); }
  inline float getShadowFar() const { return getRendererLight()->getShadowFar(); }

  inline SPointLight(SScene *scene, Renderer::IPointLight *light) : SLight(scene), mLight(light) {}

private:
  Renderer::IPointLight *getRendererLight() const override { return mLight; }
  Renderer::IPointLight *mLight;
};

class SDirectionalLight : public SLight {
public:
  // inline physx::PxVec3 getDirection() const { return getRendererLight()->getDirection(); }
  // inline void setDirection(physx::PxVec3 direction) {
  //   getRendererLight()->setDirection(direction);
  // }
  inline void setShadowParameters(float halfSize, float near, float far) {
    getRendererLight()->setShadowParameters(halfSize, near, far);
  }

  inline float getShadowHalfSize() const { return getRendererLight()->getShadowHalfSize(); }
  inline float getShadowNear() const { return getRendererLight()->getShadowNear(); }
  inline float getShadowFar() const { return getRendererLight()->getShadowFar(); }

  inline SDirectionalLight(SScene *scene, Renderer::IDirectionalLight *light)
      : SLight(scene), mLight(light) {}

private:
  Renderer::IDirectionalLight *getRendererLight() const override { return mLight; }
  Renderer::IDirectionalLight *mLight;
};

class SSpotLight : public SLight {
public:
  // inline physx::PxVec3 getPosition() const { return getRendererLight()->getPosition(); }
  // inline void setPosition(physx::PxVec3 position) { getRendererLight()->setPosition(position); }
  // inline physx::PxVec3 getDirection() const { return getRendererLight()->getDirection(); }
  // inline void setDirection(physx::PxVec3 direction) {
  //   getRendererLight()->setDirection(direction);
  // }
  inline void setFov(float fov) const { getRendererLight()->setFov(fov); }
  inline float getFov() const { return getRendererLight()->getFov(); }

  inline void setShadowParameters(float near, float far) {
    getRendererLight()->setShadowParameters(near, far);
  }
  inline float getShadowNear() const { return getRendererLight()->getShadowNear(); }
  inline float getShadowFar() const { return getRendererLight()->getShadowFar(); }

  inline SSpotLight(SScene *scene, Renderer::ISpotLight *light) : SLight(scene), mLight(light) {}

private:
  Renderer::ISpotLight *getRendererLight() const override { return mLight; }
  Renderer::ISpotLight *mLight;
};

class SActiveLight : public SLight {
public:
  // inline physx::PxVec3 getPosition() const { return getRendererLight()->getPosition(); }
  // inline void setPosition(physx::PxVec3 position) { getRendererLight()->setPosition(position); }
  inline void setFov(float fov) const { getRendererLight()->setFov(fov); }
  inline float getFov() const { return getRendererLight()->getFov(); }

  inline void setShadowParameters(float near, float far) {
    getRendererLight()->setShadowParameters(near, far);
  }
  inline float getShadowNear() const { return getRendererLight()->getShadowNear(); }
  inline float getShadowFar() const { return getRendererLight()->getShadowFar(); }

  inline SActiveLight(SScene *scene, Renderer::IActiveLight *light)
      : SLight(scene), mLight(light) {}

private:
  Renderer::IActiveLight *getRendererLight() const override { return mLight; }
  Renderer::IActiveLight *mLight;
};

} // namespace sapien
