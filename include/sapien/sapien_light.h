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
  virtual void update();

protected:
  PxTransform getParentPose() const;
  PxTransform mLocalPose{PxIdentity};
  SActorBase *mParent{};
};

class SPointLight : public SLight {
public:
  inline void setShadowParameters(float near, float far) {
    getRendererLight()->setShadowParameters(near, far);
  }
  inline float getShadowNear() const { return getRendererLight()->getShadowNear(); }
  inline float getShadowFar() const { return getRendererLight()->getShadowFar(); }

  inline SPointLight(SScene *scene, Renderer::IPointLight *light) : SLight(scene), mLight(light) {}

  Renderer::IPointLight *getRendererLight() const override { return mLight; }

private:
  Renderer::IPointLight *mLight;
};

class SDirectionalLight : public SLight {
public:
  inline void setShadowParameters(float halfSize, float near, float far) {
    getRendererLight()->setShadowParameters(halfSize, near, far);
  }

  inline float getShadowHalfSize() const { return getRendererLight()->getShadowHalfSize(); }
  inline float getShadowNear() const { return getRendererLight()->getShadowNear(); }
  inline float getShadowFar() const { return getRendererLight()->getShadowFar(); }

  inline SDirectionalLight(SScene *scene, Renderer::IDirectionalLight *light)
      : SLight(scene), mLight(light) {}

  Renderer::IDirectionalLight *getRendererLight() const override { return mLight; }

private:
  Renderer::IDirectionalLight *mLight;
};

class SSpotLight : public SLight {
public:
  inline void setFov(float fov) const { getRendererLight()->setFov(fov); }
  inline float getFov() const { return getRendererLight()->getFov(); }

  inline void setShadowParameters(float near, float far) {
    getRendererLight()->setShadowParameters(near, far);
  }
  inline float getShadowNear() const { return getRendererLight()->getShadowNear(); }
  inline float getShadowFar() const { return getRendererLight()->getShadowFar(); }

  inline SSpotLight(SScene *scene, Renderer::ISpotLight *light) : SLight(scene), mLight(light) {}

  Renderer::ISpotLight *getRendererLight() const override { return mLight; }

private:
  Renderer::ISpotLight *mLight;
};

class SActiveLight : public SLight {
public:
  inline void setFov(float fov) const { getRendererLight()->setFov(fov); }
  inline float getFov() const { return getRendererLight()->getFov(); }

  inline void setShadowParameters(float near, float far) {
    getRendererLight()->setShadowParameters(near, far);
  }
  inline float getShadowNear() const { return getRendererLight()->getShadowNear(); }
  inline float getShadowFar() const { return getRendererLight()->getShadowFar(); }

  inline SActiveLight(SScene *scene, Renderer::IActiveLight *light)
      : SLight(scene), mLight(light) {}

  Renderer::IActiveLight *getRendererLight() const override { return mLight; }

private:
  Renderer::IActiveLight *mLight;
};

class SParallelogramLight : public SLight {
public:
  inline void setShape(physx::PxVec2 edge0, physx::PxVec2 edge1) {
    mEdge0 = edge0;
    mEdge1 = edge1;
    mLight->setShape({0, edge0.x, edge0.y}, {0, edge1.x, edge1.y});
  }
  inline physx::PxVec2 getEdge0() const { return mEdge0; }
  inline physx::PxVec2 getEdge1() const { return mEdge1; }

  inline SParallelogramLight(SScene *scene, Renderer::IParallelogramLight *light)
      : SLight(scene), mLight(light) {}

  Renderer::IParallelogramLight *getRendererLight() const override { return mLight; }
  void update() override;

private:
  PxVec2 mEdge0;
  PxVec2 mEdge1;
  Renderer::IParallelogramLight *mLight;
};

} // namespace sapien
