#pragma once
#include "renderer/render_interface.h"
#include "sapien_entity.h"

namespace sapien {

class SEntityParticle : public SEntity {
public:
  SEntityParticle(SScene *scene, Renderer::IRenderPointBody *renderBody);

  PxTransform getPose() const override { return {PxVec3(0, 0, 0), physx::PxIdentity}; }

  SEntityParticle &operator=(SEntityParticle const &other) = delete;
  SEntityParticle &operator=(SEntityParticle &&other) = delete;
  SEntityParticle(SEntityParticle const &other) = delete;
  SEntityParticle(SEntityParticle &&other) = delete;

  inline Renderer::IRenderPointBody *getVisualBody() { return mRenderBody; };

private:
  Renderer::IRenderPointBody *mRenderBody{};
};

} // namespace sapien
