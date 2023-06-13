#pragma once
#include "renderer/render_interface.h"
#include "sapien_entity.h"

namespace sapien {

class SEntityDeformable : public SEntity {
public:
  SEntityDeformable(SScene *scene, Renderer::IRenderBody *renderBody);

  PxTransform getPose() const override { return {PxVec3(0, 0, 0), physx::PxIdentity}; }

  SEntityDeformable &operator=(SEntityDeformable const &other) = delete;
  SEntityDeformable &operator=(SEntityDeformable &&other) = delete;
  SEntityDeformable(SEntityDeformable const &other) = delete;
  SEntityDeformable(SEntityDeformable &&other) = delete;

  inline Renderer::IRenderBody *getVisualBody() { return mRenderBody; };

private:
  Renderer::IRenderBody *mRenderBody{};
};

} // namespace sapien
