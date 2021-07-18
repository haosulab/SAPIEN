//
// Created by jet on 7/18/21.
//

#pragma once
#include "render_interface.h"

namespace sapien::Renderer {
class KuafuRigidBody : public IPxrRigidbody {
public:
  inline void setName(std::string const &name) {};
  inline std::string getName() const { return ""; };

  inline void setUniqueId(uint32_t uniqueId) {};
  inline uint32_t getUniqueId() const { return 0; };
  inline void setSegmentationId(uint32_t segmentationId) {};
  inline uint32_t getSegmentationId() const { return 0; };
  inline void setSegmentationCustomData(std::vector<float> const &customData) {};
  inline void setInitialPose(const physx::PxTransform &transform) {};
  inline void update(const physx::PxTransform &transform) {};

  inline void setVisibility(float visibility) {};
  inline void setVisible(bool visible) {};
  inline void setRenderMode(uint32_t mode) {};

  inline void destroy() {};

  std::vector<std::unique_ptr<RenderShape>> getRenderShapes() const { return {}; };
};

class KuafuScene : public IPxrScene {
};
}