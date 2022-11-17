#pragma once
#include "render_interface.h"
#include <svulkan2/scene/scene.h>

namespace sapien {
namespace Renderer {

class SVulkan2Scene;

class SVulkan2PointBody : public IPxrPointBody {
  std::string mName{};
  SVulkan2Scene *mParentScene = nullptr;
  physx::PxTransform mInitialPose = {{0, 0, 0}, physx::PxIdentity};
  svulkan2::scene::PointObject *mObject;

public:
  SVulkan2PointBody(SVulkan2Scene *scene, svulkan2::scene::PointObject *object);
  SVulkan2PointBody(SVulkan2PointBody const &other) = delete;
  SVulkan2PointBody &operator=(SVulkan2PointBody const &other) = delete;

  inline void setName(std::string const &name) override { mName = name; };
  std::string getName() const override { return mName; };

  void setVisibility(float visibility) override;
  void setRenderMode(uint32_t mode) override;

  void setRenderedVertexCount(uint32_t count) override;
  uint32_t getRenderedVertexCount() override;

  void setAttribute(
      std::string_view name,
      Eigen::Ref<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>) override;

  /** internal use only */
  void destroyVisualObject();
  /** internal use only */
  inline svulkan2::scene::PointObject *const &getVisualObject() const { return mObject; }
  inline SVulkan2Scene *getScene() const { return mParentScene; }

#ifdef SAPIEN_DLPACK
  DLManagedTensor *getDLVertices();
#endif
};

} // namespace Renderer
} // namespace sapien
