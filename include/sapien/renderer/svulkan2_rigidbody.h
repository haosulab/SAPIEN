#pragma once
#include "render_interface.h"
#include <svulkan2/scene/scene.h>

namespace sapien {
namespace Renderer {

class SVulkan2Scene;

class SVulkan2Rigidbody : public IPxrRigidbody {
  std::string mName{};
  SVulkan2Scene *mParentScene = nullptr;
  physx::PxTransform mInitialPose = {{0, 0, 0}, physx::PxIdentity};
  std::vector<svulkan2::scene::Object *> mObjects;

  uint32_t mUniqueId{0};
  uint32_t mSegmentationId{0};

  physx::PxGeometryType::Enum mType;
  physx::PxVec3 mScale;

public:
  SVulkan2Rigidbody(SVulkan2Scene *scene, std::vector<svulkan2::scene::Object *> const &objects,
                    physx::PxGeometryType::Enum type, physx::PxVec3 scale);
  SVulkan2Rigidbody(SVulkan2Rigidbody const &other) = delete;

  SVulkan2Rigidbody &operator=(SVulkan2Rigidbody const &other) = delete;

  inline void setName(std::string const &name) override { mName = name; };
  std::string getName() const override { return mName; };

  void setUniqueId(uint32_t uniqueId) override;
  uint32_t getUniqueId() const override;
  void setSegmentationId(uint32_t segmentationId) override;
  uint32_t getSegmentationId() const override;
  void setSegmentationCustomData(const std::vector<float> &customData) override;
  void setInitialPose(const physx::PxTransform &transform) override;
  inline physx::PxTransform getInitialPose() const override { return mInitialPose; };
  void update(const physx::PxTransform &transform) override;

  void setVisibility(float visibility) override;
  void setVisible(bool visible) override;
  void setRenderMode(uint32_t mode) override;
  void setShadeFlat(bool shadeFlat) override;
  bool getShadeFlat() override;

  void destroy() override;

  physx::PxGeometryType::Enum getType() const override { return mType; }
  physx::PxVec3 getScale() const override;

  /** internal use only */
  void destroyVisualObjects();
  /** internal use only */
  inline std::vector<svulkan2::scene::Object *> const &getVisualObjects() const {
    return mObjects;
  }

  std::vector<std::shared_ptr<IPxrRenderShape>> getRenderShapes() override;

  inline SVulkan2Scene *getScene() const { return mParentScene; }
};

} // namespace Renderer
} // namespace sapien
