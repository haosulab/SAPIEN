#pragma once
#include "../component.h"
#include "sapien/array.h"
#include "sapien_renderer_system.h"
#include "sapien/math/pose.h"
#include "sapien/serialize.h"
#include <svulkan2/resource/model.h>

namespace sapien {
namespace sapien_renderer {

class PointCloudComponent : public Component {
public:
  PointCloudComponent(uint32_t capacity = 0);

  std::shared_ptr<PointCloudComponent>
  setVertices(Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &vertices);
  std::shared_ptr<PointCloudComponent> setAttribute(
      std::string const &name,
      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> const &attribute);

  inline std::shared_ptr<svulkan2::resource::SVPointSet> const &getPointSet() const {
    return mPointSet;
  }

  void onAddToScene(Scene &scene) override;
  void onRemoveFromScene(Scene &scene) override;

  void internalUpdate();

  CudaArrayHandle getCudaArray() const;

  PointCloudComponent(PointCloudComponent const &) = delete;
  PointCloudComponent &operator=(PointCloudComponent const &) = delete;
  PointCloudComponent(PointCloudComponent const &&) = delete;
  PointCloudComponent &operator=(PointCloudComponent const &&) = delete;

private:
  svulkan2::scene::Transform getTransform() const;

  std::shared_ptr<SapienRenderEngine> mEngine;
  Pose mLocalPose{};
  std::shared_ptr<svulkan2::resource::SVPointSet> mPointSet;
  svulkan2::scene::PointObject *mObject{};
  Vec3 mScale{1.f};
};

} // namespace sapien_renderer
} // namespace sapien
