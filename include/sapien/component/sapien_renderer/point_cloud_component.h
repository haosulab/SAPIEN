#pragma once
#include "../component.h"
#include "sapien/array.h"
#include "sapien/component/sapien_renderer/sapien_renderer_system.h"
#include "sapien/math/pose.h"
#include "sapien/serialize.h"
#include <svulkan2/resource/model.h>

namespace sapien {
namespace component {

class PointCloudComponent : public Component {
public:
  PointCloudComponent(uint32_t capacity = 0);

  void setVertices(Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> const &vertices);
  void setAttribute(
      std::string const &name,
      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> const &vertices);

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

} // namespace component
} // namespace sapien
