#pragma once
#include "proto/render_server.grpc.pb.h"
#include <sapien/component/component.h>
#include <numbers>

namespace sapien {
namespace render_server {

class ClientCameraComponent : public component::Component {
public:
  ClientCameraComponent(uint32_t width, uint32_t height, std::string const &shaderDir);

  void onAddToScene(Scene &scene) override;
  void onRemoveFromScene(Scene &scene) override;

  void setLocalPose(Pose const &);
  Pose getLocalPose() const;

  void setPerspectiveParameters(float near, float far, float fx, float fy, float cx, float cy,
                                float skew);

  void takePicture();

private:
  float mWidth;
  float mHeight;
  std::string mShaderDir;

  float mNear;
  float mFar;
  float mFx;
  float mFy;
  float mCx;
  float mCy;
  float mSkew;

  uint64_t mId{};

  Pose mLocalPose;
};

} // namespace render_server
} // namespace sapien
