#include "camera_component.h"
#include "client_system.h"
#include <sapien/scene.h>

namespace sapien {
namespace render_server {

ClientCameraComponent::ClientCameraComponent(uint32_t width, uint32_t height,
                                             std::string const &shaderDir)
    : mWidth(width), mHeight(height), mShaderDir(shaderDir) {
  mFx = mFy = height / 2.f / std::tan(std::numbers::pi_v<float> / 4.f);
  mCx = width / 2.f;
  mCy = height / 2.f;
  mNear = 0.01f;
  mFar = 10.f;
  mSkew = 0.f;
}

void ClientCameraComponent::onAddToScene(Scene &scene) {
  auto system = scene.getSystemWithType<ClientSystem>("render_server");

  {
    grpc::ClientContext context;
    proto::AddCameraReq req;
    proto::Id res;
    req.set_scene_id(system->getServerId());
    req.set_width(mWidth);
    req.set_height(mHeight);
    req.set_near(mNear);
    req.set_far(mFar);
    req.set_fx(mFx);
    req.set_fy(mFy);
    req.set_cx(mCx);
    req.set_cy(mCy);
    req.set_shader(mShaderDir);

    auto status = system->getStub().AddCamera(&context, req, &res);
    if (!status.ok()) {
      throw std::runtime_error("failed to add camera to scene: " + status.error_message());
    }
    mServerId = res.id();
  }
}

void ClientCameraComponent::onRemoveFromScene(Scene &scene) {
  // TODO: currently unimplemented
}

void ClientCameraComponent::setLocalPose(Pose const &pose) { mLocalPose = pose; }
Pose ClientCameraComponent::getLocalPose() const { return mLocalPose; }

void ClientCameraComponent::setPerspectiveParameters(float near, float far, float fx, float fy,
                                                     float cx, float cy, float skew) {
  if (getScene()) {
    throw std::runtime_error(
        "failed to set client camera parameters: parameters may not change once added to scene");
  }
  mNear = near;
  mFar = far;
  mFx = fx;
  mFy = fy;
  mCx = cx;
  mCy = cy;
  mSkew = skew;
}

void ClientCameraComponent::takePicture() {
  if (!getScene()) {
    throw std::runtime_error("camera is not added to scene");
  }
  auto system = getScene()->getSystemWithType<ClientSystem>("render_server");

  grpc::ClientContext context;
  proto::TakePictureReq req;
  proto::Empty res;
  req.set_scene_id(system->getServerId());
  req.set_camera_id(mServerId);

  auto status = system->getStub().TakePicture(&context, req, &res);
  if (!status.ok()) {
    throw std::runtime_error("failed to take picture: " + status.error_message());
  }
}

} // namespace render_server
} // namespace sapien
