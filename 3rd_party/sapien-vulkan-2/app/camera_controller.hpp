#pragma once
#include "svulkan2/scene/scene.h"

namespace svulkan2 {

class FPSCameraController {
  scene::Node *mCamera;
  glm::vec3 mRPY{0, 0, 0};
  glm::vec3 mXYZ{0, 0, 0};

  glm::vec3 mForward;
  glm::vec3 mUp;
  glm::vec3 mLeft;

  glm::quat mInitialRotation;

  void update();

public:
  FPSCameraController(scene::Node &node, glm::vec3 const &forward,
                      glm::vec3 const &up);

  void setRPY(float roll, float pitch, float yaw);
  void setXYZ(float x, float y, float z);

  inline glm::vec3 getRPY() const { return mRPY; };
  inline glm::vec3 getXYZ() const { return mXYZ; };

  void move(float forward, float left, float up);
  void rotate(float roll, float pitch, float yaw);
};

class ArcRotateCameraController {
  scene::Node *mCamera;

  glm::vec3 mForward;
  glm::vec3 mUp;
  glm::vec3 mLeft;
  glm::quat mInitialRotation;

  glm::vec3 center{};

  float mYaw{0.f};
  float mPitch{0.f};
  float mR{1.f};

  void update();

public:
  ArcRotateCameraController(scene::Node &node, glm::vec3 const &forward,
                            glm::vec3 const &up);
  void setCenter(float x, float y, float z);
  void rotateYawPitch(float d_yaw, float d_pitch);
  void setYawPitch(float yaw, float pitch);
  void zoom(float in);
  void setRadius(float r);

  inline glm::vec2 getYawPitch() const { return {mYaw, mPitch}; };
  inline glm::vec3 getCenter() const { return center; };
  inline float getRadius() const { return mR; };
};

FPSCameraController::FPSCameraController(scene::Node &node,
                                         glm::vec3 const &forward,
                                         glm::vec3 const &up)
    : mCamera(&node), mForward(glm::normalize(forward)),
      mUp(glm::normalize(up)), mLeft(glm::cross(mUp, mForward)) {
  mInitialRotation = glm::mat3(-mLeft, mUp, -mForward);
}

void FPSCameraController::setRPY(float roll, float pitch, float yaw) {
  mRPY = {roll, pitch, yaw};
  update();
}

void FPSCameraController::setXYZ(float x, float y, float z) {
  mXYZ = {x, y, z};
  update();
}

void FPSCameraController::move(float forward, float left, float up) {
  auto pose = glm::angleAxis(mRPY.z, mUp) * glm::angleAxis(-mRPY.y, mLeft) *
              glm::angleAxis(mRPY.x, mForward);
  mXYZ += pose * mForward * forward + pose * mLeft * left + pose * mUp * up;
  update();
}

void FPSCameraController::rotate(float roll, float pitch, float yaw) {
  mRPY += glm::vec3{roll, pitch, yaw};
  update();
}

void FPSCameraController::update() {
  mRPY.y = std::clamp(mRPY.y, -1.57f, 1.57f);
  if (mRPY.z >= 3.15) {
    mRPY.z -= 2 * glm::pi<float>();
  } else if (mRPY.z <= -3.15) {
    mRPY.z += 2 * glm::pi<float>();
  }

  auto rotation = glm::angleAxis(mRPY.z, mUp) * glm::angleAxis(-mRPY.y, mLeft) *
                  glm::angleAxis(mRPY.x, mForward) * mInitialRotation;
  mCamera->setTransform({.position = mXYZ, .rotation = rotation});
}

ArcRotateCameraController::ArcRotateCameraController(scene::Node &node,
                                                     glm::vec3 const &forward,
                                                     glm::vec3 const &up)
    : mCamera(&node), mForward(glm::normalize(forward)),
      mUp(glm::normalize(up)), mLeft(glm::cross(mUp, mForward)) {
  mInitialRotation = glm::mat3(-mLeft, mUp, -mForward);
}

void ArcRotateCameraController::setCenter(float x, float y, float z) {
  center = {x, y, z};
  update();
}

void ArcRotateCameraController::setYawPitch(float yaw, float pitch) {
  mYaw = yaw;
  mPitch = pitch;
  update();
}

void ArcRotateCameraController::rotateYawPitch(float d_yaw, float d_pitch) {
  mYaw += d_yaw;
  mPitch += d_pitch;
  if (mYaw >= glm::pi<float>()) {
    mYaw -= 2 * glm::pi<float>();
  } else if (mYaw < -glm::pi<float>()) {
    mYaw += 2 * glm::pi<float>();
  }
  mPitch = glm::clamp(mPitch, -glm::pi<float>() / 2 + 0.05f,
                      glm::pi<float>() / 2 - 0.05f);
  update();
}

void ArcRotateCameraController::setRadius(float r) {
  mR = r;
  update();
}

void ArcRotateCameraController::zoom(float in) {
  mR -= in;
  mR = std::max(0.1f, mR);
  mR = std::min(100.f, mR);
  update();
}

void ArcRotateCameraController::update() {
  auto q = glm::angleAxis(mYaw, mUp) * glm::angleAxis(mPitch, mLeft) *
           mInitialRotation;
  mCamera->setTransform(
      {.position = center - mR * (q * glm::vec3({0, 0, -1})), .rotation = q});
}

} // namespace svulkan2
