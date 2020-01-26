#include "camera_controller.h"

namespace sapien {
namespace Renderer {

FPSCameraController::FPSCameraController(Optifuser::CameraSpec &c) : camera(c) {}

glm::quat FPSCameraController::getRotation0() const {
  glm::mat3 mat = glm::mat3(glm::cross(forward, up), up, -forward);
  return glm::quat(mat);
}

void FPSCameraController::setPosition(float x, float y, float z) {
  camera.position = {x, y, z};
  update();
}

void FPSCameraController::rotateYawPitch(float dy, float dp) {
  yaw += dy;
  pitch += dp;
  if (yaw >= glm::pi<float>()) {
    yaw -= 2 * glm::pi<float>();
  } else if (yaw < -glm::pi<float>()) {
    yaw += 2 * glm::pi<float>();
  }
  pitch = glm::clamp(pitch, -glm::pi<float>() / 2 + 0.05f, glm::pi<float>() / 2 - 0.05f);
  update();
}

void FPSCameraController::moveForwardRight(float df, float dr) {
  glm::mat4 model = camera.getModelMat();
  glm::vec3 forward_global = model * glm::vec4(0, 0, -1, 0);
  glm::vec3 right_global = glm::cross(forward_global, up);
  camera.position += df * forward_global + dr * right_global;
}

void FPSCameraController::update() {
  glm::vec3 right = glm::cross(forward, up);
  camera.setRotation(glm::angleAxis(yaw, up) * glm::angleAxis(pitch, right) * getRotation0());
}

ArcRotateCameraController::ArcRotateCameraController(Optifuser::CameraSpec &c) : camera(c) {}

glm::quat ArcRotateCameraController::getRotation0() const {
  glm::mat3 mat = glm::mat3(glm::cross(forward, up), up, -forward);
  return glm::quat(mat);
}

void ArcRotateCameraController::setCenter(float x, float y, float z) {
  center = {x, y, z};
  update();
}

void ArcRotateCameraController::rotateYawPitch(float d_yaw, float d_pitch) {
  yaw += d_yaw;
  pitch += d_pitch;
  if (yaw >= glm::pi<float>()) {
    yaw -= 2 * glm::pi<float>();
  } else if (yaw < -glm::pi<float>()) {
    yaw += 2 * glm::pi<float>();
  }
  pitch = glm::clamp(pitch, -glm::pi<float>() / 2 + 0.05f, glm::pi<float>() / 2 - 0.05f);
  update();
}

void ArcRotateCameraController::zoom(float in) {
  r -= in;
  update();
}

void ArcRotateCameraController::update() {
  glm::vec3 right = glm::cross(forward, up);
  camera.setRotation(glm::angleAxis(yaw, up) * glm::angleAxis(pitch, right) * getRotation0());
  camera.position = center - r * (camera.getRotation() * glm::vec3({0, 0, -1}));
}

} // namespace Renderer
} // namespace sapien
