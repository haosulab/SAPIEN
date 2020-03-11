#pragma once
#include <optifuser.h>

namespace sapien {
namespace Renderer {

class FPSCameraController {
public:
  explicit FPSCameraController(Optifuser::CameraSpec *c);
  Optifuser::CameraSpec *camera;
  float yaw = 0.f;
  float pitch = 0.f;
  glm::vec3 forward = {1, 0, 0};
  glm::vec3 up = {0, 0, 1};

  void update();
  void setPosition(float x, float y, float z);
  void rotateYawPitch(float dy, float dp);
  void moveForwardRight(float df, float dr);
  glm::quat getRotation0() const;

  inline void changeCamera(Optifuser::CameraSpec *c) { camera = c; }
};

class ArcRotateCameraController {
public:
  explicit ArcRotateCameraController(Optifuser::CameraSpec *c);
  Optifuser::CameraSpec *camera;
  float yaw = 0.f;
  float pitch = 0.f;
  float r = 1.f;
  glm::vec3 forward = {1, 0, 0};
  glm::vec3 up = {0, 0, 1};
  glm::vec3 center;
  void update();
  void setCenter(float x, float y, float z);
  void rotateYawPitch(float d_yaw, float d_pitch);
  void zoom(float in);
  glm::quat getRotation0() const;

  inline void changeCamera(Optifuser::CameraSpec *c) { camera = c; }
};

} // namespace Renderer
} // namespace sapien
