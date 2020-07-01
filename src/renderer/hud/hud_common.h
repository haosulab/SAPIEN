#pragma once
#include <PxPhysicsAPI.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/norm.hpp>
#include <imgui.h>

namespace sapien {
namespace Renderer {
static inline void PrintPose(physx::PxTransform const &pose) {
  glm::vec3 angles = glm::eulerAngles(glm::quat(pose.q.w, pose.q.x, pose.q.y, pose.q.z)) /
                     glm::pi<float>() * 180.f;

  ImGui::Text("Position: %.4g %.4g %.4g", pose.p.x, pose.p.y, pose.p.z);
  ImGui::Text("Quat (wxyz): %.4g %.4g %.4g %.4g", pose.q.w, pose.q.x, pose.q.y, pose.q.z);
  ImGui::Text("Euler (degree): %.4g %.4g %.4g", angles.x, angles.y, angles.z);
};

static inline void PrintPose(glm::vec3 position, glm::quat rotation) {
  glm::vec3 angles = glm::eulerAngles(rotation) / glm::pi<float>() * 180.f;

  ImGui::Text("Position: %.4g %.4g %.4g", position.x, position.y, position.z);
  ImGui::Text("Quat (wxyz): %.4g %.4g %.4g %.4g", rotation.w, rotation.x, rotation.y, rotation.z);
  ImGui::Text("Euler (degree): %.4g %.4g %.4g", angles.x, angles.y, angles.z);
};
} // namespace Renderer
} // namespace sapien
