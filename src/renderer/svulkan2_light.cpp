#include "svulkan2_light.h"

namespace sapien {
namespace Renderer {

// point light
SVulkan2PointLight::SVulkan2PointLight(svulkan2::scene::PointLight &light) : mLight(&light) {}

physx::PxTransform SVulkan2PointLight::getPose() const {
  auto pose = mLight->getTransform();
  return {{pose.position.x, pose.position.y, pose.position.z},
          {pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w}};
}

void SVulkan2PointLight::setPose(physx::PxTransform const &transform) {
  mLight->setTransform(svulkan2::scene::Transform{
      .position = glm::vec3(transform.p.x, transform.p.y, transform.p.z),
      .rotation = glm::quat(transform.q.w, transform.q.x, transform.q.y, transform.q.z)});
}

physx::PxVec3 SVulkan2PointLight::getColor() const {
  auto color = mLight->getColor();
  return {color.x, color.y, color.z};
}

void SVulkan2PointLight::setColor(physx::PxVec3 color) {
  mLight->setColor({color.x, color.y, color.z});
}

bool SVulkan2PointLight::getShadowEnabled() const { return mLight->isShadowEnabled(); }
void SVulkan2PointLight::setShadowEnabled(bool enabled) { mLight->enableShadow(enabled); }

float SVulkan2PointLight::getShadowNear() const { return mLight->getShadowNear(); }
float SVulkan2PointLight::getShadowFar() const { return mLight->getShadowNear(); }

physx::PxVec3 SVulkan2PointLight::getPosition() const {
  auto p = mLight->getPosition();
  return {p.x, p.y, p.z};
}

void SVulkan2PointLight::setPosition(physx::PxVec3 position) {
  mLight->setPosition({position[0], position[1], position[2]});
}

void SVulkan2PointLight::setShadowParameters(float near, float far) {
  mLight->setShadowParameters(near, far);
}

// directional light
SVulkan2DirectionalLight::SVulkan2DirectionalLight(svulkan2::scene::DirectionalLight &light)
    : mLight(&light) {}

physx::PxTransform SVulkan2DirectionalLight::getPose() const {
  auto pose = mLight->getTransform();
  return {{pose.position.x, pose.position.y, pose.position.z},
          {pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w}};
}

void SVulkan2DirectionalLight::setPose(physx::PxTransform const &transform) {
  mLight->setTransform(svulkan2::scene::Transform{
      .position = glm::vec3(transform.p.x, transform.p.y, transform.p.z),
      .rotation = glm::quat(transform.q.w, transform.q.x, transform.q.y, transform.q.z)});
}

physx::PxVec3 SVulkan2DirectionalLight::getColor() const {
  auto color = mLight->getColor();
  return {color.x, color.y, color.z};
}

void SVulkan2DirectionalLight::setColor(physx::PxVec3 color) {
  mLight->setColor({color.x, color.y, color.z});
}

bool SVulkan2DirectionalLight::getShadowEnabled() const { return mLight->isShadowEnabled(); }
void SVulkan2DirectionalLight::setShadowEnabled(bool enabled) { mLight->enableShadow(enabled); }

float SVulkan2DirectionalLight::getShadowHalfSize() const { return mLight->getShadowScaling(); }
float SVulkan2DirectionalLight::getShadowNear() const { return mLight->getShadowNear(); }
float SVulkan2DirectionalLight::getShadowFar() const { return mLight->getShadowNear(); }

physx::PxVec3 SVulkan2DirectionalLight::getDirection() const {
  auto p = mLight->getDirection();
  return {p.x, p.y, p.z};
}

void SVulkan2DirectionalLight::setDirection(physx::PxVec3 direction) {
  mLight->setDirection({direction[0], direction[1], direction[2]});
}

void SVulkan2DirectionalLight::setShadowParameters(float halfSize, float near, float far) {
  mLight->setShadowParameters(halfSize, near, far);
}

// spot light
SVulkan2SpotLight::SVulkan2SpotLight(svulkan2::scene::SpotLight &light) : mLight(&light) {}

physx::PxTransform SVulkan2SpotLight::getPose() const {
  auto pose = mLight->getTransform();
  return {{pose.position.x, pose.position.y, pose.position.z},
          {pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w}};
}

void SVulkan2SpotLight::setPose(physx::PxTransform const &transform) {
  mLight->setTransform(svulkan2::scene::Transform{
      .position = glm::vec3(transform.p.x, transform.p.y, transform.p.z),
      .rotation = glm::quat(transform.q.w, transform.q.x, transform.q.y, transform.q.z)});
}

physx::PxVec3 SVulkan2SpotLight::getColor() const {
  auto color = mLight->getColor();
  return {color.x, color.y, color.z};
}

void SVulkan2SpotLight::setColor(physx::PxVec3 color) {
  mLight->setColor({color.x, color.y, color.z});
}

bool SVulkan2SpotLight::getShadowEnabled() const { return mLight->isShadowEnabled(); }
void SVulkan2SpotLight::setShadowEnabled(bool enabled) { mLight->enableShadow(enabled); }

float SVulkan2SpotLight::getShadowNear() const { return mLight->getShadowNear(); }
float SVulkan2SpotLight::getShadowFar() const { return mLight->getShadowNear(); }

physx::PxVec3 SVulkan2SpotLight::getPosition() const {
  auto p = mLight->getPosition();
  return {p.x, p.y, p.z};
}

void SVulkan2SpotLight::setPosition(physx::PxVec3 position) {
  mLight->setPosition({position[0], position[1], position[2]});
}

void SVulkan2SpotLight::setShadowParameters(float near, float far) {
  mLight->setShadowParameters(near, far);
}

physx::PxVec3 SVulkan2SpotLight::getDirection() const {
  auto p = mLight->getDirection();
  return {p.x, p.y, p.z};
}

void SVulkan2SpotLight::setDirection(physx::PxVec3 direction) {
  mLight->setDirection({direction[0], direction[1], direction[2]});
}

void SVulkan2SpotLight::setFov(float fov) { mLight->setFov(fov); }
float SVulkan2SpotLight::getFov() const { return mLight->getFov(); }

} // namespace Renderer
} // namespace sapien
