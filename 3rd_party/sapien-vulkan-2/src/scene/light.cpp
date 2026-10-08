/*
 * Copyright 2025 Hillbot Inc.
 * Copyright 2020-2024 UCSD SU Lab
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "svulkan2/scene/light.h"
#include "svulkan2/scene/scene.h"

namespace svulkan2 {
namespace scene {

std::array<glm::mat4, 6> PointLight::getModelMatrices(glm::vec3 const &center) {
  const glm::vec3 x(1, 0, 0);
  const glm::vec3 y(0, 1, 0);
  const glm::vec3 z(0, 0, 1);

  glm::mat4 m1 = glm::mat3(glm::mat3(z, -y, x));
  m1[3] = glm::vec4(center, 1.f);

  glm::mat4 m2 = glm::mat3(glm::mat3(-z, -y, -x));
  m2[3] = glm::vec4(center, 1.f);

  glm::mat4 m3 = glm::mat3(glm::mat3(-x, z, y));
  m3[3] = glm::vec4(center, 1.f);

  glm::mat4 m4 = glm::mat3(glm::mat3(-x, -z, -y));
  m4[3] = glm::vec4(center, 1.f);

  glm::mat4 m5 = glm::mat3(glm::mat3(-x, -y, z));
  m5[3] = glm::vec4(center, 1.f);

  glm::mat4 m6 = glm::mat3(glm::mat3(x, -y, -z));
  m6[3] = glm::vec4(center, 1.f);

  return {m1, m2, m3, m4, m5, m6};
}

PointLight::PointLight(std::string const &name) : Node(name) {}
void PointLight::enableShadow(bool enable) {
  mCastShadow = enable;
  mScene->reorderLights();
}

void PointLight::setShadowParameters(float near, float far, uint32_t size) {
  mShadowNear = near;
  mShadowFar = far;
  mShadowMapSize = size;
}

glm::mat4 PointLight::getShadowProjectionMatrix() const {
  auto mat = math::perspective(glm::pi<float>() / 2.f, 1.f, mShadowNear, mShadowFar);
  return mat;
}

DirectionalLight::DirectionalLight(std::string const &name) : Node(name){};

void DirectionalLight::enableShadow(bool enable) {
  mCastShadow = enable;
  mScene->reorderLights();
}

glm::vec3 DirectionalLight::getDirection() const {
  return glm::mat3(mTransform.rotation) * glm::vec3(0, 0, -1);
}

void DirectionalLight::setDirection(glm::vec3 const &dir) {
  auto z = -glm::normalize(dir);
  glm::vec3 x(1, 0, 0);
  glm::vec3 y;
  if (std::abs(glm::dot(z, x)) < 0.95) {
    y = glm::normalize(glm::cross(z, x));
  } else {
    y = glm::normalize(glm::cross(z, glm::vec3(0, 1, 0)));
  }
  x = glm::cross(y, z);
  mTransform.rotation = glm::quat(glm::mat3(x, y, z));
}

void DirectionalLight::setShadowParameters(float near, float far, float scaling, uint32_t size) {
  mShadowNear = near;
  mShadowFar = far;
  mShadowScaling = scaling;
  mShadowMapSize = size;
}

glm::mat4 DirectionalLight::getShadowProjectionMatrix() const {
  return math::ortho(-mShadowScaling, mShadowScaling, -mShadowScaling, mShadowScaling, mShadowNear,
                     mShadowFar);
}

SpotLight::SpotLight(std::string const &name) : Node(name){};
void SpotLight::enableShadow(bool enable) {
  mCastShadow = enable;
  mScene->reorderLights();
}

void SpotLight::setShadowParameters(float near, float far, uint32_t size) {
  mShadowNear = near;
  mShadowFar = far;
  mShadowMapSize = size;
}

void SpotLight::setDirection(glm::vec3 const &dir) {
  auto z = -glm::normalize(dir);
  glm::vec3 x(1, 0, 0);
  glm::vec3 y;
  if (std::abs(glm::dot(z, x)) < 0.95) {
    y = glm::normalize(glm::cross(z, x));
  } else {
    y = glm::normalize(glm::cross(z, glm::vec3(0, 1, 0)));
  }
  x = glm::cross(y, z);
  mTransform.rotation = glm::quat(glm::mat3(x, y, z));
}

glm::vec3 SpotLight::getDirection() const {
  return glm::mat3(mTransform.rotation) * glm::vec3(0, 0, -1);
}

void SpotLight::setFov(float fov) { mFov = fov; }
float SpotLight::getFov() const { return mFov; }

void SpotLight::setFovSmall(float fov) { mFovSmall = fov; }
float SpotLight::getFovSmall() const { return mFovSmall; }

glm::mat4 SpotLight::getShadowProjectionMatrix() const {
  return math::perspective(mFov, 1.f, mShadowNear, mShadowFar);
}

void TexturedLight::setTexture(std::shared_ptr<resource::SVTexture> texture) {
  mTexture = texture;
  mScene->updateVersion();
}

ParallelogramLight::ParallelogramLight(std::string const &name) : Node(name) {}

void ParallelogramLight::setShape(glm::vec2 halfSize, float theta) {
  mHalfSize = halfSize;
  mAngle = theta;
}

glm::vec3 ParallelogramLight::getOrigin() const {
  return glm::rotate(getRotation(), {-mHalfSize.x, -mHalfSize.y, 0.f}) + getPosition();
}

glm::vec3 ParallelogramLight::getEdgeX() const {
  return glm::rotate(getRotation(), {mHalfSize.x * 2.f, 0.f, 0.f});
}

glm::vec3 ParallelogramLight::getEdgeY() const {
  return glm::rotate(getRotation(), {glm::cos(mAngle) * mHalfSize.y * 2.f,
                                     glm::sin(mAngle) * mHalfSize.y * 2.f, 0.f});
}

} // namespace scene
} // namespace svulkan2