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
#pragma once
#include "../component.h"
#include "material.h"
#include "sapien/math/pose.h"
#include <numbers>
#include <svulkan2/scene/light.h>

namespace sapien {
class Entity;
namespace sapien_renderer {

// TODO: make sure certain parameters cannot be modified
class SapienRenderLightComponent : public Component {
public:
  Vec3 getColor() const { return mColor; }
  virtual void setColor(Vec3 color);

  bool getShadowEnabled() const { return mShadowEnabled; }
  void setShadowEnabled(bool enabled) { mShadowEnabled = enabled; }
  void enableShadow() { setShadowEnabled(true); }
  void disableShadow() { setShadowEnabled(false); }

  float getShadowNear() const { return mShadowNear; }
  void setShadowNear(float near) { mShadowNear = near; }

  float getShadowFar() const { return mShadowFar; }
  void setShadowFar(float far) { mShadowFar = far; }

  uint32_t getShadowMapSize() const { return mShadowMapSize; }
  void setShadowMapSize(uint32_t size) { mShadowMapSize = size; }

  void setLocalPose(Pose const &);
  Pose getLocalPose() const;
  Pose getGlobalPose() const;

  virtual void internalUpdate() = 0;

protected:
  Vec3 mColor{1.f, 1.f, 1.f};
  bool mShadowEnabled{true};
  float mShadowNear{0.01f};
  float mShadowFar{10.f};
  uint32_t mShadowMapSize{2048};
  Pose mLocalPose{};
};

class SapienRenderPointLightComponent : public SapienRenderLightComponent {
public:
  void onAddToScene(Scene &scene) override;
  void onRemoveFromScene(Scene &scene) override;

  void internalUpdate() override;
  void setColor(Vec3 color) override;

private:
  svulkan2::scene::PointLight *mPointLight{};
};

class SapienRenderDirectionalLightComponent : public SapienRenderLightComponent {
public:
  float getShadowHalfSize() const { return mShadowHalfSize; }
  void setShadowHalfSize(float size) { mShadowHalfSize = size; }

  void onAddToScene(Scene &scene) override;
  void onRemoveFromScene(Scene &scene) override;

  void internalUpdate() override;
  void setColor(Vec3 color) override;

private:
  svulkan2::scene::DirectionalLight *mDirectionalLight{};
  float mShadowHalfSize{10.f};
};

class SapienRenderSpotLightComponent : public SapienRenderLightComponent {
public:
  float getFovInner() const { return mFovInner; }
  void setFovInner(float fov) { mFovInner = fov; }
  float getFovOuter() const { return mFovOuter; }
  void setFovOuter(float fov) { mFovOuter = fov; }

  void onAddToScene(Scene &scene) override;
  void onRemoveFromScene(Scene &scene) override;

  void internalUpdate() override;
  void setColor(Vec3 color) override;

protected:
  float mFovInner{0.f};
  float mFovOuter{0.f};
  svulkan2::scene::SpotLight *mSpotLight{};
};

class SapienRenderTexturedLightComponent : public SapienRenderSpotLightComponent {

public:
  void onAddToScene(Scene &scene) override;
  void onRemoveFromScene(Scene &scene) override;

  void setTexture(std::shared_ptr<SapienRenderTexture2D> texture) { mTexture = texture; }
  std::shared_ptr<SapienRenderTexture2D> getTexture() const { return mTexture; }

  void internalUpdate() override;

private:
  svulkan2::scene::TexturedLight *getLight() const {
    return static_cast<svulkan2::scene::TexturedLight *>(mSpotLight);
  }
  std::shared_ptr<SapienRenderTexture2D> mTexture;
};

class SapienRenderParallelogramLightComponent : public SapienRenderLightComponent {
public:
  void onAddToScene(Scene &scene) override;
  void onRemoveFromScene(Scene &scene) override;

  void setShape(float halfWidth, float halfHeight, float angle);
  float getHalfWidth() const { return mHalfWidth; }
  float getHalfHeight() const { return mHalfHeight; }
  float getAngle() const { return mAngle; }

  void internalUpdate() override;
  void setColor(Vec3 color) override;

private:
  float mHalfWidth = 1.f;
  float mHalfHeight = 1.f;
  float mAngle = std::numbers::pi_v<float> / 2.f;

  svulkan2::scene::ParallelogramLight *mParallelogramLight{};
};

} // namespace sapien_renderer
} // namespace sapien
