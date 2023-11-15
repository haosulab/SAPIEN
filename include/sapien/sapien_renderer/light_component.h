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

  template <class Archive> void save(Archive &ar) const {
    ar(cereal::base_class<Component>(this));
    ar(mColor, mShadowEnabled, mShadowNear, mShadowFar, mShadowMapSize, mLocalPose);
  }
  template <class Archive> void load(Archive &ar) {
    ar(cereal::base_class<Component>(this));
    ar(mColor, mShadowEnabled, mShadowNear, mShadowFar, mShadowMapSize, mLocalPose);
  }

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

  template <class Archive> void save(Archive &ar) const {
    ar(cereal::base_class<SapienRenderLightComponent>(this));
  }
  template <class Archive> void load(Archive &ar) {
    ar(cereal::base_class<SapienRenderLightComponent>(this));
  }

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

  template <class Archive> void save(Archive &ar) const {
    ar(cereal::base_class<SapienRenderLightComponent>(this));
    ar(mShadowHalfSize);
  }
  template <class Archive> void load(Archive &ar) {
    ar(cereal::base_class<SapienRenderLightComponent>(this));
    ar(mShadowHalfSize);
  }

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

  template <class Archive> void save(Archive &ar) const {
    ar(cereal::base_class<SapienRenderLightComponent>(this));
    ar(mFovInner);
  }
  template <class Archive> void load(Archive &ar) {
    ar(cereal::base_class<SapienRenderLightComponent>(this));
    ar(mFovOuter);
  }

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

  template <class Archive> void save(Archive &ar) const {
    ar(cereal::base_class<SapienRenderSpotLightComponent>(this));
    ar(mTexture);
  }
  template <class Archive> void load(Archive &ar) {
    ar(cereal::base_class<SapienRenderSpotLightComponent>(this));
    ar(mTexture);
  }

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

  template <class Archive> void save(Archive &ar) const {
    ar(cereal::base_class<SapienRenderLightComponent>(this));
    ar(mHalfWidth, mHalfHeight, mAngle);
  }
  template <class Archive> void load(Archive &ar) {
    ar(cereal::base_class<SapienRenderLightComponent>(this));
    ar(mHalfWidth, mHalfHeight, mAngle);
  }

private:
  float mHalfWidth = 1.f;
  float mHalfHeight = 1.f;
  float mAngle = std::numbers::pi_v<float> / 2.f;

  svulkan2::scene::ParallelogramLight *mParallelogramLight{};
};

} // namespace sapien_renderer
} // namespace sapien

CEREAL_REGISTER_TYPE(sapien::sapien_renderer::SapienRenderLightComponent);
CEREAL_REGISTER_TYPE(sapien::sapien_renderer::SapienRenderPointLightComponent);
CEREAL_REGISTER_TYPE(sapien::sapien_renderer::SapienRenderDirectionalLightComponent)
CEREAL_REGISTER_TYPE(sapien::sapien_renderer::SapienRenderSpotLightComponent);
CEREAL_REGISTER_TYPE(sapien::sapien_renderer::SapienRenderTexturedLightComponent);
CEREAL_REGISTER_TYPE(sapien::sapien_renderer::SapienRenderParallelogramLightComponent);

CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::Component,
                                     sapien::sapien_renderer::SapienRenderLightComponent);
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::sapien_renderer::SapienRenderLightComponent,
                                     sapien::sapien_renderer::SapienRenderPointLightComponent);
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::sapien_renderer::SapienRenderLightComponent,
                                     sapien::sapien_renderer::SapienRenderDirectionalLightComponent)
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::sapien_renderer::SapienRenderLightComponent,
                                     sapien::sapien_renderer::SapienRenderSpotLightComponent);
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::sapien_renderer::SapienRenderSpotLightComponent,
                                     sapien::sapien_renderer::SapienRenderTexturedLightComponent);
CEREAL_REGISTER_POLYMORPHIC_RELATION(sapien::sapien_renderer::SapienRenderLightComponent,
                                     sapien::sapien_renderer::SapienRenderParallelogramLightComponent);
