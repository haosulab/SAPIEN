//
// Created by jet on 9/3/21.
//

#pragma once

#include "render_interface.h"
#include <core/light.hpp>
#include <kuafu_utils.hpp>
namespace sapien::Renderer {

inline void kf_warn_feature_not_available(const std::string& feature) {
  spdlog::get("SAPIEN")->warn("KF: " + feature + " not available");
}

inline glm::mat4 toGlmMat4(physx::PxMat44 mat) {
  glm::mat4 ret;
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      ret[i][j] = mat[i][j];
  return ret;
}

inline physx::PxMat44 toPxMat44(glm::mat4 mat) {
  physx::PxMat44 ret;
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      ret[i][j] = mat[i][j];
  return ret;
}


class IKuafuLight : public ILight {
public:
  inline virtual void _kfRemoveFromScene(void*) = 0;
};

class KuafuPointLight : public IKuafuLight, public IPointLight {
  std::shared_ptr<kuafu::PointLight> mLight;

public:
  explicit KuafuPointLight(std::shared_ptr<kuafu::PointLight> light)
      : mLight(std::move(light)) {};

  // TODO: do these even make sense?
  [[nodiscard]] inline physx::PxTransform getPose() const override {
    return physx::PxTransform(getPosition());
  }
  inline void setPose(physx::PxTransform const &transform) override {
    setPosition(transform.p);
  }

  [[nodiscard]] inline physx::PxVec3 getColor() const override {
    auto color = mLight->color * mLight->strength;
    return {color.r, color.g, color.b};
  }
  inline void setColor(physx::PxVec3 color) override {
    mLight->color = {color[0], color[1], color[2]};
    mLight->strength = 1.0;
  }
  [[nodiscard]] inline bool getShadowEnabled() const override { return true; };
  inline void setShadowEnabled(bool enabled) override {
    kf_warn_feature_not_available("KuafuPointLight::setShadowEnabled");
  };
  [[nodiscard]] inline physx::PxVec3 getPosition() const override {
    auto position = mLight->position;
    return {position.x, position.y, position.z};
  };
  inline void setPosition(physx::PxVec3 position) override {
    mLight->position = {position[0], position[1], position[2]};
  }
  inline void setShadowParameters(float near, float far) override {
    kf_warn_feature_not_available("KuafuPointLight::setShadowParameters");
  };
  [[nodiscard]] inline float getShadowNear() const override {
    kf_warn_feature_not_available("KuafuPointLight::getShadowNear");
    return 0;
  };
  [[nodiscard]] inline float getShadowFar() const override {
    kf_warn_feature_not_available("KuafuPointLight::getShadowFar");
    return 0;
  };

  void _kfRemoveFromScene(void* scene) override {
    auto kfScene = static_cast<kuafu::Scene*>(scene);
    kfScene->removePointLight(mLight);
  }

  // ---------- Non-override ---------- //
  [[maybe_unused]] [[nodiscard]] inline float getSoftness() const { return mLight->radius; }
  [[maybe_unused]] inline void setSoftness(float s) { mLight->radius = s; }
};

class KuafuDirectionalLight : public IKuafuLight, public IDirectionalLight {
  std::shared_ptr<kuafu::DirectionalLight> mLight;

public:
  explicit KuafuDirectionalLight(std::shared_ptr<kuafu::DirectionalLight> light)
      : mLight(std::move(light)) {};

  // TODO: do these even make sense?
  [[nodiscard]] inline physx::PxTransform getPose() const override {
    kf_warn_feature_not_available("KuafuDirectionalLight::getPose");
    return {};
  };
  inline void setPose(physx::PxTransform const &transform) override {
    kf_warn_feature_not_available("KuafuDirectionalLight::setPose");
  };

  [[nodiscard]] inline physx::PxVec3 getColor() const override {
    auto color = mLight->color * mLight->strength;
    return {color.r, color.g, color.b};
  }
  inline void setColor(physx::PxVec3 color) override {
    mLight->color = {color[0], color[1], color[2]};
    mLight->strength = 1.0;
  }
  [[nodiscard]] inline bool getShadowEnabled() const override { return true; };
  inline void setShadowEnabled(bool enabled) override {
    kf_warn_feature_not_available("KuafuDirectionalLight::setShadowEnabled");
  };
  [[nodiscard]] inline physx::PxVec3 getDirection() const override {
      auto direction = mLight->direction;
      return {direction.x, direction.y, direction.z};
  };
  inline void setDirection(physx::PxVec3 d) override {
      mLight->direction = {d[0], d[1], d[2]};
  };
  inline void setShadowParameters(float halfSize, float near, float far) override {
    kf_warn_feature_not_available("KuafuDirectionalLight::setShadowParameters");
  };
  [[nodiscard]] inline float getShadowHalfSize() const override {
    kf_warn_feature_not_available("KuafuDirectionalLight::setShadowParameters");
    return 0;
  };
  [[nodiscard]] inline float getShadowNear() const override {
    kf_warn_feature_not_available("KuafuDirectionalLight::getShadowNear");
    return 0;
  };
  [[nodiscard]] inline float getShadowFar() const override {
    kf_warn_feature_not_available("KuafuDirectionalLight::getShadowFar");
    return 0;
  };

  void _kfRemoveFromScene(void* scene) override {
    auto kfScene = static_cast<kuafu::Scene*>(scene);
    kfScene->removeDirectionalLight();
  }

  // ---------- Non-override ---------- //
  [[maybe_unused]] [[nodiscard]] inline float getSoftness() const { return mLight->softness; }
  [[maybe_unused]] inline void setSoftness(float s) { mLight->softness = s; }
};

class KuafuSpotLight : public IKuafuLight, public ISpotLight {
  std::shared_ptr<kuafu::ActiveLight> mLight;

public:
  explicit KuafuSpotLight(std::shared_ptr<kuafu::ActiveLight> light)
      : mLight(std::move(light)) {};

  [[nodiscard]] inline physx::PxTransform getPose() const override {
    // TODO: check if this is what we want
    return physx::PxTransform(toPxMat44(glm::inverse(mLight->viewMat)));
  };
  inline void setPose(physx::PxTransform const &transform) override {
    // TODO: check if this is what we want
    mLight->viewMat = glm::inverse(toGlmMat4(transform));
  };
  [[nodiscard]] inline physx::PxVec3 getColor() const override {
    auto color = mLight->color * mLight->strength;
    return {color.r, color.g, color.b};
  }
  inline void setColor(physx::PxVec3 color) override {
    mLight->color = {color[0], color[1], color[2]};
    mLight->strength = 1.0;
  }
  [[nodiscard]] inline bool getShadowEnabled() const override { return true; }
  inline void setShadowEnabled(bool enabled) override {
    kf_warn_feature_not_available("KuafuSpotLight::setShadowEnabled");
  };
  [[nodiscard]] inline physx::PxVec3 getPosition() const override { return getPose().p; };
  inline void setPosition(physx::PxVec3 position) override {
      mLight->viewMat = toGlmMat4(physx::PxTransform(position, getPose().q));
  };
  [[nodiscard]] inline physx::PxVec3 getDirection() const override {
      return {};
  };
  inline void setDirection(physx::PxVec3 direction) override {
    // TODO: not good
    auto position = getPosition();
    glm::vec3 p = {position[0], position[1], position[2]};
    glm::vec3 d = {direction[0], direction[1], direction[2]};
    mLight->viewMat = glm::lookAt(p, p + d, kuafu::utils::getPerpendicular(d));
  };
  inline void setShadowParameters(float near, float far) override {
    kf_warn_feature_not_available("KuafuSpotLight::setShadowParameters");
  };
  [[nodiscard]] inline float getShadowNear() const override {
    kf_warn_feature_not_available("KuafuSpotLight::getShadowNear");
    return 0;
  };
  [[nodiscard]] inline float getShadowFar() const override {
    kf_warn_feature_not_available("KuafuSpotLight::getShadowFar");
    return 0;
  };

  inline void setFov(float fov) override { mLight->fov = fov; };
  [[nodiscard]] inline float getFov() const override { return mLight->fov; };

  void _kfRemoveFromScene(void* scene) override {
    auto kfScene = static_cast<kuafu::Scene*>(scene);
    kfScene->removeActiveLight(mLight);
  }

  // ---------- Non-override ---------- //
//  [[maybe_unused]] [[maybe_unused]] [[nodiscard]] inline float getSoftness() const { return mLight->softness; }
//  [[maybe_unused]] [[maybe_unused]] inline void setSoftness(float s) { mLight->softness = s; }
};

class KuafuActiveLight : public IKuafuLight, public IActiveLight {
  std::shared_ptr<kuafu::ActiveLight> mLight;

public:
  explicit KuafuActiveLight(std::shared_ptr<kuafu::ActiveLight> light)
      : mLight(std::move(light)) {};

  [[nodiscard]] inline physx::PxTransform getPose() const override {
    // TODO: check if this is what we want
    return physx::PxTransform(toPxMat44(glm::inverse(mLight->viewMat)));
  };
  inline void setPose(physx::PxTransform const &transform) override {
    // TODO: check if this is what we want
    mLight->viewMat = glm::inverse(toGlmMat4(transform));
  };
  [[nodiscard]] inline physx::PxVec3 getColor() const override {
    auto color = mLight->color * mLight->strength;
    return {color.r, color.g, color.b};
  }
  inline void setColor(physx::PxVec3 color) override {
    mLight->color = {color[0], color[1], color[2]};
    mLight->strength = 1.0;
  }
  [[nodiscard]] inline bool getShadowEnabled() const override { return true; }
  void setShadowEnabled(bool enabled) override {
    kf_warn_feature_not_available("KuafuActiveLight::setShadowEnabled");
  };
  [[nodiscard]] inline physx::PxVec3 getPosition() const override { return getPose().p; };
  inline void setPosition(physx::PxVec3 position) override {
    mLight->viewMat = toGlmMat4(physx::PxTransform(position, getPose().q));
  };

  inline void setFov(float fov) override { mLight->fov = fov; };
  [[nodiscard]] inline float getFov() const override { return mLight->fov; };

  void setTexture(std::string_view path) override {
    // TODO: The texture is uploaded when with the geometries. We need access to
    //       the parent scene here and markGeometriesChanged()
    //       (Change Kuafu makes more sense)
    kf_warn_feature_not_available("KuafuActiveLight::setTexture");
    // mLight->texPath = path;
  };
  inline std::string_view getTexture() override { return mLight->texPath; }

  inline void setShadowParameters(float near, float far) override {
    kf_warn_feature_not_available("KuafuActiveLight::setShadowParameters");
  };
  [[nodiscard]] inline float getShadowNear() const override {
    kf_warn_feature_not_available("KuafuActiveLight::getShadowNear");
    return 0;
  };
  [[nodiscard]] inline float getShadowFar() const override {
    kf_warn_feature_not_available("KuafuActiveLight::getShadowFar");
    return 0;
  };

  void _kfRemoveFromScene(void* scene) override {
    auto kfScene = static_cast<kuafu::Scene*>(scene);
    kfScene->removeActiveLight(mLight);
  }

  // ---------- Non-override ---------- //
//  [[maybe_unused]] [[nodiscard]] inline float getSoftness() const { return mLight->softness; }
//  [[maybe_unused]] inline void setSoftness(float s) { mLight->softness = s; }
};

} // namespace sapien::Renderer
