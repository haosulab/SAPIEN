//
// Created by jet on 7/18/21.
//

#pragma once
#include "kuafu.hpp"
#include "kuafu_config.hpp"
#include "kuafu_scene.hpp"
#include "render_interface.h"

namespace sapien::Renderer {

// Based on Wavefront MTL
class KuafuMaterial : public IPxrMaterial {
  kuafu::NiceMaterial mKMaterial;

public:
  KuafuMaterial(kuafu::NiceMaterial m = {}) : mKMaterial(std::move(m)) {}
  inline void setBaseColor(std::array<float, 4> color) override {
    mKMaterial.diffuseColor = glm::vec3(color[0], color[1], color[2]);
    mKMaterial.alpha = color[3];
  }
  [[nodiscard]] inline std::array<float, 4> getBaseColor() const override {
    return {mKMaterial.diffuseColor.r, mKMaterial.diffuseColor.g, mKMaterial.diffuseColor.b,
            mKMaterial.alpha};
  }
  inline void setRoughness(float roughness) override { mKMaterial.roughness = roughness; }
  [[nodiscard]] inline float getRoughness() const override { return mKMaterial.roughness; }
  inline void setSpecular(float specular) override { mKMaterial.specular = specular; }
  [[nodiscard]] inline float getSpecular() const override { return mKMaterial.specular; }
  inline void setMetallic(float metallic) override { mKMaterial.metallic = metallic; }
  [[nodiscard]] inline float getMetallic() const override { return mKMaterial.metallic; }

  inline void setEmission(std::array<float, 4> color) override {
    mKMaterial.emission = glm::vec3(color[0], color[1], color[2]);
    mKMaterial.emissionStrength = color[3];
  }
  [[nodiscard]] inline std::array<float, 4> getEmission() const override {
    return {mKMaterial.emission.r, mKMaterial.emission.g, mKMaterial.emission.b,
            mKMaterial.emissionStrength};
  }
  inline void setIOR(float ior) override { mKMaterial.ior = ior; }
  [[nodiscard]] inline float getIOR() const override { return mKMaterial.ior; }
  inline void setTransmission(float t) override { mKMaterial.transmission = t; }
  [[nodiscard]] inline float getTransmission() const override { return mKMaterial.transmission; }
  inline void setDiffuseTextureFromFilename(std::string_view path) override {
    mKMaterial.diffuseTexPath = path;
  }
  [[nodiscard]] inline std::string getDiffuseTextureFilename() const override {
    return mKMaterial.diffuseTexPath;
  }
  inline void setMetallicTextureFromFilename(std::string_view path) override {
    mKMaterial.metallicTexPath = path;
  }
  [[nodiscard]] inline std::string getMetallicTextureFilename() const override {
    return mKMaterial.metallicTexPath;
  }
  inline void setRoughnessTextureFromFilename(std::string_view path) override {
    mKMaterial.roughnessTexPath = path;
  }
  [[nodiscard]] inline std::string getRoughnessTextureFilename() const override {
    return mKMaterial.roughnessTexPath;
  }
  inline void setTransmissionTextureFromFilename(std::string_view path) override {
    mKMaterial.transmissionTexPath = path;
  }
  [[nodiscard]] inline std::string getTransmissionTextureFilename() const override {
    return mKMaterial.transmissionTexPath;
  }

  [[nodiscard]] inline const kuafu::NiceMaterial &getKMaterial() const { return mKMaterial; }
};

class KuafuRenderer : public IPxrRenderer {
  std::shared_ptr<kuafu::Kuafu> pKRenderer = nullptr;
  std::vector<std::unique_ptr<KuafuScene>> mScenes;

public:
  explicit KuafuRenderer(KuafuConfig config = KuafuConfig());

  IPxrScene *createScene(std::string const &name) override;
  void removeScene(IPxrScene *scene) override;
  std::shared_ptr<IPxrMaterial> createMaterial() override;
  std::shared_ptr<IRenderMesh> createMesh(std::vector<float> const &vertices,
                                          std::vector<uint32_t> const &indices) override {
    throw std::runtime_error("KuafuRenderer::createMesh is not implemented");
  }

  static void setDefaultAssetsPath(std::string path);
  static void setLogLevel(std::string_view level);
  inline bool isRunning() { return pKRenderer->isRunning(); }
};
} // namespace sapien::Renderer
