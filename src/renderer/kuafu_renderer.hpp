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
  std::shared_ptr<kuafu::NiceMaterial> mKMaterial = nullptr;

public:
  KuafuMaterial() { mKMaterial = std::make_shared<kuafu::NiceMaterial>(); }
  void setBaseColor(std::array<float, 4> color) override;
  void setRoughness(float roughness) override;
  void setSpecular(float specular) override;
  void setMetallic(float metallic) override;

  void setEmission(std::array<float, 4> color) override;
  void setIOR(float ior) override;
  void setTransmission(float transmission) override;
  void setDiffuseTex(std::string_view path) override;

  inline auto getKMaterial() { return mKMaterial; }
};

class KuafuRenderer : public IPxrRenderer {
  std::shared_ptr<kuafu::Kuafu> pKRenderer = nullptr;
  KuafuScene mScene;

public:
  explicit KuafuRenderer(KuafuConfig config = KuafuConfig());

  IPxrScene *createScene(std::string const &name = "") override;
  void removeScene(IPxrScene *scene) override;
  std::shared_ptr<IPxrMaterial> createMaterial() override;

  static void setDefaultAssetsPath(std::string path);

  // TODO: move this function
  void setEnvironmentMap(std::string_view path) {
    pKRenderer->getScene().setEnvironmentMap(path); };
};
}