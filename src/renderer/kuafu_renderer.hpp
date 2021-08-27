//
// Created by jet on 7/18/21.
//

#pragma once
#include "render_interface.h"
#include "kuafu_scene.hpp"
#include "kuafu.hpp"

namespace sapien::Renderer {

// Based on Wavefront MTL
class KuafuMaterial : public IPxrMaterial {
  std::shared_ptr<kuafu::Material> mKMaterial = nullptr;

public:
  KuafuMaterial() { mKMaterial = std::make_shared<kuafu::Material>(); }
  void setBaseColor(std::array<float, 4> color) override;
  void setRoughness(float roughness) override;
  void setSpecular(float specular) override;
  void setMetallic(float metallic) override;

  void setTransparent(bool isTransparent, float ior = 1.4) override;
  void setMaterialType(uint32_t type) override;
  inline auto getKMaterial() { return mKMaterial; }
};

class KuafuRenderer : public IPxrRenderer {
  kuafu::Kuafu mKRenderer;
  KuafuScene mScene;

  bool mUseViewer;

public:
  explicit KuafuRenderer(bool useViewer = false): mUseViewer(useViewer) {};

  IPxrScene *createScene(std::string const &name = "") override;
  void removeScene(IPxrScene *scene) override;
  std::shared_ptr<IPxrMaterial> createMaterial() override;

  void setAssetsPath(std::string const &path);
  void init();

  // TODO: remove these
  inline auto& _getK() { return mKRenderer; }
};
}