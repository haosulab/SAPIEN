//
// Created by jet on 7/18/21.
//

#pragma once
#include "render_interface.h"
#include "kuafu_scene.hpp"
#include "kuafu.hpp"

namespace sapien::Renderer {

class KuafuRenderer : public IPxrRenderer {
  kuafu::Kuafu mKRenderer;
  KuafuScene mScene;

public:
  IPxrScene *createScene(std::string const &name = "") override;
  void removeScene(IPxrScene *scene) override;
  std::shared_ptr<IPxrMaterial> createMaterial() override;

  void setAssetsPath(std::string const &path);
  void init();
};
}