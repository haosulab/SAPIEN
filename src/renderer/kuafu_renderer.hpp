//
// Created by jet on 7/18/21.
//

#pragma once
#include "render_interface.h"
#include "kuafu.hpp"

namespace sapien::Renderer {

class KuafuRenderer : public IPxrRenderer {
  kuafu::Kuafu mInternalRenderer;

public:
  KuafuRenderer();
  IPxrScene *createScene(std::string const &name = "") override;
  void removeScene(IPxrScene *scene) override;
  std::shared_ptr<IPxrMaterial> createMaterial() override;
};
}