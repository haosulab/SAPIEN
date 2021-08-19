//
// Created by jet on 7/18/21.
//

#include "kuafu_renderer.hpp"
#include "kuafu_scene.hpp"

#include <kuafu.hpp>


namespace sapien::Renderer {

void KuafuRenderer::setAssetsPath(std::string const &path) {
  mKRenderer.getConfig().setAssetsPath(path);
}

void KuafuRenderer::init() {
  mKRenderer.init();
}

IPxrScene *KuafuRenderer::createScene(std::string const &name) {
  mScene.mKRenderer = &mKRenderer;
  return &mScene;
};

void KuafuRenderer::removeScene(IPxrScene *scene) {
  assert(false); // not implemented
};

std::shared_ptr<IPxrMaterial> KuafuRenderer::createMaterial() {
  assert(false); // not implemented
  return std::make_shared<PxrMaterial>();
};

}