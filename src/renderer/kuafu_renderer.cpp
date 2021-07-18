//
// Created by jet on 7/18/21.
//

#include "kuafu_renderer.hpp"
#include "kuafu_scene.hpp"

#include <kuafu.hpp>


namespace sapien::Renderer {

KuafuRenderer::KuafuRenderer() { mInternalRenderer.init(); }

IPxrScene *KuafuRenderer::createScene(std::string const &name) { return static_cast<KuafuScene*>(nullptr); };
void KuafuRenderer::removeScene(IPxrScene *scene) {};
std::shared_ptr<IPxrMaterial> KuafuRenderer::createMaterial() { return std::make_shared<PxrMaterial>(); };

}