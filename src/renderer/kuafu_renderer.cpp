//
// Created by jet on 7/18/21.
//

#include "kuafu_renderer.hpp"
#include "kuafu_scene.hpp"
#include "kuafu_window.hpp"

#include <kuafu.hpp>
#include <spdlog/spdlog.h>


namespace sapien::Renderer {

void KuafuMaterial::setBaseColor(std::array<float, 4> color) {
  mKMaterial->kd = glm::vec3(color[0], color[1], color[2]);
  mKMaterial->d = color[3];
}

void KuafuMaterial::setRoughness(float roughness) {
  mKMaterial->roughness = roughness;
}

void KuafuMaterial::setSpecular(float specular) {
//  mKMaterial->ns = specular;
  spdlog::get("SAPIEN")->warn("specular adjustment not supported");
}

void KuafuMaterial::setMetallic(float metallic) {
  mKMaterial->shininess = metallic;
}

void KuafuMaterial::setEmission(std::array<float, 4> color) {
  mKMaterial->emission = glm::vec3(color[0], color[1], color[2]) * color[3];
}

void KuafuMaterial::setTransparent(bool isTransparent, float ior) {
  mKMaterial->illum = isTransparent ? 1 : 2;
  mKMaterial->ior = ior;
}

void KuafuMaterial::setMaterialType(uint32_t type) {
  mKMaterial->illum = type;
}



void KuafuRenderer::setDefaultAssetsPath(std::string path) {
    kuafu::Config::setDefaultAssetsPath(std::move(path));
}


KuafuRenderer::KuafuRenderer(KuafuConfig conf) {
  auto config = conf.generate();

  std::shared_ptr<kuafu::Camera> camera = nullptr;
  std::shared_ptr<kuafu::Window> window = nullptr;

  // TODO: kuafu_urgent (window?)
  if (conf.mUseViewer) {
    camera = std::make_shared<KCamera>(
        "viewer_cam", conf.mWidth, conf.mHeight);
    camera->setPosition({-1, 0, 1});
    camera->setFront({1, 0, -1});

    window = std::make_shared<KWindow>(
        conf.mWidth, conf.mHeight, "Viewer", SDL_WINDOW_RESIZABLE);
  }

  pKRenderer = std::make_shared<kuafu::Kuafu>(config, window, camera, nullptr);

  pKRenderer->getScene().removeEnvironmentMap();
  pKRenderer->getScene().setGeometries({});
  pKRenderer->getScene().setGeometryInstances({});
}

IPxrScene *KuafuRenderer::createScene(std::string const &name) {
  mScene.pKRenderer = pKRenderer;
  return &mScene;
};

void KuafuRenderer::removeScene(IPxrScene *scene) {
  spdlog::get("SAPIEN")->warn("removeScene not implemented yet");
};

std::shared_ptr<IPxrMaterial> KuafuRenderer::createMaterial() {
  return std::make_shared<KuafuMaterial>();
};

}