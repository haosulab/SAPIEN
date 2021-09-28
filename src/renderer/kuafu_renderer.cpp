//
// Created by jet on 7/18/21.
//

#include "kuafu_renderer.hpp"
#include "kuafu_scene.hpp"
#include "kuafu_window.hpp"

#include <kuafu.hpp>
#include <spdlog/spdlog.h>


namespace sapien::Renderer {

void KuafuRenderer::setDefaultAssetsPath(std::string path) {
    kuafu::Config::setDefaultAssetsPath(std::move(path));
}

KuafuRenderer::KuafuRenderer(KuafuConfig conf) {
  auto config = conf.generate();

  std::shared_ptr<kuafu::Camera> camera = nullptr;
  std::shared_ptr<kuafu::Window> window = nullptr;

  if (conf.mUseViewer) {
    camera = std::make_shared<KCamera>(
        "viewer_cam", conf.mViewerWidth, conf.mViewerHeight);
    camera->setPosition({-1, 0, 1});
    camera->setFront({1, 0, -1});

    window = std::make_shared<KWindow>(
        conf.mViewerWidth, conf.mViewerHeight, "Viewer", SDL_WINDOW_RESIZABLE, camera);
  }

  try {
    pKRenderer = std::make_shared<kuafu::Kuafu>(config, window, camera, nullptr);
  } catch (std::exception& e) {
    kuafu::global::logger->critical(e.what());
    throw std::runtime_error(e.what());
  }

  pKRenderer->getScene().setGeometries({});
  pKRenderer->getScene().setGeometryInstances({});
}

IPxrScene *KuafuRenderer::createScene(std::string const &name) {
  mScene.pKRenderer = pKRenderer;
  return &mScene;
};

void KuafuRenderer::setLogLevel(std::string_view level) {
  auto eq = kuafu::utils::iequals;
  if (eq(level, "debug") || eq(level, "all"))
    kuafu::global::logger->set_level(spdlog::level::debug);
  else if (eq(level, "info"))
    kuafu::global::logger->set_level(spdlog::level::info);
  else if (eq(level, "warn") || eq(level, "warning"))
    kuafu::global::logger->set_level(spdlog::level::warn);
  else if (eq(level, "error") || eq(level, "err"))
    kuafu::global::logger->set_level(spdlog::level::err);
  else if (eq(level, "critical") || eq(level, "fatal"))
    kuafu::global::logger->set_level(spdlog::level::critical);
  else if (eq(level, "off"))
    kuafu::global::logger->set_level(spdlog::level::off);
  else
    kuafu::global::logger->error("Invalid log level \"{}\"", level);
}

void KuafuRenderer::removeScene(IPxrScene *scene) {
  spdlog::get("SAPIEN")->warn("KF: removeScene not implemented yet");
};

std::shared_ptr<IPxrMaterial> KuafuRenderer::createMaterial() {
  return std::make_shared<KuafuMaterial>();
};

}