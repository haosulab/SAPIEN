//
// Created by jet on 7/18/21.
//

#include "kuafu_renderer.hpp"
#include "kuafu_scene.hpp"

#include <kuafu.hpp>
#include <spdlog/spdlog.h>


namespace sapien::Renderer {

void KuafuRenderer::setDefaultAssetsPath(std::string path) {
    kuafu::Config::setDefaultAssetsPath(std::move(path));
}

KuafuRenderer::KuafuRenderer(KuafuConfig conf) {
  auto config = conf.generate();

  try {
    pKRenderer = std::make_shared<kuafu::Kuafu>(config);
  } catch (std::exception& e) {
    kuafu::global::logger->critical(e.what());
    throw std::runtime_error(e.what());
  }
}

IPxrScene *KuafuRenderer::createScene(std::string const &name) {
  static bool first = true;
  if (first)
    if (!mScenes.empty())
      throw std::runtime_error("???");

  mScenes.emplace_back(new KuafuScene);
  auto newScene = mScenes.back().get();
  newScene->pKRenderer = pKRenderer;

  if (first) {
    newScene->pKScene = pKRenderer->getScene();
    first = false;
  } else
    newScene->pKScene = pKRenderer->createScene();

  return newScene;
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
  scene->destroy();
  mScenes.erase(std::remove_if(mScenes.begin(), mScenes.end(),
                               [scene](auto& s) { return s.get() == scene; }),
                mScenes.end());
};

std::shared_ptr<IPxrMaterial> KuafuRenderer::createMaterial() {
  return std::make_shared<KuafuMaterial>();
};

}