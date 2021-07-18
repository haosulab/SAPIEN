#include "svulkan2_renderer.h"
#include <svulkan2/resource/material.h>

namespace sapien {
namespace Renderer {

std::string gDefaultViewerShaderDirectory = "";
void setDefaultViewerShaderDirectory(std::string const &dir) {
  gDefaultViewerShaderDirectory = dir;
}

std::string gDefaultCameraShaderDirectory = "";
void setDefaultCameraShaderDirectory(std::string const &dir) {
  gDefaultCameraShaderDirectory = dir;
}

void SVulkan2Renderer::setLogLevel(std::string const &level) {
  if (level == "debug") {
    svulkan2::log::getLogger()->set_level(spdlog::level::debug);
  } else if (level == "info") {
    svulkan2::log::getLogger()->set_level(spdlog::level::info);
  } else if (level == "warn" || level == "warning") {
    svulkan2::log::getLogger()->set_level(spdlog::level::warn);
  } else if (level == "err" || level == "error") {
    svulkan2::log::getLogger()->set_level(spdlog::level::err);
  } else if (level == "critical") {
    svulkan2::log::getLogger()->set_level(spdlog::level::critical);
  } else if (level == "off") {
    svulkan2::log::getLogger()->set_level(spdlog::level::off);
  } else {
    svulkan2::log::getLogger()->error("Invalid log level \"{}\"", level);
  }
}

SVulkan2Material::SVulkan2Material(
    std::shared_ptr<svulkan2::resource::SVMetallicMaterial> material)
    : mMaterial(material) {}
void SVulkan2Material::setBaseColor(std::array<float, 4> color) {
  mMaterial->setBaseColor({color[0], color[1], color[2], color[3]});
}
void SVulkan2Material::setRoughness(float roughness) { mMaterial->setRoughness(roughness); }
void SVulkan2Material::setSpecular(float specular) { mMaterial->setFresnel(specular); }
void SVulkan2Material::setMetallic(float metallic) { mMaterial->setMetallic(metallic); }

SVulkan2Renderer::SVulkan2Renderer(bool offscreenOnly, uint32_t maxNumMaterials,
                                   uint32_t maxNumTextures, uint32_t defaultMipLevels) {
  mContext = std::make_shared<svulkan2::core::Context>(
      VK_API_VERSION_1_1, !offscreenOnly, maxNumMaterials, maxNumTextures, defaultMipLevels);
  mResourceManager = mContext->createResourceManager();
}

SVulkan2Scene *SVulkan2Renderer::createScene(std::string const &name) {
  mScenes.push_back(std::make_unique<SVulkan2Scene>(this, name));
  return mScenes.back().get();
}

void SVulkan2Renderer::removeScene(IPxrScene *scene) {
  mContext->getDevice().waitIdle(); // wait for all render tasks to finish
  mScenes.erase(std::remove_if(mScenes.begin(), mScenes.end(),
                               [scene](auto &s) { return scene == s.get(); }),
                mScenes.end());
}

std::shared_ptr<IPxrMaterial> SVulkan2Renderer::createMaterial() {
  auto mat = std::make_shared<SVulkan2Material>(
      std::make_shared<svulkan2::resource::SVMetallicMaterial>());
  mat->setBaseColor({1.0, 1.0, 1.0, 1});
  return mat;
}

} // namespace Renderer
} // namespace sapien
