#ifdef _USE_VULKAN
#include "sapien_vulkan_renderer.h"

namespace sapien {
namespace Renderer {

SapienVulkanRenderer::SapienVulkanRenderer(bool offscreenOnly) {
  mContext = std::make_unique<svulkan::VulkanContext>(!offscreenOnly);
  setLogLevel("warn");
}

SapienVulkanScene *SapienVulkanRenderer::createScene(std::string const &name) {
  mScenes.push_back(std::make_unique<SapienVulkanScene>(this, name));
  return mScenes.back().get();
}

void SapienVulkanRenderer::removeScene(IPxrScene *scene) {
  mContext->getDevice().waitIdle(); // wait for all render tasks to finish
  mScenes.erase(std::remove_if(mScenes.begin(), mScenes.end(),
                               [scene](auto &s) { return scene == s.get(); }),
                mScenes.end());
}

void SapienVulkanRenderer::setLogLevel(std::string const &level) {
  if (level == "debug") {
    svulkan::log::getLogger()->set_level(spdlog::level::debug);
  } else if (level == "info") {
    svulkan::log::getLogger()->set_level(spdlog::level::info);
  } else if (level == "warn" || level == "warning") {
    svulkan::log::getLogger()->set_level(spdlog::level::warn);
  } else if (level == "err" || level == "error") {
    svulkan::log::getLogger()->set_level(spdlog::level::err);
  } else {
    svulkan::log::getLogger()->error("Invalid log level \"{}\"", level);
  }
}

} // namespace Renderer
} // namespace sapien
#endif
