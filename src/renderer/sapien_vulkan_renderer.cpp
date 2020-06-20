#ifdef _USE_VULKAN
#include "sapien_vulkan_renderer.h"

namespace sapien {
namespace Renderer {

SapienVulkanRenderer::SapienVulkanRenderer() {
  mContext = std::make_unique<svulkan::VulkanContext>();
}

SapienVulkanScene *SapienVulkanRenderer::createScene(std::string const &name) {
  mScenes.push_back(std::make_unique<SapienVulkanScene>(this, name));
  return mScenes.back().get();
}

void SapienVulkanRenderer::removeScene(IPxrScene *scene) {
  mContext->getDevice().waitIdle();  // wait for all render tasks to finish
  mScenes.erase(std::remove_if(mScenes.begin(), mScenes.end(),
                               [scene](auto &s) { return scene == s.get(); }), mScenes.end());
}

}
} // namespace sapien
#endif
