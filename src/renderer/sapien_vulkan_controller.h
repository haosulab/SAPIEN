#ifdef _USE_VULKAN
#ifdef ON_SCREEN
#pragma once

#include "sapien_vulkan_renderer.h"
#include "sapien_vulkan/camera_controller.h"

#include "sapien_vulkan/gui/gui.h"

#include "hud/hud_control_window.hpp"

namespace sapien {
class Simulation;
class SScene;
class SActorBase;

namespace Renderer {

class SapienVulkanController {
  SapienVulkanRenderer *mRenderer{};
  SScene *mScene{};
  // SapienVulkanScene *mScene{};

  std::unique_ptr<svulkan::Camera> mCamera;
  std::unique_ptr<svulkan::VulkanRenderer> mVulkanRenderer;
  std::unique_ptr<svulkan::FPSCameraController> mFPSController;
  std::unique_ptr<svulkan::VulkanWindow> mWindow;

  uint32_t mWidth;
  uint32_t mHeight;

  vk::UniqueFence mFence;
  vk::UniqueSemaphore mSemaphore;
  vk::UniqueCommandBuffer mCommandBuffer;

 public:
  explicit SapienVulkanController(SapienVulkanRenderer *renderer);
  void render();
  inline void setScene(SScene * scene) { mScene = scene; }
  inline bool isClosed() const { return mWindow->isClosed(); };

 private:
  // ImGui Modules
  HudControlWindow mHudControlWindow;
};

} // namespace Renderer
} // namespace sapien

#endif
#endif
