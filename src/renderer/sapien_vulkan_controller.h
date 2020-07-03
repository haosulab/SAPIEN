#ifdef _USE_VULKAN
#ifdef ON_SCREEN
#pragma once

#include "sapien_vulkan_renderer.h"
#include "sapien_vulkan/camera_controller.h"

#include "sapien_vulkan/gui/gui.h"

#include "hud/hud_control_window.h"
#include "hud/hud_object_window.h"

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
  std::unique_ptr<svulkan::VulkanRendererForEditor> mVulkanRenderer;
  std::unique_ptr<svulkan::FPSCameraController> mFPSController;
  std::unique_ptr<svulkan::ArcRotateCameraController> mArcRotateController;
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

  void selectActor(physx_id_t actorId);
  void focusActor(physx_id_t actorId);
  void viewFromCamera(uint32_t camera);
  void pause(bool p);

  void setFreeCameraPosition(float x, float y, float z);
  void setFreeCameraRotation(float yaw, float pitch, float roll);

  inline void setDefaultControl(bool mouse, bool keyboard) {
    mDefaultMouseClickBehavior = mouse;
    mDefaultKeyPressBehavior = keyboard;
  }

  void close();

 private:
  bool mDefaultMouseClickBehavior {true};
  bool mDefaultKeyPressBehavior {true};

  physx_id_t mSelectedId {0};
  physx_id_t mFocusedId {0};
  uint32_t mCameraView {0};

  // ImGui Modules
  HudControlWindow mHudControlWindow;
  HudObjectWindow mHudObjectWindow;

  glm::mat4 mGizmoTransform {1};
  std::vector<IPxrRigidbody*> mGizmoBody {};
  void editGizmoTransform();
  void createGizmoVisual(SActorBase *actor);

  void editContactVisualization();

 public:
  // input
  bool keyPressed(char k);
  bool keyDown(char k);
  bool mouseClick(int key);
  bool mouseDown(int key);

  std::tuple<int, int> getMousePos();
  std::tuple<float, float> getMouseDelta();
  std::tuple<float, float> getMouseWheelDelta();

};

} // namespace Renderer
} // namespace sapien

#endif
#endif
