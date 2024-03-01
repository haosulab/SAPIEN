#pragma once
#include "image.h"
#include "material.h"
#include "sapien/scene.h"

#include <svulkan2/renderer/renderer.h>
#include <svulkan2/renderer/rt_renderer.h>
#include <svulkan2/renderer/vr.h>

namespace sapien {
namespace sapien_renderer {

class SapienVRDisplay {
public:
  SapienVRDisplay();

  void setScene(std::shared_ptr<Scene> scene);
  void setCameraParameters(float near, float far);

  std::vector<uint32_t> getControllerIds() const;
  Pose getHMDPose() const;
  Pose getControllerPose(uint32_t id) const;

  uint64_t getControllerButtonPressed(uint32_t id) const;
  uint64_t getControllerButtonTouched(uint32_t id) const;
  std::array<float, 2> getControllerAxisState(uint32_t id, uint32_t axis) const;

  Pose getRootPose() const;
  void setRootPose(Pose const &pose);

  void fetchPoses();
  void updateRender();
  void render();

  std::shared_ptr<svulkan2::scene::Scene> getInternalScene() const { return mRenderScene; }

  ~SapienVRDisplay() {}

private:
  std::array<svulkan2::scene::Camera *, 2> getCameras();
  std::shared_ptr<SapienRenderEngine> mEngine;
  std::array<std::shared_ptr<svulkan2::renderer::RendererBase>, 2> mRenderers;
  std::shared_ptr<svulkan2::renderer::VRDisplay> mVR;

  std::shared_ptr<SapienRendererSystem> mRenderSystem;
  std::shared_ptr<svulkan2::scene::Scene> mRenderScene;

  // used for image layout transision
  std::unique_ptr<svulkan2::core::CommandPool> mCommandPool;
  vk::UniqueCommandBuffer mCommandBuffer;
  vk::UniqueFence mSceneRenderFence;

  uint32_t mWidth{};
  uint32_t mHeight{};

  std::array<Pose, 2> mEyePoses;
  Pose mRootPose;
};

} // namespace sapien_renderer
} // namespace sapien
