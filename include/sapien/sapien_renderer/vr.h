/*
 * Copyright 2025 Hillbot Inc.
 * Copyright 2020-2024 UCSD SU Lab
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
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

  Pose getLeftHandRootPose();
  Pose getRightHandRootPose();
  std::vector<Pose> getLeftHandSkeletalPoses();
  std::vector<Pose> getRightHandSkeletalPoses();

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
