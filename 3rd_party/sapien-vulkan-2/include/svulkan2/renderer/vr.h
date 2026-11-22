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
#include "svulkan2/common/glm.h"
#include <memory>
#include <vector>
#include <vulkan/vulkan.hpp>

namespace svulkan2 {
namespace core {
class Context;
class Image;
} // namespace core
namespace renderer {

class VRDisplay {
public:
  static void setActionManifestPath(std::string const &path);
  static std::string getActionManifestPath();

public:
  VRDisplay();
  void initDevices();

  std::array<uint32_t, 2> getScreenSize() const;
  glm::mat4 getEyePoseLeft() const;
  glm::mat4 getEyePoseRight() const;

  struct Frustum {
    float left;
    float right;
    float top;
    float bottom;
  };
  Frustum getCameraFrustumLeft() const;
  Frustum getCameraFrustumRight() const;

  std::vector<uint32_t> getControllers();
  glm::mat4 getControllerPose(uint32_t id) const;
  glm::mat4 getHMDPose() const;
  glm::mat4 getSkeletalRootPoseLeft();
  glm::mat4 getSkeletalRootPoseRight();
  std::vector<std::array<float, 8>> getSkeletalDataLeft();
  std::vector<std::array<float, 8>> getSkeletalDataRight();

  uint64_t getControllerButtonPressed(uint32_t id);
  uint64_t getControllerButtonTouched(uint32_t id);
  std::array<float, 2> getControllerAxis(uint32_t id, uint32_t axis);

  void handleInput();
  void updatePoses();
  void renderFrame(core::Image const &left, core::Image const &right);

  ~VRDisplay() {}

private:
  std::shared_ptr<core::Context> mContext;

  uint64_t mActionSetHandle{};
  uint64_t mLeftHandHandle{};
  uint64_t mRightHandHandle{};
};

} // namespace renderer
} // namespace svulkan2