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
#include "svulkan2/renderer/vr.h"
#include "../common/logger.h"
#include "svulkan2/core/context.h"
#include <openvr.h>

namespace svulkan2 {
namespace renderer {

static vr::IVRSystem *gSystem;
static std::array<vr::TrackedDevicePose_t, vr::k_unMaxTrackedDeviceCount> gTrackedDevicePose{};
static std::array<glm::mat4, vr::k_unMaxTrackedDeviceCount> gDevicePoses{};
static std::array<vr::VRControllerState_t, vr::k_unMaxTrackedDeviceCount> gDeviceState{};

static std::string gActionManifestPath{};
void VRDisplay::setActionManifestPath(std::string const &path) { gActionManifestPath = path; }

std::string VRDisplay::getActionManifestPath() { return gActionManifestPath; }

VRDisplay::VRDisplay() {
  if (!gSystem) {
    gSystem = vr::VRSystem();
  }

  mContext = core::Context::Get();
  if (!gSystem) {
    throw std::runtime_error("VR is not supported");
  }
  initDevices();
}

void VRDisplay::initDevices() {
  vr::TrackedDeviceIndex_t hmd = vr::k_unTrackedDeviceIndexInvalid;
  for (vr::TrackedDeviceIndex_t di = 0; di < vr::k_unMaxTrackedDeviceCount; ++di) {
    switch (gSystem->GetTrackedDeviceClass(di)) {
    case vr::TrackedDeviceClass::TrackedDeviceClass_HMD:
      hmd = di;
      break;
    default:
      break;
    }
  }
  if (hmd != 0) {
    throw std::runtime_error("Failed to find HMD");
  }

  if (gActionManifestPath.length()) {
    logger::info("using action manifest {}", gActionManifestPath);
    auto res = vr::VRInput()->SetActionManifestPath(gActionManifestPath.c_str());
    if (res != vr::EVRInputError::VRInputError_None) {
      logger::error("failed to read action manifest {}", res);
    }
  }

  static_assert(std::is_same<uint64_t, vr::VRActionSetHandle_t>::value);
  static_assert(std::is_same<uint64_t, vr::VRActionHandle_t>::value);
  if (vr::VRInput()->GetActionSetHandle("/actions/global", &mActionSetHandle) !=
      vr::EVRInputError::VRInputError_None) {
    mActionSetHandle = 0;
    logger::error("failed to get action set handle");
  }
  if (vr::VRInput()->GetActionHandle("/actions/global/in/HandSkeletonLeft", &mLeftHandHandle) !=
      vr::EVRInputError::VRInputError_None) {
    mLeftHandHandle = 0;
    logger::error("failed to get action handle");
  }
  if (vr::VRInput()->GetActionHandle("/actions/global/in/HandSkeletonRight", &mRightHandHandle) !=
      vr::EVRInputError::VRInputError_None) {
    mRightHandHandle = 0;
    logger::error("failed to get action handle");
  }
}

std::array<uint32_t, 2> VRDisplay::getScreenSize() const {
  uint32_t width{};
  uint32_t height{};
  gSystem->GetRecommendedRenderTargetSize(&width, &height);
  return {width, height};
}

glm::mat4 VRDisplay::getEyePoseLeft() const {
  vr::HmdMatrix34_t mat = gSystem->GetEyeToHeadTransform(vr::EVREye::Eye_Left);
  return glm::mat4(mat.m[0][0], mat.m[1][0], mat.m[2][0], 0.f, mat.m[0][1], mat.m[1][1],
                   mat.m[2][1], 0.f, mat.m[0][2], mat.m[1][2], mat.m[2][2], 0.f, mat.m[0][3],
                   mat.m[1][3], mat.m[2][3], 1.f);
}

glm::mat4 VRDisplay::getEyePoseRight() const {
  vr::HmdMatrix34_t mat = gSystem->GetEyeToHeadTransform(vr::EVREye::Eye_Right);
  return glm::mat4(mat.m[0][0], mat.m[1][0], mat.m[2][0], 0.f, mat.m[0][1], mat.m[1][1],
                   mat.m[2][1], 0.f, mat.m[0][2], mat.m[1][2], mat.m[2][2], 0.f, mat.m[0][3],
                   mat.m[1][3], mat.m[2][3], 1.f);
}

VRDisplay::Frustum VRDisplay::getCameraFrustumLeft() const {
  VRDisplay::Frustum frustum;
  // NOTE
  // GetProjectionRaw should return left, right, top, bottom
  // But in reality it returns left, right, bottom, top
  // see https://github.com/ValveSoftware/openvr/issues/816
  gSystem->GetProjectionRaw(vr::EVREye::Eye_Left, &frustum.left, &frustum.right, &frustum.bottom,
                            &frustum.top);
  return frustum;
}

VRDisplay::Frustum VRDisplay::getCameraFrustumRight() const {
  VRDisplay::Frustum frustum;
  gSystem->GetProjectionRaw(vr::EVREye::Eye_Right, &frustum.left, &frustum.right, &frustum.bottom,
                            &frustum.top);
  return frustum;
}

std::vector<vr::TrackedDeviceIndex_t> VRDisplay::getControllers() {
  std::vector<vr::TrackedDeviceIndex_t> controllers;
  for (vr::TrackedDeviceIndex_t di = 0; di < vr::k_unMaxTrackedDeviceCount; ++di) {
    switch (gSystem->GetTrackedDeviceClass(di)) {
    case vr::TrackedDeviceClass::TrackedDeviceClass_Controller:
      controllers.push_back(di);
      break;
    default:
      break;
    }
  }
  return controllers;
}

glm::mat4 VRDisplay::getControllerPose(vr::TrackedDeviceIndex_t id) const {
  return gDevicePoses.at(id);
}

glm::mat4 VRDisplay::getHMDPose() const { return gDevicePoses.at(0); }

glm::mat4 VRDisplay::getSkeletalRootPoseLeft() {
  vr::InputPoseActionData_t poseData{};
  vr::VRInput()->GetPoseActionDataForNextFrame(
      mLeftHandHandle, vr::ETrackingUniverseOrigin::TrackingUniverseRawAndUncalibrated, &poseData,
      sizeof(poseData), 0);
  auto &pose = poseData.pose.mDeviceToAbsoluteTracking;

  // clang-format off
  return glm::mat4(pose.m[0][0], pose.m[1][0], pose.m[2][0], 0.0,
                   pose.m[0][1], pose.m[1][1], pose.m[2][1], 0.0,
                   pose.m[0][2], pose.m[1][2], pose.m[2][2], 0.0,
                   pose.m[0][3], pose.m[1][3], pose.m[2][3], 1.0f);
  // clang-format on
}

glm::mat4 VRDisplay::getSkeletalRootPoseRight() {
  vr::InputPoseActionData_t poseData{};
  vr::VRInput()->GetPoseActionDataForNextFrame(
      mRightHandHandle, vr::ETrackingUniverseOrigin::TrackingUniverseRawAndUncalibrated, &poseData,
      sizeof(poseData), 0);
  auto &pose = poseData.pose.mDeviceToAbsoluteTracking;

  // clang-format off
  return glm::mat4(pose.m[0][0], pose.m[1][0], pose.m[2][0], 0.0,
                   pose.m[0][1], pose.m[1][1], pose.m[2][1], 0.0,
                   pose.m[0][2], pose.m[1][2], pose.m[2][2], 0.0,
                   pose.m[0][3], pose.m[1][3], pose.m[2][3], 1.0f);
  // clang-format on
}

std::vector<std::array<float, 8>> VRDisplay::getSkeletalDataLeft() {
  if (!mLeftHandHandle) {
    throw std::runtime_error("failed to get skeletal data.");
  }

  static std::vector<vr::VRBoneTransform_t> transforms;
  vr::InputSkeletalActionData_t data{};
  auto res = vr::VRInput()->GetSkeletalActionData(mLeftHandHandle, &data, sizeof(data));
  if (res != vr::EVRInputError::VRInputError_None) {
    logger::error("failed to get skeletal data {}", res);
  }
  if (data.bActive) {
    uint32_t boneCount{};
    vr::VRInput()->GetBoneCount(mLeftHandHandle, &boneCount);
    transforms.resize(boneCount);
    auto res = vr::VRInput()->GetSkeletalBoneData(
        mLeftHandHandle, vr::EVRSkeletalTransformSpace::VRSkeletalTransformSpace_Model,
        vr::EVRSkeletalMotionRange::VRSkeletalMotionRange_WithoutController, transforms.data(),
        transforms.size());
    if (res != vr::EVRInputError::VRInputError_None) {
      logger::error("get skeletal bone data failed");
    }
    std::vector<std::array<float, 8>> poses;
    poses.reserve(transforms.size());
    for (auto &t : transforms) {
      poses.push_back({t.orientation.w, t.orientation.x, t.orientation.y, t.orientation.z,
                       t.position.v[0], t.position.v[1], t.position.v[2], t.position.v[3]});
    }
    return poses;
  }
  return {};
}

std::vector<std::array<float, 8>> VRDisplay::getSkeletalDataRight() {
  if (!mRightHandHandle) {
    throw std::runtime_error("failed to get skeletal data.");
  }

  static std::vector<vr::VRBoneTransform_t> transforms;
  vr::InputSkeletalActionData_t data{};
  auto res = vr::VRInput()->GetSkeletalActionData(mRightHandHandle, &data, sizeof(data));
  if (res != vr::EVRInputError::VRInputError_None) {
    logger::error("failed to get skeletal data {}", res);
  }
  if (data.bActive) {
    uint32_t boneCount{};
    vr::VRInput()->GetBoneCount(mRightHandHandle, &boneCount);
    transforms.resize(boneCount);
    auto res = vr::VRInput()->GetSkeletalBoneData(
        mRightHandHandle, vr::EVRSkeletalTransformSpace::VRSkeletalTransformSpace_Model,
        vr::EVRSkeletalMotionRange::VRSkeletalMotionRange_WithoutController, transforms.data(),
        transforms.size());
    if (res != vr::EVRInputError::VRInputError_None) {
      logger::error("get skeletal bone data failed");
    }
    std::vector<std::array<float, 8>> poses;
    poses.reserve(transforms.size());
    for (auto &t : transforms) {
      poses.push_back({t.orientation.w, t.orientation.x, t.orientation.y, t.orientation.z,
                       t.position.v[0], t.position.v[1], t.position.v[2], t.position.v[3]});
    }
    return poses;
  }
  return {};
}

static glm::mat4 VRPoseToMat4(vr::HmdMatrix34_t const &pose) {
  // clang-format off
  return glm::mat4(pose.m[0][0], pose.m[1][0], pose.m[2][0], 0.0,
                   pose.m[0][1], pose.m[1][1], pose.m[2][1], 0.0,
                   pose.m[0][2], pose.m[1][2], pose.m[2][2], 0.0,
                   pose.m[0][3], pose.m[1][3], pose.m[2][3], 1.0f);
  // clang-format on
}

void VRDisplay::handleInput() {
  // Process SteamVR controller state
  for (vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount;
       unDevice++) {
    vr::VRControllerState_t state;
    if (gSystem->GetControllerState(unDevice, &state, sizeof(state))) {
      gDeviceState[unDevice] = state;
    }
  }
}

uint64_t VRDisplay::getControllerButtonPressed(vr::TrackedDeviceIndex_t id) {
  return gDeviceState.at(id).ulButtonPressed;
}

uint64_t VRDisplay::getControllerButtonTouched(vr::TrackedDeviceIndex_t id) {
  return gDeviceState.at(id).ulButtonTouched;
}

std::array<float, 2> VRDisplay::getControllerAxis(vr::TrackedDeviceIndex_t id, uint32_t axis) {
  if (axis >= vr::k_unControllerStateAxisCount) {
    throw std::runtime_error("requested controller axis out of range");
  }
  return {gDeviceState.at(id).rAxis[axis].x, gDeviceState.at(id).rAxis[axis].y};
}

void VRDisplay::updatePoses() {

  if (mActionSetHandle) {
    vr::VRActiveActionSet_t actionSet;
    actionSet.ulActionSet = mActionSetHandle;
    actionSet.ulRestrictedToDevice = vr::k_ulInvalidInputValueHandle;
    actionSet.ulSecondaryActionSet = vr::k_ulInvalidActionSetHandle;
    actionSet.nPriority = 0;
    auto res = vr::VRInput()->UpdateActionState(&actionSet, sizeof(actionSet), 1);
    if (res != vr::EVRInputError::VRInputError_None) {
      logger::error("failed to update action state {}", res);
    }
  }

  vr::VRCompositor()->WaitGetPoses(gTrackedDevicePose.data(), vr::k_unMaxTrackedDeviceCount, NULL,
                                   0);
  for (int nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice) {
    if (gTrackedDevicePose[nDevice].bPoseIsValid) {
      gDevicePoses[nDevice] = VRPoseToMat4(gTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking);
    }
  }
}

void VRDisplay::renderFrame(core::Image const &left, core::Image const &right) {
  handleInput();

  if (left.getExtent() != right.getExtent()) {
    throw std::runtime_error("left and right images have different dimensions");
  }
  if (left.getExtent() != right.getExtent()) {
    throw std::runtime_error("left and right images have different dimensions");
  }
  if (left.getSampleCount() != vk::SampleCountFlagBits::e1 ||
      right.getSampleCount() != vk::SampleCountFlagBits::e1) {
    throw std::runtime_error("msaa is not supported");
  }
  if (left.getCurrentLayout(0) != vk::ImageLayout::eTransferSrcOptimal ||
      right.getCurrentLayout(0) != vk::ImageLayout::eTransferSrcOptimal) {
    throw std::runtime_error("image layout must be transfer source.");
  }

  vr::VRTextureBounds_t bounds;
  bounds.uMin = 0.0f;
  bounds.uMax = 1.0f;
  bounds.vMin = 0.0f;
  bounds.vMax = 1.0f;

  vr::VRVulkanTextureData_t vulkanData;
  vulkanData.m_nImage = (uint64_t)VkImage(left.getVulkanImage());
  vulkanData.m_pDevice = mContext->getDevice();
  vulkanData.m_pPhysicalDevice = mContext->getPhysicalDevice();
  vulkanData.m_pInstance = mContext->getInstance();
  vulkanData.m_pQueue = mContext->getQueue().getVulkanQueue();
  vulkanData.m_nQueueFamilyIndex = mContext->getGraphicsQueueFamilyIndex();

  vulkanData.m_nWidth = left.getExtent().width;
  vulkanData.m_nHeight = left.getExtent().height;
  vulkanData.m_nFormat = VkFormat(left.getFormat());
  vulkanData.m_nSampleCount = 1;

  vr::Texture_t texture = {&vulkanData, vr::TextureType_Vulkan, vr::ColorSpace_Auto};
  if (vr::VRCompositor()->Submit(vr::Eye_Left, &texture, &bounds) !=
      vr::EVRCompositorError::VRCompositorError_None) {
    throw std::runtime_error("failed to submit");
  }

  vulkanData.m_nImage = (uint64_t)VkImage(right.getVulkanImage());
  if (vr::VRCompositor()->Submit(vr::Eye_Right, &texture, &bounds) !=
      vr::EVRCompositorError::VRCompositorError_None) {
    throw std::runtime_error("failed to submit");
  }
}

} // namespace renderer
} // namespace svulkan2