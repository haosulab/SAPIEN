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
#include "sapien/sapien_renderer/vr.h"
#include "sapien/sapien_renderer/sapien_renderer_default.h"
#include "sapien/sapien_renderer/sapien_renderer_system.h"
#include <svulkan2/renderer/rt_renderer.h>

namespace sapien {
namespace sapien_renderer {

static Pose Mat4ToPose(glm::mat4 const &m) {
  glm::vec3 scale;
  glm::quat quat;
  glm::vec3 pos;
  glm::vec3 skew;
  glm::vec4 pers;
  glm::decompose(m, scale, quat, pos, skew, pers);
  return Pose({pos.x, pos.y, pos.z}, {quat.w, quat.x, quat.y, quat.z});
}

SapienVRDisplay::SapienVRDisplay() {
  mEngine = SapienRenderEngine::Get();
  auto &renderConfig = SapienRendererDefault::Get();
  auto config = std::make_shared<svulkan2::RendererConfig>();
  config->shaderDir = renderConfig.viewerShaderDirectory;

  mRenderers = {svulkan2::renderer::RendererBase::Create(config),
                svulkan2::renderer::RendererBase::Create(config)};

  mVR = std::make_shared<svulkan2::renderer::VRDisplay>();

  auto [width, height] = mVR->getScreenSize();
  mWidth = width;
  mHeight = height;

  if (mWidth == 0 || mHeight == 0) {
    throw std::runtime_error("VR device reported invalid screen size.");
  }

  for (auto &r : mRenderers) {
    r->resize(width, height);
  }

  mEyePoses = {Mat4ToPose(mVR->getEyePoseLeft()), Mat4ToPose(mVR->getEyePoseRight())};

  mCommandPool = mEngine->getContext()->createCommandPool();
  mCommandBuffer = mCommandPool->allocateCommandBuffer();
  mSceneRenderFence =
      mEngine->getContext()->getDevice().createFenceUnique({vk::FenceCreateFlagBits::eSignaled});
}

std::array<svulkan2::scene::Camera *, 2> SapienVRDisplay::getCameras() {
  if (!mRenderScene) {
    throw std::runtime_error(
        "failed to operate camera, did you forget to call viewer.set_scene ?");
  }
  auto cams = mRenderScene->getCameras();

  svulkan2::scene::Camera *left{};
  svulkan2::scene::Camera *right{};
  for (auto cam : cams) {
    if (cam->getName() == "_vr_controller_left") {
      left = cam;
    }
    if (cam->getName() == "_vr_controller_right") {
      right = cam;
    }
  }

  if (!left) {
    left = &mRenderScene->addCamera();
    left->setName("_vr_controller_left");
    auto frustumLeft = mVR->getCameraFrustumLeft();
    float fx = svulkan2::math::clip2focal(frustumLeft.left, frustumLeft.right, mWidth);
    float fy = svulkan2::math::clip2focal(-frustumLeft.top, -frustumLeft.bottom, mHeight);

    float cx = svulkan2::math::clip2principal(frustumLeft.left, frustumLeft.right, mWidth);
    float cy = svulkan2::math::clip2principal(-frustumLeft.top, -frustumLeft.bottom, mHeight);

    printf("left: %f %f %f %f %d %d\n", fx, fy, cx, cy, mWidth, mHeight);
    left->setPerspectiveParameters(0.05, 50, fx, fy, cx, cy, mWidth, mHeight, 0.f);
  }

  if (!right) {
    right = &mRenderScene->addCamera();
    right->setName("_vr_controller_right");
    auto frustumRight = mVR->getCameraFrustumRight();
    float fx = svulkan2::math::clip2focal(frustumRight.left, frustumRight.right, mWidth);
    float fy = svulkan2::math::clip2focal(-frustumRight.top, -frustumRight.bottom, mHeight);

    float cx = svulkan2::math::clip2principal(frustumRight.left, frustumRight.right, mWidth);
    float cy = svulkan2::math::clip2principal(-frustumRight.top, -frustumRight.bottom, mHeight);

    printf("right: %f %f %f %f %d %d\n", fx, fy, cx, cy, mWidth, mHeight);
    right->setPerspectiveParameters(0.05, 50, fx, fy, cx, cy, mWidth, mHeight, 0.f);
  }

  return {left, right};
}

void SapienVRDisplay::setCameraParameters(float near, float far) {
  for (auto c : getCameras()) {
    c->setPerspectiveParameters(near, far, c->getFx(), c->getFy(), c->getCx(), c->getCy(), mWidth,
                                mHeight, 0.f);
  }
}

void SapienVRDisplay::setScene(std::shared_ptr<Scene> scene) {
  mRenderSystem = scene->getSapienRendererSystem();
  mRenderScene = scene->getSapienRendererSystem()->getScene();
  for (auto r : mRenderers) {
    r->setScene(mRenderScene);
  }
}

std::vector<uint32_t> SapienVRDisplay::getControllerIds() const { return mVR->getControllers(); }

void SapienVRDisplay::fetchPoses() { mVR->updatePoses(); }

Pose SapienVRDisplay::getHMDPose() const {
  return POSE_GL_TO_ROS * Mat4ToPose(mVR->getHMDPose()) * POSE_ROS_TO_GL;
}

Pose SapienVRDisplay::getControllerPose(uint32_t id) const {
  return POSE_GL_TO_ROS * Mat4ToPose(mVR->getControllerPose(id)) * POSE_ROS_TO_GL;
}

Pose SapienVRDisplay::getLeftHandRootPose() {
  return POSE_GL_TO_ROS * Mat4ToPose(mVR->getSkeletalRootPoseLeft()) * POSE_ROS_TO_GL;
}

Pose SapienVRDisplay::getRightHandRootPose() {
  return POSE_GL_TO_ROS * Mat4ToPose(mVR->getSkeletalRootPoseRight()) * POSE_ROS_TO_GL;
}
std::vector<Pose> SapienVRDisplay::getLeftHandSkeletalPoses() {
  auto poses = mVR->getSkeletalDataLeft();
  std::vector<Pose> res;
  res.reserve(poses.size());
  for (auto &p : poses) {
    res.push_back(POSE_GL_TO_ROS * Pose{{p[4], p[5], p[6]}, {p[0], p[1], p[2], p[3]}} *
                  POSE_ROS_TO_GL);
  }
  return res;
}
std::vector<Pose> SapienVRDisplay::getRightHandSkeletalPoses() {
  auto poses = mVR->getSkeletalDataRight();
  std::vector<Pose> res;
  res.reserve(poses.size());
  for (auto &p : poses) {
    res.push_back(POSE_GL_TO_ROS * Pose{{p[4], p[5], p[6]}, {p[0], p[1], p[2], p[3]}} *
                  POSE_ROS_TO_GL);
  }
  return res;
}

uint64_t SapienVRDisplay::getControllerButtonPressed(uint32_t id) const {
  return mVR->getControllerButtonPressed(id);
}
uint64_t SapienVRDisplay::getControllerButtonTouched(uint32_t id) const {
  return mVR->getControllerButtonTouched(id);
}
std::array<float, 2> SapienVRDisplay::getControllerAxisState(uint32_t id, uint32_t axis) const {
  return mVR->getControllerAxis(id, axis);
}

void SapienVRDisplay::updateRender() {
  fetchPoses();
  auto cams = getCameras();
  Pose h2w = mRootPose * POSE_GL_TO_ROS * Mat4ToPose(mVR->getHMDPose());

  Pose left = h2w * mEyePoses[0];
  cams[0]->setPosition({left.p.x, left.p.y, left.p.z});
  cams[0]->setRotation({left.q.w, left.q.x, left.q.y, left.q.z});

  Pose right = h2w * mEyePoses[1];
  cams[1]->setPosition({right.p.x, right.p.y, right.p.z});
  cams[1]->setRotation({right.q.w, right.q.x, right.q.y, right.q.z});

  if (mRenderSystem) {
    mRenderSystem->step();
  }
}

Pose SapienVRDisplay::getRootPose() const { return mRootPose; }
void SapienVRDisplay::setRootPose(Pose const &pose) { mRootPose = pose; }

void SapienVRDisplay::render() {
  if (mEngine->getContext()->getDevice().waitForFences(mSceneRenderFence.get(), VK_TRUE,
                                                       UINT64_MAX) != vk::Result::eSuccess) {
    throw std::runtime_error("failed on wait for fence.");
  }
  mEngine->getContext()->getDevice().resetFences(mSceneRenderFence.get());

  auto cams = getCameras();
  mRenderers[0]->render(*cams[0], {}, {}, {}, {});
  mRenderers[1]->render(*cams[1], {}, {}, {}, {});

  auto &imageLeft = mRenderers[0]->getRenderImage("Color");
  auto &imageRight = mRenderers[1]->getRenderImage("Color");

  // transition image layout
  mCommandBuffer->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
  imageLeft.transitionLayout(
      mCommandBuffer.get(), mRenderers[0]->getRenderTargetImageLayout("Color"),
      vk::ImageLayout::eTransferSrcOptimal, vk::AccessFlagBits::eMemoryWrite,
      vk::AccessFlagBits::eTransferRead, vk::PipelineStageFlagBits::eAllGraphics,
      vk::PipelineStageFlagBits::eTransfer);
  imageRight.transitionLayout(
      mCommandBuffer.get(), mRenderers[1]->getRenderTargetImageLayout("Color"),
      vk::ImageLayout::eTransferSrcOptimal, vk::AccessFlagBits::eMemoryWrite,
      vk::AccessFlagBits::eTransferRead, vk::PipelineStageFlagBits::eAllGraphics,
      vk::PipelineStageFlagBits::eTransfer);
  mCommandBuffer->end();
  mEngine->getContext()->getQueue().submit(mCommandBuffer.get(), mSceneRenderFence.get());

  // present to steam
  mVR->renderFrame(imageLeft, imageRight);
}

} // namespace sapien_renderer
} // namespace sapien
