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
#include "sapien/sapien_renderer/camera_component.h"
#include "../logger.h"
#include "sapien/entity.h"
#include "sapien/math/conversion.h"
#include "sapien/sapien_renderer/sapien_renderer_default.h"
#include "sapien/sapien_renderer/sapien_renderer_system.h"
#include "sapien/sapien_renderer/texture.h"
#include "sapien/scene.h"
#include <numbers>
#include <svulkan2/renderer/renderer.h>
#include <svulkan2/renderer/renderer_base.h>

#ifdef near
#undef near
#endif
#ifdef far
#undef far
#endif

namespace sapien {
namespace sapien_renderer {

struct SapienRenderCameraInternal {
  uint32_t mWidth;
  uint32_t mHeight;
  std::shared_ptr<SapienRenderEngine> mEngine;
  std::shared_ptr<svulkan2::scene::Scene> mScene;
  vk::UniqueSemaphore mSemaphore;
  uint64_t mFrameCounter{0};
  std::unique_ptr<svulkan2::renderer::RendererBase> mRenderer;
  svulkan2::scene::Camera *mCamera;

  // CPU image buffer
  std::unordered_map<std::string, std::vector<char>> mImageBuffers;

  // GPU image buffer
#ifdef SAPIEN_CUDA
  std::unordered_map<std::string, std::shared_ptr<svulkan2::core::Buffer>> mCudaImageBuffers;
#endif
  std::unique_ptr<svulkan2::core::CommandPool> mCommandPool;
  vk::UniqueCommandBuffer mCommandBuffer;

  SapienRenderCameraInternal(uint32_t width, uint32_t height, std::string const &shaderDir,
                             std::shared_ptr<svulkan2::scene::Scene> scene) {
    mWidth = width;
    mHeight = height;

    mEngine = SapienRenderEngine::Get();
    mScene = scene;

    auto &renderConfig = SapienRendererDefault::Get();
    auto config = std::make_shared<svulkan2::RendererConfig>();
    switch (renderConfig.msaa) {
    case 1:
      config->msaa = vk::SampleCountFlagBits::e1;
      break;
    case 2:
      config->msaa = vk::SampleCountFlagBits::e2;
      break;
    case 4:
      config->msaa = vk::SampleCountFlagBits::e4;
      break;
    case 8:
      config->msaa = vk::SampleCountFlagBits::e8;
      break;
    default:
      throw std::runtime_error("MSAA count must be one of [1, 2, 4, 8]");
    }

    for (auto &[name, format] : renderConfig.renderTargetFormats) {
      config->textureFormat[name] = format;
    }

    config->shaderDir = shaderDir.length() ? shaderDir : renderConfig.cameraShaderDirectory;
    mRenderer = svulkan2::renderer::RendererBase::Create(config);
    mRenderer->resize(width, height);
    mCamera = &scene->addCamera();
    mRenderer->setScene(scene);

    if (auto rtRenderer = dynamic_cast<svulkan2::renderer::RTRenderer *>(mRenderer.get())) {
      rtRenderer->setCustomProperty("spp", renderConfig.rayTracingSamplesPerPixel);
      rtRenderer->setCustomProperty("maxDepth", renderConfig.rayTracingPathDepth);
      rtRenderer->setCustomProperty("aperture", renderConfig.rayTracingDoFAperture);
      rtRenderer->setCustomProperty("focusPlane", renderConfig.rayTracingDoFPlane);
      if (renderConfig.rayTracingRussianRouletteMinBounces >= 0) {
        rtRenderer->setCustomProperty("russianRoulette", 1);
        rtRenderer->setCustomProperty("russianRouletteMinBounces",
                                      renderConfig.rayTracingRussianRouletteMinBounces);
      } else {
        rtRenderer->setCustomProperty("russianRoulette", 0);
      }
      rtRenderer->enableDenoiser(renderConfig.rayTracingDenoiserType, "HdrColor", "Albedo",
                                 "Normal");
    }
  }

  void waitForRender() {
    if (!mSemaphore) {
      return;
    }
    auto result = mEngine->getContext()->getDevice().waitSemaphores(
        vk::SemaphoreWaitInfo({}, mSemaphore.get(), mFrameCounter), UINT64_MAX);
    if (result != vk::Result::eSuccess) {
      throw std::runtime_error("take picture failed: wait for fence failed");
    }
  }

  void takePicture() {
    if (!mSemaphore) {
      mSemaphore = mEngine->getContext()->createTimelineSemaphore(mFrameCounter);
    }
    waitForRender();
    mFrameCounter++;
    mRenderer->render(*mCamera, {}, {}, {}, mSemaphore.get(), mFrameCounter);
  }

  std::vector<std::string> getImageNames() const { return mRenderer->getRenderTargetNames(); }

  SapienRenderImageCpu getImage(std::string const &name) {
    waitForRender();
    auto &image = mRenderer->getRenderImage(name);
    vk::Format format = image.getFormat();
    uint32_t formatSize = svulkan2::getFormatSize(format);
    size_t size = mWidth * mHeight * formatSize;
    mImageBuffers[name].resize(size);
    image.download(mImageBuffers[name].data(), size);
    return SapienRenderImageCpu(mWidth, mHeight, format, mImageBuffers[name].data());
  }

  SapienRenderImageCuda getImageCuda(std::string const &name) {
#ifdef SAPIEN_CUDA
    waitForRender();

    auto context = mEngine->getContext();
    if (!mCommandPool) {
      mCommandPool = context->createCommandPool();
      mCommandBuffer = mCommandPool->allocateCommandBuffer();
    }

    mCommandBuffer->reset();
    mCommandBuffer->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});

    auto &image = mRenderer->getRenderImage(name);
    auto extent = image.getExtent();
    vk::Format format = image.getFormat();
    vk::DeviceSize size =
        extent.width * extent.height * extent.depth * svulkan2::getFormatSize(format);

    if (!mCudaImageBuffers.contains(name)) {
      mCudaImageBuffers[name] = svulkan2::core::Buffer::Create(
          size, vk::BufferUsageFlagBits::eTransferSrc | vk::BufferUsageFlagBits::eTransferDst,
          VMA_MEMORY_USAGE_GPU_ONLY, VmaAllocationCreateFlags{}, true);
    }
    auto buffer = mCudaImageBuffers.at(name);
    image.recordCopyToBuffer(mCommandBuffer.get(), buffer->getVulkanBuffer(), 0, size, {0, 0, 0},
                             extent);
    mCommandBuffer->end();
    context->getQueue().submitAndWait(mCommandBuffer.get());

    return SapienRenderImageCuda(mWidth, mHeight, format, buffer->getCudaPtr(),
                                 buffer->getCudaDeviceId());
#else
    throw std::runtime_error("sapien is not copmiled with CUDA support");
#endif
  }

  svulkan2::renderer::RendererBase &getRenderer() const { return *mRenderer; }
  svulkan2::scene::Camera &getCamera() const { return *mCamera; }

  ~SapienRenderCameraInternal() { mScene->removeNode(*mCamera); }
};

void SapienRenderCameraComponent::setProperty(std::string const &name, int property) {
  mProperties[name] = property;
  if (mCamera) {
    mCamera->mRenderer->setCustomProperty(name, property);
  }
}
void SapienRenderCameraComponent::setProperty(std::string const &name, float property) {
  mProperties[name] = property;
  if (mCamera) {
    mCamera->mRenderer->setCustomProperty(name, property);
  }
}
void SapienRenderCameraComponent::setTexture(std::string const &name,
                                             std::shared_ptr<SapienRenderTexture> texture) {
  if (!mCamera) {
    throw std::runtime_error("camera texture is only available for cameras added to scene");
  }
  mCamera->mRenderer->setCustomTexture(name, texture->getTexture());
}
void SapienRenderCameraComponent::setTextureArray(
    std::string const &name, std::vector<std::shared_ptr<SapienRenderTexture>> texture) {
  if (!mCamera) {
    throw std::runtime_error("camera texture is only available for cameras added to scene");
  }

  std::vector<std::shared_ptr<svulkan2::resource::SVTexture>> ts;
  for (auto &t : texture) {
    ts.push_back(t->getTexture());
  }
  mCamera->mRenderer->setCustomTextureArray(name, ts);
}

SapienRenderCameraComponent::SapienRenderCameraComponent(uint32_t width, uint32_t height,
                                                         std::string const &shaderDir)
    : mWidth(width), mHeight(height) {
  mFx = mFy = height / 2.f / std::tan(std::numbers::pi_v<float> / 4.f);
  mCx = width / 2.f;
  mCy = height / 2.f;
  mNear = 0.01f;
  mFar = 10.f;
  mSkew = 0.f;
  mShaderDir = shaderDir;
}

void SapienRenderCameraComponent::setAutoUpload(bool enable) {
  auto scene = getScene();
  if (!scene) {
    throw std::runtime_error("The camera needs to be added to scene.");
  }

  auto system = scene->getSapienRendererSystem();
  if (auto r = dynamic_cast<svulkan2::renderer::Renderer *>(&mCamera->getRenderer())) {
    // TODO: do it for rt renderer
    r->setAutoUploadEnabled(enable);
  }
}

svulkan2::core::Image &SapienRenderCameraComponent::getInternalImage(std::string const &name) {
  if (!mGpuInitialized) {
    throw std::runtime_error("The camera needs to be initialized");
  }
  return mCamera->getRenderer().getRenderImage(name);
}

svulkan2::renderer::RendererBase &SapienRenderCameraComponent::getInternalRenderer() {
  return mCamera->getRenderer();
}

svulkan2::scene::Camera &SapienRenderCameraComponent::getInternalCamera() {
  return mCamera->getCamera();
}

void SapienRenderCameraComponent::onAddToScene(Scene &scene) {
  auto system = scene.getSapienRendererSystem();
  mCamera = std::make_unique<SapienRenderCameraInternal>(getWidth(), getHeight(), mShaderDir,
                                                         system->getScene());
  mCamera->mCamera->setPerspectiveParameters(mNear, mFar, mFx, mFy, mCx, mCy, mWidth, mHeight,
                                             mSkew);

  for (auto &[k, v] : mProperties) {
    if (std::holds_alternative<int>(v)) {
      mCamera->mRenderer->setCustomProperty(k, std::get<int>(v));
    } else {
      mCamera->mRenderer->setCustomProperty(k, std::get<float>(v));
    }
  }
  system->registerComponent(
      std::static_pointer_cast<SapienRenderCameraComponent>(shared_from_this()));
}

void SapienRenderCameraComponent::onRemoveFromScene(Scene &scene) {
  auto system = scene.getSapienRendererSystem();
  mCamera = nullptr;
  system->unregisterComponent(
      std::static_pointer_cast<SapienRenderCameraComponent>(shared_from_this()));
}

void SapienRenderCameraComponent::takePicture() {
  if (!mCamera) {
    throw std::runtime_error("failed to take picture: the camera is not added to scene");
  }
  mCamera->takePicture();
  mUpdatedWithoutTakingPicture = false;
}

std::vector<std::string> SapienRenderCameraComponent::getImageNames() const {
  if (!mCamera) {
    throw std::runtime_error("failed to get image names: the camera is not added to scene");
  }
  return mCamera->getImageNames();
}

SapienRenderImageCpu SapienRenderCameraComponent::getImage(std::string const &name) {
  if (!mCamera) {
    throw std::runtime_error("failed to get image: the camera is not added to scene");
  }
  if (mUpdatedWithoutTakingPicture) {
    logger::warn("getting picture without taking picture since last camera update");
  }
  auto image = mCamera->getImage(name);
  return image;
}

SapienRenderImageCuda SapienRenderCameraComponent::getImageCuda(std::string const &name) {
#ifdef SAPIEN_CUDA
  if (!mCamera) {
    throw std::runtime_error("failed to get image: the camera is not added to scene");
  }
  if (mUpdatedWithoutTakingPicture) {
    logger::warn("getting picture without taking picture since last camera update");
  }
  auto image = mCamera->getImageCuda(name);
  return image;
#else
  throw std::runtime_error("sapien is not copmiled with CUDA support");
#endif
}

static Mat4 mat4glm2eigen(glm::mat4 const &mat) {
  return Eigen::Map<const Eigen::Matrix<float, 4, 4, Eigen::ColMajor>>(&mat[0][0], 4, 4);
}

// getters
Mat4 SapienRenderCameraComponent::getModelMatrix() const {
  return PoseToEigenMat4(getGlobalPose() * POSE_GL_TO_ROS);
}
Mat4 SapienRenderCameraComponent::getProjectionMatrix() const {
  switch (mMode) {
  case CameraMode::ePerspective:
    return mat4glm2eigen(
        svulkan2::math::fullPerspective(mNear, mFar, mFx, mFy, mCx, mCy, mWidth, mHeight, mSkew));
  case CameraMode::eOrthographic:
    return mat4glm2eigen(svulkan2::math::ortho(mLeft, mRight, mBottom, mTop, mNear, mFar));
  }
  throw std::runtime_error("corrupted camera mode");
}
Mat3 SapienRenderCameraComponent::getIntrinsicMatrix() const {
  checkMode(CameraMode::ePerspective);
  Mat3 mat = Mat3::Identity();
  mat(0, 0) = getFocalLengthX();
  mat(1, 1) = getFocalLengthY();
  mat(0, 2) = getPrincipalPointX();
  mat(1, 2) = getPrincipalPointY();
  mat(0, 1) = getSkew();
  return mat;

  // TODO: support ortho, check if the following is right
  // Mat3 mat = Mat3::Identity();
  // mat(0, 0) = getWidth() / (getOrthoRight() - getOrthoLeft());
  // mat(1, 1) = getHeight() / (getOrthoTop() - getOrthoBottom());
  // mat(0, 2) = getOrthoLeft() / (getOrthoLeft() - getOrthoRight());
  // mat(1, 2) = getOrthoTop() / (getOrthoTop() - getOrthoBottom());
}
Mat34 SapienRenderCameraComponent::getExtrinsicMatrix() const {
  Mat34 ros2cv;
  ros2cv << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0;
  return ros2cv * PoseToEigenMat4(getGlobalPose().getInverse());
}

// setters
void SapienRenderCameraComponent::setPerspectiveParameters(float near, float far, float fx,
                                                           float fy, float cx, float cy,
                                                           float skew) {
  mMode = CameraMode::ePerspective;
  mNear = near;
  mFar = far;
  mFx = fx;
  mFy = fy;
  mCx = cx;
  mCy = cy;
  mSkew = skew;
  if (mCamera) {
    mCamera->mCamera->setPerspectiveParameters(mNear, mFar, mFx, mFy, mCx, mCy, mWidth, mHeight,
                                               mSkew);
  }
}

void SapienRenderCameraComponent::setOrthographicParameters(float near, float far, float top) {
  float aspect = mWidth / mHeight;
  setOrthographicParameters(near, far, -top * aspect, top * aspect, -top, top);
}

void SapienRenderCameraComponent::setOrthographicParameters(float near, float far, float left,
                                                            float right, float bottom, float top) {
  mMode = CameraMode::eOrthographic;
  mNear = near;
  mFar = far;
  mLeft = left;
  mRight = right;
  mBottom = bottom;
  mTop = top;
  if (mCamera) {
    mCamera->mCamera->setOrthographicParameters(near, far, left, right, bottom, top, mWidth,
                                                mHeight);
  }
}

void SapienRenderCameraComponent::setFocalLengths(float fx, float fy) {
  checkMode(CameraMode::ePerspective);
  setPerspectiveParameters(getNear(), getFar(), fx, fy, getPrincipalPointX(), getPrincipalPointY(),
                           getSkew());
}
void SapienRenderCameraComponent::setFovX(float fovx, bool computeY) {
  checkMode(CameraMode::ePerspective);
  float fx = getWidth() / 2.f / std::tan(fovx / 2);
  float fy = computeY ? fx : getFocalLengthY();
  setFocalLengths(fx, fy);
}

void SapienRenderCameraComponent::setFovY(float fovy, bool computeX) {
  checkMode(CameraMode::ePerspective);
  float fy = getHeight() / 2.f / std::tan(fovy / 2);
  float fx = computeX ? fy : getFocalLengthX();
  setFocalLengths(fx, fy);
}
void SapienRenderCameraComponent::setNear(float near) {
  checkMode(CameraMode::ePerspective); // TODO: implement for ortho
  setPerspectiveParameters(near, getFar(), getFocalLengthX(), getFocalLengthY(),
                           getPrincipalPointX(), getPrincipalPointY(), getSkew());
}
void SapienRenderCameraComponent::setFar(float far) {
  checkMode(CameraMode::ePerspective); // TODO: implement for ortho
  setPerspectiveParameters(getNear(), far, getFocalLengthX(), getFocalLengthY(),
                           getPrincipalPointX(), getPrincipalPointY(), getSkew());
}
void SapienRenderCameraComponent::setPrincipalPoint(float cx, float cy) {
  checkMode(CameraMode::ePerspective);
  setPerspectiveParameters(getNear(), getFar(), getFocalLengthX(), getFocalLengthY(), cx, cy,
                           getSkew());
}
void SapienRenderCameraComponent::setSkew(float s) {
  checkMode(CameraMode::ePerspective);
  setPerspectiveParameters(getNear(), getFar(), getFocalLengthX(), getFocalLengthY(),
                           getPrincipalPointX(), getPrincipalPointY(), s);
}

void SapienRenderCameraComponent::setLocalPose(Pose const &pose) { mLocalPose = pose; }
Pose SapienRenderCameraComponent::getLocalPose() const { return mLocalPose; }
Pose SapienRenderCameraComponent::getGlobalPose() const { return getPose() * mLocalPose; }

void SapienRenderCameraComponent::internalUpdate() {
  if (mCamera) {
    auto pose = getGlobalPose() * POSE_GL_TO_ROS;
    mCamera->mCamera->setTransform({.position = {pose.p.x, pose.p.y, pose.p.z},
                                    .rotation = {pose.q.w, pose.q.x, pose.q.y, pose.q.z}});
  }
  mUpdatedWithoutTakingPicture = true;
}

void SapienRenderCameraComponent::checkMode(CameraMode mode) const {
  if (mMode == mode) {
    return;
  }
  if (mMode == CameraMode::ePerspective) {
    throw std::runtime_error("this function is only available for perspective camera");
  } else {
    throw std::runtime_error("this function is only available for orthographic camera");
  }
}

void SapienRenderCameraComponent::gpuInit() {
  if (mGpuInitialized) {
    return;
  }

  if (!mCamera) {
    throw std::runtime_error("failed to init: the camera is not added to scene.");
  }

  setAutoUpload(true);
  auto device = SapienRenderEngine::Get()->getContext()->getDevice();
  auto fence = device.createFenceUnique({});
  mCamera->getRenderer().render(mCamera->getCamera(), {}, {}, {}, fence.get());
  if (device.waitForFences(fence.get(), true, UINT64_MAX) != vk::Result::eSuccess) {
    throw std::runtime_error("failed to initialize camera: the camera failed to render");
  }
  setAutoUpload(false);
  mGpuInitialized = true;
}

CudaArrayHandle SapienRenderCameraComponent::getCudaBuffer() {
  if (!mCamera) {
    throw std::runtime_error(
        "failed to access camera cuda buffer: the camera is not added to scene.");
  }

  if (!mGpuInitialized) {
    throw std::runtime_error(
        "failed to access camera cuda buffer: the camera is not initialized on the GPU.");
  }

  if (auto r = dynamic_cast<svulkan2::renderer::Renderer *>(&mCamera->getRenderer())) {
    auto &buffer = r->getCameraBuffer();
#ifdef SAPIEN_CUDA
    return CudaArrayHandle{.shape = {static_cast<int>(buffer.getSize() / sizeof(float))},
                           .strides = {4},
                           .type = "f4",
                           .cudaId = buffer.getCudaDeviceId(),
                           .ptr = buffer.getCudaPtr()};
#else
    return CudaArrayHandle{.shape = {static_cast<int>(buffer.getSize() / sizeof(float))},
                           .strides = {4},
                           .type = "f4"};
#endif
  } else {
    throw std::runtime_error("only rasterization renderer supports camera cuda buffer.");
  }
}

void SapienRenderCameraComponent::setGpuBatchedPoseIndex(int index) { mGpuPoseIndex = index; }
int SapienRenderCameraComponent::getGpuBatchedPoseIndex() const { return mGpuPoseIndex; }

SapienRenderCameraComponent::~SapienRenderCameraComponent() {}

} // namespace sapien_renderer
} // namespace sapien
