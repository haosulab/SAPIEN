#include "sapien/component/sapien_renderer/camera_component.h"
#include "../../logger.h"
#include "sapien/component/sapien_renderer/sapien_renderer_default.h"
#include "sapien/component/sapien_renderer/sapien_renderer_system.h"
#include "sapien/component/sapien_renderer/texture.h"
#include "sapien/entity.h"
#include "sapien/math/conversion.h"
#include "sapien/scene.h"
#include <svulkan2/renderer/renderer_base.h>

namespace sapien {
namespace component {

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
    mSemaphore = mEngine->getContext()->createTimelineSemaphore(0);
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
    auto result = mEngine->getContext()->getDevice().waitSemaphores(
        vk::SemaphoreWaitInfo({}, mSemaphore.get(), mFrameCounter), UINT64_MAX);
    if (result != vk::Result::eSuccess) {
      throw std::runtime_error("take picture failed: wait for fence failed");
    }
  }

  void takePicture() {
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
      mCudaImageBuffers[name] = std::make_shared<svulkan2::core::Buffer>(
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
  return mCamera->getImage(name);
}

SapienRenderImageCuda SapienRenderCameraComponent::getImageCuda(std::string const &name) {
#ifdef SAPIEN_CUDA
  if (!mCamera) {
    throw std::runtime_error("failed to get image: the camera is not added to scene");
  }
  if (mUpdatedWithoutTakingPicture) {
    logger::warn("getting picture without taking picture since last camera update");
  }
  return mCamera->getImageCuda(name);
#else
  throw std::runtime_error("sapien is not copmiled with CUDA support");
#endif
}

// getters
Mat4 SapienRenderCameraComponent::getModelMatrix() const {
  return PoseToEigenMat4(getGlobalPose() * POSE_GL_TO_ROS);
}
Mat4 SapienRenderCameraComponent::getProjectionMatrix() const {
  Mat4 mat = Mat4::Identity();
  float fx = getFocalLengthX();
  float fy = getFocalLengthY();
  float width = getWidth();
  float height = getHeight();
  float far = getFar();
  float near = getNear();
  float cx = getPrincipalPointX();
  float cy = getPrincipalPointY();
  float skew = getSkew();
  mat(0, 0) = (2.f * fx) / width;
  mat(1, 1) = -(2.f * fy) / height;
  mat(2, 2) = -far / (far - near);
  mat(2, 3) = -far * near / (far - near);
  mat(3, 2) = -1.f;
  mat(0, 2) = -2.f * cx / width + 1;
  mat(1, 2) = -2.f * cy / height + 1;
  mat(3, 3) = 0.f;
  mat(0, 1) = -2 * skew / width;
  return mat;
}
Mat3 SapienRenderCameraComponent::getIntrinsicMatrix() const {
  Mat3 mat = Mat3::Identity();
  mat(0, 0) = getFocalLengthX();
  mat(1, 1) = getFocalLengthY();
  mat(0, 2) = getPrincipalPointX();
  mat(1, 2) = getPrincipalPointY();
  mat(0, 1) = getSkew();
  return mat;
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
void SapienRenderCameraComponent::setFocalLengths(float fx, float fy) {
  setPerspectiveParameters(getNear(), getFar(), fx, fy, getPrincipalPointX(), getPrincipalPointY(),
                           getSkew());
}
void SapienRenderCameraComponent::setFovX(float fovx, bool computeY) {
  float fx = getWidth() / 2.f / std::tan(fovx / 2);
  float fy = computeY ? fx : getFocalLengthY();
  setFocalLengths(fx, fy);
}

void SapienRenderCameraComponent::setFovY(float fovy, bool computeX) {
  float fy = getHeight() / 2.f / std::tan(fovy / 2);
  float fx = computeX ? fy : getFocalLengthX();
  setFocalLengths(fx, fy);
}
void SapienRenderCameraComponent::setNear(float near) {
  setPerspectiveParameters(near, getFar(), getFocalLengthX(), getFocalLengthY(),
                           getPrincipalPointX(), getPrincipalPointY(), getSkew());
}
void SapienRenderCameraComponent::setFar(float far) {
  setPerspectiveParameters(getNear(), far, getFocalLengthX(), getFocalLengthY(),
                           getPrincipalPointX(), getPrincipalPointY(), getSkew());
}
void SapienRenderCameraComponent::setPrincipalPoint(float cx, float cy) {
  setPerspectiveParameters(getNear(), getFar(), getFocalLengthX(), getFocalLengthY(), cx, cy,
                           getSkew());
}
void SapienRenderCameraComponent::setSkew(float s) {
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
SapienRenderCameraComponent::~SapienRenderCameraComponent() {}

} // namespace component
} // namespace sapien
