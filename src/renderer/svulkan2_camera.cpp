#include "sapien/renderer/dlpack.hpp"
#include "sapien/renderer/svulkan2_renderer.h"

namespace sapien {
namespace Renderer {

SVulkan2Camera::SVulkan2Camera(uint32_t width, uint32_t height, float fovy, float near, float far,
                               SVulkan2Scene *scene, std::string const &shaderDir)
    : mWidth(width), mHeight(height), mScene(scene) {
  auto context = mScene->getParentRenderer()->mContext;
  auto config = std::make_shared<svulkan2::RendererConfig>();
  config->culling = mScene->getParentRenderer()->mCullMode;
  config->colorFormat4 = vk::Format::eR32G32B32A32Sfloat;
  config->depthFormat = vk::Format::eD32Sfloat;
  config->shaderDir = shaderDir;

  mRenderer = std::make_unique<svulkan2::renderer::Renderer>(config);
  mRenderer->resize(width, height);

  mCamera = &mScene->getScene()->addCamera();
  mCamera->setPerspectiveParameters(near, far, fovy, width, height);

  mSemaphore = context->createTimelineSemaphore(0);
  mRenderer->setScene(*scene->getScene());
}

float SVulkan2Camera::getPrincipalPointX() const { return mCamera->getCx(); }
float SVulkan2Camera::getPrincipalPointY() const { return mCamera->getCy(); }
float SVulkan2Camera::getFocalX() const { return mCamera->getFx(); }
float SVulkan2Camera::getFocalY() const { return mCamera->getFy(); }
float SVulkan2Camera::getNear() const { return mCamera->getNear(); }
float SVulkan2Camera::getFar() const { return mCamera->getFar(); }
float SVulkan2Camera::getSkew() const { return mCamera->getSkew(); }

void SVulkan2Camera::setPerspectiveCameraParameters(float near, float far, float fx, float fy,
                                                    float cx, float cy, float skew) {
  mCamera->setPerspectiveParameters(near, far, fx, fy, cx, cy, mWidth, mHeight, skew);
}

void SVulkan2Camera::takePicture() {
  auto context = mScene->getParentRenderer()->mContext;
  waitForRender();
  mFrameCounter++;
  mRenderer->render(*mCamera, {}, {}, {}, mSemaphore.get(), mFrameCounter);
}

#ifdef SAPIEN_DLPACK
std::shared_ptr<IAwaitable<std::vector<DLManagedTensor *>>>
SVulkan2Camera::takePictureAndGetDLTensorsAsync(ThreadPool &thread,
                                                std::vector<std::string> const &names) {
  auto context = mScene->getParentRenderer()->mContext;
  mFrameCounter++;
  thread.submit([context, frame = mFrameCounter, this, names = std::move(names)]() {
    uint64_t waitFrame = frame - 1;
    auto result = context->getDevice().waitSemaphores(
        vk::SemaphoreWaitInfo({}, mSemaphore.get(), waitFrame), UINT64_MAX);
    if (result != vk::Result::eSuccess) {
      throw std::runtime_error("take picture failed: wait failed");
    }
    if (!mCommandPool) {
      mCommandPool = context->createCommandPool();
      mCommandBuffer = mCommandPool->allocateCommandBuffer();
    }
    mCommandBuffer->reset();
    mCommandBuffer->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
    mRenderer->render(*mCamera, {}, {}, {}, {});

    for (auto &name : names) {
      auto target = mRenderer->getRenderTarget(name);
      auto extent = target->getImage().getExtent();
      vk::Format format = target->getFormat();
      vk::DeviceSize size =
          extent.width * extent.height * extent.depth * svulkan2::getFormatSize(format);
      if (!mImageBuffers.contains(name)) {
        mImageBuffers[name] = std::make_shared<svulkan2::core::Buffer>(
            size, vk::BufferUsageFlagBits::eTransferSrc | vk::BufferUsageFlagBits::eTransferDst,
            VMA_MEMORY_USAGE_GPU_ONLY);
      }
      auto buffer = mImageBuffers.at(name);
      target->getImage().recordCopyToBuffer(mCommandBuffer.get(), buffer->getVulkanBuffer(), 0,
                                            size, {0, 0, 0}, extent);
    }
    mCommandBuffer->end();
    context->getQueue().submit(mCommandBuffer.get(), {}, {}, {}, mSemaphore.get(), frame, {});
  });

  return std::make_shared<AwaitableSemaphore<std::vector<DLManagedTensor *>>>(
      [this, names]() {
        std::vector<DLManagedTensor *> tensors;
        for (auto &name : names) {
          auto target = mRenderer->getRenderTarget(name);
          auto extent = target->getImage().getExtent();
          vk::Format format = target->getFormat();
          auto buffer = mImageBuffers.at(name);
          std::vector<long> sizes2 = {extent.height, extent.width};
          uint8_t dtype;
          switch (format) {
          case vk::Format::eR32G32B32A32Sfloat:
            dtype = DLDataTypeCode::kDLFloat;
            sizes2.push_back(4);
            break;
          case vk::Format::eD32Sfloat:
            dtype = DLDataTypeCode::kDLFloat;
            break;
          case vk::Format::eR32G32B32A32Uint:
            // dtype = DLDataTypeCode::kDLUInt;
            dtype = DLDataTypeCode::kDLInt;
            sizes2.push_back(4);
            break;
          default:
            throw std::runtime_error(
                "Failed to get tensor from cuda buffer: unsupported buffer format");
          }
          tensors.push_back(dl_wrapper(buffer, buffer->getCudaPtr(), buffer->getCudaDeviceId(),
                                       sizes2, {dtype, 32, 1}));
        }
        return tensors;
      },
      mSemaphore.get(), mFrameCounter, context->getDevice());
}
#endif

void SVulkan2Camera::waitForRender() {
  auto context = mScene->getParentRenderer()->mContext;
  auto result = context->getDevice().waitSemaphores(
      vk::SemaphoreWaitInfo({}, mSemaphore.get(), mFrameCounter), UINT64_MAX);
  if (result != vk::Result::eSuccess) {
    throw std::runtime_error("take picture failed: wait for fence failed");
  }
}

std::vector<std::string> SVulkan2Camera::getRenderTargetNames() {
  return mRenderer->getRenderTargetNames();
}

std::vector<float> SVulkan2Camera::getFloatImage(std::string const &name) {
  waitForRender();
  return std::get<0>(mRenderer->download<float>(name));
}

std::vector<uint32_t> SVulkan2Camera::getUintImage(std::string const &textureName) {
  waitForRender();
  return std::get<0>(mRenderer->download<uint32_t>(textureName));
}

#ifdef SAPIEN_DLPACK
DLManagedTensor *SVulkan2Camera::getDLImage(std::string const &name) {
  waitForRender();

  auto context = mScene->getParentRenderer()->mContext;
  if (!mCommandPool) {
    mCommandPool = context->createCommandPool();
    mCommandBuffer = mCommandPool->allocateCommandBuffer();
  }
  mCommandBuffer->reset();
  mCommandBuffer->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
  mRenderer->render(*mCamera, {}, {}, {}, {});

  auto target = mRenderer->getRenderTarget(name);
  auto extent = target->getImage().getExtent();
  vk::Format format = target->getFormat();
  vk::DeviceSize size =
      extent.width * extent.height * extent.depth * svulkan2::getFormatSize(format);
  if (!mImageBuffers.contains(name)) {
    mImageBuffers[name] = std::make_shared<svulkan2::core::Buffer>(
        size, vk::BufferUsageFlagBits::eTransferSrc | vk::BufferUsageFlagBits::eTransferDst,
        VMA_MEMORY_USAGE_GPU_ONLY);
  }
  auto buffer = mImageBuffers.at(name);
  target->getImage().recordCopyToBuffer(mCommandBuffer.get(), buffer->getVulkanBuffer(), 0, size,
                                        {0, 0, 0}, extent);
  std::vector<long> sizes2 = {extent.height, extent.width};
  uint8_t dtype;
  switch (format) {
  case vk::Format::eR32G32B32A32Sfloat:
    dtype = DLDataTypeCode::kDLFloat;
    sizes2.push_back(4);
    break;
  case vk::Format::eD32Sfloat:
    dtype = DLDataTypeCode::kDLFloat;
    break;
  case vk::Format::eR32G32B32A32Uint:
    // dtype = DLDataTypeCode::kDLUInt;
    dtype = DLDataTypeCode::kDLInt;
    sizes2.push_back(4);
    break;
  default:
    throw std::runtime_error("Failed to get tensor from cuda buffer: unsupported buffer format");
  }
  mCommandBuffer->end();
  context->getQueue().submitAndWait(mCommandBuffer.get());
  return dl_wrapper(buffer, buffer->getCudaPtr(), buffer->getCudaDeviceId(), sizes2,
                    {dtype, 32, 1});
}
#endif

glm::mat4 SVulkan2Camera::getModelMatrix() const { return mCamera->computeWorldModelMatrix(); }
glm::mat4 SVulkan2Camera::getProjectionMatrix() const { return mCamera->getProjectionMatrix(); }

glm::mat4 SVulkan2Camera::getCameraMatrix() const {
  if (mCamera->getCameraType() == svulkan2::scene::Camera::Type::ePerspective) {
    auto matrix = glm::mat4(1.0);
    matrix[0][0] = mCamera->getFx();
    matrix[1][1] = mCamera->getFy();
    matrix[2][0] = mCamera->getCx();
    matrix[2][1] = mCamera->getCy();
    matrix[1][0] = mCamera->getSkew();
    return matrix;
  }
  throw std::runtime_error("only perspective cameras support camera matrix");
}

physx::PxTransform SVulkan2Camera::getPose() const {
  auto &T = mCamera->getTransform();
  auto p = T.position;
  auto q = T.rotation;
  return physx::PxTransform({p.x, p.y, p.z}, physx::PxQuat(q.x, q.y, q.z, q.w));
}

void SVulkan2Camera::setPose(physx::PxTransform const &pose) {
  auto p = pose;
  mCamera->setTransform(
      {.position = {p.p.x, p.p.y, p.p.z}, .rotation = {p.q.w, p.q.x, p.q.y, p.q.z}});
}

void SVulkan2Camera::setOrthographicParameters(float near, float far, float scaling, float width,
                                               float height) {
  mCamera->setOrthographicParameters(near, far, scaling, mWidth, mHeight);
}

std::string SVulkan2Camera::getMode() const {
  switch (mCamera->getCameraType()) {
  case svulkan2::scene::Camera::Type::eUndefined:
    return "unknown";
  case svulkan2::scene::Camera::Type::ePerspective:
    return "perspective";
  case svulkan2::scene::Camera::Type::eOrthographic:
    return "orthographic";
  case svulkan2::scene::Camera::Type::eMatrix:
    return "matrix";
  }
  throw std::runtime_error("invalid camera type");
}

} // namespace Renderer
} // namespace sapien
