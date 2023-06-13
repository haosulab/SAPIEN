#include "sapien/renderer/dlpack.hpp"
#include "sapien/renderer/render_config.h"
#include "sapien/renderer/svulkan2_renderer.h"
#include <spdlog/spdlog.h>

namespace sapien {
namespace Renderer {

SVulkan2Camera::SVulkan2Camera(uint32_t width, uint32_t height, float fovy, float near, float far,
                               SVulkan2Scene *scene, std::string const &shaderDir)
    : mWidth(width), mHeight(height), mScene(scene) {
  auto &renderConfig = GetRenderConfig();

  auto context = mScene->getParentRenderer()->getContext();

  auto config = std::make_shared<svulkan2::RendererConfig>();
  *config = *mScene->getParentRenderer()->getDefaultRendererConfig();

  for (auto &[name, format] : renderConfig.renderTargetFormats) {
    if (format == "r32g32b32a32float" || format == "4f4") {
      config->textureFormat[name] = vk::Format::eR32G32B32A32Sfloat;
    } else if (format == "r8g8b8a8unorm" || format == "4u1") {
      config->textureFormat[name] = vk::Format::eR8G8B8A8Unorm;
    } else {
      throw std::runtime_error("only r32g32b32a32float (4f4) and r8g8b8a8unorm (4u1) are "
                               "supported custom texture formats");
    }
  }

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

  config->shaderDir = shaderDir;

  mRenderer = svulkan2::renderer::RendererBase::Create(config);
  mRenderer->resize(width, height);
  mCamera = &mScene->getScene()->addCamera();
  mCamera->setPerspectiveParameters(near, far, fovy, width, height);
  mSemaphore = context->createTimelineSemaphore(0);
  mRenderer->setScene(scene->getScene());

  // RT renderer
  if (auto rtRenderer = dynamic_cast<svulkan2::renderer::RTRenderer *>(mRenderer.get())) {
    rtRenderer->setCustomProperty("spp", renderConfig.rayTracingSamplesPerPixel);
    rtRenderer->setCustomProperty("maxDepth", renderConfig.rayTracingPathDepth);
    if (renderConfig.rayTracingRussianRouletteMinBounces >= 0) {
      rtRenderer->setCustomProperty("russianRoulette", 1);
      rtRenderer->setCustomProperty("russianRouletteMinBounces",
                                    renderConfig.rayTracingRussianRouletteMinBounces);
    } else {
      rtRenderer->setCustomProperty("russianRoulette", 0);
    }
    if (renderConfig.rayTracingUseDenoiser) {
      rtRenderer->enableDenoiser("HdrColor", "Albedo", "Normal");
    }
  }
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

void SVulkan2Camera::setIntProperty(std::string const &name, int property) {
  mRenderer->setCustomProperty(name, property);
}
void SVulkan2Camera::setFloatProperty(std::string const &name, float property) {
  mRenderer->setCustomProperty(name, property);
}

void SVulkan2Camera::setCustomTexture(std::string const &name,
                                      std::shared_ptr<IRenderTexture> texture) {
  if (auto t = std::dynamic_pointer_cast<SVulkan2Texture>(texture)) {
    mRenderer->setCustomTexture(name, t->getTexture());
  } else {
    throw std::runtime_error("failed to set custom texture: invalid texture.");
  }
}

void SVulkan2Camera::setCustomTextureArray(std::string const &name,
                                           std::vector<std::shared_ptr<IRenderTexture>> textures) {
  std::vector<std::shared_ptr<svulkan2::resource::SVTexture>> sts;
  for (auto tex : textures) {
    if (auto t = std::dynamic_pointer_cast<SVulkan2Texture>(tex)) {
      sts.push_back(t->getTexture());
    } else {
      throw std::runtime_error("failed to set custom texture array: invalid texture.");
    }
  }
  mRenderer->setCustomTextureArray(name, sts);
}

void SVulkan2Camera::takePicture() {
  auto context = mScene->getParentRenderer()->getContext();
  waitForRender();
  mFrameCounter++;
  mRenderer->render(*mCamera, {}, {}, {}, mSemaphore.get(), mFrameCounter);
}

#ifdef SAPIEN_DLPACK
std::shared_ptr<IAwaitable<std::vector<DLManagedTensor *>>>
SVulkan2Camera::takePictureAndGetDLTensorsAsync(ThreadPool &thread,
                                                std::vector<std::string> const &names) {
  auto context = mScene->getParentRenderer()->getContext();
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
      auto &image = mRenderer->getRenderImage(name);
      auto extent = image.getExtent();
      vk::Format format = image.getFormat();
      vk::DeviceSize size =
          extent.width * extent.height * extent.depth * svulkan2::getFormatSize(format);
      if (!mImageBuffers.contains(name)) {
        mImageBuffers[name] = std::make_shared<svulkan2::core::Buffer>(
            size, vk::BufferUsageFlagBits::eTransferSrc | vk::BufferUsageFlagBits::eTransferDst,
            VMA_MEMORY_USAGE_GPU_ONLY, VmaAllocationCreateFlags{}, true);
      }
      auto buffer = mImageBuffers.at(name);
      image.recordCopyToBuffer(mCommandBuffer.get(), buffer->getVulkanBuffer(), 0, size, {0, 0, 0},
                               extent);
    }
    mCommandBuffer->end();
    context->getQueue().submit(mCommandBuffer.get(), {}, {}, {}, mSemaphore.get(), frame, {});
  });

  return std::make_shared<AwaitableSemaphore<std::vector<DLManagedTensor *>>>(
      [this, names]() {
        std::vector<DLManagedTensor *> tensors;
        for (auto &name : names) {
          auto &image = mRenderer->getRenderImage(name);
          auto extent = image.getExtent();
          vk::Format format = image.getFormat();
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
  auto context = mScene->getParentRenderer()->getContext();
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

std::vector<uint8_t> SVulkan2Camera::getUint8Image(std::string const &name) {
  waitForRender();
  return std::get<0>(mRenderer->download<uint8_t>(name));
}

std::string SVulkan2Camera::getImageFormat(std::string const &name) {
  waitForRender();
  switch (mRenderer->getRenderImage(name).getFormat()) {
  case vk::Format::eR8Unorm:
  case vk::Format::eR8G8B8A8Unorm:
    return "u1";
  case vk::Format::eR32Sfloat:
  case vk::Format::eR32G32B32A32Sfloat:
  case vk::Format::eD32Sfloat:
    return "f4";
  case vk::Format::eR32G32B32A32Uint: // FIXME: use int instead of uint
  case vk::Format::eR32G32B32A32Sint:
    return "i4";
  default:
    throw std::runtime_error("image format not supported");
  }
}

#ifdef SAPIEN_DLPACK
DLManagedTensor *SVulkan2Camera::getDLImage(std::string const &name) {
  waitForRender();

  auto context = mScene->getParentRenderer()->getContext();
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
  if (!mImageBuffers.contains(name)) {
    mImageBuffers[name] = std::make_shared<svulkan2::core::Buffer>(
        size, vk::BufferUsageFlagBits::eTransferSrc | vk::BufferUsageFlagBits::eTransferDst,
        VMA_MEMORY_USAGE_GPU_ONLY, VmaAllocationCreateFlags{}, true);
  }
  auto buffer = mImageBuffers.at(name);
  image.recordCopyToBuffer(mCommandBuffer.get(), buffer->getVulkanBuffer(), 0, size, {0, 0, 0},
                           extent);
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
