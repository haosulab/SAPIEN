#include "svulkan2_renderer.h"

namespace sapien {
namespace Renderer {

SVulkan2Camera::SVulkan2Camera(uint32_t width, uint32_t height, float fovy, float near, float far,
                               SVulkan2Scene *scene, std::string const &shaderDir)
    : mWidth(width), mHeight(height), mScene(scene) {
  auto context = mScene->getParentRenderer()->mContext;
  auto config = std::make_shared<svulkan2::RendererConfig>();
  config->culling = mScene->getParentRenderer()->mCullMode;
  config->colorFormat = vk::Format::eR32G32B32A32Sfloat;
  config->depthFormat = vk::Format::eD32Sfloat;
  config->shaderDir = shaderDir;

  mRenderer = std::make_unique<svulkan2::renderer::Renderer>(config);
  mRenderer->resize(width, height);

  mCamera = &mScene->getScene()->addCamera();
  mCamera->setPerspectiveParameters(near, far, fovy, width, height);
  mCommandBuffer = context->createCommandBuffer();
  mFence = context->getDevice().createFenceUnique({vk::FenceCreateFlagBits::eSignaled});
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
  auto result = context->getDevice().waitForFences(mFence.get(), VK_TRUE, UINT64_MAX);
  if (result != vk::Result::eSuccess) {
    throw std::runtime_error("take picture failed: wait for fence failed");
  }
  context->getDevice().resetFences(mFence.get());
  mRenderer->render(*mCamera, {}, {}, {}, mFence.get());
}

void SVulkan2Camera::waitForFence() {
  auto result = mScene->getParentRenderer()->mContext->getDevice().waitForFences(
      mFence.get(), VK_TRUE, UINT64_MAX);
  if (result != vk::Result::eSuccess) {
    throw std::runtime_error("take picture failed: wait for fence failed");
  }
}

std::vector<std::string> SVulkan2Camera::getRenderTargetNames() {
  return mRenderer->getRenderTargetNames();
}

std::vector<float> SVulkan2Camera::getFloatImage(std::string const &name) {
  waitForFence();
  return std::get<0>(mRenderer->download<float>(name));
}

std::vector<uint32_t> SVulkan2Camera::getUintImage(std::string const &textureName) {
  waitForFence();
  return std::get<0>(mRenderer->download<uint32_t>(textureName));
}

// DLPack deleter
static void deleter(DLManagedTensor *self) {
  delete[] self->dl_tensor.shape;
  delete static_cast<std::shared_ptr<svulkan2::core::CudaBuffer> *>(self->manager_ctx);
}

DLManagedTensor *SVulkan2Camera::getDLImage(std::string const &name) {
  auto [buffer, sizes, format] = mRenderer->transferToCuda(name);

  long size = sizes[0] * sizes[1];
  std::vector<long> sizes2 = {sizes[0], sizes[1]};

  uint8_t dtype;
  switch (format) {
  case vk::Format::eR32G32B32A32Sfloat:
    dtype = DLDataTypeCode::kDLFloat;
    size *= 4;
    sizes2.push_back(4);
    break;
  case vk::Format::eD32Sfloat:
    dtype = DLDataTypeCode::kDLFloat;
    break;
  case vk::Format::eR32G32B32A32Uint:
    size *= 4;
    dtype = DLDataTypeCode::kDLUInt;
    sizes2.push_back(4);
    break;
  default:
    throw std::runtime_error("Failed to get tensor from cuda buffer: unsupported buffer format");
  }

  assert(static_cast<long>(buffer->getSize()) == size * 4);

  void *pointer = buffer->getCudaPointer();

  DLManagedTensor *tensor = new DLManagedTensor();

  auto container = new std::shared_ptr<svulkan2::core::CudaBuffer>(buffer);

  int64_t *shape = new int64_t[sizes2.size()];
  for (size_t i = 0; i < sizes2.size(); ++i) {
    shape[i] = sizes2[i];
  }

  if (sizes2.size() >= 2) {
    int64_t t = shape[0];
    shape[0] = shape[1];
    shape[1] = t;
  }

  tensor->dl_tensor.data = pointer;
  tensor->dl_tensor.device = {DLDeviceType::kDLGPU, buffer->getCudaDeviceId()};
  tensor->dl_tensor.ndim = static_cast<int>(sizes2.size());
  tensor->dl_tensor.dtype = {dtype, 32, 1};
  tensor->dl_tensor.shape = shape;
  tensor->dl_tensor.strides = nullptr;
  tensor->dl_tensor.byte_offset = 0;

  tensor->manager_ctx = container;
  tensor->deleter = deleter;

  return tensor;
}

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
