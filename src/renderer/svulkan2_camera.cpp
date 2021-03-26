#include "svulkan2_renderer.h"

namespace sapien {
namespace Renderer {

SVulkan2Camera::SVulkan2Camera(std::string const &name, uint32_t width, uint32_t height,
                               float fovy, float near, float far, SVulkan2Scene *scene,
                               std::string const &shaderDir)
    : mName(name), mWidth(width), mHeight(height), mScene(scene) {
  auto &context = mScene->getParentRenderer()->mContext;
  auto config = std::make_shared<svulkan2::RendererConfig>();
  config->colorFormat = vk::Format::eR32G32B32A32Sfloat;
  config->depthFormat = vk::Format::eD32Sfloat;
  config->shaderDir = shaderDir;

  mRenderer = std::make_unique<svulkan2::renderer::Renderer>(*context, config);
  mRenderer->resize(width, height);

  mCamera = &mScene->getScene()->addCamera();
  mCamera->setPerspectiveParameters(near, far, fovy, width / static_cast<float>(height));
  mCommandBuffer = context->createCommandBuffer();
  mFence = context->getDevice().createFenceUnique({vk::FenceCreateFlagBits::eSignaled});
  mRenderer->setScene(*scene->getScene());
}

void SVulkan2Camera::takePicture() {
  auto &context = mScene->getParentRenderer()->mContext;
  auto result = context->getDevice().waitForFences(mFence.get(), VK_TRUE, UINT64_MAX);
  if (result != vk::Result::eSuccess) {
    throw std::runtime_error("take picture failed: wait for fence failed");
  }
  context->getDevice().resetFences(mFence.get());
  mRenderer->render(*mCamera, {}, {}, {}, mFence.get());
}

std::vector<float> SVulkan2Camera::getColorRGBA() {
  throw std::runtime_error("This method is no longer supported, use "
                           "get{Uint32/Uint8/Float}Texture(\"textureName\") instead.");
}

std::vector<float> SVulkan2Camera::getAlbedoRGBA() {
  throw std::runtime_error("This method is no longer supported, use "
                           "get{Uint32/Uint8/Float}Texture(\"textureName\") instead.");
}
std::vector<float> SVulkan2Camera::getNormalRGBA() {
  throw std::runtime_error("This method is no longer supported, use "
                           "get{Uint32/Uint8/Float}Texture(\"textureName\") instead.");
}
std::vector<float> SVulkan2Camera::getDepth() {
  throw std::runtime_error("This method is no longer supported, use "
                           "get{Uint32/Uint8/Float}Texture(\"textureName\") instead.");
}
std::vector<int> SVulkan2Camera::getSegmentation() {
  throw std::runtime_error("This method is no longer supported, use "
                           "get{Uint32/Uint8/Float}Texture(\"textureName\") instead.");
}
std::vector<int> SVulkan2Camera::getObjSegmentation() {
  throw std::runtime_error("This method is no longer supported, use "
                           "get{Uint32/Uint8/Float}Texture(\"textureName\") instead.");
}

void SVulkan2Camera::waitForFence() {
  // auto result = mScene->getParentRenderer()->mContext->getDevice().waitForFences(
  //     mFence.get(), VK_TRUE, UINT64_MAX);
  mScene->getParentRenderer()->mContext->getDevice().waitIdle();
  // if (result != vk::Result::eSuccess) {
  //   throw std::runtime_error("take picture failed: wait for fence failed");
  // }
}

std::tuple<std::vector<float>, std::array<uint32_t, 3>>
SVulkan2Camera::getFloatTexture(std::string const &textureName) {
  waitForFence();
  return mRenderer->download<float>(textureName);
}
std::tuple<std::vector<uint32_t>, std::array<uint32_t, 3>>
SVulkan2Camera::getUint32Texture(std::string const &textureName) {
  waitForFence();
  return mRenderer->download<uint32_t>(textureName);
}
std::tuple<std::vector<uint8_t>, std::array<uint32_t, 3>>
SVulkan2Camera::getUint8Texture(std::string const &textureName) {
  waitForFence();
  return mRenderer->download<uint8_t>(textureName);
}

glm::mat4 SVulkan2Camera::getModelMatrix() const { return mCamera->computeWorldModelMatrix(); }
glm::mat4 SVulkan2Camera::getProjectionMatrix() const { return mCamera->getProjectionMatrix(); }

glm::mat4 SVulkan2Camera::getCameraMatrix() const {
  if (mCamera->getCameraType() == svulkan2::scene::Camera::Type::ePerspective) {
    float fovy = mCamera->getFovy();
    float f = static_cast<float>(mHeight) / std::tan(fovy / 2) / 2;
    auto matrix = glm::mat4(1.0);
    matrix[0][0] = f;
    matrix[2][0] = static_cast<float>(mWidth) / 2;
    matrix[1][1] = f;
    matrix[2][1] = static_cast<float>(mHeight) / 2;
    return matrix;
  } else if (mCamera->getCameraType() == svulkan2::scene::Camera::Type::eFullPerspective) {
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

void SVulkan2Camera::setInitialPose(physx::PxTransform const &pose) {
  mInitialPose = pose;
  mCamera->setTransform({.position = {pose.p.x, pose.p.y, pose.p.z},
                         .rotation = {pose.q.w, pose.q.x, pose.q.y, pose.q.z}});
}
void SVulkan2Camera::setPose(physx::PxTransform const &pose) {
  auto p = pose * mInitialPose;
  mCamera->setTransform(
      {.position = {p.p.x, p.p.y, p.p.z}, .rotation = {p.q.w, p.q.x, p.q.y, p.q.z}});
}

void SVulkan2Camera::setPerspectiveParameters(float near, float far, float fovy, float aspect) {
  mCamera->setPerspectiveParameters(near, far, fovy, aspect);
}

void SVulkan2Camera::setFullPerspectiveParameters(float near, float far, float fx, float fy,
                                                  float cx, float cy, float width, float height,
                                                  float skew) {
  mCamera->setFullPerspectiveParameters(near, far, fx, fy, cx, cy, width, height, skew);
}

void SVulkan2Camera::setOrthographicParameters(float near, float far, float aspect,
                                               float scaling) {
  mCamera->setOrthographicParameters(near, far, aspect, scaling);
}

std::string SVulkan2Camera::getMode() const {
  switch (mCamera->getCameraType()) {
  case svulkan2::scene::Camera::Type::eUndefined:
    return "unknown";
  case svulkan2::scene::Camera::Type::ePerspective:
    return "perspective";
  case svulkan2::scene::Camera::Type::eFullPerspective:
    return "full_perspective";
  case svulkan2::scene::Camera::Type::eOrthographic:
    return "orthographic";
  case svulkan2::scene::Camera::Type::eMatrix:
    return "matrix";
  }
  throw std::runtime_error("invalid camera type");
}

} // namespace Renderer
} // namespace sapien
