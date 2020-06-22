#ifdef _USE_VULKAN
#include "sapien_vulkan_renderer.h"

namespace sapien {
namespace Renderer {
SapienVulkanCamera::SapienVulkanCamera(std::string const &name, uint32_t width, uint32_t height, float fovy,
                                       float near, float far, SapienVulkanScene *scene,
                                       std::string const &shaderDir)
    : mName(name), mWidth(width), mHeight(height), mScene(scene) {
  auto &context = mScene->getParentRenderer()->mContext;
  mRenderer = context->createVulkanRenderer();
  mRenderer->resize(width, height);

  mCamera = context->createCamera();
  mCamera->fovy = fovy;
  mCamera->aspect = static_cast<float>(width) / height;
  mCamera->position = {0, 0, 0};
  mCamera->rotation = {1, 0, 0, 0};
  mCamera->near = near;
  mCamera->far = far;

  mCommandBuffer = svulkan::createCommandBuffer(context->getDevice(), context->getCommandPool(),
                                                vk::CommandBufferLevel::ePrimary);
  mFence = context->getDevice().createFenceUnique({vk::FenceCreateFlagBits::eSignaled});
}

void SapienVulkanCamera::takePicture() {
  auto &context = mScene->getParentRenderer()->mContext;
  context->getDevice().waitForFences({mFence.get()}, VK_TRUE, UINT64_MAX);
  context->getDevice().resetFences({mFence.get()});

  mCommandBuffer->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
  mRenderer->render(mCommandBuffer.get(), *mScene->getScene(), *mCamera);
  mCommandBuffer->end();

  vk::PipelineStageFlags waitStage = vk::PipelineStageFlagBits::eColorAttachmentOutput;
  vk::SubmitInfo info(0, nullptr, &waitStage, 1, &mCommandBuffer.get(),
                      0, nullptr);
  context->getGraphicsQueue().submit(info, mFence.get());
}

std::vector<float> SapienVulkanCamera::getColorRGBA() {
  waitForFence();
  return mRenderer->downloadLighting();
}

std::vector<float> SapienVulkanCamera::getAlbedoRGBA() {
  waitForFence();
  return mRenderer->downloadAlbedo();
}

std::vector<float> SapienVulkanCamera::getNormalRGBA() {
  waitForFence();
  return mRenderer->downloadNormal();
}

std::vector<float> SapienVulkanCamera::getDepth() {
  waitForFence();
  return mRenderer->downloadDepth();
}

std::vector<int> SapienVulkanCamera::getSegmentation() {
  waitForFence();

  std::vector<int> output;
  auto segmentation = mRenderer->downloadSegmentation();
  for (uint32_t i = 1; i < segmentation.size(); i += 4) {
    output.push_back(segmentation[i]);
  }
  return output;
}

std::vector<int> SapienVulkanCamera::getObjSegmentation() {
  waitForFence();

  std::vector<int> output;
  auto segmentation = mRenderer->downloadSegmentation();
  for (uint32_t i = 0; i < segmentation.size(); i += 4) {
    output.push_back(segmentation[i]);
  }
  return output;
}

std::vector<float> SapienVulkanCamera::getPositionRGBA() {
  waitForFence();
  return mRenderer->downloadPosition();
}


physx::PxTransform SapienVulkanCamera::getPose() const {
  auto p = mCamera->position;
  auto q = mCamera->rotation;
  return physx::PxTransform({p.x, p.y, p.z}, physx::PxQuat(q.x, q.y, q.z, q.w));
}

void SapienVulkanCamera::setInitialPose(physx::PxTransform const &pose) {
  mInitialPose = pose;
  mCamera->position = {pose.p.x, pose.p.y, pose.p.z};
  mCamera->rotation = {pose.q.w, pose.q.x, pose.q.y, pose.q.z};
}

void SapienVulkanCamera::setPose(physx::PxTransform const &pose) {
  auto p = pose * mInitialPose;
  mCamera->position = {p.p.x, p.p.y, p.p.z};
  mCamera->rotation = {p.q.w, p.q.x, p.q.y, p.q.z};
}

void SapienVulkanCamera::changeModeToOrthographic(float scaling) {
  mCamera->ortho = true;
  mCamera->scaling = scaling;
}

void SapienVulkanCamera::changeModeToPerspective(float fovy) {
  mCamera->ortho = false;
  mCamera->fovy = fovy;
}

bool SapienVulkanCamera::isOrthographic() const {
  return mCamera->ortho;
}

void SapienVulkanCamera::waitForFence() {
  auto &context = mScene->getParentRenderer()->mContext;
  context->getDevice().waitForFences({mFence.get()}, VK_TRUE, UINT64_MAX);
}

glm::mat4 SapienVulkanCamera::getModelMatrix() const {
  return mCamera->getModelMat();
}

glm::mat4 SapienVulkanCamera::getProjectionMatrix() const {
  return mCamera->getProjectionMat();
}

} // namespace Renderer
} // namespace sapien
#endif
