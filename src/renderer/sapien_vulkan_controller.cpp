#ifdef _USE_VULKAN
#ifdef ON_SCREEN
#include "sapien_vulkan_controller.h"
#include "sapien_scene.h"

namespace sapien {
namespace Renderer {

SapienVulkanController::SapienVulkanController(SapienVulkanRenderer *renderer)
    : mRenderer(renderer), mWidth(0), mHeight(0) {
  mCamera = mRenderer->mContext->createCamera();
  mVulkanRenderer = mRenderer->mContext->createVulkanRenderer();
  mFPSController = std::make_unique<svulkan::FPSCameraController>(
      *mCamera, glm::vec3{1.f, 0.f, 0.f}, glm::vec3{0.f, 0.f, 1.f});
  mWindow = mRenderer->mContext->createWindow();
  mWindow->initImgui(mRenderer->mContext->getDescriptorPool(), mRenderer->mContext->getCommandPool());

  mCommandBuffer = svulkan::createCommandBuffer(mRenderer->mContext->getDevice(),
                                                mRenderer->mContext->getCommandPool(),
                                                vk::CommandBufferLevel::ePrimary);
  mFence = mRenderer->mContext->getDevice().createFenceUnique({vk::FenceCreateFlagBits::eSignaled});
  mSemaphore = mRenderer->mContext->getDevice().createSemaphoreUnique({});

  mFPSController->setXYZ(0, 0, 0);
}

void SapienVulkanController::render() {
  if (mWindow->isClosed()) {
    return;
  }
  if (!mScene) {
    return;
  }

  int width, height;
  glfwGetFramebufferSize(mWindow->getWindow(), &width, &height);
  if (static_cast<uint32_t>(width) != mWidth || static_cast<uint32_t>(height) != mHeight) {
    mRenderer->mContext->getDevice().waitIdle();
    mWindow->updateSize(width, height);
    mVulkanRenderer->resize(mWindow->getWidth(), mWindow->getHeight());
    mWidth = mWindow->getWidth();
    mHeight = mWindow->getHeight();
    mCamera->aspect = static_cast<float>(mWidth) / mHeight;
  }

  // get the next image
  try {
    mWindow->newFrame();
  } catch (vk::OutOfDateKHRError &e) {
    mRenderer->mContext->getDevice().waitIdle();
    return;
  }

  {
    ImGui::NewFrame();
    ImGui::Render();
  }

  // wait for previous frame to finish
  {
    mRenderer->mContext->getDevice().waitForFences(mFence.get(), VK_TRUE, UINT64_MAX);
    mRenderer->mContext->getDevice().resetFences(mFence.get());
  }

  auto scene = static_cast<SapienVulkanScene *>(mScene->getRendererScene());
  // draw
  {
    mCommandBuffer->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
    mVulkanRenderer->render(mCommandBuffer.get(), *scene->getScene(), *mCamera);
    mVulkanRenderer->display(mCommandBuffer.get(), mWindow->getBackBuffer(),
                             mWindow->getBackBufferFormat(), mWindow->getWidth(), mWindow->getHeight());
    mCommandBuffer->end();

    auto imageAcquiredSemaphore = mWindow->getImageAcquiredSemaphore();
    vk::PipelineStageFlags waitStage = vk::PipelineStageFlagBits::eColorAttachmentOutput;
    vk::SubmitInfo info(1, &imageAcquiredSemaphore, &waitStage, 1, &mCommandBuffer.get(),
                        1, &mSemaphore.get());
    mRenderer->mContext->getGraphicsQueue().submit(info, {});
  }

  auto swapchain = mWindow->getSwapchain();
  auto fidx = mWindow->getFrameIndex();
  vk::PresentInfoKHR info(1, &mSemaphore.get(), 1, &swapchain, &fidx);

  try {
    mWindow->presentFrameWithImgui(mRenderer->mContext->getGraphicsQueue(), mWindow->getPresentQueue(),
                                   mSemaphore.get(), mFence.get());
  } catch (vk::OutOfDateKHRError &e) {
    return;
  }
  // wait idle

  if (mWindow->isKeyDown('q')) {
    mWindow->close();
  }

  if (mWindow->isMouseKeyDown(1)) {
    auto [x, y] = mWindow->getMouseDelta();
    float r = 1e-3;
    mFPSController->rotate(0, -r * y, -r * x);
  }

  constexpr float r = 1e-3;
  if (mWindow->isKeyDown('w')) {
    mFPSController->move(r, 0, 0);
  }
  if (mWindow->isKeyDown('s')) {
    mFPSController->move(-r, 0, 0);
  }
  if (mWindow->isKeyDown('a')) {
    mFPSController->move(0, r, 0);
  }
  if (mWindow->isKeyDown('d')) {
    mFPSController->move(0, -r, 0);
  }
}

} // namespace Renderer
} // namespace sapien
#endif
#endif
