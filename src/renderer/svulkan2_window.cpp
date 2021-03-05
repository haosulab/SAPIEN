#include "svulkan2_window.h"

namespace sapien {
namespace Renderer {

#ifdef _DEBUG
FPSCameraController::FPSCameraController(svulkan2::scene::Node &node, glm::vec3 const &forward,
                                         glm::vec3 const &up)
    : mCamera(&node), mForward(glm::normalize(forward)), mUp(glm::normalize(up)),
      mLeft(glm::cross(mUp, mForward)) {
  mInitialRotation = glm::mat3(-mLeft, mUp, -mForward);
}

void FPSCameraController::setRPY(float roll, float pitch, float yaw) {
  mRPY = {roll, pitch, yaw};
  update();
}

void FPSCameraController::setXYZ(float x, float y, float z) {
  mXYZ = {x, y, z};
  update();
}

void FPSCameraController::move(float forward, float left, float up) {
  auto pose = glm::angleAxis(mRPY.z, mUp) * glm::angleAxis(-mRPY.y, mLeft) *
              glm::angleAxis(mRPY.x, mForward);
  mXYZ += pose * mForward * forward + pose * mLeft * left + pose * mUp * up;
  update();
}

void FPSCameraController::rotate(float roll, float pitch, float yaw) {
  mRPY += glm::vec3{roll, pitch, yaw};
  update();
}

void FPSCameraController::update() {
  mRPY.y = std::clamp(mRPY.y, -1.57f, 1.57f);
  if (mRPY.z >= 3.15) {
    mRPY.z -= 2 * glm::pi<float>();
  } else if (mRPY.z <= -3.15) {
    mRPY.z += 2 * glm::pi<float>();
  }

  auto rotation = glm::angleAxis(mRPY.z, mUp) * glm::angleAxis(-mRPY.y, mLeft) *
                  glm::angleAxis(mRPY.x, mForward) * mInitialRotation;
  mCamera->setTransform({.position = mXYZ, .rotation = rotation});
}
#endif

SVulkan2Window::SVulkan2Window(SVulkan2Renderer &renderer) : mRenderer(&renderer) {
  auto config = std::make_shared<svulkan2::RendererConfig>();
  config->shaderDir = "../shader/full";
  config->colorFormat = vk::Format::eR32G32B32A32Sfloat;
  mSVulkanRenderer = std::make_unique<svulkan2::renderer::Renderer>(*mRenderer->mContext, config);

  mWindow = renderer.mContext->createWindow(800, 600);
  mWindow->initImgui();
  renderer.mContext->getDevice().waitIdle();

  mSceneRenderSemaphore = renderer.mContext->getDevice().createSemaphoreUnique({});
  mSceneRenderFence =
      renderer.mContext->getDevice().createFenceUnique({vk::FenceCreateFlagBits::eSignaled});
  mCommandBuffer = renderer.mContext->createCommandBuffer();
  mSVulkanRenderer->resize(800, 600);

#ifdef _DEBUG
#endif
}

SVulkan2Window::~SVulkan2Window() {
  if (!mClosed) {
    close();
  }
}

void SVulkan2Window::close() {
  mClosed = true;
  mRenderer->mContext->getDevice().waitIdle();
  mWindow->close();
  mRenderer->mContext->getDevice().waitIdle();
}

void SVulkan2Window::hide() { glfwHideWindow(mWindow->getGLFWWindow()); }

void SVulkan2Window::show() { glfwShowWindow(mWindow->getGLFWWindow()); }

void SVulkan2Window::setScene(SVulkan2Scene *scene) { mScene = scene; }

void SVulkan2Window::render() {
  if (!mScene) {
    return;
  }

  auto cams = mScene->getScene()->getCameras();
  svulkan2::scene::Camera *camera{};
  for (auto cam : cams) {
    if (cam->getName() == "_controller") {
      camera = cam;
    }
  }
  if (!camera) {
    camera = &mScene->getScene()->addCamera();
    camera->setPerspectiveParameters(0.1, 10, 1, 800.f / 600.f);
    camera->setName("_controller");
    camera->setPosition({-5, 0, 0});
  }

  mWindow->newFrame();
  ImGui::NewFrame();
  ImGui::ShowDemoWindow();
  ImGui::Render();
  mRenderer->mContext->getDevice().waitForFences(mSceneRenderFence.get(), VK_TRUE, UINT64_MAX);
  mRenderer->mContext->getDevice().resetFences(mSceneRenderFence.get());

  {
    // draw
    mCommandBuffer->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
    mSVulkanRenderer->render(mCommandBuffer.get(), *mScene->getScene(), *camera);
    mSVulkanRenderer->display(mCommandBuffer.get(), "Color", mWindow->getBackbuffer(),
                              mWindow->getBackBufferFormat(), mWindow->getWidth(),
                              mWindow->getHeight());
    mCommandBuffer->end();
    auto imageAcquiredSemaphore = mWindow->getImageAcquiredSemaphore();
    vk::PipelineStageFlags waitStage = vk::PipelineStageFlagBits::eColorAttachmentOutput;
    vk::SubmitInfo info(1, &imageAcquiredSemaphore, &waitStage, 1, &mCommandBuffer.get(), 1,
                        &mSceneRenderSemaphore.get());
    mRenderer->mContext->getQueue().submit(info, {});
  }

  auto swapchain = mWindow->getSwapchain();
  auto fidx = mWindow->getFrameIndex();
  vk::PresentInfoKHR info(1, &mSceneRenderSemaphore.get(), 1, &swapchain, &fidx);
  mWindow->presentFrameWithImgui(mSceneRenderSemaphore.get(), mSceneRenderFence.get());

#ifdef _DEBUG
  if (!mCameraController) {
    mCameraController =
        std::make_unique<FPSCameraController>(*camera, glm::vec3{1, 0, 0}, glm::vec3{0, 0, 1});
    mCameraController->move(0, 0, 0);
  }
  float r = 1e-1;
  if (mWindow->isMouseKeyDown(1)) {
    auto [x, y] = mWindow->getMouseDelta();
    mCameraController->rotate(0, -0.01 * y, -0.01 * x);
  }
  if (mWindow->isKeyDown('w')) {
    mCameraController->move(r, 0, 0);
  }
  if (mWindow->isKeyDown('s')) {
    mCameraController->move(-r, 0, 0);
  }
  if (mWindow->isKeyDown('a')) {
    mCameraController->move(0, r, 0);
  }
  if (mWindow->isKeyDown('d')) {
    mCameraController->move(0, -r, 0);
  }
  if (mWindow->isKeyDown('q')) {
    glfwSetWindowShouldClose(mWindow->getGLFWWindow(), 1);
  }
#endif
}

bool SVulkan2Window::windowCloseRequested() {
  return glfwWindowShouldClose(mWindow->getGLFWWindow());
}

} // namespace Renderer
} // namespace sapien
