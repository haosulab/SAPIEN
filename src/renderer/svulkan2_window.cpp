#include "svulkan2_window.h"

namespace sapien {
namespace Renderer {

#ifdef _DEBUG
FPSCameraControllerDebug::FPSCameraControllerDebug(svulkan2::scene::Node &node,
                                                   glm::vec3 const &forward, glm::vec3 const &up)
    : mCamera(&node), mForward(glm::normalize(forward)), mUp(glm::normalize(up)),
      mLeft(glm::cross(mUp, mForward)) {
  mInitialRotation = glm::mat3(-mLeft, mUp, -mForward);
}

void FPSCameraControllerDebug::setRPY(float roll, float pitch, float yaw) {
  mRPY = {roll, pitch, yaw};
  update();
}

void FPSCameraControllerDebug::setXYZ(float x, float y, float z) {
  mXYZ = {x, y, z};
  update();
}

void FPSCameraControllerDebug::move(float forward, float left, float up) {
  auto pose = glm::angleAxis(mRPY.z, mUp) * glm::angleAxis(-mRPY.y, mLeft) *
              glm::angleAxis(mRPY.x, mForward);
  mXYZ += pose * mForward * forward + pose * mLeft * left + pose * mUp * up;
  update();
}

void FPSCameraControllerDebug::rotate(float roll, float pitch, float yaw) {
  mRPY += glm::vec3{roll, pitch, yaw};
  update();
}

void FPSCameraControllerDebug::update() {
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

SVulkan2Window::SVulkan2Window(SVulkan2Renderer &renderer, std::string const &shaderDir)
    : mRenderer(&renderer), mShaderDir(shaderDir) {
  auto config = std::make_shared<svulkan2::RendererConfig>();
  config->shaderDir = mShaderDir.length() ? mShaderDir : gDefaultShaderDirectory;
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

void SVulkan2Window::setCameraParameters(float near, float far, float fovy) {
  getCamera()->setPerspectiveParameters(
      near, far, fovy, static_cast<float>(mWindow->getWidth()) / mWindow->getHeight());
}
void SVulkan2Window::setCameraPosition(glm::vec3 const &pos) { getCamera()->setPosition(pos); }
void SVulkan2Window::setCameraRotation(glm::quat const &rot) { getCamera()->setRotation(rot); }

std::vector<std::string> SVulkan2Window::getDisplayTargetNames() const {
  return mSVulkanRenderer->getDisplayTargetNames();
}

void SVulkan2Window::render(std::string const &targetName,
                            std::vector<std::shared_ptr<svulkan2::ui::Window>> uiWindows) {
  if (!mScene) {
    return;
  }

  svulkan2::scene::Camera *camera = getCamera();

  mWindow->newFrame();
  ImGui::NewFrame();
  // ImGui::ShowDemoWindow();
  for (auto w : uiWindows) {
    w->build();
  }
  ImGui::Render();
  mRenderer->mContext->getDevice().waitForFences(mSceneRenderFence.get(), VK_TRUE, UINT64_MAX);
  mRenderer->mContext->getDevice().resetFences(mSceneRenderFence.get());

  {
    // draw
    mCommandBuffer->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
    mSVulkanRenderer->render(mCommandBuffer.get(), *mScene->getScene(), *camera);
    mSVulkanRenderer->display(mCommandBuffer.get(), targetName, mWindow->getBackbuffer(),
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
    mCameraController = std::make_unique<FPSCameraControllerDebug>(*camera, glm::vec3{1, 0, 0},
                                                                   glm::vec3{0, 0, 1});
    mCameraController->move(0, 0, 0);
  }
  float r = 1e-1;
  if (mWindow->isMouseKeyDown(1)) {
    auto [x, y] = mWindow->getMouseDelta();
    mCameraController->rotate(0, -0.01 * y, -0.01 * x);
  }
  if (mWindow->isKeyDown("w")) {
    mCameraController->move(r, 0, 0);
  }
  if (mWindow->isKeyDown("s")) {
    mCameraController->move(-r, 0, 0);
  }
  if (mWindow->isKeyDown("a")) {
    mCameraController->move(0, r, 0);
  }
  if (mWindow->isKeyDown("d")) {
    mCameraController->move(0, -r, 0);
  }
  if (mWindow->isKeyDown("q")) {
    glfwSetWindowShouldClose(mWindow->getGLFWWindow(), 1);
  }
#endif
}

bool SVulkan2Window::windowCloseRequested() {
  return glfwWindowShouldClose(mWindow->getGLFWWindow());
}

svulkan2::scene::Camera *SVulkan2Window::getCamera() {
  if (!mScene) {
    throw std::runtime_error("failed to operate camera, you need to set scene first.");
  }
  auto cams = mScene->getScene()->getCameras();
  for (auto cam : cams) {
    if (cam->getName() == "_controller") {
      return cam;
    }
  }
  auto camera = &mScene->getScene()->addCamera();
  camera->setPerspectiveParameters(0.1, 10, 1,
                                   static_cast<float>(mWindow->getWidth()) / mWindow->getHeight());
  camera->setName("_controller");
  camera->setPosition({0, 0, 0});
  return camera;
}

std::tuple<std::vector<float>, std::array<uint32_t, 3>>
SVulkan2Window::downloadFloatTarget(std::string const &name) {
  auto format = mSVulkanRenderer->getRenderTarget(name)->getFormat();
  if (format != vk::Format::eR32G32B32A32Sfloat && format != vk::Format::eD32Sfloat) {
    throw std::runtime_error("failed to download: " + name + " is not a float render target.");
  }
  return mSVulkanRenderer->download<float>(name);
}

std::tuple<std::vector<uint32_t>, std::array<uint32_t, 3>>
SVulkan2Window::downloadUint32Target(std::string const &name) {
  if (mSVulkanRenderer->getRenderTarget(name)->getFormat() != vk::Format::eR32G32B32A32Uint) {
    throw std::runtime_error("failed to download: " + name + " is not a uint32 render target.");
  }
  return mSVulkanRenderer->download<uint32_t>(name);
}

std::tuple<std::vector<uint8_t>, std::array<uint32_t, 3>>
SVulkan2Window::downloadUint8Target(std::string const &name) {
  if (mSVulkanRenderer->getRenderTarget(name)->getFormat() != vk::Format::eR8G8B8A8Unorm) {
    throw std::runtime_error("failed to download: " + name + " is not a uint8 render target.");
  }
  return mSVulkanRenderer->download<uint8_t>(name);
}

std::vector<float> SVulkan2Window::downloadFloatTargetPixel(std::string const &name, uint32_t x,
                                                            uint32_t y) {
  auto format = mSVulkanRenderer->getRenderTarget(name)->getFormat();
  if (format != vk::Format::eR32G32B32A32Sfloat && format != vk::Format::eD32Sfloat) {
    throw std::runtime_error("failed to download: " + name + " is not a float render target.");
  }
  return std::get<0>(mSVulkanRenderer->downloadRegion<float>(
      name, vk::Offset2D{static_cast<int>(x), static_cast<int>(y)}, vk::Extent2D{1, 1}));
}

std::vector<uint32_t> SVulkan2Window::downloadUint32TargetPixel(std::string const &name,
                                                                uint32_t x, uint32_t y) {
  if (mSVulkanRenderer->getRenderTarget(name)->getFormat() != vk::Format::eR32G32B32A32Uint) {
    throw std::runtime_error("failed to download: " + name + " is not a uint32 render target.");
  }
  return std::get<0>(mSVulkanRenderer->downloadRegion<uint32_t>(
      name, vk::Offset2D{static_cast<int>(x), static_cast<int>(y)}, vk::Extent2D{1, 1}));
}

std::vector<uint8_t> SVulkan2Window::downloadUint8TargetPixel(std::string const &name, uint32_t x,
                                                              uint32_t y) {
  if (mSVulkanRenderer->getRenderTarget(name)->getFormat() != vk::Format::eR8G8B8A8Unorm) {
    throw std::runtime_error("failed to download: " + name + " is not a uint8 render target.");
  }
  return std::get<0>(mSVulkanRenderer->downloadRegion<uint8_t>(
      name, vk::Offset2D{static_cast<int>(x), static_cast<int>(y)}, vk::Extent2D{1, 1}));
}

bool SVulkan2Window::isKeyDown(std::string const &key) { return mWindow->isKeyDown(key); }
bool SVulkan2Window::isKeyPressed(std::string const &key) { return mWindow->isKeyPressed(key); }
bool SVulkan2Window::isMouseKeyDown(int key) { return mWindow->isMouseKeyDown(key); }
bool SVulkan2Window::isMouseKeyClicked(int key) { return mWindow->isMouseKeyClicked(key); }

std::array<float, 2> SVulkan2Window::getMousePosition() {
  auto [x, y] = mWindow->getMousePosition();
  return {x, y};
}
std::array<float, 2> SVulkan2Window::getMouseDelta() {
  auto [x, y] = mWindow->getMouseDelta();
  return {x, y};
}
std::array<float, 2> SVulkan2Window::getMouseWheelDelta() {
  auto [x, y] = mWindow->getMouseDelta();
  return {x, y};
}

} // namespace Renderer
} // namespace sapien
