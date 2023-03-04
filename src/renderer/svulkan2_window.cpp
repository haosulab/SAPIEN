#include "sapien/renderer/svulkan2_window.h"
#include "sapien/renderer/render_config.h"
#include <easy/profiler.h>

namespace sapien {
namespace Renderer {

#ifdef _DEBUG_VIEWER
FPSCameraControllerDebug::FPSCameraControllerDebug(svulkan2::scene::Node &node,
                                                   glm::vec3 const &forward, glm::vec3 const &up)
    : pCamera(&node), mForward(glm::normalize(forward)), mUp(glm::normalize(up)),
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
  pCamera->setTransform({.position = mXYZ, .rotation = rotation});
}
#endif

SVulkan2Window::SVulkan2Window(std::shared_ptr<SVulkan2Renderer> renderer, int width, int height,
                               std::string const &shaderDir)
    : mRenderer(renderer), mShaderDir(shaderDir) {
  auto &renderConfig = GetRenderConfig();

  auto config = std::make_shared<svulkan2::RendererConfig>();
  *config = *renderer->getDefaultRendererConfig();

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

  config->shaderDir = mShaderDir.length() ? mShaderDir : renderConfig.viewerShaderDirectory;

  mSVulkanRenderer = svulkan2::renderer::RendererBase::Create(config);

  // glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
  mWindow = renderer->getContext()->createWindow(width, height);
  mWindow->initImgui();
  renderer->getContext()->getDevice().waitIdle();

  mViewportWidth = width;
  mViewportHeight = height;

  if (auto rtRenderer = dynamic_cast<svulkan2::renderer::RTRenderer *>(mSVulkanRenderer.get())) {
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

void SVulkan2Window::setShader(std::string const &shaderDir) {
  mRequiresRebuild = true;
  mRenderer->getContext()->getDevice().waitIdle();

  auto config = std::make_shared<svulkan2::RendererConfig>();
  *config = *mRenderer->getDefaultRendererConfig();
  config->shaderDir = shaderDir;
  mSVulkanRenderer = svulkan2::renderer::RendererBase::Create(config);
  if (mScene) {
    mSVulkanRenderer->setScene(mScene->getScene());
  }
  mSVulkanRenderer->resize(mViewportWidth, mViewportHeight);
}

void SVulkan2Window::setDropCallback(std::function<void(std::vector<std::string>)> callback) {
  mWindow->setDropCallback(callback);
}

void SVulkan2Window::unsetDropCallback() { mWindow->unsetDropCallback(); }

SVulkan2Window::~SVulkan2Window() {
  if (!mClosed) {
    close();
  }
}

void SVulkan2Window::close() {
  mClosed = true;
  mRenderer->getContext()->getDevice().waitIdle();
  mWindow->close();
  mRenderer->getContext()->getDevice().waitIdle();
}

void SVulkan2Window::hide() { glfwHideWindow(mWindow->getGLFWWindow()); }

void SVulkan2Window::show() { glfwShowWindow(mWindow->getGLFWWindow()); }

void SVulkan2Window::setScene(SVulkan2Scene *scene) {
  mScene = scene;
  mSVulkanRenderer->setScene(mScene->getScene());
}

void SVulkan2Window::setCameraParameters(float near, float far, float fovy) {
  uint32_t width = std::max(mWindow->getWidth(), 1u);
  uint32_t height = std::max(mWindow->getHeight(), 1u);
  getCamera()->setPerspectiveParameters(near, far, fovy, width, height);
}
void SVulkan2Window::setCameraIntrinsicParameters(float near, float far, float fx, float fy,
                                                  float cx, float cy, float skew) {
  uint32_t width = std::max(mWindow->getWidth(), 1u);
  uint32_t height = std::max(mWindow->getHeight(), 1u);
  getCamera()->setPerspectiveParameters(near, far, fx, fy, cx, cy, width, height, skew);
}

void SVulkan2Window::setCameraPosition(glm::vec3 const &pos) { getCamera()->setPosition(pos); }
void SVulkan2Window::setCameraRotation(glm::quat const &rot) { getCamera()->setRotation(rot); }

void SVulkan2Window::setCameraProperty(std::string const &name, float property) {
  mSVulkanRenderer->setCustomProperty(name, property);
}
void SVulkan2Window::setCameraProperty(std::string const &name, int property) {
  if (auto rtRenderer = dynamic_cast<svulkan2::renderer::RTRenderer *>(mSVulkanRenderer.get())) {
    if (name == "_denoiser") {
      if (property == 0) {
        rtRenderer->disableDenoiser();
      } else {
        rtRenderer->enableDenoiser("HdrColor", "Albedo", "Normal");
      }
    }
  }
  mSVulkanRenderer->setCustomProperty(name, property);
}

glm::vec3 SVulkan2Window::getCameraPosition() { return getCamera()->getPosition(); }
glm::quat SVulkan2Window::getCameraRotation() { return getCamera()->getRotation(); }
glm::mat4 SVulkan2Window::getCameraProjectionMatrix() {
  return getCamera()->getProjectionMatrix();
}

float SVulkan2Window::getCameraNear() { return getCamera()->getNear(); }
float SVulkan2Window::getCameraFar() { return getCamera()->getFar(); }
float SVulkan2Window::getCameraFovy() { return getCamera()->getFovy(); };

std::vector<std::string> SVulkan2Window::getDisplayTargetNames() const {
  return mSVulkanRenderer->getDisplayTargetNames();
}

void SVulkan2Window::rebuild() {
  mRenderer->getContext()->getDevice().waitIdle();
  do {
    glfwGetFramebufferSize(mWindow->getGLFWWindow(), &mViewportWidth, &mViewportHeight);
  } while (!mWindow->updateSize(mViewportWidth, mViewportHeight));
  mSceneRenderSemaphore = mRenderer->getContext()->getDevice().createSemaphoreUnique({});
  mSceneRenderFence =
      mRenderer->getContext()->getDevice().createFenceUnique({vk::FenceCreateFlagBits::eSignaled});
  mSVulkanRenderer->resize(mViewportWidth, mViewportHeight);
  mRenderer->getContext()->getDevice().waitIdle();
  mRequiresRebuild = false;

  if (mScene) {
    auto cam = getCamera();
    uint32_t width = std::max(mWindow->getWidth(), 1u);
    uint32_t height = std::max(mWindow->getHeight(), 1u);
    cam->setPerspectiveParameters(cam->getNear(), cam->getFar(), cam->getFovy(), width, height);
  }
}

void SVulkan2Window::resize(int width, int height) {
  glfwSetWindowSize(mWindow->getGLFWWindow(), width, height);
  mRequiresRebuild = true;
}

void SVulkan2Window::render(std::string const &targetName,
                            std::vector<std::shared_ptr<svulkan2::ui::Window>> uiWindows) {

  if (!mScene) {
    return;
  }
  if (mRequiresRebuild) {
    rebuild();
  }

  svulkan2::scene::Camera *camera = getCamera();

  try {
    mWindow->newFrame();
  } catch (vk::OutOfDateKHRError &e) {
    mRequiresRebuild = true;
    return;
  }

  ImGui::NewFrame();
  ImGuizmo::BeginFrame();

  // setup docking window
  ImGuiWindowFlags flags = ImGuiWindowFlags_MenuBar;
  flags |= ImGuiWindowFlags_NoDocking;
  ImGuiViewport *viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(viewport->Pos);
  ImGui::SetNextWindowSize(viewport->Size);
  ImGui::SetNextWindowViewport(viewport->ID);
  ImGui::SetNextWindowBgAlpha(0.f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
  flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize |
           ImGuiWindowFlags_NoMove;
  flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
  ImGui::Begin("DockSpace Demo", 0, flags);
  ImGui::PopStyleVar();

  ImGui::DockSpace(ImGui::GetID("Dockspace"), ImVec2(0, 0),
                   ImGuiDockNodeFlags_PassthruCentralNode |
                       ImGuiDockNodeFlags_NoDockingInCentralNode);
  ImGui::End();
  ImGui::PopStyleVar();

  if (uiWindows.size() == 0) {
    ImGui::ShowDemoWindow();
  }
  for (auto w : uiWindows) {
    if (!w) {
      throw std::runtime_error("failed to render UI windows: a windows is null");
    }
    w->build();
  }
  ImGui::Render();
  if (mRenderer->getContext()->getDevice().waitForFences(mSceneRenderFence.get(), VK_TRUE,
                                                         UINT64_MAX) != vk::Result::eSuccess) {
    throw std::runtime_error("failed on wait for fence.");
  }
  mRenderer->getContext()->getDevice().resetFences(mSceneRenderFence.get());

  {
    // draw
    mSVulkanRenderer->render(*camera, {}, {}, {}, {});
    auto imageAcquiredSemaphore = mWindow->getImageAcquiredSemaphore();
    mSVulkanRenderer->display(targetName, mWindow->getBackbuffer(), mWindow->getBackBufferFormat(),
                              mWindow->getWidth(), mWindow->getHeight(), {imageAcquiredSemaphore},
                              {vk::PipelineStageFlagBits::eColorAttachmentOutput},
                              {mSceneRenderSemaphore.get()}, {});
  }

  auto swapchain = mWindow->getSwapchain();
  auto fidx = mWindow->getFrameIndex();
  vk::PresentInfoKHR info(1, &mSceneRenderSemaphore.get(), 1, &swapchain, &fidx);
  try {
    mWindow->presentFrameWithImgui(mSceneRenderSemaphore.get(), mSceneRenderFence.get());
  } catch (vk::OutOfDateKHRError &e) {
    mRequiresRebuild = true;
    return;
  }

#ifdef _DEBUG_VIEWER
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

  uint32_t width = std::max(mWindow->getWidth(), 1u);
  uint32_t height = std::max(mWindow->getHeight(), 1u);

  camera->setPerspectiveParameters(0.1, 10, 1, width, height);
  camera->setName("_controller");
  camera->setPosition({0, 0, 0});
  return camera;
}

std::tuple<std::vector<float>, std::array<uint32_t, 3>>
SVulkan2Window::downloadFloatTarget(std::string const &name) {
  auto format = mSVulkanRenderer->getRenderImage(name).getFormat();
  if (format != vk::Format::eR32G32B32A32Sfloat && format != vk::Format::eD32Sfloat) {
    throw std::runtime_error("failed to download: " + name + " is not a float render target.");
  }
  return mSVulkanRenderer->download<float>(name);
}

std::array<uint32_t, 2> SVulkan2Window::getRenderTargetSize(std::string const &name) const {
  auto &image = mSVulkanRenderer->getRenderImage(name);
  return {image.getExtent().width, image.getExtent().height};
}

std::tuple<std::vector<uint32_t>, std::array<uint32_t, 3>>
SVulkan2Window::downloadUint32Target(std::string const &name) {
  if (mSVulkanRenderer->getRenderImage(name).getFormat() != vk::Format::eR32G32B32A32Uint) {
    throw std::runtime_error("failed to download: " + name + " is not a uint32 render target.");
  }
  return mSVulkanRenderer->download<uint32_t>(name);
}

std::tuple<std::vector<uint8_t>, std::array<uint32_t, 3>>
SVulkan2Window::downloadUint8Target(std::string const &name) {
  if (mSVulkanRenderer->getRenderImage(name).getFormat() != vk::Format::eR8G8B8A8Unorm) {
    throw std::runtime_error("failed to download: " + name + " is not a uint8 render target.");
  }
  return mSVulkanRenderer->download<uint8_t>(name);
}

std::vector<float> SVulkan2Window::downloadFloatTargetPixel(std::string const &name, uint32_t x,
                                                            uint32_t y) {
  auto format = mSVulkanRenderer->getRenderImage(name).getFormat();
  if (format != vk::Format::eR32G32B32A32Sfloat && format != vk::Format::eD32Sfloat) {
    throw std::runtime_error("failed to download: " + name + " is not a float render target.");
  }
  return std::get<0>(mSVulkanRenderer->downloadRegion<float>(
      name, vk::Offset2D{static_cast<int>(x), static_cast<int>(y)}, vk::Extent2D{1, 1}));
}

std::vector<uint32_t> SVulkan2Window::downloadUint32TargetPixel(std::string const &name,
                                                                uint32_t x, uint32_t y) {
  if (mSVulkanRenderer->getRenderImage(name).getFormat() != vk::Format::eR32G32B32A32Uint) {
    throw std::runtime_error("failed to download: " + name + " is not a uint32 render target.");
  }
  return std::get<0>(mSVulkanRenderer->downloadRegion<uint32_t>(
      name, vk::Offset2D{static_cast<int>(x), static_cast<int>(y)}, vk::Extent2D{1, 1}));
}

std::vector<uint8_t> SVulkan2Window::downloadUint8TargetPixel(std::string const &name, uint32_t x,
                                                              uint32_t y) {
  if (mSVulkanRenderer->getRenderImage(name).getFormat() != vk::Format::eR8G8B8A8Unorm) {
    throw std::runtime_error("failed to download: " + name + " is not a uint8 render target.");
  }
  return std::get<0>(mSVulkanRenderer->downloadRegion<uint8_t>(
      name, vk::Offset2D{static_cast<int>(x), static_cast<int>(y)}, vk::Extent2D{1, 1}));
}

bool SVulkan2Window::isShiftDown() { return mWindow->isShiftDown(); }
bool SVulkan2Window::isCtrlDown() { return mWindow->isCtrlDown(); }
bool SVulkan2Window::isAltDown() { return mWindow->isAltDown(); }
bool SVulkan2Window::isSuperDown() { return mWindow->isSuperDown(); }

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
  auto [x, y] = mWindow->getMouseWheelDelta();
  return {x, y};
}

std::array<int, 2> SVulkan2Window::getWindowSize() {
  int width, height;
  glfwGetWindowSize(mWindow->getGLFWWindow(), &width, &height);
  return {width, height};
}

float SVulkan2Window::getFPS() { return ImGui::GetIO().Framerate; }

void SVulkan2Window::setCursorEnabled(bool enabled) { mWindow->setCursorEnabled(enabled); }

bool SVulkan2Window::getCursorEnabled() const { return mWindow->getCursorEnabled(); }

} // namespace Renderer
} // namespace sapien
