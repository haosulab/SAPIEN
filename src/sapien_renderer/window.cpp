/*
 * Copyright 2025 Hillbot Inc.
 * Copyright 2020-2024 UCSD SU Lab
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "sapien/sapien_renderer/window.h"
#include "sapien/math/conversion.h"
#include "sapien/sapien_renderer/sapien_renderer_default.h"
#include "sapien/sapien_renderer/sapien_renderer_system.h"
#include <svulkan2/renderer/rt_renderer.h>
#include <svulkan2/scene/scene_group.h>

namespace sapien {
namespace sapien_renderer {

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

SapienRendererWindow::SapienRendererWindow(int width, int height, std::string const &shaderDir)
    : mShaderDir(shaderDir) {
  mEngine = SapienRenderEngine::Get();
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

  config->shaderDir = mShaderDir.length() ? mShaderDir : renderConfig.viewerShaderDirectory;

  mSVulkanRenderer = svulkan2::renderer::RendererBase::Create(config);

  mWindow = mEngine->getContext()->createWindow(width, height);
  mWindow->initImgui();
  mEngine->getContext()->getDevice().waitIdle();

  mViewportWidth = width;
  mViewportHeight = height;

  if (auto rtRenderer = dynamic_cast<svulkan2::renderer::RTRenderer *>(mSVulkanRenderer.get())) {
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

void SapienRendererWindow::setShader(std::string const &shaderDir) {
  mRequiresRebuild = true;
  mEngine->getContext()->getDevice().waitIdle();

  auto config = std::make_shared<svulkan2::RendererConfig>();
  config->shaderDir = shaderDir;
  mSVulkanRenderer = svulkan2::renderer::RendererBase::Create(config);
  if (mRenderScene) {
    mSVulkanRenderer->setScene(mRenderScene);
    // mSVulkanRenderer->setScene(mScene->getSapienRendererSystem()->getScene());
  }
  mSVulkanRenderer->resize(mViewportWidth, mViewportHeight);
}

void SapienRendererWindow::setDropCallback(
    std::function<void(std::vector<std::string>)> callback) {
  mWindow->setDropCallback(callback);
}

void SapienRendererWindow::unsetDropCallback() { mWindow->unsetDropCallback(); }

void SapienRendererWindow::setFocusCallback(std::function<void(int)> callback) {
  mWindow->setFocusCallback(callback);
}

void SapienRendererWindow::unsetFocusCallback() { mWindow->unsetFocusCallback(); }

SapienRendererWindow::~SapienRendererWindow() {
  if (!mClosed) {
    close();
  }
}

void SapienRendererWindow::close() {
  mClosed = true;
  mEngine->getContext()->getDevice().waitIdle();
  mWindow->close();
  mEngine->getContext()->getDevice().waitIdle();
}

void SapienRendererWindow::hide() { mWindow->hide(); }

void SapienRendererWindow::show() { mWindow->show(); }

void SapienRendererWindow::setScene(std::shared_ptr<Scene> scene) {
  mRenderSystems = {scene->getSapienRendererSystem()};
  mRenderScene = scene->getSapienRendererSystem()->getScene();
  mSVulkanRenderer->setScene(mRenderScene);
}

void SapienRendererWindow::setScenes(std::vector<std::shared_ptr<Scene>> const &scenes,
                                     std::vector<Vec3> const &offsets) {
  if (scenes.size() <= 1) {
    throw std::runtime_error(
        "failed to set scenes: the function should be called with 2 or more scenes.");
  }
  if (scenes.size() != offsets.size()) {
    throw std::runtime_error("failed to set scenes: scenes and offsets must have the same size.");
  }

  mRenderSystems = {};
  std::vector<std::shared_ptr<svulkan2::scene::Scene>> allScenes;
  std::vector<svulkan2::scene::Transform> allTransforms;
  for (uint32_t i = 0; i < scenes.size(); ++i) {
    auto s = scenes[i];
    mRenderSystems.push_back(s->getSapienRendererSystem());
    allScenes.push_back(s->getSapienRendererSystem()->getScene());
    allTransforms.push_back(
        svulkan2::scene::Transform{.position = {offsets[i].x, offsets[i].y, offsets[i].z}});
  }

  mRenderScene = std::make_shared<svulkan2::scene::SceneGroup>(allScenes, allTransforms);
  mSVulkanRenderer->setScene(mRenderScene);
}

void SapienRendererWindow::updateRender() {
  for (auto &s : mRenderSystems) {
    s->step();
  }

  if (dynamic_cast<svulkan2::scene::SceneGroup *>(mRenderScene.get())) {
    mRenderScene->updateModelMatrices();
  }
}

void SapienRendererWindow::setCameraParameters(float near, float far, float fovy) {
  uint32_t width = std::max(mWindow->getWidth(), 1u);
  uint32_t height = std::max(mWindow->getHeight(), 1u);
  getCamera()->setPerspectiveParameters(near, far, fovy, width, height);
}

void SapienRendererWindow::setCameraIntrinsicParameters(float near, float far, float fx, float fy,
                                                        float cx, float cy, float skew) {
  uint32_t width = std::max(mWindow->getWidth(), 1u);
  uint32_t height = std::max(mWindow->getHeight(), 1u);
  getCamera()->setPerspectiveParameters(near, far, fx, fy, cx, cy, width, height, skew);
}

void SapienRendererWindow::setCameraOrthoParameters(float near, float far, float top) {
  uint32_t width = std::max(mWindow->getWidth(), 1u);
  uint32_t height = std::max(mWindow->getHeight(), 1u);
  getCamera()->setOrthographicParameters(near, far, top, width, height);
}

CameraMode SapienRendererWindow::getCameraMode() {
  switch (getCamera()->getCameraType()) {
  case svulkan2::scene::Camera::Type::eOrthographic:
    return CameraMode::eOrthographic;
  case svulkan2::scene::Camera::Type::ePerspective:
    return CameraMode::ePerspective;
  default:
    throw std::runtime_error("unsupported camera mode");
  }
}

void SapienRendererWindow::setCameraPose(Pose const &pose) {
  auto glpose = pose * POSE_GL_TO_ROS;
  auto cam = getCamera();
  cam->setPosition({glpose.p.x, glpose.p.y, glpose.p.z});
  cam->setRotation({glpose.q.w, glpose.q.x, glpose.q.y, glpose.q.z});
}

Pose SapienRendererWindow::getCameraPose() {
  auto cam = getCamera();
  auto p = cam->getPosition();
  auto q = cam->getRotation();
  auto glpose = Pose({p.x, p.y, p.z}, {q.w, q.x, q.y, q.z});
  return glpose * POSE_GL_TO_ROS.getInverse();
}

void SapienRendererWindow::setCameraPosition(Vec3 const &pos) {
  auto pose = getCameraPose();
  pose.p = {pos.x, pos.y, pos.z};
  setCameraPose(pose);
}
void SapienRendererWindow::setCameraRotation(Quat const &rot) {
  auto pose = getCameraPose();
  pose.q = {rot.w, rot.x, rot.y, rot.z};
  setCameraPose(pose);
}

Vec3 SapienRendererWindow::getCameraPosition() {
  auto pose = getCameraPose();
  return {pose.p.x, pose.p.y, pose.p.z};
}
Quat SapienRendererWindow::getCameraRotation() {
  auto pose = getCameraPose();
  return {pose.q.w, pose.q.x, pose.q.y, pose.q.z};
}

int SapienRendererWindow::getCameraPropertyInt(std::string const &name) const {
  return mSVulkanRenderer->getCustomPropertyInt(name);
}
float SapienRendererWindow::getCameraPropertyFloat(std::string const &name) const {
  return mSVulkanRenderer->getCustomPropertyFloat(name);
}

void SapienRendererWindow::setCameraProperty(std::string const &name, float property) {
  mSVulkanRenderer->setCustomProperty(name, property);
}
void SapienRendererWindow::setCameraProperty(std::string const &name, int property) {
  mSVulkanRenderer->setCustomProperty(name, property);
}

void SapienRendererWindow::setCameraTexture(std::string const &name,
                                            std::shared_ptr<SapienRenderTexture2D> texture) {
  mSVulkanRenderer->setCustomTexture(name, texture->getTexture());
}

void SapienRendererWindow::setCameraTextureArray(
    std::string const &name, std::vector<std::shared_ptr<SapienRenderTexture2D>> textures) {
  std::vector<std::shared_ptr<svulkan2::resource::SVTexture>> sts;
  for (auto tex : textures) {
    sts.push_back(tex->getTexture());
  }
  mSVulkanRenderer->setCustomTextureArray(name, sts);
}
float SapienRendererWindow::getContentScale() { return mWindow->getContentScale(); }
void SapienRendererWindow::setContentScale(float scale) { return mWindow->setContentScale(scale); }

glm::mat4 SapienRendererWindow::getCameraProjectionMatrix() {
  return getCamera()->getProjectionMatrix();
}

Mat4 SapienRendererWindow::getCameraModelMatrix() {
  return PoseToEigenMat4(getCameraPose() * POSE_GL_TO_ROS);
}

float SapienRendererWindow::getCameraNear() { return getCamera()->getNear(); }
float SapienRendererWindow::getCameraFar() { return getCamera()->getFar(); }
float SapienRendererWindow::getCameraFovy() { return getCamera()->getFovy(); }
float SapienRendererWindow::getCameraOrthoTop() { return getCamera()->getOrthographicTop(); }

std::vector<std::string> SapienRendererWindow::getDisplayTargetNames() const {
  return mSVulkanRenderer->getDisplayTargetNames();
}

void SapienRendererWindow::rebuild() {
  mEngine->getContext()->getDevice().waitIdle();
  do {
    auto [vw, vh] = mWindow->getWindowFramebufferSize();
    mViewportWidth = vw;
    mViewportHeight = vh;
  } while (!mWindow->updateSize(mViewportWidth, mViewportHeight));
  mSceneRenderSemaphore = mEngine->getContext()->getDevice().createSemaphoreUnique({});
  mSceneRenderFence =
      mEngine->getContext()->getDevice().createFenceUnique({vk::FenceCreateFlagBits::eSignaled});
  mSVulkanRenderer->resize(mViewportWidth, mViewportHeight);
  mEngine->getContext()->getDevice().waitIdle();
  mRequiresRebuild = false;

  if (mRenderScene) {
    auto cam = getCamera();
    uint32_t width = std::max(mWindow->getWidth(), 1u);
    uint32_t height = std::max(mWindow->getHeight(), 1u);
    cam->setPerspectiveParameters(cam->getNear(), cam->getFar(), cam->getFovy(), width, height);
  }
}

void SapienRendererWindow::resize(int width, int height) {
  mWindow->setWindowSize(width, height);
  mRequiresRebuild = true;
}

void SapienRendererWindow::render(std::string const &targetName,
                                  std::vector<std::shared_ptr<svulkan2::ui::Widget>> uiWindows) {

  if (!mRenderScene) {
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

  mWindow->imguiBeginFrame();

  for (auto w : uiWindows) {
    if (!w) {
      throw std::runtime_error("failed to render UI windows: a windows is null");
    }
    w->build();
  }

  mWindow->imguiRender();
  mWindow->imguiEndFrame();

  if (mEngine->getContext()->getDevice().waitForFences(mSceneRenderFence.get(), VK_TRUE,
                                                       UINT64_MAX) != vk::Result::eSuccess) {
    throw std::runtime_error("failed on wait for fence.");
  }
  mEngine->getContext()->getDevice().resetFences(mSceneRenderFence.get());

  {
    // draw
    mSVulkanRenderer->render(*camera, {}, {}, {}, {});
    auto imageAcquiredSemaphore = mWindow->getImageAcquiredSemaphore();
    mSVulkanRenderer->display(targetName, mWindow->getBackbuffer(), mWindow->getBackBufferFormat(),
                              mWindow->getWidth(), mWindow->getHeight(), {imageAcquiredSemaphore},
                              {vk::PipelineStageFlagBits::eColorAttachmentOutput},
                              {mSceneRenderSemaphore.get()}, 
#ifdef SAPIEN_MACOS
                              {mSceneRenderFence.get()}
#else
                              {}
#endif
                              );
  }

#if !defined(SAPIEN_MACOS)
  auto swapchain = mWindow->getSwapchain();
  auto fidx = mWindow->getFrameIndex();
  vk::PresentInfoKHR info(1, &mSceneRenderSemaphore.get(), 1, &swapchain, &fidx);
#endif
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
#endif
}

bool SapienRendererWindow::windowCloseRequested() { return mWindow->isCloseRequested(); }

svulkan2::scene::Camera *SapienRendererWindow::getCamera() {
  if (!mRenderScene) {
    throw std::runtime_error(
        "failed to operate camera, did you forget to call viewer.set_scene ?");
  }
  auto cams = mRenderScene->getCameras();
  // auto cams = mScene->getSapienRendererSystem()->getScene()->getCameras();
  for (auto cam : cams) {
    if (cam->getName() == "_controller") {
      return cam;
    }
  }
  auto camera = &mRenderScene->addCamera();
  // auto camera = &mScene->getSapienRendererSystem()->getScene()->addCamera();

  uint32_t width = std::max(mWindow->getWidth(), 1u);
  uint32_t height = std::max(mWindow->getHeight(), 1u);

  camera->setPerspectiveParameters(0.1, 10, 1, width, height);
  camera->setName("_controller");
  camera->setPosition({0, 0, 0});
  return camera;
}

SapienRenderImageCpu SapienRendererWindow::getImage(std::string const &name) {
  auto &image = mSVulkanRenderer->getRenderImage(name);
  vk::Format format = image.getFormat();
  uint32_t formatSize = svulkan2::getFormatSize(format);
  uint32_t width = image.getExtent().width;
  uint32_t height = image.getExtent().height;
  size_t size = image.getExtent().width * image.getExtent().height * formatSize;
  mImageBuffers[name].resize(size);
  image.download(mImageBuffers[name].data(), size);
  return SapienRenderImageCpu(width, height, format, mImageBuffers[name].data());
}

CpuArray SapienRendererWindow::getImagePixel(std::string const &name, uint32_t x, uint32_t y) {
  auto &image = mSVulkanRenderer->getRenderImage(name);
  uint32_t width = image.getExtent().width;
  uint32_t height = image.getExtent().height;
  if (x >= width || y >= height) {
    throw std::runtime_error("failed to get pixel: location out of bounds");
  }
  vk::Format format = image.getFormat();
  uint32_t formatSize = svulkan2::getFormatSize(format);
  std::vector<char> buffer(formatSize);
  image.downloadPixel(buffer.data(), formatSize, vk::Offset3D(x, y, 0));

  int channels = svulkan2::getFormatChannels(format);
  int itemsize = getFormatChannelSize(format);
  std::string type = getFormatTypestr(format);

  assert(static_cast<int>(buffer.size()) == channels * itemsize);

  return CpuArray{.shape = {channels}, .type = type, .data = buffer};
}

std::array<uint32_t, 2> SapienRendererWindow::getRenderTargetSize(std::string const &name) const {
  auto &image = mSVulkanRenderer->getRenderImage(name);
  return {image.getExtent().width, image.getExtent().height};
}

bool SapienRendererWindow::isShiftDown() { return mWindow->isShiftDown(); }
bool SapienRendererWindow::isCtrlDown() { return mWindow->isCtrlDown(); }
bool SapienRendererWindow::isAltDown() { return mWindow->isAltDown(); }
bool SapienRendererWindow::isSuperDown() { return mWindow->isSuperDown(); }

bool SapienRendererWindow::isKeyDown(std::string const &key) { return mWindow->isKeyDown(key); }
bool SapienRendererWindow::isKeyPressed(std::string const &key) {
  return mWindow->isKeyPressed(key);
}
bool SapienRendererWindow::isMouseKeyDown(int key) { return mWindow->isMouseKeyDown(key); }
bool SapienRendererWindow::isMouseKeyClicked(int key) { return mWindow->isMouseKeyClicked(key); }

std::array<float, 2> SapienRendererWindow::getMousePosition() {
  auto [x, y] = mWindow->getMousePosition();
  return {x, y};
}
std::array<float, 2> SapienRendererWindow::getMouseDelta() {
  auto [x, y] = mWindow->getMouseDelta();
  return {x, y};
}
std::array<float, 2> SapienRendererWindow::getMouseWheelDelta() {
  auto [x, y] = mWindow->getMouseWheelDelta();
  return {x, y};
}

std::array<int, 2> SapienRendererWindow::getWindowSize() { return mWindow->getWindowSize(); }

float SapienRendererWindow::getFPS() { return mWindow->imguiGetFramerate(); }

void SapienRendererWindow::setCursorEnabled(bool enabled) { mWindow->setCursorEnabled(enabled); }

bool SapienRendererWindow::getCursorEnabled() const { return mWindow->getCursorEnabled(); }

svulkan2::renderer::RTRenderer::DenoiserType SapienRendererWindow::getDenoiserType() const {
  if (auto r = dynamic_cast<svulkan2::renderer::RTRenderer *>(mSVulkanRenderer.get())) {
    return r->getDenoiserType();
  }
  return svulkan2::renderer::RTRenderer::DenoiserType::eNONE;
}

void SapienRendererWindow::setDenoiserType(svulkan2::renderer::RTRenderer::DenoiserType type) {
  if (auto r = dynamic_cast<svulkan2::renderer::RTRenderer *>(mSVulkanRenderer.get())) {
    r->enableDenoiser(type, "HdrColor", "Albedo", "Normal");
  }
}

} // namespace sapien_renderer
} // namespace sapien
