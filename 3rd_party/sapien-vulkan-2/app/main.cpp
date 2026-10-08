#include "camera_controller.hpp"
#include "svulkan2/common/fs.h"
#include "svulkan2/core/context.h"
#include "svulkan2/renderer/renderer.h"
#include "svulkan2/renderer/rt_renderer.h"
#include "svulkan2/scene/scene.h"
#include "svulkan2/shader/compute.h"
#include "svulkan2/shader/shader.h"
#include "svulkan2/ui/ui.h"
#include <GLFW/glfw3.h>
#include <chrono>
#include <iostream>

// clang-format off
#include <imgui.h>
#include <ImGuizmo.h>
#include <ImGuiFileDialog.h>
// clang-format on

// #define STB_IMAGE_WRITE_IMPLEMENTATION
// #include <stb_image_write.h>

using namespace svulkan2;

static bool gSwapchainRebuild = true;
static bool gClosed = false;

static fs::path assetDir;

static void glfw_resize_callback(GLFWwindow *, int w, int h) { gSwapchainRebuild = true; }

static void window_close_callback(GLFWwindow *window) {
  gClosed = true;
  glfwSetWindowShouldClose(window, GLFW_FALSE);
}

static void setupSphereArray(svulkan2::scene::Scene &scene) {
  for (uint32_t i = 0; i < 3; ++i) {
    for (uint32_t j = 0; j < 3; ++j) {
      float metallic = i / 5.f;
      float roughness = j / 5.f;
      float specular = 0.5f;
      auto shape = resource::SVShape::Create(
          resource::SVMesh::CreateUVSphere(32, 16),
          std::make_shared<resource::SVMetallicMaterial>(
              glm::vec4{0, 0, 0, 1}, glm::vec4{1, 1, 1, 1}, specular, roughness, metallic));
      auto &o = scene.addObject(
          resource::SVModel::FromData({shape}),
          {.position = {((int)i - 1) / 8.f, 0.2 + j / 8.f, 0}, .scale = {0.05, 0.05, 0.05}});
      o.setSegmentation({0, 0, i, j});
    }
  }
}

static void setupGround(svulkan2::scene::Scene &scene) {
  auto plane = resource::SVMesh::CreateYZPlane();
  auto shape =
      resource::SVShape::Create(plane, std::make_shared<resource::SVMetallicMaterial>(
                                           glm::vec4{0, 0, 0, 1}, glm::vec4{0.8, 0.8, 0.8, 1.0}));
  scene.addObject(resource::SVModel::FromData({shape}),
                  {.rotation = {0.7071068, 0, 0, 0.7071068}, .scale = {10, 10, 10}});
}

static void setupCornellBox(svulkan2::scene::Scene &scene, resource::SVResourceManager &manager) {
  auto model = manager.CreateModelFromFile(assetDir / "cornell-box.glb");
  auto &o = scene.addObject(model);
  o.setTransform({.position = {-0.5, 0.01, 0}, .scale = {0.1, 0.1, 0.1}});
}

static void setupMonkey(svulkan2::scene::Scene &scene, resource::SVResourceManager &manager) {
  auto model = manager.CreateModelFromFile(assetDir / "monkey.glb");
  auto &o = scene.addObject(model);
  o.setTransform({.position = {0.5, 0.075, 0}, .scale = {0.05, 0.05, 0.05}});
}

static void setupPoints(svulkan2::scene::Scene &scene, resource::SVResourceManager &manager) {
  {
    auto pointset = std::make_shared<resource::SVPointSet>(2);
    pointset->setVertexAttribute("position", std::vector<float>{0, 0.2, 0, 0, 0, 0});
    pointset->setVertexAttribute("color", std::vector<float>{1, 0, 0, 1, 0, 1, 0, 1});
    pointset->setVertexAttribute("scale", std::vector<float>{0.05, 0.1});

    auto &o = scene.addPointObject(pointset);
    o.setTransform({.position = {0.7, 0.1, 0}, .scale = {1, 1, 1}});
  }

  {
    auto pointset = std::make_shared<resource::SVPointSet>(2);
    pointset->setVertexAttribute("position", std::vector<float>{0, 0.2, 0, 0, 0, 0});
    pointset->setVertexAttribute("color", std::vector<float>{0, 0, 1, 1, 0, 1, 1, 1});
    pointset->setVertexAttribute("scale", std::vector<float>{0.1, 0.05});

    auto &o = scene.addPointObject(pointset);
    o.setTransform({.position = {-0.7, 0.1, 0}, .scale = {1, 1, 1}});
  }
}

int main() {
  assetDir = fs::path(__FILE__).parent_path() / "assets";

  svulkan2::logger::setLogLevel("info");

  auto context = svulkan2::core::Context::Create(5000, 5000, 4);
  auto manager = context->createResourceManager();
  auto config = std::make_shared<RendererConfig>();

  config->shaderDir = "../shader/default";
  config->msaa = vk::SampleCountFlagBits::e1;
  config->colorFormat4 = vk::Format::eR32G32B32A32Sfloat;

  // auto renderer = std::make_shared<renderer::Renderer>(config);
  auto renderer = std::make_shared<renderer::RTRenderer>("../shader/rt");

  // renderer->enableDenoiser(renderer::RTRenderer::DenoiserType::eOIDN, "HdrColor", "Albedo",
  //                          "Normal");

  auto scene = std::make_shared<svulkan2::scene::Scene>();

  // camera
  auto &cameraNode = scene->addCamera();
  cameraNode.setPerspectiveParameters(0.05, 50, 1.05, 1920, 1080);
  FPSCameraController controller(cameraNode, {0, 0, -1}, {0, 1, 0});
  controller.setXYZ(0, 0.3, 1);

  setupGround(*scene);
  setupSphereArray(*scene);
  setupCornellBox(*scene, *manager);
  setupMonkey(*scene, *manager);
  setupPoints(*scene, *manager);

  // lights
  auto &dl = scene->addDirectionalLight();
  dl.setPosition({0, 0, 0});
  dl.setDirection({0, -1, 0.1});
  dl.setColor({0.5, 0.5, 0.5});
  dl.enableShadow(true);
  dl.setShadowParameters(-10, 10, 0.1, 2048); // NOTE scale of 0.1 clips shadow in raster mode

  auto &pl = scene->addPointLight();
  pl.setPosition({-0.5, 0.5, 0});
  pl.setColor({0.0, 0.2, 0.2});
  pl.enableShadow(true);
  pl.setShadowParameters(0.01, 10, 2048);

  auto &tl = scene->addTexturedLight();
  tl.setPosition({0.5, 0.75, 0});
  tl.setDirection({0, -1, 0.2});
  tl.setFov(1.5);
  tl.setFovSmall(1.5);
  tl.setColor({1, 0, 0});
  tl.enableShadow(true);
  tl.setShadowParameters(0.05, 10, 2048);
  tl.setTexture(context->getResourceManager()->CreateTextureFromFile(
      assetDir / "flashlight.jpg", 1, vk::Filter::eLinear, vk::Filter::eLinear,
      vk::SamplerAddressMode::eClampToBorder, vk::SamplerAddressMode::eClampToBorder, true));

  auto cubemap = context->getResourceManager()->CreateCubemapFromFile(
      assetDir / "rosendal_mountain_midmorning_4k.exr", 5);
  scene->setEnvironmentMap(cubemap);

  auto window = context->createWindow(1920, 1080);

  window->initImgui();
  context->getDevice().waitIdle();
  vk::UniqueSemaphore sceneRenderSemaphore = context->getDevice().createSemaphoreUnique({});
  vk::UniqueFence sceneRenderFence =
      context->getDevice().createFenceUnique({vk::FenceCreateFlagBits::eSignaled});

  // setup glfw
  glfwSetFramebufferSizeCallback(window->getGLFWWindow(), glfw_resize_callback);
  glfwSetWindowCloseCallback(window->getGLFWWindow(), window_close_callback);

  renderer->setScene(scene);

  context->getDevice().waitIdle();

  auto gizmo = ui::Widget::Create<ui::Gizmo>()->Matrix(glm::mat4(1));

  auto filechooser = ui::Widget::Create<ui::FileChooser>();

  auto uiWindow = ui::Widget::Create<ui::Window>()
                      ->Size({400, 400})
                      ->Label("main window")
                      ->append(gizmo)
                      ->append(filechooser);

  scene->updateModelMatrices();

  int count = 0;
  while (!window->isClosed()) {
    count += 1;

    if (gSwapchainRebuild) {
      context->getDevice().waitIdle();
      int width, height;
      glfwGetFramebufferSize(window->getGLFWWindow(), &width, &height);
      if (!window->updateSize(width, height)) {
        continue;
      }
      gSwapchainRebuild = false;
      renderer->resize(width, height);
      context->getDevice().waitIdle();
      cameraNode.setWidth(window->getWidth());
      cameraNode.setHeight(window->getHeight());
      continue;
    }

    try {
      window->newFrame();
    } catch (vk::OutOfDateKHRError &e) {
      gSwapchainRebuild = true;
      context->getDevice().waitIdle();
      continue;
    }

    ImGui::NewFrame();
    ImGuizmo::BeginFrame();

    ImGuiWindowFlags flags = ImGuiWindowFlags_MenuBar;
    flags |= ImGuiWindowFlags_NoDocking;
    ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->Pos);
    ImGui::SetNextWindowSize(viewport->Size);
    ImGui::SetNextWindowViewport(viewport->ID);
    ImGui::SetNextWindowBgAlpha(0.f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
             ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::Begin("DockSpace Demo", 0, flags);
    ImGui::PopStyleVar();

    filechooser->open();

    ImGui::DockSpace(ImGui::GetID("Dockspace"), ImVec2(0, 0),
                     ImGuiDockNodeFlags_PassthruCentralNode |
                         ImGuiDockNodeFlags_NoDockingInCentralNode);
    ImGui::End();
    ImGui::PopStyleVar();

    ImGui::ShowDemoWindow();

    uiWindow->build();

    ImGui::Render();

    // wait for previous frame to finish
    {
      if (context->getDevice().waitForFences(sceneRenderFence.get(), VK_TRUE, UINT64_MAX) !=
          vk::Result::eSuccess) {
        throw std::runtime_error("Failed on wait for fence.");
      }
      context->getDevice().resetFences(sceneRenderFence.get());
    }

    {
      renderer->render(cameraNode, std::vector<vk::Semaphore>{}, {}, {}, {});
      auto imageAcquiredSemaphore = window->getImageAcquiredSemaphore();
      renderer->display("Color", window->getBackbuffer(), window->getBackBufferFormat(),
                        window->getWidth(), window->getHeight(), {imageAcquiredSemaphore},
                        {vk::PipelineStageFlagBits::eColorAttachmentOutput},
                        {sceneRenderSemaphore.get()}, {});
    }

    try {
      window->presentFrameWithImgui(sceneRenderSemaphore.get(), sceneRenderFence.get());
    } catch (vk::OutOfDateKHRError &e) {
      gSwapchainRebuild = true;
      context->getDevice().waitIdle();
    }

    // context->getDevice().waitIdle();

    if (gClosed) {
      window->close();
      break;
    }

    bool update = false;

    // user input
    float r = 1e-2;
    if (window->isMouseKeyDown(1)) {
      auto [x, y] = window->getMouseDelta();
      controller.rotate(0, -r * y, -r * x);
      update = true;
    }
    if (window->isKeyDown("w")) {
      controller.move(r, 0, 0);
      update = true;
    }
    if (window->isKeyDown("s")) {
      controller.move(-r, 0, 0);
      update = true;
    }
    if (window->isKeyDown("a")) {
      controller.move(0, r, 0);
      update = true;
    }
    if (window->isKeyDown("d")) {
      controller.move(0, -r, 0);
      update = true;
    }

    if (update) {
      scene->updateModelMatrices();
    }

    // update gizmo
    {
      auto model = cameraNode.computeWorldModelMatrix();
      auto view = glm::affineInverse(model);
      auto proj = cameraNode.getProjectionMatrix();
      proj[1][1] *= -1;
      proj[2][1] *= -1;
      gizmo->setCameraParameters(view, proj);
    }
  }

  context->getDevice().waitIdle();

  return 0;
}
