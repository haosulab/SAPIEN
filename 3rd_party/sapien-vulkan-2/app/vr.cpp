#include "camera_controller.hpp"
#include "svulkan2/common/fs.h"
#include "svulkan2/core/context.h"
#include "svulkan2/renderer/renderer.h"
#include "svulkan2/renderer/rt_renderer.h"
#include "svulkan2/scene/scene.h"
#include "svulkan2/shader/compute.h"
#include "svulkan2/shader/shader.h"
#include "svulkan2/ui/ui.h"
#include <chrono>
#include <csignal>
#include <iostream>

#include "svulkan2/renderer/vr.h"

// clang-format off
#include <imgui.h>
#include <ImGuizmo.h>
#include <ImGuiFileDialog.h>
// clang-format on

// #define STB_IMAGE_WRITE_IMPLEMENTATION
// #include <stb_image_write.h>

using namespace svulkan2;

static fs::path assetDir;

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

bool exited = false;
void handleSIGINT(int signal) {
  exited = true;
  std::cout << "SIGINT received. Exiting gracefully." << std::endl;
}

int main() {
  // renderer::VRDisplay::setActionManifestPath("/home/fx/.sapien/steamvr_actions.json");

  assetDir = fs::path(__FILE__).parent_path() / "assets";

  svulkan2::logger::setLogLevel("info");

  auto context = svulkan2::core::Context::Create(true, 5000, 5000, 4);
  auto manager = context->createResourceManager();
  auto config = std::make_shared<RendererConfig>();

  config->shaderDir = "../shader/default";
  config->msaa = vk::SampleCountFlagBits::e1;
  config->colorFormat4 = vk::Format::eR32G32B32A32Sfloat;

  auto rendererLeft = std::make_shared<renderer::Renderer>(config);
  auto rendererRight = std::make_shared<renderer::Renderer>(config);

  auto vr = renderer::VRDisplay();
  auto [width, height] = vr.getScreenSize();

  auto scene = std::make_shared<svulkan2::scene::Scene>();

  rendererLeft->resize(width, height);
  rendererRight->resize(width, height);
  rendererLeft->setScene(scene);
  rendererRight->setScene(scene);

  setupGround(*scene);
  setupSphereArray(*scene);
  setupCornellBox(*scene, *manager);
  setupMonkey(*scene, *manager);

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

  auto leftCamPose = vr.getEyePoseLeft();
  auto rightCamPose = vr.getEyePoseRight();

  // camera
  auto &cameraLeftNode = scene->addCamera();
  auto &cameraRightNode = scene->addCamera();

  {
    auto frustumLeft = vr.getCameraFrustumLeft();
    float f = math::clip2focal(frustumLeft.left, frustumLeft.right, width);
    float cx = math::clip2principal(frustumLeft.left, frustumLeft.right, width);
    float cy = math::clip2principal(-frustumLeft.top, -frustumLeft.bottom, height);
    cameraLeftNode.setPerspectiveParameters(0.05, 50, f, f, cx, cy, width, height, 0.f);
  }

  {
    auto frustumRight = vr.getCameraFrustumRight();
    float f = math::clip2focal(frustumRight.left, frustumRight.right, width);
    float cx = math::clip2principal(frustumRight.left, frustumRight.right, width);
    float cy = math::clip2principal(-frustumRight.top, -frustumRight.bottom, height);
    cameraRightNode.setPerspectiveParameters(0.05, 50, f, f, cx, cy, width, height, 0.f);
  }

  auto cubemap = context->getResourceManager()->CreateCubemapFromFile(
      assetDir / "rosendal_mountain_midmorning_4k.exr", 5);
  scene->setEnvironmentMap(cubemap);

  context->getDevice().waitIdle();
  vk::UniqueSemaphore sceneRenderSemaphore = context->getDevice().createSemaphoreUnique({});
  vk::UniqueFence sceneRenderFence =
      context->getDevice().createFenceUnique({vk::FenceCreateFlagBits::eSignaled});

  context->getDevice().waitIdle();
  scene->updateModelMatrices();

  auto pool = context->createCommandPool();
  auto cb = pool->allocateCommandBuffer();

  std::signal(SIGINT, handleSIGINT);

  int count = 0;
  while (!exited) {
    count += 1;

    // wait for previous frame to finish
    {
      if (context->getDevice().waitForFences(sceneRenderFence.get(), VK_TRUE, UINT64_MAX) !=
          vk::Result::eSuccess) {
        throw std::runtime_error("Failed on wait for fence.");
      }
      context->getDevice().resetFences(sceneRenderFence.get());
    }

    vr.updatePoses();

    glm::vec3 scale;
    glm::quat quat;
    glm::vec3 pos;
    glm::vec3 skew;
    glm::vec4 pers;

    glm::decompose(vr.getHMDPose(), scale, quat, pos, skew, pers);

    glm::decompose(vr.getHMDPose() * leftCamPose, scale, quat, pos, skew, pers);
    cameraLeftNode.setTransform({.position = pos, .rotation = quat});

    glm::decompose(vr.getHMDPose() * rightCamPose, scale, quat, pos, skew, pers);
    cameraRightNode.setTransform({.position = pos, .rotation = quat});

    vr.getSkeletalDataLeft();

    scene->updateModelMatrices();
    {
      rendererLeft->render(cameraLeftNode, std::vector<vk::Semaphore>{}, {}, {}, {});
      rendererRight->render(cameraRightNode, std::vector<vk::Semaphore>{}, {}, {}, {});

      context->getDevice().waitIdle();

      auto &imageLeft = rendererLeft->getRenderImage("Color");
      auto &imageRight = rendererRight->getRenderImage("Color");

      cb->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
      imageLeft.transitionLayout(
          cb.get(), rendererLeft->getRenderTargetImageLayout("Color"),
          vk::ImageLayout::eTransferSrcOptimal, vk::AccessFlagBits::eMemoryWrite,
          vk::AccessFlagBits::eTransferRead, vk::PipelineStageFlagBits::eAllGraphics,
          vk::PipelineStageFlagBits::eTransfer);
      imageRight.transitionLayout(
          cb.get(), rendererRight->getRenderTargetImageLayout("Color"),
          vk::ImageLayout::eTransferSrcOptimal, vk::AccessFlagBits::eMemoryWrite,
          vk::AccessFlagBits::eTransferRead, vk::PipelineStageFlagBits::eAllGraphics,
          vk::PipelineStageFlagBits::eTransfer);
      cb->end();

      context->getQueue().submit(cb.get(), sceneRenderFence.get());

      vr.renderFrame(imageLeft, imageRight);
    }
  }

  context->getDevice().waitIdle();
  return 0;
}
