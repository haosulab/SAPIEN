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
#include "svulkan2/renderer/gui.h"
#include "../common/logger.h"
#include "svulkan2/common/fonts/roboto.hpp"
#include "svulkan2/core/context.h"
#include <GLFW/glfw3.h>
#include <filesystem>

// clang-format off
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>
#include <ImGuizmo.h>
// clang-format on

namespace svulkan2 {
namespace renderer {

static void checkVKResult(VkResult err) {
  if (err != 0) {
    logger::error("Vulkan result check failed.");
  }
}

static std::string gImguiIniFileLocation = "";
void GuiWindow::setImguiIniFileLocation(std::string const &path) { gImguiIniFileLocation = path; }
std::string GuiWindow::getImguiIniFileLocation() { return gImguiIniFileLocation; }

static vk::UniqueRenderPass createImguiRenderPass(vk::Device device, vk::Format format) {

  vk::AttachmentDescription attachment{{},
                                       format,
                                       vk::SampleCountFlagBits::e1,
                                       vk::AttachmentLoadOp::eLoad,
                                       vk::AttachmentStoreOp::eStore,
                                       vk::AttachmentLoadOp::eDontCare,
                                       vk::AttachmentStoreOp::eDontCare,
                                       vk::ImageLayout::eColorAttachmentOptimal,
                                       vk::ImageLayout::ePresentSrcKHR};
  vk::AttachmentReference colorAttachment{0, vk::ImageLayout::eColorAttachmentOptimal};

  vk::SubpassDescription subpass({}, vk::PipelineBindPoint::eGraphics, 0, nullptr, 1,
                                 &colorAttachment);
  vk::SubpassDependency dependency{VK_SUBPASS_EXTERNAL,
                                   0,
                                   vk::PipelineStageFlagBits::eAllCommands,
                                   vk::PipelineStageFlagBits::eColorAttachmentOutput,
                                   vk::AccessFlagBits::eMemoryWrite,
                                   vk::AccessFlagBits::eColorAttachmentWrite};

  vk::RenderPassCreateInfo info({}, 1, &attachment, 1, &subpass, 1, &dependency);
  return device.createRenderPassUnique(info);
}

static void windowCallback(GLFWwindow *window, int count, const char **paths) {
  static_cast<GuiWindow *>(glfwGetWindowUserPointer(window))->dropCallback(count, paths);
}

static void windowFocusCallback(GLFWwindow *window, int focused) {
  static_cast<GuiWindow *>(glfwGetWindowUserPointer(window))->focusCallback(focused);
}

GuiWindow::GuiWindow(std::vector<vk::Format> const &requestFormats,
                     vk::ColorSpaceKHR requestColorSpace, uint32_t width, uint32_t height,
                     std::vector<vk::PresentModeKHR> const &requestModes, uint32_t minImageCount)
    : mMinImageCount(minImageCount) {
  mContext = core::Context::Get();
  createGlfwWindow(width, height);
  selectSurfaceFormat(requestFormats, requestColorSpace);
  selectPresentMode(requestModes);
}

void GuiWindow::newFrame() {
  glfwPollEvents();
  mSemaphoreIndex = (mSemaphoreIndex + 1) % mFrameSemaphores.size();
  auto result = mContext->getDevice().acquireNextImageKHR(
      mSwapchain.get(), UINT64_MAX,
      mFrameSemaphores[mSemaphoreIndex].mImageAcquiredSemaphore.get(), {});
  if (result.result == vk::Result::eSuboptimalKHR) {
    throw vk::OutOfDateKHRError("Suboptimal");
  } else if (result.result != vk::Result::eSuccess) {
    throw std::runtime_error("Acquire image failed");
  }
  mFrameIndex = result.value;

  ImGui_ImplVulkan_NewFrame();
  ImGui_ImplGlfw_NewFrame();
}

bool GuiWindow::presentFrameWithImgui(vk::Semaphore renderCompleteSemaphore,
                                      vk::Fence frameCompleteFence) {
  vk::ClearValue clearValue{};
  mContext->getDevice().resetCommandPool(mFrames[mFrameIndex].mImguiCommandPool.get(), {});
  mFrames[mFrameIndex].mImguiCommandBuffer->begin(
      {vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
  vk::RenderPassBeginInfo info(mImguiRenderPass.get(),
                               mFrames[mFrameIndex].mImguiFramebuffer.get(),
                               {{0, 0}, {mWidth, mHeight}}, 1, &clearValue);
  mFrames[mFrameIndex].mImguiCommandBuffer->beginRenderPass(info, vk::SubpassContents::eInline);

  ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(),
                                  mFrames[mFrameIndex].mImguiCommandBuffer.get());
  mFrames[mFrameIndex].mImguiCommandBuffer->endRenderPass();
  mFrames[mFrameIndex].mImguiCommandBuffer->end();

  vk::PipelineStageFlags waitStage = vk::PipelineStageFlagBits::eColorAttachmentOutput;
  mContext->getQueue().submit(
      mFrames[mFrameIndex].mImguiCommandBuffer.get(), renderCompleteSemaphore, waitStage,
      mFrameSemaphores[mSemaphoreIndex].mImguiCompleteSemaphore.get(), frameCompleteFence);
  return mContext->getQueue().present(
             mFrameSemaphores[mSemaphoreIndex].mImguiCompleteSemaphore.get(), mSwapchain.get(),
             mFrameIndex) == vk::Result::eSuccess;
}

static void applyStyle(float scale) {
  ImGuiStyle style{};
  ImVec4 *colors = style.Colors;
  colors[ImGuiCol_Text] = ImVec4(1.000f, 1.000f, 1.000f, 1.000f);
  colors[ImGuiCol_TextDisabled] = ImVec4(0.500f, 0.500f, 0.500f, 1.000f);
  colors[ImGuiCol_WindowBg] = ImVec4(0.180f, 0.180f, 0.180f, 0.900f);
  colors[ImGuiCol_ChildBg] = ImVec4(0.280f, 0.280f, 0.280f, 0.000f);
  colors[ImGuiCol_PopupBg] = ImVec4(0.313f, 0.313f, 0.313f, 1.000f);
  colors[ImGuiCol_Border] = ImVec4(0.266f, 0.266f, 0.266f, 1.000f);
  colors[ImGuiCol_BorderShadow] = ImVec4(0.000f, 0.000f, 0.000f, 0.000f);
  colors[ImGuiCol_FrameBg] = ImVec4(0.160f, 0.160f, 0.160f, 1.000f);
  colors[ImGuiCol_FrameBgHovered] = ImVec4(0.200f, 0.200f, 0.200f, 1.000f);
  colors[ImGuiCol_FrameBgActive] = ImVec4(0.280f, 0.280f, 0.280f, 1.000f);
  colors[ImGuiCol_TitleBg] = ImVec4(0.148f, 0.148f, 0.148f, 1.000f);
  colors[ImGuiCol_TitleBgActive] = ImVec4(0.148f, 0.148f, 0.148f, 1.000f);
  colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.148f, 0.148f, 0.148f, 1.000f);
  colors[ImGuiCol_MenuBarBg] = ImVec4(0.195f, 0.195f, 0.195f, 1.000f);
  colors[ImGuiCol_ScrollbarBg] = ImVec4(0.160f, 0.160f, 0.160f, 1.000f);
  colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.277f, 0.277f, 0.277f, 1.000f);
  colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.300f, 0.300f, 0.300f, 1.000f);
  colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(1.000f, 0.391f, 0.000f, 1.000f);
  colors[ImGuiCol_CheckMark] = ImVec4(1.000f, 1.000f, 1.000f, 1.000f);
  colors[ImGuiCol_SliderGrab] = ImVec4(0.391f, 0.391f, 0.391f, 1.000f);
  colors[ImGuiCol_SliderGrabActive] = ImVec4(1.000f, 0.391f, 0.000f, 1.000f);
  colors[ImGuiCol_Button] = ImVec4(1.000f, 1.000f, 1.000f, 0.100f);
  colors[ImGuiCol_ButtonHovered] = ImVec4(1.000f, 1.000f, 1.000f, 0.156f);
  colors[ImGuiCol_ButtonActive] = ImVec4(1.000f, 1.000f, 1.000f, 0.391f);
  colors[ImGuiCol_Header] = ImVec4(0.313f, 0.313f, 0.313f, 0.800f);
  colors[ImGuiCol_HeaderHovered] = ImVec4(0.469f, 0.469f, 0.469f, 1.000f);
  colors[ImGuiCol_HeaderActive] = ImVec4(0.469f, 0.469f, 0.469f, 1.000f);
  colors[ImGuiCol_Separator] = colors[ImGuiCol_Border];
  colors[ImGuiCol_SeparatorHovered] = ImVec4(0.391f, 0.391f, 0.391f, 1.000f);
  colors[ImGuiCol_SeparatorActive] = ImVec4(1.000f, 0.391f, 0.000f, 1.000f);
  colors[ImGuiCol_ResizeGrip] = ImVec4(1.000f, 1.000f, 1.000f, 0.250f);
  colors[ImGuiCol_ResizeGripHovered] = ImVec4(1.000f, 1.000f, 1.000f, 0.670f);
  colors[ImGuiCol_ResizeGripActive] = ImVec4(1.000f, 0.391f, 0.000f, 1.000f);
  colors[ImGuiCol_Tab] = ImVec4(0.098f, 0.098f, 0.098f, 1.000f);
  colors[ImGuiCol_TabHovered] = ImVec4(0.352f, 0.352f, 0.352f, 1.000f);
  colors[ImGuiCol_TabActive] = ImVec4(0.195f, 0.195f, 0.195f, 1.000f);
  colors[ImGuiCol_TabUnfocused] = ImVec4(0.098f, 0.098f, 0.098f, 1.000f);
  colors[ImGuiCol_TabUnfocusedActive] = ImVec4(0.195f, 0.195f, 0.195f, 1.000f);
  colors[ImGuiCol_DockingPreview] = ImVec4(1.000f, 0.391f, 0.000f, 0.781f);
  colors[ImGuiCol_DockingEmptyBg] = ImVec4(0.180f, 0.180f, 0.180f, 1.000f);
  colors[ImGuiCol_PlotLines] = ImVec4(0.469f, 0.469f, 0.469f, 1.000f);
  colors[ImGuiCol_PlotLinesHovered] = ImVec4(1.000f, 0.391f, 0.000f, 1.000f);
  colors[ImGuiCol_PlotHistogram] = ImVec4(0.586f, 0.586f, 0.586f, 1.000f);
  colors[ImGuiCol_PlotHistogramHovered] = ImVec4(1.000f, 0.391f, 0.000f, 1.000f);
  colors[ImGuiCol_TextSelectedBg] = ImVec4(1.000f, 1.000f, 1.000f, 0.156f);
  colors[ImGuiCol_DragDropTarget] = ImVec4(1.000f, 0.391f, 0.000f, 1.000f);
  colors[ImGuiCol_NavHighlight] = ImVec4(1.000f, 0.391f, 0.000f, 1.000f);
  colors[ImGuiCol_NavWindowingHighlight] = ImVec4(1.000f, 0.391f, 0.000f, 1.000f);
  colors[ImGuiCol_NavWindowingDimBg] = ImVec4(0.000f, 0.000f, 0.000f, 0.586f);
  colors[ImGuiCol_ModalWindowDimBg] = ImVec4(0.000f, 0.000f, 0.000f, 0.586f);

  style.ChildRounding = 4.0f;
  style.FrameBorderSize = 1.0f;
  style.FrameRounding = 2.0f;
  style.GrabMinSize = 7.0f;
  style.PopupRounding = 2.0f;
  style.ScrollbarRounding = 12.0f;
  style.ScrollbarSize = 13.0f;
  style.TabBorderSize = 1.0f;
  style.TabRounding = 0.0f;
  style.WindowRounding = 4.0f;

  style.ScaleAllSizes(scale);
  ImGui::GetStyle() = style;
}

void GuiWindow::setContentScale(float scale) {
  mContentScale = scale;
  if (ImGui::GetCurrentContext()) {
    applyStyle(mContentScale);
    auto &io = ImGui::GetIO();
    if (io.FontDefault) {
      io.FontDefault->Scale = scale / 2.f;
    }
  }
}

void GuiWindow::initImgui() {
  auto device = mContext->getDevice();

  vk::Instance instance = mContext->getInstance();
  ImGui_ImplVulkan_LoadFunctions(
      [](const char *name, void *instance) {
        return reinterpret_cast<vk::Instance *>(instance)->getProcAddr(name);
      },
      &instance);

  // create a pool for allocating descriptor sets
#ifdef VK_USE_PLATFORM_MACOS_MVK
  vk::DescriptorPoolSize pool_sizes[] = {{vk::DescriptorType::eSampler, 100},
                                         {vk::DescriptorType::eCombinedImageSampler, 100},
                                         {vk::DescriptorType::eSampledImage, 100},
                                         {vk::DescriptorType::eStorageImage, 100},
                                         {vk::DescriptorType::eUniformTexelBuffer, 100},
                                         {vk::DescriptorType::eStorageTexelBuffer, 100},
                                         {vk::DescriptorType::eUniformBuffer, 100},
                                         {vk::DescriptorType::eStorageBuffer, 100},
                                         {vk::DescriptorType::eUniformBufferDynamic, 100},
                                         {vk::DescriptorType::eStorageBufferDynamic, 100},
                                         {vk::DescriptorType::eInputAttachment, 100}};
  auto info = vk::DescriptorPoolCreateInfo(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet,
                                           100 * 11, 11, pool_sizes);
#else
  vk::DescriptorPoolSize pool_sizes[] = {{vk::DescriptorType::eSampler, 1000},
                                         {vk::DescriptorType::eCombinedImageSampler, 1000},
                                         {vk::DescriptorType::eSampledImage, 1000},
                                         {vk::DescriptorType::eStorageImage, 1000},
                                         {vk::DescriptorType::eUniformTexelBuffer, 1000},
                                         {vk::DescriptorType::eStorageTexelBuffer, 1000},
                                         {vk::DescriptorType::eUniformBuffer, 1000},
                                         {vk::DescriptorType::eStorageBuffer, 1000},
                                         {vk::DescriptorType::eUniformBufferDynamic, 1000},
                                         {vk::DescriptorType::eStorageBufferDynamic, 1000},
                                         {vk::DescriptorType::eInputAttachment, 1000}};
  auto info = vk::DescriptorPoolCreateInfo(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet,
                                           1000 * 11, 11, pool_sizes);
#endif
  mDescriptorPool = device.createDescriptorPoolUnique(info);

  mImguiRenderPass = createImguiRenderPass(device, mSurfaceFormat.format);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();

  auto &io = ImGui::GetIO();

  if (gImguiIniFileLocation.length()) {
    auto f = std::filesystem::path(gImguiIniFileLocation);
    std::filesystem::create_directories(f.parent_path());
    io.IniFilename = gImguiIniFileLocation.c_str();
  }

  {
    int monitorCount = 0;
    auto monitors = glfwGetMonitors(&monitorCount);

    if (mContentScale == 0.f) {
      for (int i = 0; i < monitorCount; ++i) {
        float xscale = 0.f;
        float yscale = 0.f;
        glfwGetMonitorContentScale(monitors[i], &xscale, &yscale);
        logger::info("Monitor {} scale: {}", i, xscale);
        assert(xscale == yscale); // this should always be true
        mContentScale = std::max(yscale, std::max(xscale, mContentScale));
      }
      if (mContentScale < 0.1f) {
        mContentScale = 1.f;
      }
      logger::info("Largest monitor DPI scale: {}", mContentScale);
    }

    auto font =
        io.Fonts->AddFontFromMemoryCompressedBase85TTF(roboto_compressed_data_base85, 28.f);
    if (font != nullptr) {
      io.FontDefault = font;
    }
    setContentScale(mContentScale);
  }

  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

  ImGui_ImplGlfw_InitForVulkan(mWindow, true);
  ImGui_ImplVulkan_InitInfo initInfo = {};
  initInfo.Instance = mContext->getInstance();
  initInfo.PhysicalDevice = mContext->getPhysicalDevice();
  initInfo.Device = device;
  initInfo.QueueFamily = mContext->getGraphicsQueueFamilyIndex();
  initInfo.Queue = mContext->getQueue().getVulkanQueue();

  initInfo.PipelineCache = {};
  initInfo.DescriptorPool = mDescriptorPool.get();
  initInfo.Allocator = {};
  initInfo.MinImageCount = mMinImageCount;
  initInfo.ImageCount = mMinImageCount;
  initInfo.CheckVkResultFn = checkVKResult;
  ImGui_ImplVulkan_Init(&initInfo, mImguiRenderPass.get());

  auto pool = mContext->createCommandPool();
  auto commandBuffer = pool->allocateCommandBuffer();
  commandBuffer->begin(vk::CommandBufferBeginInfo(vk::CommandBufferUsageFlagBits::eOneTimeSubmit));
  ImGui_ImplVulkan_CreateFontsTexture(commandBuffer.get());
  commandBuffer->end();
  mContext->getQueue().submitAndWait(commandBuffer.get());
  logger::info("Imgui initialized");
  updateSize(mWidth, mHeight);
}

void GuiWindow::imguiBeginFrame() {
  ImGui::NewFrame();
  ImGuizmo::BeginFrame();

  if (ImGui::GetIO().WantCaptureMouse) {
    mMouseWheelDelta = {0, 0};
  } else {
    mMouseWheelDelta = {ImGui::GetIO().MouseWheel, ImGui::GetIO().MouseWheelH};
  }

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
  ImGui::Begin("DockSpace", 0, flags);
  ImGui::PopStyleVar();

  ImGui::DockSpace(ImGui::GetID("Dockspace"), ImVec2(0, 0),
                   ImGuiDockNodeFlags_PassthruCentralNode |
                       ImGuiDockNodeFlags_NoDockingInCentralNode);
  ImGui::End();
  ImGui::PopStyleVar();
}

void GuiWindow::imguiEndFrame() { ImGui::EndFrame(); }

void GuiWindow::imguiRender() { ImGui::Render(); }
float GuiWindow::imguiGetFramerate() { return ImGui::GetIO().Framerate; }

void GuiWindow::createGlfwWindow(uint32_t width, uint32_t height) {
  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  glfwWindowHintString(GLFW_X11_CLASS_NAME, "sapien");
  glfwWindowHintString(GLFW_X11_INSTANCE_NAME, "SAPIEN");
  mWindow = glfwCreateWindow(width, height, "SAPIEN", nullptr, nullptr);

  VkSurfaceKHR tmpSurface;

  auto result = glfwCreateWindowSurface(mContext->getInstance(), mWindow, nullptr, &tmpSurface);
  if (result != VK_SUCCESS) {
    throw std::runtime_error("create window failed: glfwCreateWindowSurface failed");
  }
  mSurface = vk::UniqueSurfaceKHR(tmpSurface, mContext->getInstance());

  if (!mContext->getPhysicalDevice().getSurfaceSupportKHR(mContext->getGraphicsQueueFamilyIndex(),
                                                          mSurface.get())) {
    throw std::runtime_error("create window failed: graphics device does not "
                             "have present capability");
  }

  glfwSetWindowUserPointer(mWindow, this);
  glfwSetDropCallback(mWindow, windowCallback);

  glfwSetWindowFocusCallback(mWindow, windowFocusCallback);
}

void GuiWindow::dropCallback(int count, const char **paths) {
  std::vector<std::string> v;
  for (int i = 0; i < count; ++i) {
    v.push_back(paths[i]);
  }
  if (mDropCallback) {
    mDropCallback(v);
  }
}

void GuiWindow::setDropCallback(std::function<void(std::vector<std::string>)> callback) {
  mDropCallback = callback;
}
void GuiWindow::unsetDropCallback() { mDropCallback = {}; }

void GuiWindow::focusCallback(int focus) {
  if (mFocusCallback) {
    mFocusCallback(focus);
  }
}
void GuiWindow::setFocusCallback(std::function<void(int)> callback) { mFocusCallback = callback; }
void GuiWindow::unsetFocusCallback() { mFocusCallback = {}; }

void GuiWindow::selectSurfaceFormat(std::vector<vk::Format> const &requestFormats,
                                    vk::ColorSpaceKHR requestColorSpace) {
  assert(requestFormats.size() > 0);

  auto avail_formats = mContext->getPhysicalDevice().getSurfaceFormatsKHR(mSurface.get());
  if (avail_formats.size() == 0) {
    throw std::runtime_error("No surface format is available");
  }

  if (avail_formats.size() == 1) {
    if (avail_formats[0].format == vk::Format::eUndefined) {
      vk::SurfaceFormatKHR ret;
      ret.format = requestFormats[0];
      ret.colorSpace = requestColorSpace;
      mSurfaceFormat = ret;
      return;
    }
    // No point in searching another format
    mSurfaceFormat = avail_formats[0];
    return;
  }

  // Request several formats, the first found will be used
  for (uint32_t request_i = 0; request_i < requestFormats.size(); request_i++) {
    for (uint32_t avail_i = 0; avail_i < avail_formats.size(); avail_i++) {
      if (avail_formats[avail_i].format == requestFormats[request_i] &&
          avail_formats[avail_i].colorSpace == requestColorSpace) {
        mSurfaceFormat = avail_formats[avail_i];
        return;
      }
    }
  }

  logger::warn("SelectSurfaceFormat: None of the requested surface formats is "
               "available");
  // If none of the requested image formats could be found, use the first
  // available
  mSurfaceFormat = avail_formats[0];
}

void GuiWindow::selectPresentMode(std::vector<vk::PresentModeKHR> const &requestModes) {
  assert(requestModes.size() > 0);

  // Request a certain mode and confirm that it is available. If not use
  // VK_PRESENT_MODE_FIFO_KHR which is mandatory
  auto avail_modes = mContext->getPhysicalDevice().getSurfacePresentModesKHR(mSurface.get());

  for (uint32_t request_i = 0; request_i < requestModes.size(); request_i++) {
    for (uint32_t avail_i = 0; avail_i < avail_modes.size(); avail_i++) {
      if (requestModes[request_i] == avail_modes[avail_i]) {
        mPresentMode = requestModes[request_i];
        return;
      }
    }
  }

  mPresentMode = vk::PresentModeKHR::eFifo; // always available
}

bool GuiWindow::updateSize(uint32_t w, uint32_t h) {
  if (!recreateSwapchain(w, h)) {
    return false;
  }
  recreateImguiResources();
  mContext->getDevice().waitIdle();
  return true;
}

bool GuiWindow::recreateSwapchain(uint32_t w, uint32_t h) {
  if (mMinImageCount == 0) {
    throw std::runtime_error("Invalid min image count specified");
  }

  auto device = mContext->getDevice();
  auto cap = mContext->getPhysicalDevice().getSurfaceCapabilitiesKHR(mSurface.get());
  if (cap.minImageExtent.width > w || cap.maxImageExtent.width < w ||
      cap.minImageExtent.height > h || cap.maxImageExtent.height < h) {
    logger::info("swapchain create ignored: requested size ({}, {}); available "
                 "{}-{}, {}-{}",
                 w, h, cap.minImageExtent.width, cap.maxImageExtent.width,
                 cap.minImageExtent.height, cap.maxImageExtent.height);
    return false;
  }

  vk::SwapchainCreateInfoKHR info(
      {}, mSurface.get(), mMinImageCount, mSurfaceFormat.format, mSurfaceFormat.colorSpace, {w, h},
      1, vk::ImageUsageFlagBits::eColorAttachment | vk::ImageUsageFlagBits::eTransferDst,
      vk::SharingMode::eExclusive, 0, nullptr, vk::SurfaceTransformFlagBitsKHR::eIdentity,
      vk::CompositeAlphaFlagBitsKHR::eOpaque, mPresentMode, VK_TRUE, mSwapchain.get());

  if (info.minImageCount < cap.minImageCount) {
    info.minImageCount = cap.minImageCount;
  } else if (cap.maxImageCount != 0 && info.minImageCount > cap.maxImageCount) {
    info.minImageCount = cap.maxImageCount;
  }

  if (cap.currentExtent.width != 0xffffffff) {
    mWidth = info.imageExtent.width = cap.currentExtent.width;
    mHeight = info.imageExtent.height = cap.currentExtent.height;
  } else {
    mWidth = w;
    mHeight = h;
  }
  mSwapchain = device.createSwapchainKHRUnique(info);
  auto images = device.getSwapchainImagesKHR(mSwapchain.get());

  assert(images.size() >= mMinImageCount);
  assert(images.size() < 16);

  mFrames.resize(images.size());
  mFrameSemaphores.resize(images.size());
  for (uint32_t i = 0; i < images.size(); ++i) {
    mFrameSemaphores[i].mImageAcquiredSemaphore = device.createSemaphoreUnique({});
    mFrames[i].mBackbuffer = images[i];
    vk::ImageViewCreateInfo info{{},
                                 mFrames[i].mBackbuffer,
                                 vk::ImageViewType::e2D,
                                 mSurfaceFormat.format,
                                 {vk::ComponentSwizzle::eR, vk::ComponentSwizzle::eG,
                                  vk::ComponentSwizzle::eB, vk::ComponentSwizzle::eA},
                                 {vk::ImageAspectFlagBits::eColor, 0, 1, 0, 1}};
    mFrames[i].mBackbufferView = device.createImageViewUnique(info);
  }
  return true;
}

void GuiWindow::recreateImguiResources() {
  auto device = mContext->getDevice();
  for (uint32_t i = 0; i < mFrames.size(); ++i) {
    mFrames[i].mImguiCommandBuffer.reset();
    mFrames[i].mImguiCommandPool.reset();
  }
  for (uint32_t i = 0; i < mFrames.size(); ++i) {
    mFrames[i].mImguiCommandPool =
        device.createCommandPoolUnique({vk::CommandPoolCreateFlagBits::eResetCommandBuffer,
                                        mContext->getGraphicsQueueFamilyIndex()});
    mFrames[i].mImguiCommandBuffer =
        std::move(device
                      .allocateCommandBuffersUnique({mFrames[i].mImguiCommandPool.get(),
                                                     vk::CommandBufferLevel::ePrimary, 1})
                      .front());

    vk::FramebufferCreateInfo info({}, mImguiRenderPass.get(), 1,
                                   &mFrames[i].mBackbufferView.get(), mWidth, mHeight, 1);
    mFrames[i].mImguiFramebuffer = device.createFramebufferUnique(info);
  }
  for (uint32_t i = 0; i < mFrameSemaphores.size(); ++i) {
    mFrameSemaphores[i].mImguiCompleteSemaphore = device.createSemaphoreUnique({});
  }
}

void GuiWindow::close() {
  if (!mClosed) {
    mClosed = true;
    glfwSetWindowShouldClose(mWindow, true);

    mContext->getDevice().waitIdle();

    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    mImguiRenderPass.reset();
    mFrameSemaphores.clear();
    mFrames.clear();
    mSwapchain.reset();
    mSurface.reset();
  }
}

GuiWindow::~GuiWindow() {
  close();
  glfwDestroyWindow(mWindow);
}

static ImGuiKey findKeyCode(std::string const &key) {
  static std::unordered_map<std::string, ImGuiKey> keyMap = {{"a", ImGuiKey_A},
                                                             {"b", ImGuiKey_B},
                                                             {"c", ImGuiKey_C},
                                                             {"d", ImGuiKey_D},
                                                             {"e", ImGuiKey_E},
                                                             {"f", ImGuiKey_F},
                                                             {"g", ImGuiKey_G},
                                                             {"h", ImGuiKey_H},
                                                             {"i", ImGuiKey_I},
                                                             {"j", ImGuiKey_J},
                                                             {"k", ImGuiKey_K},
                                                             {"l", ImGuiKey_L},
                                                             {"m", ImGuiKey_M},
                                                             {"n", ImGuiKey_N},
                                                             {"o", ImGuiKey_O},
                                                             {"p", ImGuiKey_P},
                                                             {"q", ImGuiKey_Q},
                                                             {"r", ImGuiKey_R},
                                                             {"s", ImGuiKey_S},
                                                             {"t", ImGuiKey_T},
                                                             {"u", ImGuiKey_U},
                                                             {"v", ImGuiKey_V},
                                                             {"w", ImGuiKey_W},
                                                             {"x", ImGuiKey_X},
                                                             {"y", ImGuiKey_Y},
                                                             {"z", ImGuiKey_Z},
                                                             {" ", ImGuiKey_Space},
                                                             {"space", ImGuiKey_Space},
                                                             {"esc", ImGuiKey_Escape},
                                                             {"escape", ImGuiKey_Escape},
                                                             {"tab", ImGuiKey_Tab},
                                                             {"enter", ImGuiKey_Enter},
                                                             {"insert", ImGuiKey_Insert},
                                                             {"home", ImGuiKey_Home},
                                                             {"delete", ImGuiKey_Delete},
                                                             {"end", ImGuiKey_End},
                                                             {"pageup", ImGuiKey_PageUp},
                                                             {"pagedown", ImGuiKey_PageDown},
                                                             {"up", ImGuiKey_UpArrow},
                                                             {"down", ImGuiKey_DownArrow},
                                                             {"left", ImGuiKey_LeftArrow},
                                                             {"right", ImGuiKey_RightArrow},
                                                             {"0", ImGuiKey_0},
                                                             {"1", ImGuiKey_1},
                                                             {"2", ImGuiKey_2},
                                                             {"3", ImGuiKey_3},
                                                             {"4", ImGuiKey_4},
                                                             {"5", ImGuiKey_5},
                                                             {"6", ImGuiKey_6},
                                                             {"7", ImGuiKey_7},
                                                             {"8", ImGuiKey_8},
                                                             {"9", ImGuiKey_9}};

  if (keyMap.find(key) == keyMap.end()) {
    throw std::runtime_error("unknown key " + key);
  }
  return keyMap.at(key);
}

bool GuiWindow::isKeyDown(std::string const &key) {
  auto code = findKeyCode(key);
  if (ImGui::GetIO().WantTextInput || ImGui::GetIO().WantCaptureKeyboard) {
    return false;
  }
  return ImGui::IsKeyDown(code);
}

bool GuiWindow::isKeyPressed(std::string const &key) {
  auto code = findKeyCode(key);
  if (ImGui::GetIO().WantTextInput || ImGui::GetIO().WantCaptureKeyboard) {
    return false;
  }
  return ImGui::IsKeyPressed(code);
}

bool GuiWindow::isShiftDown() { return ImGui::GetIO().KeyShift; }
bool GuiWindow::isCtrlDown() { return ImGui::GetIO().KeyCtrl; }
bool GuiWindow::isAltDown() { return ImGui::GetIO().KeyAlt; }
bool GuiWindow::isSuperDown() { return ImGui::GetIO().KeySuper; }

std::array<float, 2> GuiWindow::getMouseDelta() {
  auto v = ImGui::GetIO().MouseDelta;
  return {v.x, v.y};
}

std::array<float, 2> GuiWindow::getMouseWheelDelta() { return mMouseWheelDelta; }

std::array<float, 2> GuiWindow::getMousePosition() {
  auto v = ImGui::GetIO().MousePos;
  return {v.x, v.y};
}

bool GuiWindow::isMouseKeyDown(int key) {
  return !ImGui::GetIO().WantCaptureMouse && ImGui::IsMouseDown(key);
}

bool GuiWindow::isMouseKeyClicked(int key) {
  if (!mCursorEnabled) {
    return ImGui::IsMouseClicked(key);
  }
  return !ImGui::GetIO().WantCaptureMouse && ImGui::IsMouseClicked(key);
}

void GuiWindow::setCursorEnabled(bool enabled) {
  mCursorEnabled = enabled;
  if (enabled) {
    glfwSetInputMode(mWindow, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    ImGui::GetIO().ConfigFlags &= ~ImGuiConfigFlags_NoMouse;
  } else {
    glfwSetInputMode(mWindow, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NoMouse;
  }
}

bool GuiWindow::getCursorEnabled() const { return mCursorEnabled; }

void GuiWindow::setWindowSize(int width, int height) { glfwSetWindowSize(mWindow, width, height); }
std::array<int, 2> GuiWindow::getWindowSize() const {
  int width, height;
  glfwGetWindowSize(mWindow, &width, &height);
  return {width, height};
}
std::array<int, 2> GuiWindow::getWindowFramebufferSize() const {
  int width, height;
  glfwGetFramebufferSize(mWindow, &width, &height);
  return {width, height};
}
bool GuiWindow::isCloseRequested() const { return glfwWindowShouldClose(mWindow); }
void GuiWindow::hide() { glfwHideWindow(mWindow); }
void GuiWindow::show() { glfwShowWindow(mWindow); }

} // namespace renderer
} // namespace svulkan2