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
#pragma once
#include <functional>
#include <iostream>
#include <memory>
#include <vector>
#include <vulkan/vulkan.hpp>

struct GLFWwindow;

namespace svulkan2 {
namespace core {
class Context;
}
namespace renderer {

struct Frame {
  vk::Image mBackbuffer;
  vk::UniqueImageView mBackbufferView;

  vk::UniqueFramebuffer mImguiFramebuffer;
  vk::UniqueCommandPool mImguiCommandPool;
  vk::UniqueCommandBuffer mImguiCommandBuffer;
};

struct VulkanFrameSemaphores {
  vk::UniqueSemaphore mImageAcquiredSemaphore;
  vk::UniqueSemaphore mImguiCompleteSemaphore;
};

class GuiWindow {
public:
  static void setImguiIniFileLocation(std::string const &);
  static std::string getImguiIniFileLocation();

private:
  std::shared_ptr<core::Context> mContext;
  GLFWwindow *mWindow;
  vk::UniqueSurfaceKHR mSurface;

  uint32_t mMinImageCount;

  float mContentScale{0.f};

  uint32_t mWidth{0};
  uint32_t mHeight{0};
  vk::SurfaceFormatKHR mSurfaceFormat{};
  vk::PresentModeKHR mPresentMode{vk::PresentModeKHR::eFifo};

  uint32_t mFrameIndex{0};
  uint32_t mSemaphoreIndex{0};
  std::vector<Frame> mFrames{};
  std::vector<VulkanFrameSemaphores> mFrameSemaphores{};

  vk::UniqueDescriptorPool mDescriptorPool;

  vk::UniqueSwapchainKHR mSwapchain{};

  // imgui render pass
  vk::UniqueRenderPass mImguiRenderPass;

  bool mCursorEnabled{true};

  bool mClosed{};

  std::array<float, 2> mMouseWheelDelta{0, 0};

  std::function<void(std::vector<std::string>)> mDropCallback{};
  std::function<void(int)> mFocusCallback{};

public:
  [[nodiscard]] inline vk::SwapchainKHR getSwapchain() const { return mSwapchain.get(); }
  [[nodiscard]] inline vk::Format getBackBufferFormat() const { return mSurfaceFormat.format; }
  [[nodiscard]] inline uint32_t getWidth() const { return mWidth; }
  [[nodiscard]] inline uint32_t getHeight() const { return mHeight; }
  [[nodiscard]] inline uint32_t getFrameIndex() const { return mFrameIndex; }
  [[nodiscard]] inline float getContentScale() const { return mContentScale; }

  void setContentScale(float scale);

  void dropCallback(int count, const char **paths);
  void setDropCallback(std::function<void(std::vector<std::string>)> callback);
  void unsetDropCallback();

  void focusCallback(int focus);
  void setFocusCallback(std::function<void(int)> callback);
  void unsetFocusCallback();

public:
  /** Acquire new frame, poll events, call ImGui NewFrame */
  void newFrame();

  /** Send current frame to the present queue. Waits for
   * renderCopmleteSemaphore, signals frameCompleteFence. */
  bool presentFrameWithImgui(vk::Semaphore renderCompleteSemaphore, vk::Fence frameCompleteFence);

  /** Create ImGui Context and init ImGui Vulkan implementation. */
  void initImgui();

  void imguiBeginFrame();
  void imguiEndFrame();
  void imguiRender();
  float imguiGetFramerate();

  inline vk::Semaphore getImageAcquiredSemaphore() {
    return mFrameSemaphores[mSemaphoreIndex].mImageAcquiredSemaphore.get();
  }

  inline vk::Image getBackbuffer() const { return mFrames[mFrameIndex].mBackbuffer; }

  GuiWindow(std::vector<vk::Format> const &requestFormats, vk::ColorSpaceKHR requestColorSpace,
            uint32_t width, uint32_t height, std::vector<vk::PresentModeKHR> const &requestModes,
            uint32_t minImageCount);

  GuiWindow(GuiWindow const &other) = delete;
  GuiWindow &operator=(GuiWindow const &other) = delete;

  GuiWindow(GuiWindow &&other) = default;
  GuiWindow &operator=(GuiWindow &&other) = default;

  ~GuiWindow();

  bool updateSize(uint32_t w, uint32_t h);

  inline GLFWwindow *getGLFWWindow() const { return mWindow; }

  void close();
  inline bool isClosed() const { return mClosed; }

  void setWindowSize(int width, int height);
  std::array<int, 2> getWindowSize() const;
  std::array<int, 2> getWindowFramebufferSize() const;
  bool isCloseRequested() const;
  void hide();
  void show();

public:
  bool isKeyDown(std::string const &key);
  bool isKeyPressed(std::string const &key);

  bool isShiftDown();
  bool isCtrlDown();
  bool isAltDown();
  bool isSuperDown();

  std::array<float, 2> getMouseDelta();

  std::array<float, 2> getMouseWheelDelta();

  std::array<float, 2> getMousePosition();

  bool isMouseKeyDown(int key);

  bool isMouseKeyClicked(int key);

  void setCursorEnabled(bool enabled);
  bool getCursorEnabled() const;

private:
  /** Called at initialization time  */
  void selectSurfaceFormat(std::vector<vk::Format> const &requestFormats,
                           vk::ColorSpaceKHR requestColorSpace);
  /** Called at initialization time  */
  void selectPresentMode(std::vector<vk::PresentModeKHR> const &requestModes);

  void createGlfwWindow(uint32_t width, uint32_t height);

  /** Called when the window is resized to recreate the sawpchain */
  bool recreateSwapchain(uint32_t w, uint32_t h);

  /** Called after swapchain recreation to update ImGui related resources */
  void recreateImguiResources();
};

} // namespace renderer
} // namespace svulkan2