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
#include "svulkan2/core/queue.h"
#include "svulkan2/core/device.h"
#include "svulkan2/common/profiler.h"

namespace svulkan2 {
namespace core {
Queue::Queue(Device &device, uint32_t familyIndex) : mDevice(device) {
  mQueue = mDevice.getInternal().getQueue(familyIndex, 0);
}

void Queue::submit(vk::ArrayProxyNoTemporaries<vk::CommandBuffer const> const &commandBuffers,
                   vk::ArrayProxyNoTemporaries<vk::Semaphore const> const &waitSemaphores,
                   vk::ArrayProxyNoTemporaries<vk::PipelineStageFlags const> const
                       &waitStageMasks, // none of the commands in this submission can reach
                                        // the wait stage unless the semaphore is signaled
                   vk::ArrayProxyNoTemporaries<uint64_t const> const &waitValues,
                   vk::ArrayProxyNoTemporaries<vk::Semaphore const> const &signalSemaphores,
                   vk::ArrayProxyNoTemporaries<uint64_t const> const &signalValues,
                   vk::Fence fence) {
  vk::TimelineSemaphoreSubmitInfo timelineInfo(waitValues, signalValues);
  vk::SubmitInfo info(waitSemaphores, waitStageMasks, commandBuffers, signalSemaphores);
  info.setPNext(&timelineInfo);
  std::lock_guard lock(mMutex);
  mQueue.submit(info, fence);
}

void Queue::submit(vk::ArrayProxyNoTemporaries<vk::CommandBuffer const> const &commandBuffers,
                   vk::ArrayProxyNoTemporaries<vk::Semaphore const> const &waitSemaphores,
                   vk::ArrayProxyNoTemporaries<vk::PipelineStageFlags const> const &waitStageMasks,
                   vk::ArrayProxyNoTemporaries<vk::Semaphore const> const &signalSemaphores,
                   vk::Fence fence) {

  std::vector<vk::CommandBuffer> commandBuffers_(commandBuffers.begin(), commandBuffers.end());
  std::vector<vk::Semaphore> waitSemaphores_(waitSemaphores.begin(), waitSemaphores.end());
  std::vector<vk::Semaphore> signalSemaphores_(signalSemaphores.begin(), signalSemaphores.end());
  std::vector<vk::PipelineStageFlags> waitStageMasks_(waitStageMasks.begin(),
                                                      waitStageMasks.end());
  std::lock_guard lock(mMutex);
  vk::SubmitInfo info(waitSemaphores_, waitStageMasks_, commandBuffers_, signalSemaphores_);
  mQueue.submit(info, fence);
}

void Queue::submit(vk::ArrayProxyNoTemporaries<vk::CommandBuffer const> const &commandBuffers,
                   vk::Fence fence) {
  return submit(commandBuffers, {}, {}, {}, fence);
}

vk::Result
Queue::submitAndWait(vk::ArrayProxyNoTemporaries<vk::CommandBuffer const> const &commandBuffers) {
  auto fence = mDevice.getInternal().createFenceUnique({});
  submit(commandBuffers, fence.get());
  return mDevice.getInternal().waitForFences(fence.get(), VK_TRUE, UINT64_MAX);
}

vk::Result Queue::present(vk::ArrayProxyNoTemporaries<vk::Semaphore const> const &waitSemaphores,
                          vk::ArrayProxyNoTemporaries<vk::SwapchainKHR const> const &swapchains,
                          vk::ArrayProxyNoTemporaries<uint32_t const> const &imageIndices) {
  std::vector<vk::Semaphore> waitSemaphores_(waitSemaphores.begin(), waitSemaphores.end());
  std::vector<vk::SwapchainKHR> swapchains_(swapchains.begin(), swapchains.end());
  std::vector<uint32_t> imageIndices_(imageIndices.begin(), imageIndices.end());

  std::lock_guard lock(mMutex);
  vk::PresentInfoKHR info(waitSemaphores_, swapchains_, imageIndices_);
  return mQueue.presentKHR(info);
}

void Queue::waitIdle() const { mQueue.waitIdle(); }

Queue::~Queue(){};

} // namespace core
} // namespace svulkan2