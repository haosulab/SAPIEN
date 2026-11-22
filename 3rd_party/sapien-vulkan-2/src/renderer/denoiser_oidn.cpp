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
#ifdef SVULKAN2_CUDA_INTEROP
#include "../common/logger.h"
#include "denoiser.h"
#include "svulkan2/core/context.h"
#include <OpenImageDenoise/oidn.hpp>
#include <cuda_runtime.h>

namespace svulkan2 {
namespace renderer {

static inline bool checkCudaRuntime(cudaError_t error, std::string const &message = "") {
  if (error != cudaSuccess) {
    logger::error("{} CUDA Error: {}", message, cudaGetErrorName(error));
    return false;
  }
  return true;
}

bool DenoiserOidn::init(bool albedo, bool normal, bool hdr) {
  int device;
  if (!checkCudaRuntime(cudaGetDevice(&device))) {
    return false;
  }
  if (!checkCudaRuntime(cudaStreamCreate(&mCudaStream))) {
    return false;
  }
  mDevice = oidn::newCUDADevice(device, mCudaStream);
  mDevice.commit();

  mCommandPool = core::Context::Get()->createCommandPool();
  mCommandBufferIn = mCommandPool->allocateCommandBuffer();
  mCommandBufferOut = mCommandPool->allocateCommandBuffer();

  mPixelSize = 16;

  mAlbedo = albedo;
  mNormal = normal;
  mHdr = hdr;

  return true;
}

void DenoiserOidn::allocate(uint32_t width, uint32_t height) {
  mWidth = width;
  mHeight = height;
  free();

  mFilter = mDevice.newFilter("RT");

  mInputBuffer = core::Buffer::Create(width * height * mPixelSize,
                                      vk::BufferUsageFlagBits::eTransferDst |
                                          vk::BufferUsageFlagBits::eTransferSrc,
                                      VMA_MEMORY_USAGE_GPU_ONLY, VmaAllocationCreateFlags{}, true);
  mInputBufferOidn = mDevice.newBuffer(mInputBuffer->getCudaPtr(), mInputBuffer->getSize());
  mFilter.setImage("color", mInputBufferOidn, oidn::Format::Float3, mWidth, mHeight, 0, 16);

  mFilter.setImage("output", mInputBufferOidn, oidn::Format::Float3, mWidth, mHeight, 0, 16);

  if (useAlbedo()) {
    mAlbedoBuffer = core::Buffer::Create(
        width * height * mPixelSize,
        vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eTransferSrc,
        VMA_MEMORY_USAGE_GPU_ONLY, VmaAllocationCreateFlags{}, true);
    mAlbedoBufferOidn = mDevice.newBuffer(mAlbedoBuffer->getCudaPtr(), mAlbedoBuffer->getSize());
    mFilter.setImage("albedo", mAlbedoBufferOidn, oidn::Format::Float3, mWidth, mHeight, 0, 16);
  }
  if (useNormal()) {
    mNormalBuffer = core::Buffer::Create(
        width * height * mPixelSize,
        vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eTransferSrc,
        VMA_MEMORY_USAGE_GPU_ONLY, VmaAllocationCreateFlags{}, true);
    mNormalBufferOidn = mDevice.newBuffer(mNormalBuffer->getCudaPtr(), mNormalBuffer->getSize());
    mFilter.setImage("normal", mNormalBufferOidn, oidn::Format::Float3, mWidth, mHeight, 0, 16);
  }
  mFilter.set("hdr", mHdr);
  mFilter.commit();

  vk::Device device = core::Context::Get()->getDevice();
  // create timeline semaphore with value 0
  vk::SemaphoreTypeCreateInfo timelineCreateInfo(vk::SemaphoreType::eTimeline, 0);
  vk::SemaphoreCreateInfo createInfo{};
  vk::ExportSemaphoreCreateInfo exportCreateInfo(
      vk::ExternalSemaphoreHandleTypeFlagBits::eOpaqueFd);
  createInfo.setPNext(&exportCreateInfo);
  exportCreateInfo.setPNext(&timelineCreateInfo);
  mSem = device.createSemaphoreUnique(createInfo);

  // export vulkan semaphore as cuda semaphore
  int fd =
      device.getSemaphoreFdKHR({mSem.get(), vk::ExternalSemaphoreHandleTypeFlagBits::eOpaqueFd});
  cudaExternalSemaphoreHandleDesc desc = {};
  desc.flags = 0;
  desc.handle.fd = fd;
  desc.type = cudaExternalSemaphoreHandleTypeTimelineSemaphoreFd;
  checkCudaRuntime(cudaImportExternalSemaphore(&mCudaSem, &desc));
}

void DenoiserOidn::free() {
  core::Context::Get()->getDevice().waitIdle();
  if (mCudaSem) {
    cudaDestroyExternalSemaphore(mCudaSem);
    mCudaSem = {};
  }

  mFilter = {};
  mNormalBufferOidn = {};
  mNormalBuffer = {};
  mAlbedoBufferOidn = {};
  mAlbedoBuffer = {};
  mInputBufferOidn = {};
  mInputBuffer = {};
}

void DenoiserOidn::denoise(core::Image &color, core::Image *albedo, core::Image *normal) {

  if (color.getFormat() != vk::Format::eR32G32B32A32Sfloat ||
      (albedo && albedo->getFormat() != vk::Format::eR32G32B32A32Sfloat) ||
      (normal && normal->getFormat() != vk::Format::eR32G32B32A32Sfloat)) {
    throw std::runtime_error("denoiser only supports R32G32B32A32Sfloat format");
  }

  // copy external image to buffer
  mCommandBufferIn->reset();
  mCommandBufferIn->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});

  color.recordCopyToBuffer(mCommandBufferIn.get(), mInputBuffer->getVulkanBuffer(), 0,
                           mInputBuffer->getSize(), {0, 0, 0}, color.getExtent());

  if (useAlbedo() && albedo) {
    albedo->recordCopyToBuffer(mCommandBufferIn.get(), mAlbedoBuffer->getVulkanBuffer(), 0,
                               mAlbedoBuffer->getSize(), {0, 0, 0}, color.getExtent());
  }
  if (useNormal() && normal) {
    normal->recordCopyToBuffer(mCommandBufferIn.get(), mNormalBuffer->getVulkanBuffer(), 0,
                               mNormalBuffer->getSize(), {0, 0, 0}, color.getExtent());
  }

  mCommandBufferIn->end();

  core::Context::Get()->getQueue().submit(mCommandBufferIn.get(), {}, {}, {}, mSem.get(),
                                          ++mSemValue, {});

  cudaExternalSemaphoreWaitParams waitParams{};
  waitParams.params.fence.value = mSemValue;
  cudaWaitExternalSemaphoresAsync(&mCudaSem, &waitParams, 1, mCudaStream);

  // run
  // mFilter.executeAsync();
  mFilter.execute();

  char const *err;
  if (mDevice.getError(err) != oidn::Error::None) {
    logger::error("OIDN Error: {}", err);
  }

  // copy output buffer to external image
  cudaExternalSemaphoreSignalParams sigParams{};
  sigParams.flags = 0;
  sigParams.params.fence.value = ++mSemValue;
  cudaSignalExternalSemaphoresAsync(&mCudaSem, &sigParams, 1, mCudaStream);

  mCommandBufferOut->reset();
  mCommandBufferOut->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});
  color.recordCopyFromBuffer(mCommandBufferOut.get(), mInputBuffer->getVulkanBuffer(), 0,
                             mInputBuffer->getSize(), {0, 0, 0}, color.getExtent());
  mCommandBufferOut->end();

  vk::PipelineStageFlags waitStage = vk::PipelineStageFlagBits::eTransfer;
  core::Context::Get()->getQueue().submit(mCommandBufferOut.get(), mSem.get(), waitStage,
                                          mSemValue, {}, {}, {});
}

DenoiserOidn::~DenoiserOidn() {
  logger::info("OIDN finished");
  free();

  mDevice = {};
  if (mCudaStream) {
    checkCudaRuntime(cudaStreamDestroy(mCudaStream));
  }
}
} // namespace renderer
} // namespace svulkan2

#endif