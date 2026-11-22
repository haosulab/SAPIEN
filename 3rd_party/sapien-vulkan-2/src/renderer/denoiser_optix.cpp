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
#include "optix_function_table_definition.h"
#include "svulkan2/core/context.h"
#include <cuda_runtime.h>
#include <filesystem>
#include <optional>
#include <optix_stubs.h>
#include <vector>

namespace svulkan2 {
namespace renderer {

static inline bool checkOptix(OptixResult error, std::string const &message = "") {
  if (error != OPTIX_SUCCESS) {
    logger::error("{} OptiX Error: {}", message, optixGetErrorName(error));
  }
  return true;
}

static inline bool checkCudaRuntime(cudaError_t error, std::string const &message = "") {
  if (error != cudaSuccess) {
    logger::error("{} CUDA Error: {}", message, cudaGetErrorName(error));
    return false;
  }
  return true;
}

DenoiserOptix::Context::Context() {
  if (cudaFree(0) != cudaSuccess) {
    throw std::runtime_error("failed to init cuda runtime");
  }
  if (optixInit() != OPTIX_SUCCESS) {
    throw std::runtime_error("failed to init optix");
  }
  if (optixDeviceContextCreate(nullptr, nullptr, &optixDevice) != OPTIX_SUCCESS) {
    throw std::runtime_error("failed to create optix device");
  }
}

DenoiserOptix::Context::~Context() {
  if (optixDevice) {
    optixDeviceContextDestroy(optixDevice);
  }
}

std::weak_ptr<DenoiserOptix::Context> DenoiserOptix::gContext = {};

bool DenoiserOptix::useAlbedo() { return mOptions.guideAlbedo; }

bool DenoiserOptix::useNormal() { return mOptions.guideNormal; }

bool DenoiserOptix::init(bool albedo, bool normal, bool hdr) {
  OptixPixelFormat pixelFormat = OptixPixelFormat::OPTIX_PIXEL_FORMAT_FLOAT4;

  std::string const error = "Failed to initialize denoiser. Please make sure the renderer runs "
                            "on NVIDIA GPU with driver version >= 522.25.";

  if (!(mContext = gContext.lock())) {
    try {
      gContext = mContext = std::make_shared<Context>();
    } catch (std::runtime_error const &e) {
      logger::error("{}", error);
      return false;
    }
  }

  mCommandPool = core::Context::Get()->createCommandPool();
  mCommandBufferIn = mCommandPool->allocateCommandBuffer();
  mCommandBufferOut = mCommandPool->allocateCommandBuffer();

  bool success = checkCudaRuntime(cudaStreamCreate(&mCudaStream));
  if (!success) {
    logger::error("{}", error);
  }

  mOptions.guideAlbedo = albedo;
  mOptions.guideNormal = normal;
  success = checkOptix(optixDenoiserCreate(
      mContext->optixDevice, hdr ? OPTIX_DENOISER_MODEL_KIND_HDR : OPTIX_DENOISER_MODEL_KIND_LDR,
      &mOptions, &mDenoiser));

  if (!success) {
    logger::error("{}", error);
  }

  mPixelFormat = pixelFormat;
  switch (pixelFormat) {
  case OPTIX_PIXEL_FORMAT_FLOAT3:
    mPixelSize = static_cast<uint32_t>(3 * sizeof(float));
    break;
  case OPTIX_PIXEL_FORMAT_FLOAT4:
    mPixelSize = static_cast<uint32_t>(4 * sizeof(float));
    break;
  case OPTIX_PIXEL_FORMAT_UCHAR3:
    mPixelSize = static_cast<uint32_t>(3 * sizeof(uint8_t));
    break;
  case OPTIX_PIXEL_FORMAT_UCHAR4:
    mPixelSize = static_cast<uint32_t>(4 * sizeof(uint8_t));
    break;
  case OPTIX_PIXEL_FORMAT_HALF3:
    mPixelSize = static_cast<uint32_t>(3 * sizeof(uint16_t));
    break;
  case OPTIX_PIXEL_FORMAT_HALF4:
    mPixelSize = static_cast<uint32_t>(4 * sizeof(uint16_t));
    break;
  default:
    throw std::runtime_error("Unsupported OPTIX_PIXEL_FORMAT!");
    break;
  }

  return true;
}

void DenoiserOptix::allocate(uint32_t width, uint32_t height) {
  mWidth = width;
  mHeight = height;

  free();

  checkOptix(optixDenoiserComputeMemoryResources(mDenoiser, width, height, &mSizes));

  checkCudaRuntime(cudaMalloc((void **)&mStatePtr, mSizes.stateSizeInBytes));
  checkCudaRuntime(cudaMalloc((void **)&mScratchPtr, mSizes.withoutOverlapScratchSizeInBytes));

  checkOptix(optixDenoiserSetup(mDenoiser, mCudaStream, width, height, mStatePtr,
                                mSizes.stateSizeInBytes, mScratchPtr,
                                mSizes.withoutOverlapScratchSizeInBytes));

  mInputBuffer = core::Buffer::Create(width * height * mPixelSize,
                                      vk::BufferUsageFlagBits::eTransferDst |
                                          vk::BufferUsageFlagBits::eTransferSrc,
                                      VMA_MEMORY_USAGE_GPU_ONLY, VmaAllocationCreateFlags{}, true);
  auto ptr = mInputBuffer->getCudaPtr();
  std::memcpy(&mInputPtr, &ptr, sizeof(ptr));

  mOutputBuffer = core::Buffer::Create(
      width * height * mPixelSize,
      vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eTransferSrc,
      VMA_MEMORY_USAGE_GPU_ONLY, VmaAllocationCreateFlags{}, true);
  ptr = mOutputBuffer->getCudaPtr();
  std::memcpy(&mOutputPtr, &ptr, sizeof(ptr));

  if (useAlbedo()) {
    mAlbedoBuffer = core::Buffer::Create(
        width * height * mPixelSize,
        vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eTransferSrc,
        VMA_MEMORY_USAGE_GPU_ONLY, VmaAllocationCreateFlags{}, true);
    ptr = mAlbedoBuffer->getCudaPtr();
    std::memcpy(&mAlbedoPtr, &ptr, sizeof(ptr));
  }

  if (useNormal()) {
    mNormalBuffer = core::Buffer::Create(
        width * height * mPixelSize,
        vk::BufferUsageFlagBits::eTransferDst | vk::BufferUsageFlagBits::eTransferSrc,
        VMA_MEMORY_USAGE_GPU_ONLY, VmaAllocationCreateFlags{}, true);
    ptr = mNormalBuffer->getCudaPtr();
    std::memcpy(&mNormalPtr, &ptr, sizeof(ptr));
  }

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

void DenoiserOptix::free() {
  core::Context::Get()->getDevice().waitIdle();
  if (mCudaSem) {
    cudaDestroyExternalSemaphore(mCudaSem);
    mCudaSem = {};
  }

  checkCudaRuntime(cudaFree((void *)mStatePtr));
  mStatePtr = {};
  checkCudaRuntime(cudaFree((void *)mScratchPtr));
  mScratchPtr = {};

  mInputBuffer.reset();
  mInputPtr = {};
  mOutputBuffer.reset();
  mOutputPtr = {};
  mAlbedoBuffer.reset();
  mAlbedoPtr = {};
  mNormalBuffer.reset();
  mNormalPtr = {};
}

void DenoiserOptix::denoise(core::Image &color, core::Image *albedo, core::Image *normal) {
  if (color.getFormat() != vk::Format::eR32G32B32A32Sfloat ||
      (albedo && albedo->getFormat() != vk::Format::eR32G32B32A32Sfloat) ||
      (normal && normal->getFormat() != vk::Format::eR32G32B32A32Sfloat)) {
    throw std::runtime_error("denoiser only supports R32G32B32A32Sfloat format");
  }

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

  uint32_t width = color.getExtent().width;
  uint32_t height = color.getExtent().height;

  mParams.blendFactor = 0.f;
  mParams.denoiseAlpha = OPTIX_DENOISER_ALPHA_MODE_COPY;
  mParams.temporalModeUsePreviousLayers = 0;
  mParams.hdrIntensity = 0;
  mParams.hdrAverageColor = 0;

  mLayer.input =
      OptixImage2D{mInputPtr, width, height, mPixelSize * width, mPixelSize, mPixelFormat};
  mLayer.output =
      OptixImage2D{mOutputPtr, width, height, mPixelSize * width, mPixelSize, mPixelFormat};
  mLayer.previousOutput = {0};

  mGuideLayer.flow = {};
  mGuideLayer.outputInternalGuideLayer = {};
  mGuideLayer.previousOutputInternalGuideLayer = {};
  mGuideLayer.albedo = {};
  mGuideLayer.normal = {};
  if (useAlbedo() && albedo) {
    mGuideLayer.albedo =
        OptixImage2D{mAlbedoPtr, width, height, mPixelSize * width, mPixelSize, mPixelFormat};
  }
  if (useNormal() && normal) {
    mGuideLayer.normal =
        OptixImage2D{mNormalPtr, width, height, mPixelSize * width, mPixelSize, mPixelFormat};
  }
  checkOptix(optixDenoiserInvoke(mDenoiser, mCudaStream, &mParams, mStatePtr,
                                 mSizes.stateSizeInBytes, &mGuideLayer, &mLayer, 1, 0, 0,
                                 mScratchPtr, mSizes.withoutOverlapScratchSizeInBytes),
             "Failed to denoise");

  cudaExternalSemaphoreSignalParams sigParams{};
  sigParams.flags = 0;
  sigParams.params.fence.value = ++mSemValue;
  cudaSignalExternalSemaphoresAsync(&mCudaSem, &sigParams, 1, mCudaStream);

  mCommandBufferOut->reset();
  mCommandBufferOut->begin({vk::CommandBufferUsageFlagBits::eOneTimeSubmit});

  color.recordCopyFromBuffer(mCommandBufferOut.get(), mOutputBuffer->getVulkanBuffer(), 0,
                             mOutputBuffer->getSize(), {0, 0, 0}, color.getExtent());

  mCommandBufferOut->end();

  vk::PipelineStageFlags waitStage = vk::PipelineStageFlagBits::eTransfer;
  core::Context::Get()->getQueue().submit(mCommandBufferOut.get(), mSem.get(), waitStage,
                                          mSemValue, {}, {}, {});
}

DenoiserOptix::~DenoiserOptix() {
  free();

  if (mDenoiser) {
    optixDenoiserDestroy(mDenoiser);
    mDenoiser = {};
  }

  if (mCudaStream) {
    checkCudaRuntime(cudaStreamDestroy(mCudaStream));
  }
}

} // namespace renderer
} // namespace svulkan2

#endif