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

#pragma once
#include "svulkan2/core/buffer.h"
#include "svulkan2/core/command_pool.h"
#include "svulkan2/core/image.h"
#include <OpenImageDenoise/oidn.hpp>
#include <cstdio>
#include <cuda.h>
#include <cuda_runtime.h>
#include <memory>
#include <optix.h>

static_assert(OPTIX_VERSION == 70600);

namespace svulkan2 {
namespace renderer {

class Denoiser {
public:
  virtual bool init(bool albedo, bool normal, bool hdr) = 0;
  virtual void allocate(uint32_t width, uint32_t height) = 0;
  virtual void free() = 0;
  virtual void denoise(core::Image &color, core::Image *albedo, core::Image *normal) = 0;

  virtual uint32_t getWidth() const = 0;
  virtual uint32_t getHeight() const = 0;

  virtual ~Denoiser(){};
};

class DenoiserOptix : public Denoiser {
public:
  bool init(bool albedo, bool normal, bool hdr) override;
  void allocate(uint32_t width, uint32_t height) override;
  void free() override;
  void denoise(core::Image &color, core::Image *albedo, core::Image *normal) override;

  uint32_t getWidth() const override { return mWidth; }
  uint32_t getHeight() const override { return mHeight; }

  ~DenoiserOptix();

private:
  class Context {
  public:
    Context();
    ~Context();

    OptixDeviceContext optixDevice{};
  };

  static std::weak_ptr<Context> gContext;
  std::shared_ptr<Context> mContext;

  CUstream mCudaStream{};
  OptixDenoiser mDenoiser{};
  OptixDenoiserOptions mOptions{};

  bool useAlbedo();
  bool useNormal();

  OptixPixelFormat mPixelFormat{};
  uint32_t mPixelSize{};
  OptixDenoiserSizes mSizes{};

  OptixDenoiserParams mParams{};

  CUdeviceptr mStatePtr{};
  CUdeviceptr mScratchPtr{};

  OptixDenoiserGuideLayer mGuideLayer;
  OptixDenoiserLayer mLayer;

  uint32_t mWidth{};
  uint32_t mHeight{};

  std::unique_ptr<core::Buffer> mInputBuffer;
  CUdeviceptr mInputPtr{};
  std::unique_ptr<core::Buffer> mOutputBuffer;
  CUdeviceptr mOutputPtr{};

  std::unique_ptr<core::Buffer> mAlbedoBuffer;
  CUdeviceptr mAlbedoPtr{};
  std::unique_ptr<core::Buffer> mNormalBuffer;
  CUdeviceptr mNormalPtr{};

  std::unique_ptr<core::CommandPool> mCommandPool;
  vk::UniqueCommandBuffer mCommandBufferIn;
  vk::UniqueCommandBuffer mCommandBufferOut;

  vk::UniqueSemaphore mSem{};
  cudaExternalSemaphore_t mCudaSem{};
  uint64_t mSemValue{0};
};

class DenoiserOidn : public Denoiser {
public:
  bool init(bool albedo, bool normal, bool hdr) override;
  void allocate(uint32_t width, uint32_t height) override;
  void free() override;
  void denoise(core::Image &color, core::Image *albedo, core::Image *normal) override;

  uint32_t getWidth() const override { return mWidth; }
  uint32_t getHeight() const override { return mHeight; }
  bool useAlbedo() const { return mAlbedo; }
  bool useNormal() const { return mAlbedo; }

  ~DenoiserOidn();

private:
  bool mAlbedo{};
  bool mNormal{};
  bool mHdr{};

  uint32_t mPixelSize{};

  uint32_t mWidth;
  uint32_t mHeight;

  cudaStream_t mCudaStream{};
  oidn::DeviceRef mDevice;
  oidn::FilterRef mFilter;

  std::unique_ptr<core::Buffer> mInputBuffer;
  oidn::BufferRef mInputBufferOidn;

  std::unique_ptr<core::Buffer> mAlbedoBuffer;
  oidn::BufferRef mAlbedoBufferOidn;

  std::unique_ptr<core::Buffer> mNormalBuffer;
  oidn::BufferRef mNormalBufferOidn;

  std::unique_ptr<core::CommandPool> mCommandPool;
  vk::UniqueCommandBuffer mCommandBufferIn;
  vk::UniqueCommandBuffer mCommandBufferOut;

  vk::UniqueSemaphore mSem{};
  cudaExternalSemaphore_t mCudaSem{};
  uint64_t mSemValue{0};
};

} // namespace renderer
}; // namespace svulkan2

#endif