#pragma once

#include "./sapien_renderer_system.h"

struct CUstream_st;

namespace sapien {
namespace sapien_renderer {

class BatchedCamera {
public:
  BatchedCamera(std::vector<std::shared_ptr<SapienRenderCameraComponent>> cameras,
                std::vector<std::string> renderTargets);
  std::vector<std::shared_ptr<SapienRenderCameraComponent>> const &getCameras() const {
    return mCameras;
  }

  void setCudaStream(CUstream_st *stream) { mCudaStream = stream; }

  void takePicture();
  CudaArrayHandle getPictureCuda(std::string const &name);

  ~BatchedCamera();

private:
  std::vector<std::shared_ptr<SapienRenderCameraComponent>> mCameras;
  std::map<std::string, std::shared_ptr<svulkan2::core::Buffer>> mCudaImageBuffers;
  std::map<std::string, CudaArrayHandle> mCudaImageHandles;

  std::unique_ptr<svulkan2::core::CommandPool> mCommandPool;
  vk::UniqueCommandBuffer mCommandBuffer;

  CUstream_st *mCudaStream{nullptr};

  vk::UniqueSemaphore mSemaphore;
  uint64_t mFrameCounter{0};
  cudaExternalSemaphore_t mCudaSem{nullptr};
};

class BatchedRenderSystem {
public:
  BatchedRenderSystem(std::vector<std::shared_ptr<SapienRendererSystem>> systems);

  void init();

  void setPoseSource(CudaArrayHandle const &poses);

  std::shared_ptr<BatchedCamera>
  createCameraBatch(std::vector<std::shared_ptr<SapienRenderCameraComponent>> cameras,
                    std::vector<std::string> renderTargets);

  void update();
  void setCudaStream(uintptr_t);

  ~BatchedRenderSystem();

private:
  std::vector<std::shared_ptr<SapienRendererSystem>> mSystems;
  std::vector<uint64_t> mSceneVersions;

  /** external poses array */
  CudaArrayHandle mCudaPoseHandle;

  std::vector<std::shared_ptr<BatchedCamera>> mCameraBatches;

  // size of a mat4 element in the transform buffer
  int mTransformBufferElementByteOffset{0};

  int mShapeCount{0};
  CudaArray mCudaSceneTransformRefBuffer;
  CudaArray mCudaShapeDataBuffer;

  int mCameraCount{0};
  CudaArray mCudaCameraDataBuffer;

  CUstream_st *mCudaStream{nullptr};

  // semaphore to notify scene update
  void notifyUpdate();
  vk::UniqueSemaphore mSem{};
  cudaExternalSemaphore_t mCudaSem{nullptr};
  uint64_t mSemValue{0};
};

} // namespace sapien_renderer
} // namespace sapien
