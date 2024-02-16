#pragma once
#include <stdexcept>
#include <string>

#include <cuda.h>
#include <cuda_runtime.h>

#define checkCudaErrors(call)                                                                     \
  do {                                                                                            \
    cudaError_t err = call;                                                                       \
    if (err != cudaSuccess) {                                                                     \
      throw std::runtime_error("CUDA error: " + std::string(cudaGetErrorString(err)));            \
    }                                                                                             \
  } while (0)

#define checkCudaDriverErrors(call)                                                               \
  do {                                                                                            \
    CUresult err = call;                                                                          \
    if (err != CUDA_SUCCESS) {                                                                    \
      throw std::runtime_error("CUDA failed");                                                    \
    }                                                                                             \
  } while (0)

namespace sapien {

struct CudaLib {
  static CudaLib &Get();
  CudaLib();
  void *libcuda{};

  decltype(::cuCtxGetCurrent) *cuCtxGetCurrent{};
  decltype(::cuEventCreate) *cuEventCreate{};
  decltype(::cuEventDestroy) *cuEventDestroy{};
  decltype(::cuEventRecord) *cuEventRecord{};
  decltype(::cuStreamWaitEvent) *cuStreamWaitEvent{};
  decltype(::cuEventSynchronize) *cuEventSynchronize{};
};

struct CudaEvent {
  CudaEvent() {}

  void init() {
    checkCudaErrors(cudaGetDevice(&cudaId));
    checkCudaDriverErrors(CudaLib::Get().cuEventCreate(&event, 0));
  }

  CudaEvent(CudaEvent const &) = delete;
  CudaEvent &operator=(CudaEvent const &) = delete;

  CudaEvent(CudaEvent &&other) {
    // copy
    cudaId = other.cudaId;
    event = other.event;

    // destory other
    other.event = nullptr;
  }

  CudaEvent &operator=(CudaEvent &&other) {
    // destroy self
    if (event) {
      checkCudaDriverErrors(CudaLib::Get().cuEventDestroy(event));
    }

    // copy
    cudaId = other.cudaId;
    event = other.event;

    // destroy other
    other.event = nullptr;

    return *this;
  }

  /** record cuda event on the current cuda runtime device
   *  @param stream cuda stream, must be created from the same cuda runtime device */
  void record(cudaStream_t stream) {
    if (!event) {
      init();
    }
    checkCudaDriverErrors(CudaLib::Get().cuEventRecord(event, stream));
  }

  void wait(cudaStream_t stream) const {
    if (!event) {
      // no need to wait if no one records to this event
      return;
    }
    checkCudaDriverErrors(CudaLib::Get().cuStreamWaitEvent(stream, event, 0));
  }

  void synchronize() const {
    if (!event) {
      // no need to wait if no one records to this event
    }
    checkCudaDriverErrors(CudaLib::Get().cuEventSynchronize(event));
  }

  ~CudaEvent() {
    if (event) {
      cudaSetDevice(cudaId);
      CudaLib::Get().cuEventDestroy(event);
    }
  }

  int cudaId{-1};
  CUevent event{nullptr};
};

}; // namespace sapien
