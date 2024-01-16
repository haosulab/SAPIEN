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

namespace sapien {

struct CudaContext {
  static CudaContext &Get();
  CudaContext();
  void *libcuda{};
  decltype(::cuEventRecord) *cuEventRecord{};
  decltype(::cuStreamWaitEvent) *cuStreamWaitEvent{};
  decltype(::cuEventSynchronize) *cuEventSynchronize{};
};

struct CudaEvent {
  CudaEvent() {}

  void init() {
    checkCudaErrors(cudaGetDevice(&cudaId));
    checkCudaErrors(cudaEventCreate(&event));
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
      checkCudaErrors(cudaEventDestroy(event));
    }

    // copy
    cudaId = other.cudaId;
    event = other.event;

    // destroy other
    other.event = nullptr;

    return *this;
  }

  void record(cudaStream_t stream) const {
    if (!event) {
      throw std::runtime_error("cuda event is not initialized");
    }
    CudaContext::Get().cuEventRecord(event, stream);
  }

  void wait(cudaStream_t stream) const {
    if (!event) {
      throw std::runtime_error("cuda event is not initialized");
    }
    CudaContext::Get().cuStreamWaitEvent(stream, event, 0);
  }

  void synchronize() const {
    if (!event) {
      throw std::runtime_error("cuda event is not initialized");
    }
    CudaContext::Get().cuEventSynchronize(event);
  }

  ~CudaEvent() {
    if (event) {
      cudaEventDestroy(event);
    }
  }

  int cudaId{-1};
  cudaEvent_t event{nullptr};
};

}; // namespace sapien
