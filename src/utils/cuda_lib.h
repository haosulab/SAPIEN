#pragma once
#include <cuda.h>
#include <cuda_runtime.h>

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

} // namespace sapien
