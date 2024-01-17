#include "sapien/utils/cuda.h"
#include <cuda.h>
#include <dlfcn.h>

namespace sapien {

CudaLib::CudaLib() {
  libcuda = dlopen("libcuda.so", RTLD_LAZY);

  cudaFree(0); // establish cuda context with runtime API

  this->cuCtxGetCurrent = (decltype(::cuCtxGetCurrent) *)dlsym(libcuda, "cuCtxGetCurrent");
  this->cuEventCreate = (decltype(::cuEventCreate) *)dlsym(libcuda, "cuEventCreate");
  this->cuEventDestroy = (decltype(::cuEventDestroy) *)dlsym(libcuda, "cuEventDestroy");
  this->cuEventRecord = (decltype(::cuEventRecord) *)dlsym(libcuda, "cuEventRecord");
  this->cuStreamWaitEvent = (decltype(::cuStreamWaitEvent) *)dlsym(libcuda, "cuStreamWaitEvent");
  this->cuEventSynchronize =
      (decltype(::cuEventSynchronize) *)dlsym(libcuda, "cuEventSynchronize");
}

CudaLib &CudaLib::Get() {
  static CudaLib lib;
  if (!lib.libcuda) {
    throw std::runtime_error("failed to load libcuda.so");
  }
  return lib;
}

} // namespace sapien
