#include "sapien/utils/cuda.h"
#include <cuda.h>
#include <dlfcn.h>

namespace sapien {

CudaContext::CudaContext() {
  libcuda = dlopen("libcuda.so", RTLD_LAZY);
  this->cuEventRecord = (decltype(::cuEventRecord) *)dlsym(libcuda, "cuEventRecord");
  this->cuStreamWaitEvent = (decltype(::cuStreamWaitEvent) *)dlsym(libcuda, "cuStreamWaitEvent");
}

CudaContext &CudaContext::Get() {
  static CudaContext context;
  if (!context.libcuda) {
    throw std::runtime_error("failed to load libcuda.so");
  }
  return context;
}

} // namespace sapien
