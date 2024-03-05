#include "sapien/utils/cuda.h"
#include <cuda.h>
#if _WIN64
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#else
#include <dlfcn.h>
#endif

namespace sapien {

CudaLib::CudaLib() {
#if _WIN64
  HMODULE handle = LoadLibrary("libcuda.so");
  libcuda = (void*)handle;
  cudaFree(0); // establish cuda context with runtime API
  
  this->cuCtxGetCurrent = (decltype(::cuCtxGetCurrent) *)GetProcAddress(handle, "cuCtxGetCurrent");
  this->cuEventCreate = (decltype(::cuEventCreate) *)GetProcAddress(handle, "cuEventCreate");
  this->cuEventDestroy = (decltype(::cuEventDestroy) *)GetProcAddress(handle, "cuEventDestroy");
  this->cuEventRecord = (decltype(::cuEventRecord) *)GetProcAddress(handle, "cuEventRecord");
  this->cuStreamWaitEvent = (decltype(::cuStreamWaitEvent) *)GetProcAddress(handle, "cuStreamWaitEvent");
  this->cuEventSynchronize =
      (decltype(::cuEventSynchronize) *)GetProcAddress(handle, "cuEventSynchronize");
#else
  libcuda = dlopen("libcuda.so", RTLD_LAZY);

  cudaFree(0); // establish cuda context with runtime API

  this->cuCtxGetCurrent = (decltype(::cuCtxGetCurrent) *)dlsym(libcuda, "cuCtxGetCurrent");
  this->cuEventCreate = (decltype(::cuEventCreate) *)dlsym(libcuda, "cuEventCreate");
  this->cuEventDestroy = (decltype(::cuEventDestroy) *)dlsym(libcuda, "cuEventDestroy");
  this->cuEventRecord = (decltype(::cuEventRecord) *)dlsym(libcuda, "cuEventRecord");
  this->cuStreamWaitEvent = (decltype(::cuStreamWaitEvent) *)dlsym(libcuda, "cuStreamWaitEvent");
  this->cuEventSynchronize =
      (decltype(::cuEventSynchronize) *)dlsym(libcuda, "cuEventSynchronize");
#endif
}

CudaLib &CudaLib::Get() {
  static CudaLib lib;
  if (!lib.libcuda) {
    throw std::runtime_error("failed to load libcuda.so");
  }
  return lib;
}

} // namespace sapien
