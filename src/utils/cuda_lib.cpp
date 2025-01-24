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
#ifdef SAPIEN_CUDA
#include "./cuda_lib.h"
#include <stdexcept>

#if _WIN64
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#else
#include <dlfcn.h>
#endif

namespace sapien {

CudaLib::CudaLib() {
#if _WIN64
  HMODULE handle = LoadLibrary("nvcuda.dll");
  libcuda = (void *)handle;
  cudaFree(0); // establish cuda context with runtime API

  this->cuCtxGetCurrent = (decltype(::cuCtxGetCurrent) *)GetProcAddress(handle, "cuCtxGetCurrent");
  this->cuEventCreate = (decltype(::cuEventCreate) *)GetProcAddress(handle, "cuEventCreate");
  this->cuEventDestroy = (decltype(::cuEventDestroy) *)GetProcAddress(handle, "cuEventDestroy");
  this->cuEventRecord = (decltype(::cuEventRecord) *)GetProcAddress(handle, "cuEventRecord");
  this->cuStreamWaitEvent =
      (decltype(::cuStreamWaitEvent) *)GetProcAddress(handle, "cuStreamWaitEvent");
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
    throw std::runtime_error("failed to load libcuda");
  }
  return lib;
}

} // namespace sapien
#endif