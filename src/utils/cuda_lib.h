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
#endif