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
#include <stdexcept>
#include <string>

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

using cudaStream_t = struct CUstream_st *;
using CUevent = struct CUevent_st *;

namespace sapien {

int getCudaPtrDevice(void *ptr);

struct CudaEvent {
  CudaEvent() {}
  void init();
  CudaEvent(CudaEvent const &) = delete;
  CudaEvent &operator=(CudaEvent const &) = delete;
  CudaEvent(CudaEvent &&other);
  CudaEvent &operator=(CudaEvent &&other);

  /** record cuda event on the current cuda runtime device
   *  @param stream cuda stream, must be created from the same cuda runtime device */
  void record(cudaStream_t stream);

  void wait(cudaStream_t stream) const;

  void synchronize() const;

  ~CudaEvent();

  int cudaId{-1};
  CUevent event{nullptr};
};

}; // namespace sapien
#endif