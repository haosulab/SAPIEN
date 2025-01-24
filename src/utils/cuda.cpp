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
#include "sapien/utils/cuda.h"
#include <cuda.h>

#include "./cuda_lib.h"
#include <cuda.h>
#include <cuda_runtime.h>

namespace sapien {

void CudaEvent::init() {
  checkCudaErrors(cudaGetDevice(&cudaId));
  checkCudaDriverErrors(CudaLib::Get().cuEventCreate(&event, 0));
}

CudaEvent::CudaEvent(CudaEvent &&other) {
  // copy
  cudaId = other.cudaId;
  event = other.event;

  // destory other
  other.event = nullptr;
}

CudaEvent &CudaEvent::operator=(CudaEvent &&other) {
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
void CudaEvent::record(cudaStream_t stream) {
  if (!event) {
    init();
  }
  checkCudaDriverErrors(CudaLib::Get().cuEventRecord(event, stream));
}

void CudaEvent::wait(cudaStream_t stream) const {
  if (!event) {
    // no need to wait if no one records to this event
    return;
  }
  checkCudaDriverErrors(CudaLib::Get().cuStreamWaitEvent(stream, event, 0));
}

void CudaEvent::synchronize() const {
  if (!event) {
    // no need to wait if no one records to this event
  }
  checkCudaDriverErrors(CudaLib::Get().cuEventSynchronize(event));
}

CudaEvent::~CudaEvent() {
  if (event) {
    cudaSetDevice(cudaId);
    CudaLib::Get().cuEventDestroy(event);
  }
}

int getCudaPtrDevice(void *ptr) {
  cudaPointerAttributes attr{};
  checkCudaErrors(cudaPointerGetAttributes(&attr, ptr));
  if (!attr.devicePointer) {
    throw std::runtime_error("invalid cuda pointer");
  }
  return attr.device;
}

} // namespace sapien
#endif