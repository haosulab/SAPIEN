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
#include <sstream>
#include "sapien/array.h"
#include "sapien/utils/cuda.h"
#include "sapien/utils/typestr.h"

#ifdef SAPIEN_CUDA
#include <dlpack/dlpack.h>
#include <cuda_runtime.h>
#endif

namespace sapien {

template <typename T> static std::string VectorToString(std::vector<T> const &v) {
  std::stringstream ss;
  ss << "(";
  for (size_t i = 0; i < v.size(); ++i) {
    ss << v.at(i);
    if (i + 1 != v.size()) {
      ss << ", ";
    }
  }
  ss << ")";
  return ss.str();
}

static bool CheckShape(std::vector<int> const &tested, std::vector<int> const &expected) {
  if (tested.size() != expected.size()) {
    return false;
  }
  for (size_t i = 0; i < tested.size(); ++i) {
    if (expected.at(i) != -1 && tested.at(i) != expected.at(i)) {
      return false;
    }
  }
  return true;
}

#ifdef SAPIEN_CUDA
static DLDataType TypestrToDLDataType(std::string const &d) {
  char code = typestrCode(d);
  int size = typestrBytes(d);

  DLDataType type;
  switch (code) {
  case 'i':
    type.code = kDLInt;
    break;
  case 'u':
    type.code = kDLUInt;
    break;
  case 'f':
    type.code = kDLFloat;
    break;
  default:
    throw std::runtime_error("failed to convert to dlpack: unsupported type");
  }
  type.bits = size * 8;
  type.lanes = 1;
  return type;
}
#endif

std::vector<int> ShapeToStrides(std::vector<int> const &shape, int elemSize) {
  int acc = elemSize;
  std::vector<int> strides;
  for (uint32_t i = 0; i < shape.size(); ++i) {
    strides.push_back(acc);
    acc *= shape.at(shape.size() - i - 1);
  }
  std::reverse(strides.begin(), strides.end());
  return strides;
}

bool CudaArrayHandle::isContiguous() const {
  return strides == ShapeToStrides(shape, typestrBytes(type));
}

void CudaArrayHandle::checkCongiguous() const {
  if (!isContiguous()) {
    throw std::runtime_error("Assertion failed: cuda array is not congiguous");
  }
}
void CudaArrayHandle::checkShape(std::vector<int> const &expected) const {
  if (!CheckShape(shape, expected)) {
    throw std::runtime_error("Assertion failed: cuda array shape " + VectorToString(shape) +
                             " is not compatible with exepcted shape " + VectorToString(expected));
  }
}
void CudaArrayHandle::checkStride(std::vector<int> const &expected) const {
  if (!CheckShape(strides, expected)) {
    throw std::runtime_error("Assertion failed: cuda array strides " + VectorToString(strides) +
                             " does not match exepcted strides " + VectorToString(expected));
  }
}

int CudaArrayHandle::bytes() const {
  int size = 1;
  for (auto s : shape) {
    size *= s;
  }
  return size * typestrBytes(type);
}

#ifdef SAPIEN_CUDA
static void DLManagedTensorDeleter(DLManagedTensor *self) {
  delete self->dl_tensor.strides;
  delete self->dl_tensor.shape;
  delete self;
}

DLManagedTensor *CudaArrayHandle::toDLPack() const {
  auto tensor = new DLManagedTensor();
  tensor->manager_ctx = nullptr;
  tensor->deleter = &DLManagedTensorDeleter;

  tensor->dl_tensor.data = ptr;
  tensor->dl_tensor.device = {kDLCUDA, cudaId};
  tensor->dl_tensor.ndim = shape.size();
  tensor->dl_tensor.dtype = TypestrToDLDataType(type);

  tensor->dl_tensor.shape = new int64_t[shape.size()];
  for (uint32_t i = 0; i < shape.size(); ++i) {
    tensor->dl_tensor.shape[i] = shape[i];
  }

  tensor->dl_tensor.strides = new int64_t[strides.size()];
  for (uint32_t i = 0; i < strides.size(); ++i) {
    tensor->dl_tensor.strides[i] = strides[i] / (tensor->dl_tensor.dtype.bits / 8);
  }
  tensor->dl_tensor.byte_offset = 0;

  return tensor;
}
#endif

CudaArray CudaArray::FromData(void *data, int size) {
  CudaArray buffer({size}, "u1");
#ifdef SAPIEN_CUDA
  checkCudaErrors(cudaMemcpy(buffer.ptr, data, size, cudaMemcpyHostToDevice));
#endif
  return buffer;
}

CudaArray::CudaArray(std::vector<int> shape_, std::string type_)
    : shape(shape_), type(type_), cudaId(-1), ptr(nullptr) {
#ifdef SAPIEN_CUDA
  size_t size = 1;
  for (auto s : shape) {
    size *= s;
  }
  size *= typestrBytes(type);
  if (size > 0) {
    checkCudaErrors(cudaGetDevice(&cudaId));
    checkCudaErrors(cudaMalloc(&ptr, size));
    checkCudaErrors(cudaMemset(ptr, 0, size));
  }
#endif
}

CudaArray::CudaArray(CudaArray &&other) {
#ifdef SAPIEN_CUDA
  shape = other.shape;
  type = other.type;
  cudaId = other.cudaId;
  ptr = other.ptr;

  other.shape.clear();
  other.ptr = nullptr;
#endif
}

CudaArray &CudaArray::operator=(CudaArray &&other) {
#ifdef SAPIEN_CUDA
  if (this != &other) {
    if (cudaId >= 0) {
      checkCudaErrors(cudaSetDevice(cudaId));
    }
    checkCudaErrors(cudaFree(ptr));
    shape = other.shape;
    type = other.type;
    cudaId = other.cudaId;
    ptr = other.ptr;

    other.shape.clear();
    other.ptr = nullptr;
  }
  return *this;
#endif
}

CudaArray::~CudaArray() {
#ifdef SAPIEN_CUDA
  cudaFree(ptr);
#endif
}

CudaHostArray::CudaHostArray(std::vector<int> shape_, std::string type_)
    : shape(shape_), type(type_), ptr(nullptr) {
#ifdef SAPIEN_CUDA
  size_t size = 1;
  for (auto s : shape) {
    size *= s;
  }
  size *= typestrBytes(type);
  if (size > 0) {
    checkCudaErrors(cudaHostAlloc(&ptr, size, cudaHostAllocPortable));
    // checkCudaErrors(cudaMallocHost(&ptr, size));
  }
#endif
}

CudaHostArray::CudaHostArray(CudaHostArray &&other) {
#ifdef SAPIEN_CUDA
  shape = other.shape;
  type = other.type;
  ptr = other.ptr;

  other.shape.clear();
  other.ptr = nullptr;
#endif
}

CudaHostArray::~CudaHostArray() {
#ifdef SAPIEN_CUDA
  cudaFreeHost(ptr);
#endif
}

CudaHostArray &CudaHostArray::operator=(CudaHostArray &&other) {
#ifdef SAPIEN_CUDA
  if (this != &other) {
    checkCudaErrors(cudaFreeHost(ptr));
    shape = other.shape;
    type = other.type;
    ptr = other.ptr;

    other.shape.clear();
    other.ptr = nullptr;
  }
  return *this;
#endif
}

void CudaHostArray::copyFrom(const CudaArray &array) {
#ifdef SAPIEN_CUDA
  if (array.shape != shape) {
    throw std::runtime_error("failed to copy: arrays have different shapes");
  }
  if (array.type != type) {
    throw std::runtime_error("failed to copy: arrays have different types");
  }
  if (!array.ptr) {
    return;
  }
  checkCudaErrors(cudaMemcpy(ptr, array.ptr, array.bytes(), cudaMemcpyDeviceToHost));
#endif
}

CudaArrayHandle CudaArray::handle() const {
  return CudaArrayHandle{.shape = shape,
                         .strides = ShapeToStrides(shape, typestrBytes(type)),
                         .type = type,
                         .cudaId = cudaId,
                         .ptr = ptr};
}

#ifdef SAPIEN_CUDA
static void CudaArrayDLManagedTensorDeleter(DLManagedTensor *self) {
  delete static_cast<CudaArray *>(self->manager_ctx);
  delete self->dl_tensor.shape;
  delete self;
}

DLManagedTensor *CudaArray::moveToDLPack() {
  auto tensor = new DLManagedTensor();
  tensor->deleter = &CudaArrayDLManagedTensorDeleter;

  tensor->dl_tensor.data = ptr;
  tensor->dl_tensor.device = {kDLCUDA, cudaId};
  tensor->dl_tensor.ndim = shape.size();
  tensor->dl_tensor.dtype = TypestrToDLDataType(type);

  tensor->dl_tensor.shape = new int64_t[shape.size()];
  for (uint32_t i = 0; i < shape.size(); ++i) {
    tensor->dl_tensor.shape[i] = shape[i];
  }

  tensor->dl_tensor.strides = nullptr;
  tensor->dl_tensor.byte_offset = 0;

  tensor->manager_ctx = new CudaArray(std::move(*this));
  return tensor;
}
#endif

int CudaArray::bytes() const {
  int size = 1;
  for (auto s : shape) {
    size *= s;
  }
  return size * typestrBytes(type);
}

} // namespace sapien
