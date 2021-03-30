#pragma once

#ifdef SAPIEN_DLPACK_INTEROP

#include <dlpack/dlpack.h>
#include <memory>
#include <svulkan2/core/cuda_buffer.h>
// #include <torch/extension.h>
#include <vector>

void deleter(DLManagedTensor *self) {
  delete[] self->dl_tensor.shape;
  delete static_cast<std::shared_ptr<svulkan2::core::CudaBuffer> *>(self->manager_ctx);
}

DLManagedTensor *DLTensorFromCudaBuffer(std::shared_ptr<svulkan2::core::CudaBuffer> buffer,
                                        std::vector<long> const &sizes, vk::Format format) {
  long size = 1;
  for (auto s : sizes) {
    size *= s;
  }
  std::vector<long> sizes2 = sizes;

  uint8_t dtype;
  switch (format) {
  case vk::Format::eR32G32B32A32Sfloat:
    dtype = DLDataTypeCode::kDLFloat;
    size *= 4;
    sizes2.push_back(4);
    break;
  case vk::Format::eD32Sfloat:
    dtype = DLDataTypeCode::kDLFloat;
    break;
  case vk::Format::eR32G32B32A32Uint:
    size *= 4;
    dtype = DLDataTypeCode::kDLUInt;
    sizes2.push_back(4);
    break;
  default:
    throw std::runtime_error("Failed to get tensor from cuda buffer: unsupported buffer format");
  }

  assert(buffer->getSize() == size * 4);

  void *pointer = buffer->getCudaPointer();

  DLManagedTensor *tensor = new DLManagedTensor();

  auto container = new std::shared_ptr<svulkan2::core::CudaBuffer>(buffer);

  int64_t *shape = new int64_t[sizes2.size()];
  for (size_t i = 0; i < sizes2.size(); ++i) {
    shape[i] = sizes2[i];
  }

  if (sizes2.size() >= 2) {
    int64_t t = shape[0];
    shape[0] = shape[1];
    shape[1] = t;
  }

  tensor->dl_tensor.data = pointer;
  tensor->dl_tensor.device = {DLDeviceType::kDLGPU, 0};
  tensor->dl_tensor.ndim = static_cast<int>(sizes2.size());
  tensor->dl_tensor.dtype = {dtype, 32, 1};
  tensor->dl_tensor.shape = shape;
  tensor->dl_tensor.strides = nullptr;
  tensor->dl_tensor.byte_offset = 0;

  tensor->manager_ctx = container;
  tensor->deleter = deleter;

  return tensor;
};

#endif
