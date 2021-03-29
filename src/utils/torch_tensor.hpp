#pragma once

#ifdef SAPIEN_TORCH_INTEROP

#include <memory>
#include <svulkan2/core/cuda_buffer.h>
#include <torch/extension.h>
#include <vector>

torch::Tensor TorchTensorFromCudaBuffer(std::shared_ptr<svulkan2::core::CudaBuffer> buffer,
                                        std::vector<long> const &sizes, vk::Format format) {
  long size = 1;
  for (auto s : sizes) {
    size *= s;
  }
  std::vector<long> sizes2 = sizes;

  torch::ScalarType dtype;
  switch (format) {
  case vk::Format::eR32G32B32A32Sfloat:
    dtype = torch::ScalarType::Float;
    size *= 4;
    sizes2.push_back(4);
    break;
  case vk::Format::eD32Sfloat:
    dtype = torch::ScalarType::Float;
    break;
  case vk::Format::eR32G32B32A32Uint:
    size *= 4;
    dtype = torch::ScalarType::Int;
    sizes2.push_back(4);
    break;
  default:
    throw std::runtime_error("Failed to get tensor from cuda buffer: unsupported buffer format");
  }

  assert(buffer->getSize() == size * 4);

  void *pointer = buffer->getCudaPointer();

  auto tensor = torch::from_blob(
      pointer, sizes2, [buffer](void *) {},
      torch::TensorOptions().device(torch::kCUDA).dtype(dtype));

  return tensor;
}

#endif
