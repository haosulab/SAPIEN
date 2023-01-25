#ifdef SAPIEN_DLPACK

#pragma once
#include <dlpack/dlpack.h>
#include <memory>
#include <vector>

namespace sapien {
namespace Renderer {

template <typename T> void dl_deleter(DLManagedTensor *self) {
  delete[] self->dl_tensor.shape;
  delete static_cast<std::shared_ptr<T> *>(self->manager_ctx);
  delete self;
}

template <typename T>
DLManagedTensor *dl_wrapper(std::shared_ptr<T> container, void *cudaPtr, int cudaId,
                            std::vector<int64_t> const &shape, DLDataType type) {
  DLManagedTensor *tensor = new DLManagedTensor();
  std::shared_ptr<T> *_container = new std::shared_ptr<T>(container);
  int64_t *_shape = new int64_t[shape.size()];
  for (uint32_t i = 0; i < shape.size(); ++i) {
    _shape[i] = shape[i];
  }
  tensor->dl_tensor.data = cudaPtr;
  tensor->dl_tensor.device = {DLDeviceType::kDLGPU, cudaId};
  tensor->dl_tensor.ndim = shape.size();
  tensor->dl_tensor.dtype = type;
  tensor->dl_tensor.shape = _shape;
  tensor->dl_tensor.strides = nullptr;
  tensor->dl_tensor.byte_offset = 0;
  tensor->manager_ctx = _container;
  tensor->deleter = dl_deleter<T>;
  return tensor;
}

} // namespace Renderer
} // namespace sapien

#endif
