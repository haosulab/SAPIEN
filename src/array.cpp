#include "sapien/array.h"
#include "sapien/utils/cuda.h"
#include "sapien/utils/typestr.h"
#include <dlpack/dlpack.h>

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

CudaArray::CudaArray(std::vector<int> shape_, std::string type_)
    : shape(shape_), type(type_), cudaId(-1), ptr(nullptr) {
#ifdef SAPIEN_CUDA
  size_t size = 1;
  for (auto s : shape) {
    size *= s;
  }
  size *= typestrBytes(type);
  checkCudaErrors(cudaGetDevice(&cudaId));
  checkCudaErrors(cudaMalloc(&ptr, size));
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
    if (ptr) {
      checkCudaErrors(cudaFree(ptr));
    }
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

CudaArrayHandle CudaArray::handle() const {
  return CudaArrayHandle{.shape = shape,
                         .strides = ShapeToStrides(shape, typestrBytes(type)),
                         .type = type,
                         .cudaId = cudaId,
                         .ptr = ptr};
}

CudaArray::~CudaArray() {
#ifdef SAPIEN_CUDA
  if (ptr) {
    cudaFree(ptr);
  }
#endif
}

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

} // namespace sapien