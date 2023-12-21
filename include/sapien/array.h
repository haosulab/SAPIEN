#pragma once
#include <string>
#include <vector>
#include <vulkan/vulkan.hpp>

struct DLManagedTensorVersioned;
struct DLManagedTensor;

namespace sapien {

std::vector<int> ShapeToStrides(std::vector<int> const &shape, int elemSize);

struct CudaArrayHandle {
  std::vector<int> shape;
  std::vector<int> strides;
  std::string type;
  int cudaId{-1};
  void *ptr{nullptr};

  bool isContiguous() const;
  DLManagedTensor *toDLPack() const;

  void checkCongiguous() const;
  void checkShape(std::vector<int> const &expected) const;
  void checkStride(std::vector<int> const &expected) const;

  int bytes() const;
};

struct CudaArray {
  std::vector<int> shape;
  std::string type;
  int cudaId{-1};
  void *ptr{nullptr};

  CudaArrayHandle handle() const;

  DLManagedTensor *moveToDLPack();

  CudaArray() {}
  CudaArray(std::vector<int> shape_, std::string type_);
  CudaArray(CudaArray const &) = delete;
  CudaArray &operator=(CudaArray const &) = delete;
  CudaArray(CudaArray &&other);
  CudaArray &operator=(CudaArray &&other);
  ~CudaArray();
};

struct CpuArrayHandle {
  std::vector<int> shape;
  std::vector<int> strides;
  std::string type;
  void *ptr{};
};

struct CpuArray {
  std::vector<int> shape;
  std::string type;

  std::vector<char> data;
};

} // namespace sapien
