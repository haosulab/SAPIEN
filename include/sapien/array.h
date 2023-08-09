#pragma once
#include <string>
#include <vector>
#include <vulkan/vulkan.hpp>

namespace sapien {

struct CpuArrayHandle {
  std::vector<int> shape;
  std::vector<int> strides;
  std::string type;
  void *ptr;
};

struct CudaArrayHandle {
  std::vector<int> shape;
  std::vector<int> strides;
  std::string type;
  int cudaId;
  void *ptr;
};

struct CpuArray {
  std::vector<int> shape;
  std::vector<int> strides;
  std::string type;

  std::vector<char> data;
};

} // namespace sapien
