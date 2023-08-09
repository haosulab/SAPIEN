#pragma once
#include <cstdint>
#include <stdlib.h>

namespace sapien {
namespace component {

class CudaDataSource {
public:
  virtual uintptr_t getCudaPointer() const = 0;
  virtual uintptr_t getCudaStream() const { return 0; }
  virtual size_t getSize() const = 0;
};

} // namespace component
} // namespace sapien
