#pragma once
#include <cstdint>

namespace sapien {
using physx_id_t = uint32_t;

class IDGenerator {
public:
  inline physx_id_t next() { return _id++; }

  inline IDGenerator() : _id(1) {}

private:
  physx_id_t _id;
};

} // namespace sapien
