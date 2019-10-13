#pragma once
#include <cstdint>

namespace sapien {
using physx_id_t = uint32_t;

class IDGenerator {
public:
  static IDGenerator *RenderId();
  static IDGenerator *LinkId();
  physx_id_t next() { return _id++; }

private:
  IDGenerator() : _id(1) {}
  physx_id_t _id;
};

}
