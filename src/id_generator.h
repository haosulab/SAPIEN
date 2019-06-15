#pragma once
#include <cstdint>

using physx_id_t = uint64_t;

class IDGenerator {
public:
  static IDGenerator *instance();
  physx_id_t next() { return _id++; }

private:
  IDGenerator() : _id(1) {}
  physx_id_t _id;
};
