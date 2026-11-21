#pragma once
#include <cstdint>

struct CUstream_st;
namespace sapien::physx {
void gather_rows(void *dst, void const *src, int const *rows, int count,
                 int rowBytes, int strideBytes, CUstream_st *stream);
}
