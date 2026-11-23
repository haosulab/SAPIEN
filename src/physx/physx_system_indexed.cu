#include "physx_system_indexed.cuh"

namespace sapien::physx {
// Copy words, including integer indices, without interpreting their bit patterns.
__global__ void gather_rows_kernel(uint32_t *dst, uint32_t const *src, int const *rows,
                                  uint64_t count, int width, int stride) {
  uint64_t i = uint64_t(blockIdx.x) * blockDim.x + threadIdx.x;
  if (i < count) {
    dst[i] = src[uint64_t(rows[i / width]) * stride + i % width];
  }
}

void gather_rows(void *dst, void const *src, int const *rows, int count,
                 int rowBytes, int strideBytes, CUstream_st *stream) {
  uint64_t words = uint64_t(count) * (rowBytes / 4);
  if (words) {
    gather_rows_kernel<<<(words + 127) / 128, 128, 0, stream>>>(
        static_cast<uint32_t *>(dst), static_cast<uint32_t const *>(src), rows,
        words, rowBytes / 4, strideBytes / 4);
  }
}
}
