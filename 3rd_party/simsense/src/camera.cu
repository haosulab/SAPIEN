/*
 * Copyright 2025 Hillbot Inc.
 * Copyright 2020-2024 UCSD SU Lab
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "simsense/camera.h"

namespace simsense {

__device__ __forceinline__ float curand_gamma(float shape, float scale, curandState_t *state) {
  float alpha = shape;
  float beta = 1 / scale;

  if (alpha >= 1) {
    float d = alpha - 1 / 3.0, c = 1 / sqrt(9 * d);
    do {
      float z = curand_normal(state);
      float u = curand_uniform(state);
      float v = pow(1.0f + c * z, 3.0f);
      if (z > -1 / c && log(u) < (z * z / 2 + d - d * v + d * log(v))) {
        return d * v / beta;
      }
    } while (true);
  } else {
    float r = curand_gamma(shape + 1, scale, state);
    float u = curand_uniform(state);
    return r * pow(u, 1 / alpha);
  }
}

__device__ __forceinline__ float atomicMinFloat(float *addr, float value) {
  float old;
  old = (value >= 0) ? __int_as_float(atomicMin((int *)addr, __float_as_int(value)))
                     : __uint_as_float(atomicMax((unsigned int *)addr, __float_as_uint(value)));
  return old;
}

__global__ void initInfraredNoise(curandState_t *states, int seed, const int size) {
  const int id = blockIdx.x * blockDim.x + threadIdx.x;
  if (id >= size) {
    return;
  }

  curand_init(seed, id, 0, &states[id]);
}

__global__ void simInfraredNoise(uint8_t *src, uint8_t *dst, curandState_t *states, const int rows,
                                 const int cols, float speckleShape, float speckleScale,
                                 float gaussianMu, float gaussianSigma) {
  const int pos = blockIdx.x * blockDim.x + threadIdx.x;
  if (pos >= rows * cols) {
    return;
  }

  int result = (int)round(src[pos] * curand_gamma(speckleShape, speckleScale, &states[pos]) +
                          gaussianMu + gaussianSigma * curand_normal(&states[pos]));
  if (result < 0) {
    result = 0;
  }
  if (result > 255) {
    result = 255;
  }
  dst[pos] = result;
}

__global__ void remap(const float *mapx, const float *mapy, const uint8_t *src, uint8_t *dst,
                      const int rows, const int cols) {
  const int pos = blockIdx.x * blockDim.x + threadIdx.x;
  if (pos >= rows * cols) {
    return;
  }
  float sx = mapx[pos];
  float sy = mapy[pos];

  // Eliminate error
  if (sx - round(sx) <= ERROR_MARGIN || round(sx) - sx <= ERROR_MARGIN) {
    sx = round(sx);
  }
  if (sy - round(sy) <= ERROR_MARGIN || round(sy) - sy <= ERROR_MARGIN) {
    sy = round(sy);
  }

  // Fit the mapped point into boundary
  if (sx < 0) {
    sx = 0;
  }
  if (sx > cols - 1) {
    sx = cols - 1;
  }
  if (sy < 0) {
    sy = 0;
  }
  if (sy > rows - 1) {
    sy = rows - 1;
  }

  // Bilinear interpolation, with BORDER_VALUE for pixels out of image boundary
  int x1 = floor(sx);
  int x2 = floor(sx) + 1;
  int y1 = floor(sy);
  int y2 = floor(sy) + 1;
  int I11 = (x1 < 0 || x1 >= cols || y1 < 0 || y1 >= rows) ? BORDER_VALUE : src[y1 * cols + x1];
  int I12 = (x1 < 0 || x1 >= cols || y2 < 0 || y2 >= rows) ? BORDER_VALUE : src[y2 * cols + x1];
  int I21 = (x2 < 0 || x2 >= cols || y1 < 0 || y1 >= rows) ? BORDER_VALUE : src[y1 * cols + x2];
  int I22 = (x2 < 0 || x2 >= cols || y2 < 0 || y2 >= rows) ? BORDER_VALUE : src[y2 * cols + x2];

  dst[pos] = (uint8_t)round((x2 - sx) * (y2 - sy) * I11 + (x2 - sx) * (sy - y1) * I12 +
                            (sx - x1) * (y2 - sy) * I21 + (sx - x1) * (sy - y1) * I22);
}

__global__ void copySubArea(uint8_t *src, uint8_t *dest, int srcWidth, int bboxWidth,
                            int bboxHeight, int bboxStartX, int bboxStartY) {
  const int pos = blockIdx.x * blockDim.x + threadIdx.x;
  if (pos >= bboxWidth * bboxHeight) {
    return;
  }

  int destX = pos % bboxWidth;
  int destY = pos / bboxWidth;

  int srcX = bboxStartX + destX;
  int srcY = bboxStartY + destY;

  int srcIndex = srcY * srcWidth + srcX;
  int destIndex = destY * bboxWidth + destX;

  dest[destIndex] = src[srcIndex];
}

__global__ void pasteSubArea(float *src, float *dest, int destWidth, int bboxWidth, int bboxHeight,
                             int destStartX, int destStartY) {
  const int pos = blockIdx.x * blockDim.x + threadIdx.x;
  if (pos >= bboxWidth * bboxHeight) {
    return;
  }

  int srcX = pos % bboxWidth;
  int srcY = pos / bboxWidth;

  int destX = destStartX + srcX;
  int destY = destStartY + srcY;

  int srcIndex = srcY * bboxWidth + srcX;
  int destIndex = destY * destWidth + destX;

  dest[destIndex] = src[srcIndex];
}

__global__ void disp2Depth(const float *disp, float *depth, const int size, const float focalLen,
                           const float baselineLen) {
  const int pos = blockIdx.x * blockDim.x + threadIdx.x;
  if (pos >= size) {
    return;
  }

  depth[pos] = (disp[pos] <= 0) ? 0 : focalLen * baselineLen / disp[pos];
}

__global__ void initRgbDepth(float *rgbDepth, const int size, const float maxDepth) {
  const int pos = blockIdx.x * blockDim.x + threadIdx.x;
  if (pos >= size) {
    return;
  }

  rgbDepth[pos] = maxDepth;
}

__global__ void depthRegistration(float *rgbDepth, const float *depth, const float *a1,
                                  const float *a2, const float *a3, float b1, float b2, float b3,
                                  const int size, const int rgbRows, const int rgbCols) {
  const int pos = blockIdx.x * blockDim.x + threadIdx.x;
  if (pos >= size) {
    return;
  }

  float z = depth[pos];
  float zRgb = a3[pos] * z + b3;
  int x = (int)round((a1[pos] * z + b1) / zRgb);
  int y = (int)round((a2[pos] * z + b2) / zRgb);

  if (zRgb > 0 && x >= 0 && x < rgbCols && y >= 0 && y < rgbRows) {
    float *ptr = rgbDepth + y * rgbCols + x;
    atomicMinFloat(ptr, zRgb); // Update if occluded
  }
}

// Dilate in a 2x2 region where the original projected location is in the bottom right of this
// region.
__global__ void depthDilation(float *depth, const int rgbRows, const int rgbCols,
                              const float maxDepth) {
  const int pos = blockIdx.x * blockDim.x + threadIdx.x;
  if (pos >= rgbRows * rgbCols) {
    return;
  }

  int y = pos / rgbCols;
  int x = pos % rgbCols;
  float z = depth[pos];

  // Left
  if (z < maxDepth && x - 1 >= 0) {
    float *ptr = depth + y * rgbCols + x - 1;
    atomicMinFloat(ptr, z); // Update if occluded
  }

  // Top
  if (z < maxDepth && y - 1 >= 0) {
    float *ptr = depth + (y - 1) * rgbCols + x;
    atomicMinFloat(ptr, z); // Update if occluded
  }

  // Top-left
  if (z < maxDepth && x - 1 >= 0 && y - 1 >= 0) {
    float *ptr = depth + (y - 1) * rgbCols + x - 1;
    atomicMinFloat(ptr, z); // Update if occluded
  }
}

__global__ void correctDepthRange(float *depth, const int size, const float minDepth,
                                  const float maxDepth) {
  const int pos = blockIdx.x * blockDim.x + threadIdx.x;
  if (pos >= size) {
    return;
  }

  if (depth[pos] < minDepth || depth[pos] >= maxDepth) {
    depth[pos] = 0;
  }
}

__global__ void depth2PointCloud(const float *depth, float *pc, const int rows, const int cols,
                                 const float fx, const float fy, const float s, const float cx,
                                 const float cy) {
  const int pos = blockIdx.x * blockDim.x + threadIdx.x;
  if (pos >= rows * cols) {
    return;
  }

  const int u = pos % cols;
  const int v = pos / cols;

  // x right, y down, z forward
  float z = depth[pos];
  float x = z * ((u - cx) / fx + s * (cy - v) / (fx * fy));
  float y = z * (v - cy) / fy;

  pc[3 * pos] = x;
  pc[3 * pos + 1] = y;
  pc[3 * pos + 2] = z;
}

__global__ void depth2RgbPointCloud(const float *depth, const float *rgba, float *pc,
                                    const int rows, const int cols, const float fx, const float fy,
                                    const float s, const float cx, const float cy) {
  const int pos = blockIdx.x * blockDim.x + threadIdx.x;
  if (pos >= rows * cols) {
    return;
  }

  const int u = pos % cols;
  const int v = pos / cols;

  // x right, y down, z forward
  float z = depth[pos];
  float x = z * ((u - cx) / fx + s * (cy - v) / (fx * fy));
  float y = z * (v - cy) / fy;

  // (x, y, z, r, g, b)
  pc[6 * pos] = x;
  pc[6 * pos + 1] = y;
  pc[6 * pos + 2] = z;
  pc[6 * pos + 3] = rgba[4 * pos];
  pc[6 * pos + 4] = rgba[4 * pos + 1];
  pc[6 * pos + 5] = rgba[4 * pos + 2];
}

} // namespace simsense