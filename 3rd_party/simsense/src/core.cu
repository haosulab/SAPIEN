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
#include "simsense/aggr.h"
#include "simsense/camera.h"
#include "simsense/core.h"
#include "simsense/cost.h"
#include "simsense/csct.h"
#include "simsense/filter.h"
#include "simsense/lrcheck.h"
#include "simsense/util.h"
#include "simsense/wta.h"
#include <curand.h>
#include <curand_kernel.h>
#include <iostream>

#define gpuErrCheck(ans)                                                                          \
  { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort = true) {
  if (code != cudaSuccess) {
    fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
    if (abort) {
      exit(code);
    }
  }
}

namespace simsense {

void cudaFreeChecked(void *ptr) { gpuErrCheck(cudaFree(ptr)); }

__global__ void float2uint8(const float *src, uint8_t *dst, const int rows, const int cols) {
  const int pos = blockIdx.x * blockDim.x + threadIdx.x;
  if (pos >= rows * cols) {
    return;
  }

  int temp = (int)(src[4 * pos] * 255); // RGBA image, step size is 4
  uint8_t val;
  if (temp < 0) {
    val = 0;
  } else if (temp > 255) {
    val = 255;
  } else {
    val = temp;
  }

  dst[pos] = val;
}

// Constructor without registration
DepthSensorEngine::DepthSensorEngine(
    uint32_t _rows, uint32_t _cols, float _focalLen, float _baselineLen, float _minDepth,
    float _maxDepth, uint64_t infraredNoiseSeed, float _speckleShape, float _speckleScale,
    float _gaussianMu, float _gaussianSigma, bool _rectified, uint8_t _censusWidth,
    uint8_t _censusHeight, uint32_t _maxDisp, uint8_t _bfWidth, uint8_t _bfHeight, uint8_t _p1,
    uint8_t _p2, uint8_t _uniqRatio, uint8_t _lrMaxDiff, uint8_t _mfSize, Mat2d<float> mapLx,
    Mat2d<float> mapLy, Mat2d<float> mapRx, Mat2d<float> mapRy, float _mainFx, float _mainFy,
    float _mainSkew, float _mainCx, float _mainCy) {

  // Create streams and free memory if necessary
  gpuErrCheck(cudaStreamCreate(&stream1));
  gpuErrCheck(cudaStreamCreate(&stream2));
  gpuErrCheck(cudaStreamCreate(&stream3));

  // Intialize class variables
  speckleShape = _speckleShape;
  speckleScale = _speckleScale;
  gaussianMu = _gaussianMu;
  gaussianSigma = _gaussianSigma;
  censusWidth = _censusWidth;
  censusHeight = _censusHeight;
  bfWidth = _bfWidth;
  bfHeight = _bfHeight;
  p1 = _p1;
  p2 = _p2;
  uniqRatio = _uniqRatio;
  lrMaxDiff = _lrMaxDiff;
  mfSize = _mfSize;
  rows = _rows;
  cols = _cols;
  size = rows * cols;
  maxDisp = _maxDisp;
  int size3d = size * maxDisp;
  focalLen = _focalLen;
  baselineLen = _baselineLen;
  minDepth = _minDepth;
  maxDepth = _maxDepth;
  rectified = _rectified;
  registration = false;
  mainFx = _mainFx;
  mainFy = _mainFy;
  mainSkew = _mainSkew;
  mainCx = _mainCx;
  mainCy = _mainCy;
  computed = false;

  // Allocate GPU memory
  gpuErrCheck(cudaMalloc((void **)&d_irNoiseStates0, sizeof(curandState_t) * size));
  gpuErrCheck(cudaMalloc((void **)&d_irNoiseStates1, sizeof(curandState_t) * size));
  initInfraredNoise<<<(size + WARP_SIZE - 1) / WARP_SIZE, WARP_SIZE, 0, stream1>>>(
      static_cast<curandState_t *>(d_irNoiseStates0), infraredNoiseSeed, size);
  initInfraredNoise<<<(size + WARP_SIZE - 1) / WARP_SIZE, WARP_SIZE, 0, stream2>>>(
      static_cast<curandState_t *>(d_irNoiseStates1), infraredNoiseSeed + 1, size);

  gpuErrCheck(cudaMalloc((void **)&d_mapLx, sizeof(float) * size));
  gpuErrCheck(cudaMalloc((void **)&d_mapLy, sizeof(float) * size));
  gpuErrCheck(cudaMalloc((void **)&d_mapRx, sizeof(float) * size));
  gpuErrCheck(cudaMalloc((void **)&d_mapRy, sizeof(float) * size));
  gpuErrCheck(
      cudaMemcpyAsync(d_mapLx, mapLx.data(), sizeof(float) * size, cudaMemcpyHostToDevice));
  gpuErrCheck(
      cudaMemcpyAsync(d_mapLy, mapLy.data(), sizeof(float) * size, cudaMemcpyHostToDevice));
  gpuErrCheck(
      cudaMemcpyAsync(d_mapRx, mapRx.data(), sizeof(float) * size, cudaMemcpyHostToDevice));
  gpuErrCheck(
      cudaMemcpyAsync(d_mapRy, mapRy.data(), sizeof(float) * size, cudaMemcpyHostToDevice));

  gpuErrCheck(cudaMalloc((void **)&d_rawim0, sizeof(uint8_t) * size));
  gpuErrCheck(cudaMalloc((void **)&d_rawim1, sizeof(uint8_t) * size));
  gpuErrCheck(cudaMalloc((void **)&d_noisyim0, sizeof(uint8_t) * size));
  gpuErrCheck(cudaMalloc((void **)&d_noisyim1, sizeof(uint8_t) * size));
  if (!rectified) {
    gpuErrCheck(cudaMalloc((void **)&d_recim0, sizeof(uint8_t) * size));
    gpuErrCheck(cudaMalloc((void **)&d_recim1, sizeof(uint8_t) * size));
  }

  gpuErrCheck(cudaMalloc((void **)&d_census0, sizeof(uint32_t) * size));
  gpuErrCheck(cudaMalloc((void **)&d_census1, sizeof(uint32_t) * size));

  if (bfWidth * bfHeight != 1) {
    gpuErrCheck(cudaMalloc((void **)&d_rawcost, sizeof(cost_t) * size3d));
    gpuErrCheck(cudaMalloc((void **)&d_hsum, sizeof(cost_t) * size3d));
  }
  gpuErrCheck(cudaMalloc((void **)&d_cost, sizeof(cost_t) * size3d));

  gpuErrCheck(cudaMalloc((void **)&d_L0, sizeof(cost_t) * size3d));
  gpuErrCheck(cudaMalloc((void **)&d_L1, sizeof(cost_t) * size3d));
  gpuErrCheck(cudaMalloc((void **)&d_L2, sizeof(cost_t) * size3d));
  gpuErrCheck(cudaMalloc((void **)&d_LAll, sizeof(cost_t) * size3d));

  gpuErrCheck(cudaMalloc((void **)&d_leftDisp, sizeof(float) * size));
  gpuErrCheck(cudaMalloc((void **)&d_rightDisp, sizeof(uint16_t) * size));

  if (mfSize != 1) {
    gpuErrCheck(cudaMalloc((void **)&d_filteredDisp, sizeof(float) * size));
  }

  gpuErrCheck(cudaMalloc((void **)&d_depth, sizeof(float) * size));
  depthContainer = std::shared_ptr<float>(d_depth, cudaFreeChecked);
  h_depth = new float[size];

  gpuErrCheck(cudaMalloc((void **)&d_pc, sizeof(float) * 3 * size));
  pcContainer = std::shared_ptr<float>(d_pc, cudaFreeChecked);
  h_pc = new float[3 * size];

  gpuErrCheck(cudaMalloc((void **)&d_rgbPc, sizeof(float) * 6 * size));
  rgbPcContainer = std::shared_ptr<float>(d_rgbPc, cudaFreeChecked);
  h_rgbPc = new float[6 * size];

  gpuErrCheck(cudaDeviceSynchronize());
}

// Constructor with registration
DepthSensorEngine::DepthSensorEngine(
    uint32_t _rows, uint32_t _cols, uint32_t _rgbRows, uint32_t _rgbCols, float _focalLen,
    float _baselineLen, float _minDepth, float _maxDepth, uint64_t infraredNoiseSeed,
    float _speckleShape, float _speckleScale, float _gaussianMu, float _gaussianSigma,
    bool _rectified, uint8_t _censusWidth, uint8_t _censusHeight, uint32_t _maxDisp,
    uint8_t _bfWidth, uint8_t _bfHeight, uint8_t _p1, uint8_t _p2, uint8_t _uniqRatio,
    uint8_t _lrMaxDiff, uint8_t _mfSize, Mat2d<float> mapLx, Mat2d<float> mapLy,
    Mat2d<float> mapRx, Mat2d<float> mapRy, Mat2d<float> a1, Mat2d<float> a2, Mat2d<float> a3,
    float _b1, float _b2, float _b3, bool _dilation, float _mainFx, float _mainFy, float _mainSkew,
    float _mainCx, float _mainCy) {
  // Create streams and free memory if necessary
  gpuErrCheck(cudaStreamCreate(&stream1));
  gpuErrCheck(cudaStreamCreate(&stream2));
  gpuErrCheck(cudaStreamCreate(&stream3));

  // Intialize class variables
  speckleShape = _speckleShape;
  speckleScale = _speckleScale;
  gaussianMu = _gaussianMu;
  gaussianSigma = _gaussianSigma;
  censusWidth = _censusWidth;
  censusHeight = _censusHeight;
  bfWidth = _bfWidth;
  bfHeight = _bfHeight;
  p1 = _p1;
  p2 = _p2;
  uniqRatio = _uniqRatio;
  lrMaxDiff = _lrMaxDiff;
  mfSize = _mfSize;
  rows = _rows;
  cols = _cols;
  size = rows * cols;
  maxDisp = _maxDisp;
  int size3d = size * maxDisp;
  focalLen = _focalLen;
  baselineLen = _baselineLen;
  minDepth = _minDepth;
  maxDepth = _maxDepth;
  rectified = _rectified;
  registration = true;
  dilation = _dilation;
  rgbRows = _rgbRows;
  rgbCols = _rgbCols;
  rgbSize = rgbRows * rgbCols;
  b1 = _b1;
  b2 = _b2;
  b3 = _b3;
  mainFx = _mainFx;
  mainFy = _mainFy;
  mainSkew = _mainSkew;
  mainCx = _mainCx;
  mainCy = _mainCy;
  computed = false;

  // Allocate GPU memory
  gpuErrCheck(cudaMalloc((void **)&d_irNoiseStates0, sizeof(curandState_t) * size));
  gpuErrCheck(cudaMalloc((void **)&d_irNoiseStates1, sizeof(curandState_t) * size));
  initInfraredNoise<<<(size + WARP_SIZE - 1) / WARP_SIZE, WARP_SIZE, 0, stream1>>>(
      static_cast<curandState_t *>(d_irNoiseStates0), infraredNoiseSeed, size);
  initInfraredNoise<<<(size + WARP_SIZE - 1) / WARP_SIZE, WARP_SIZE, 0, stream2>>>(
      static_cast<curandState_t *>(d_irNoiseStates1), infraredNoiseSeed + 1, size);

  gpuErrCheck(cudaMalloc((void **)&d_mapLx, sizeof(float) * size));
  gpuErrCheck(cudaMalloc((void **)&d_mapLy, sizeof(float) * size));
  gpuErrCheck(cudaMalloc((void **)&d_mapRx, sizeof(float) * size));
  gpuErrCheck(cudaMalloc((void **)&d_mapRy, sizeof(float) * size));
  gpuErrCheck(
      cudaMemcpyAsync(d_mapLx, mapLx.data(), sizeof(float) * size, cudaMemcpyHostToDevice));
  gpuErrCheck(
      cudaMemcpyAsync(d_mapLy, mapLy.data(), sizeof(float) * size, cudaMemcpyHostToDevice));
  gpuErrCheck(
      cudaMemcpyAsync(d_mapRx, mapRx.data(), sizeof(float) * size, cudaMemcpyHostToDevice));
  gpuErrCheck(
      cudaMemcpyAsync(d_mapRy, mapRy.data(), sizeof(float) * size, cudaMemcpyHostToDevice));

  gpuErrCheck(cudaMalloc((void **)&d_rawim0, sizeof(uint8_t) * size));
  gpuErrCheck(cudaMalloc((void **)&d_rawim1, sizeof(uint8_t) * size));
  gpuErrCheck(cudaMalloc((void **)&d_noisyim0, sizeof(uint8_t) * size));
  gpuErrCheck(cudaMalloc((void **)&d_noisyim1, sizeof(uint8_t) * size));
  if (!rectified) {
    gpuErrCheck(cudaMalloc((void **)&d_recim0, sizeof(uint8_t) * size));
    gpuErrCheck(cudaMalloc((void **)&d_recim1, sizeof(uint8_t) * size));
  }
  gpuErrCheck(cudaMalloc((void **)&d_bboxim0, sizeof(uint8_t) * size));
  gpuErrCheck(cudaMalloc((void **)&d_bboxim1, sizeof(uint8_t) * size));

  gpuErrCheck(cudaMalloc((void **)&d_census0, sizeof(uint32_t) * size));
  gpuErrCheck(cudaMalloc((void **)&d_census1, sizeof(uint32_t) * size));

  if (bfWidth * bfHeight != 1) {
    gpuErrCheck(cudaMalloc((void **)&d_rawcost, sizeof(cost_t) * size3d));
    gpuErrCheck(cudaMalloc((void **)&d_hsum, sizeof(cost_t) * size3d));
  }
  gpuErrCheck(cudaMalloc((void **)&d_cost, sizeof(cost_t) * size3d));

  gpuErrCheck(cudaMalloc((void **)&d_L0, sizeof(cost_t) * size3d));
  gpuErrCheck(cudaMalloc((void **)&d_L1, sizeof(cost_t) * size3d));
  gpuErrCheck(cudaMalloc((void **)&d_L2, sizeof(cost_t) * size3d));
  gpuErrCheck(cudaMalloc((void **)&d_LAll, sizeof(cost_t) * size3d));

  gpuErrCheck(cudaMalloc((void **)&d_leftDisp, sizeof(float) * size));
  gpuErrCheck(cudaMalloc((void **)&d_rightDisp, sizeof(uint16_t) * size));
  if (mfSize != 1) {
    gpuErrCheck(cudaMalloc((void **)&d_filteredDisp, sizeof(float) * size));
  }
  gpuErrCheck(cudaMalloc((void **)&d_bboxDisp, sizeof(float) * size));

  gpuErrCheck(cudaMalloc((void **)&d_a1, sizeof(float) * size));
  gpuErrCheck(cudaMalloc((void **)&d_a2, sizeof(float) * size));
  gpuErrCheck(cudaMalloc((void **)&d_a3, sizeof(float) * size));
  gpuErrCheck(cudaMemcpyAsync(d_a1, a1.data(), sizeof(float) * size, cudaMemcpyHostToDevice));
  gpuErrCheck(cudaMemcpyAsync(d_a2, a2.data(), sizeof(float) * size, cudaMemcpyHostToDevice));
  gpuErrCheck(cudaMemcpyAsync(d_a3, a3.data(), sizeof(float) * size, cudaMemcpyHostToDevice));

  gpuErrCheck(cudaMalloc((void **)&d_depth, sizeof(float) * size));
  gpuErrCheck(cudaMalloc((void **)&d_rgbDepth, sizeof(float) * rgbSize));
  depthContainer = std::shared_ptr<float>(d_rgbDepth, cudaFreeChecked);
  h_depth = new float[rgbSize];

  gpuErrCheck(cudaMalloc((void **)&d_pc, sizeof(float) * 3 * rgbSize));
  pcContainer = std::shared_ptr<float>(d_pc, cudaFreeChecked);
  h_pc = new float[3 * rgbSize];

  gpuErrCheck(cudaMalloc((void **)&d_rgbPc, sizeof(float) * 6 * rgbSize));
  rgbPcContainer = std::shared_ptr<float>(d_rgbPc, cudaFreeChecked);
  h_rgbPc = new float[6 * rgbSize];

  gpuErrCheck(cudaDeviceSynchronize());
}

void DepthSensorEngine::compute(Mat2d<uint8_t> left, Mat2d<uint8_t> right, bool bbox,
                                uint32_t bboxStartX, uint32_t bboxStartY, uint32_t bboxWidth,
                                uint32_t bboxHeight) {
  computed = false; // Start computing

  // Instance check
  if (left.rows() != right.rows() || left.cols() != right.cols()) {
    throw std::runtime_error("Both images must have the same size");
  }
  if (cols != left.cols() || rows != left.rows()) {
    throw std::runtime_error("Input image size different from initiated");
  }

  // Upload to GPU
  gpuErrCheck(
      cudaMemcpyAsync(d_rawim0, left.data(), sizeof(uint8_t) * size, cudaMemcpyHostToDevice));
  gpuErrCheck(
      cudaMemcpyAsync(d_rawim1, right.data(), sizeof(uint8_t) * size, cudaMemcpyHostToDevice));

  computeDepth(d_rawim0, d_rawim1, bbox, bboxStartX, bboxStartY, bboxWidth,
               bboxHeight); // Result stored at d_rgbDepth or d_depth
  computed = true;          // Computation done
}

void DepthSensorEngine::compute(void *leftCuda, void *rightCuda, bool bbox, uint32_t bboxStartX,
                                uint32_t bboxStartY, uint32_t bboxWidth, uint32_t bboxHeight) {
  computed = false; // Start computing

  // Get infrared (first) channel from the given RGBA data and convert float to uint8_t
  float2uint8<<<(size + WARP_SIZE - 1) / (WARP_SIZE), WARP_SIZE, 0, stream1>>>(
      (float *)leftCuda, d_rawim0, rows, cols);
  float2uint8<<<(size + WARP_SIZE - 1) / (WARP_SIZE), WARP_SIZE, 0, stream2>>>(
      (float *)rightCuda, d_rawim1, rows, cols);

  computeDepth(d_rawim0, d_rawim1, bbox, bboxStartX, bboxStartY, bboxWidth,
               bboxHeight); // Result stored at d_rgbDepth or d_depth
  computed = true;          // Computation done
}

Mat2d<float> DepthSensorEngine::getMat2d() {
  if (!computed) {
    throw std::runtime_error("No computed data stored");
  }

  // Download to CPU
  if (registration) {
    gpuErrCheck(cudaMemcpy(h_depth, d_rgbDepth, sizeof(float) * rgbSize, cudaMemcpyDeviceToHost));
    Mat2d<float> depth(rgbRows, rgbCols, h_depth);
    return depth;
  } else {
    gpuErrCheck(cudaMemcpy(h_depth, d_depth, sizeof(float) * size, cudaMemcpyDeviceToHost));
    Mat2d<float> depth(rows, cols, h_depth);
    return depth;
  }
}

Mat2d<float> DepthSensorEngine::getPointCloudMat2d() {
  if (!computed) {
    throw std::runtime_error("No computed data stored");
  }

  int localRows = registration ? rgbRows : rows;
  int localCols = registration ? rgbCols : cols;
  int localSize = registration ? rgbSize : size;
  float *d_localDepth = registration ? d_rgbDepth : d_depth;

  // Raise depth to point cloud
  depth2PointCloud<<<(localSize + WARP_SIZE - 1) / (WARP_SIZE), WARP_SIZE, 0, stream1>>>(
      d_localDepth, d_pc, localRows, localCols, mainFx, mainFy, mainSkew, mainCx, mainCy);
  gpuErrCheck(cudaDeviceSynchronize());

  // Download to CPU
  gpuErrCheck(cudaMemcpy(h_pc, d_pc, sizeof(float) * 3 * localSize, cudaMemcpyDeviceToHost));
  Mat2d<float> pc(localSize, 3, h_pc);
  return pc;
}

int DepthSensorEngine::getCudaId() {
  int cudaId;
  gpuErrCheck(cudaGetDevice(&cudaId));
  return cudaId;
}

void *DepthSensorEngine::getCudaPtr() {
  if (!computed) {
    throw std::runtime_error("No computed data stored");
  }
  return (registration) ? d_rgbDepth : d_depth;
}

void *DepthSensorEngine::getPointCloudCudaPtr() {
  if (!computed) {
    throw std::runtime_error("No computed data stored");
  }

  int localRows = registration ? rgbRows : rows;
  int localCols = registration ? rgbCols : cols;
  int localSize = registration ? rgbSize : size;
  float *d_localDepth = registration ? d_rgbDepth : d_depth;

  // Raise depth to point cloud
  depth2PointCloud<<<(localSize + WARP_SIZE - 1) / (WARP_SIZE), WARP_SIZE, 0, stream1>>>(
      d_localDepth, d_pc, localRows, localCols, mainFx, mainFy, mainSkew, mainCx, mainCy);
  gpuErrCheck(cudaDeviceSynchronize());

  return d_pc;
}

Mat2d<float> DepthSensorEngine::getRgbPointCloudMat2d(void *rgbaCuda) {
  if (!computed) {
    throw std::runtime_error("No computed data stored");
  }

  int localRows = registration ? rgbRows : rows;
  int localCols = registration ? rgbCols : cols;
  int localSize = registration ? rgbSize : size;
  float *d_localDepth = registration ? d_rgbDepth : d_depth;

  // Raise depth to point cloud
  depth2RgbPointCloud<<<(localSize + WARP_SIZE - 1) / (WARP_SIZE), WARP_SIZE, 0, stream1>>>(
      d_localDepth, (float *)rgbaCuda, d_rgbPc, localRows, localCols, mainFx, mainFy, mainSkew,
      mainCx, mainCy);
  gpuErrCheck(cudaDeviceSynchronize());

  gpuErrCheck(cudaMemcpy(h_rgbPc, d_rgbPc, sizeof(float) * 6 * localSize, cudaMemcpyDeviceToHost));
  Mat2d<float> rgbPc(localSize, 6, h_rgbPc);

  return rgbPc;
}

void *DepthSensorEngine::getRgbPointCloudCudaPtr(void *rgbaCuda) {
  if (!computed) {
    throw std::runtime_error("No computed data stored");
  }

  int localRows = registration ? rgbRows : rows;
  int localCols = registration ? rgbCols : cols;
  int localSize = registration ? rgbSize : size;
  float *d_localDepth = registration ? d_rgbDepth : d_depth;

  // Raise depth to point cloud
  depth2RgbPointCloud<<<(localSize + WARP_SIZE - 1) / (WARP_SIZE), WARP_SIZE, 0, stream1>>>(
      d_localDepth, (float *)rgbaCuda, d_rgbPc, localRows, localCols, mainFx, mainFy, mainSkew,
      mainCx, mainCy);
  gpuErrCheck(cudaDeviceSynchronize());

  return d_rgbPc;
}

void DepthSensorEngine::setInfraredNoiseParameters(float _speckleShape, float _speckleScale,
                                                   float _gaussianMu, float _gaussianSigma) {
  speckleShape = _speckleShape;
  speckleScale = _speckleScale;
  gaussianMu = _gaussianMu;
  gaussianSigma = _gaussianSigma;
}

void DepthSensorEngine::setPenalties(uint8_t _p1, uint8_t _p2) {
  p1 = _p1;
  p2 = _p2;
}

void DepthSensorEngine::setCensusWindowSize(uint8_t _censusWidth, uint8_t _censusHeight) {
  censusWidth = _censusWidth;
  censusHeight = _censusHeight;
}

void DepthSensorEngine::setMatchingBlockSize(uint8_t _bfWidth, uint8_t _bfHeight) {
  bfWidth = _bfWidth;
  bfHeight = _bfHeight;
}

void DepthSensorEngine::setUniquenessRatio(uint8_t _uniqRatio) { uniqRatio = _uniqRatio; }

void DepthSensorEngine::setLrMaxDiff(uint8_t _lrMaxDiff) { lrMaxDiff = _lrMaxDiff; }

DepthSensorEngine::~DepthSensorEngine() {
  gpuErrCheck(cudaStreamDestroy(stream1));
  gpuErrCheck(cudaStreamDestroy(stream2));
  gpuErrCheck(cudaStreamDestroy(stream3));

  gpuErrCheck(cudaFree(d_irNoiseStates0));
  gpuErrCheck(cudaFree(d_irNoiseStates1));

  gpuErrCheck(cudaFree(d_mapLx));
  gpuErrCheck(cudaFree(d_mapLy));
  gpuErrCheck(cudaFree(d_mapRx));
  gpuErrCheck(cudaFree(d_mapRy));

  gpuErrCheck(cudaFree(d_rawim0));
  gpuErrCheck(cudaFree(d_rawim1));
  gpuErrCheck(cudaFree(d_noisyim0));
  gpuErrCheck(cudaFree(d_noisyim1));
  if (!rectified) {
    gpuErrCheck(cudaFree(d_recim0));
    gpuErrCheck(cudaFree(d_recim1));
  }
  gpuErrCheck(cudaFree(d_bboxim0));
  gpuErrCheck(cudaFree(d_bboxim1));

  gpuErrCheck(cudaFree(d_census0));
  gpuErrCheck(cudaFree(d_census1));

  if (bfWidth * bfHeight != 1) {
    gpuErrCheck(cudaFree(d_rawcost));
    gpuErrCheck(cudaFree(d_hsum));
  }
  gpuErrCheck(cudaFree(d_cost));

  gpuErrCheck(cudaFree(d_L0));
  gpuErrCheck(cudaFree(d_L1));
  gpuErrCheck(cudaFree(d_L2));
  gpuErrCheck(cudaFree(d_LAll));

  gpuErrCheck(cudaFree(d_leftDisp));
  gpuErrCheck(cudaFree(d_rightDisp));
  gpuErrCheck(cudaFree(d_bboxDisp));

  if (mfSize != 1) {
    gpuErrCheck(cudaFree(d_filteredDisp));
  }

  if (registration) {
    gpuErrCheck(cudaFree(d_a1));
    gpuErrCheck(cudaFree(d_a2));
    gpuErrCheck(cudaFree(d_a3));
    gpuErrCheck(cudaFree(d_depth)); // d_rgbDepth is managed by container instead of d_depth
  }
  delete h_depth;
  delete h_pc;
  delete h_rgbPc;

  gpuErrCheck(cudaDeviceSynchronize());
}

__forceinline__ void DepthSensorEngine::computeDepth(uint8_t *d_rawim0, uint8_t *d_rawim1,
                                                     bool bbox, uint32_t bboxStartX,
                                                     uint32_t bboxStartY, uint32_t bboxWidth,
                                                     uint32_t bboxHeight) {
  gpuErrCheck(cudaDeviceSynchronize());

#ifdef PRINT_RUNTIME
  float runtime;
  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
#endif

  uint8_t *d_srcim0 = d_rawim0;
  uint8_t *d_srcim1 = d_rawim1;
  // Infrared Noise Simulation
  if (speckleShape > 0) {
#ifdef PRINT_RUNTIME
    cudaEventRecord(start);
#endif
    simInfraredNoise<<<(size + WARP_SIZE - 1) / (WARP_SIZE), WARP_SIZE, 0, stream1>>>(
        d_srcim0, d_noisyim0, static_cast<curandState_t *>(d_irNoiseStates0), rows, cols,
        speckleShape, speckleScale, gaussianMu, gaussianSigma);
    simInfraredNoise<<<(size + WARP_SIZE - 1) / (WARP_SIZE), WARP_SIZE, 0, stream2>>>(
        d_srcim1, d_noisyim1, static_cast<curandState_t *>(d_irNoiseStates1), rows, cols,
        speckleShape, speckleScale, gaussianMu, gaussianSigma);
#ifdef PRINT_RUNTIME
    cudaEventRecord(stop);
    gpuErrCheck(cudaDeviceSynchronize());
    cudaEventElapsedTime(&runtime, start, stop);
    printf("Runtime of IR noise simulation: %f ms\n", runtime);
#endif
    d_srcim0 = d_noisyim0;
    d_srcim1 = d_noisyim1;
  }

  // Rectification
  if (!rectified) {
    gpuErrCheck(cudaDeviceSynchronize());
#ifdef PRINT_RUNTIME
    cudaEventRecord(start);
#endif
    remap<<<(size + 8 * WARP_SIZE - 1) / (8 * WARP_SIZE), 8 * WARP_SIZE, 0, stream1>>>(
        d_mapLx, d_mapLy, d_srcim0, d_recim0, rows, cols);
    remap<<<(size + 8 * WARP_SIZE - 1) / (8 * WARP_SIZE), 8 * WARP_SIZE, 0, stream2>>>(
        d_mapRx, d_mapRy, d_srcim1, d_recim1, rows, cols);
#ifdef PRINT_RUNTIME
    cudaEventRecord(stop);
    gpuErrCheck(cudaDeviceSynchronize());
    cudaEventElapsedTime(&runtime, start, stop);
    printf("Runtime of rectification: %f ms\n", runtime);
#endif
    d_srcim0 = d_recim0;
    d_srcim1 = d_recim1;
  }

  // Clip out bbox area
  uint32_t origCols = cols;
  uint32_t origRows = rows;
  if (bbox) {
    uint64_t bboxSize = bboxWidth * bboxHeight;
    gpuErrCheck(cudaDeviceSynchronize());
    copySubArea<<<(bboxSize + WARP_SIZE - 1) / WARP_SIZE, WARP_SIZE, 0, stream1>>>(
        d_srcim0, d_bboxim0, cols, bboxWidth, bboxHeight, bboxStartX, bboxStartY);
    copySubArea<<<(bboxSize + WARP_SIZE - 1) / WARP_SIZE, WARP_SIZE, 0, stream2>>>(
        d_srcim1, d_bboxim1, cols, bboxWidth, bboxHeight, bboxStartX, bboxStartY);
    d_srcim0 = d_bboxim0;
    d_srcim1 = d_bboxim1;
    cols = bboxWidth;
    rows = bboxHeight;
    size = bboxSize;
  }

  // Center-symmetric census transform
  dim3 CSCTBlockSize;
  CSCTBlockSize.x = WARP_SIZE;
  CSCTBlockSize.y = WARP_SIZE;
  dim3 CSCTGridSize;
  CSCTGridSize.x = (cols + CSCTBlockSize.x - 1) / CSCTBlockSize.x;
  CSCTGridSize.y = (rows + CSCTBlockSize.y - 1) / CSCTBlockSize.y;
  int CSCTWinCols = (WARP_SIZE + censusWidth - 1);
  int CSCTWinRows = (WARP_SIZE + censusHeight - 1);
  int CSCTSharedMemSize = 2 * CSCTWinCols * CSCTWinRows * sizeof(uint8_t);
  gpuErrCheck(cudaDeviceSynchronize());
#ifdef PRINT_RUNTIME
  cudaEventRecord(start);
#endif
  CSCT<<<CSCTGridSize, CSCTBlockSize, CSCTSharedMemSize, stream1>>>(
      d_srcim0, d_srcim1, d_census0, d_census1, rows, cols, censusWidth, censusHeight);
#ifdef PRINT_RUNTIME
  cudaEventRecord(stop);
  gpuErrCheck(cudaDeviceSynchronize());
  cudaEventElapsedTime(&runtime, start, stop);
  printf("Runtime of CSCT: %f ms\n", runtime);
#endif

  // Hamming distance
  dim3 costGridSize;
  costGridSize.x = (cols + maxDisp - 1) / maxDisp;
  costGridSize.y = rows;
  gpuErrCheck(cudaDeviceSynchronize());
#ifdef PRINT_RUNTIME
  cudaEventRecord(start);
#endif
  if (bfWidth * bfHeight == 1) {
    hammingCost<<<costGridSize, maxDisp, 3 * maxDisp * sizeof(uint32_t), stream1>>>(
        d_census0, d_census1, d_cost, rows, cols, maxDisp);
  } else {
    // Apply box filter
    hammingCost<<<costGridSize, maxDisp, 3 * maxDisp * sizeof(uint32_t), stream1>>>(
        d_census0, d_census1, d_rawcost, rows, cols, maxDisp);
    gpuErrCheck(cudaDeviceSynchronize());
    boxFilterHorizontal<<<rows, maxDisp, 0, stream1>>>(d_rawcost, d_hsum, rows, cols, maxDisp,
                                                       bfWidth);
    gpuErrCheck(cudaDeviceSynchronize());
    boxFilterVertical<<<cols, maxDisp, 0, stream1>>>(d_hsum, d_cost, rows, cols, maxDisp,
                                                     bfHeight);
  }
#ifdef PRINT_RUNTIME
  cudaEventRecord(stop);
  gpuErrCheck(cudaDeviceSynchronize());
  cudaEventElapsedTime(&runtime, start, stop);
  printf("Runtime of cost calculation + box filter: %f ms\n", runtime);
#endif

  // Cost aggregation
  gpuErrCheck(cudaDeviceSynchronize());
  int P1 = p1 * bfWidth * bfHeight;
  int P2 = p2 * bfWidth * bfHeight;
  aggrLeft2Right<<<rows, maxDisp, maxDisp * sizeof(cost_t), stream1>>>(d_cost, d_L0, P1, P2, rows,
                                                                       cols, maxDisp);
  aggrRight2Left<<<rows, maxDisp, maxDisp * sizeof(cost_t), stream2>>>(d_cost, d_L1, P1, P2, rows,
                                                                       cols, maxDisp);
  aggrTop2Bottom<<<cols, maxDisp, maxDisp * sizeof(cost_t), stream3>>>(d_cost, d_L2, P1, P2, rows,
                                                                       cols, maxDisp);
  gpuErrCheck(cudaDeviceSynchronize());
#ifdef PRINT_RUNTIME
  cudaEventRecord(start);
#endif
  aggrBottom2Top<<<cols, maxDisp, maxDisp * sizeof(cost_t), stream1>>>(
      d_cost, d_LAll, d_L0, d_L1, d_L2, P1, P2, rows, cols, maxDisp);
#ifdef PRINT_RUNTIME
  cudaEventRecord(stop);
  gpuErrCheck(cudaDeviceSynchronize());
  cudaEventElapsedTime(&runtime, start, stop);
  printf("Runtime of cost aggregation: %f ms\n", runtime);
#endif

  // Winner takes all
  dim3 WTAGridSize;
  WTAGridSize.x = cols;
  WTAGridSize.y = rows;
  int WTAThrNum = ((maxDisp + WARP_SIZE - 1) / WARP_SIZE) * WARP_SIZE;
  gpuErrCheck(cudaDeviceSynchronize());
#ifdef PRINT_RUNTIME
  cudaEventRecord(start);
#endif
  winnerTakesAll<<<WTAGridSize, WTAThrNum, maxDisp * sizeof(cost_t), stream1>>>(
      d_LAll, d_leftDisp, d_rightDisp, cols, maxDisp, uniqRatio);
#ifdef PRINT_RUNTIME
  cudaEventRecord(stop);
  gpuErrCheck(cudaDeviceSynchronize());
  cudaEventElapsedTime(&runtime, start, stop);
  printf("Runtime of winner takes all: %f ms\n", runtime);
#endif

  if (lrMaxDiff != 255) {
    gpuErrCheck(cudaDeviceSynchronize());
#ifdef PRINT_RUNTIME
    cudaEventRecord(start);
#endif
    lrConsistencyCheck<<<(size + 8 * WARP_SIZE - 1) / (8 * WARP_SIZE), 8 * WARP_SIZE, 0,
                         stream1>>>(d_leftDisp, d_rightDisp, rows, cols, lrMaxDiff);
#ifdef PRINT_RUNTIME
    cudaEventRecord(stop);
    gpuErrCheck(cudaDeviceSynchronize());
    cudaEventElapsedTime(&runtime, start, stop);
    printf("Runtime of left-right consistency check: %f ms\n", runtime);
#endif
  }

  float *d_destDisp = d_leftDisp;
  // Apply median filter
  if (mfSize != 1) {
    int mfSharedMemSize = 2 * WARP_SIZE * mfSize * mfSize * sizeof(float);
    gpuErrCheck(cudaDeviceSynchronize());
#ifdef PRINT_RUNTIME
    cudaEventRecord(start);
#endif
    medianFilter<<<(size + 2 * WARP_SIZE - 1) / (2 * WARP_SIZE), 2 * WARP_SIZE, mfSharedMemSize,
                   stream1>>>(d_destDisp, d_filteredDisp, rows, cols, mfSize);
#ifdef PRINT_RUNTIME
    cudaEventRecord(stop);
    gpuErrCheck(cudaDeviceSynchronize());
    cudaEventElapsedTime(&runtime, start, stop);
    printf("Runtime of median filter: %f ms\n", runtime);
#endif
    d_destDisp = d_filteredDisp;
  }

  // Pasting bbox area back to the original disparity map
  if (bbox) {
    gpuErrCheck(cudaDeviceSynchronize());
    pasteSubArea<<<(size + WARP_SIZE - 1) / WARP_SIZE, WARP_SIZE, 0, stream1>>>(
        d_destDisp, d_bboxDisp, origCols, cols, rows, bboxStartX, bboxStartY);
    cols = origCols;
    rows = origRows;
    size = cols * rows;
    d_destDisp = d_bboxDisp;
  }

  // Convert disparity into depth
  gpuErrCheck(cudaDeviceSynchronize());
  disp2Depth<<<(size + 8 * WARP_SIZE - 1) / (8 * WARP_SIZE), 8 * WARP_SIZE, 0, stream1>>>(
      d_destDisp, d_depth, size, focalLen, baselineLen);

  if (registration) {
    // Transfrom the depth map from left camera's frame to RGB camera's frame
    gpuErrCheck(cudaDeviceSynchronize());
#ifdef PRINT_RUNTIME
    cudaEventRecord(start);
#endif
    initRgbDepth<<<(rgbSize + 8 * WARP_SIZE - 1) / (8 * WARP_SIZE), 8 * WARP_SIZE, 0, stream1>>>(
        d_rgbDepth, rgbSize, maxDepth);
    depthRegistration<<<(size + 8 * WARP_SIZE - 1) / (8 * WARP_SIZE), 8 * WARP_SIZE, 0, stream1>>>(
        d_rgbDepth, d_depth, d_a1, d_a2, d_a3, b1, b2, b3, size, rgbRows, rgbCols);
    if (dilation) {
      depthDilation<<<(rgbSize + 8 * WARP_SIZE - 1) / (8 * WARP_SIZE), 8 * WARP_SIZE, 0,
                      stream1>>>(d_rgbDepth, rgbRows, rgbCols, maxDepth);
    }
    correctDepthRange<<<(rgbSize + 8 * WARP_SIZE - 1) / (8 * WARP_SIZE), 8 * WARP_SIZE, 0,
                        stream1>>>(d_rgbDepth, rgbSize, minDepth, maxDepth);
#ifdef PRINT_RUNTIME
    cudaEventRecord(stop);
    gpuErrCheck(cudaDeviceSynchronize());
    cudaEventElapsedTime(&runtime, start, stop);
    printf("Runtime of registration: %f ms\n", runtime);
#endif
  } else {
    correctDepthRange<<<(size + 8 * WARP_SIZE - 1) / (8 * WARP_SIZE), 8 * WARP_SIZE, 0, stream1>>>(
        d_depth, size, minDepth, maxDepth);
  }

  gpuErrCheck(cudaDeviceSynchronize());
}

} // namespace simsense