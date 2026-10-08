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
#ifndef SIMSENSE_CAMERA_H
#define SIMSENSE_CAMERA_H

#include "config.h"
#include <cmath>
#include <curand.h>
#include <curand_kernel.h>
#include <stdint.h>

#define ERROR_MARGIN 0.01
#define BORDER_VALUE 0

namespace simsense {

__global__ void initInfraredNoise(curandState_t *states, int seed, const int size);

__global__ void simInfraredNoise(uint8_t *src, uint8_t *dst, curandState_t *states, const int rows,
                                 const int cols, float speckleShape, float speckleScale,
                                 float gaussianMu, float gaussianSigma);

__global__ void remap(const float *mapx, const float *mapy, const uint8_t *src, uint8_t *dst,
                      const int rows, const int cols);

__global__ void copySubArea(uint8_t *src, uint8_t *dest, int srcWidth, int bboxWidth,
                            int bboxHeight, int bboxStartX, int bboxStartY);

__global__ void pasteSubArea(float *src, float *dest, int destWidth, int bboxWidth, int bboxHeight,
                             int destStartX, int destStartY);

__global__ void disp2Depth(const float *disp, float *depth, const int size, const float focalLen,
                           const float baselineLen);

__global__ void initRgbDepth(float *rgbDepth, const int size, const float maxDepth);

__global__ void depthRegistration(float *rgbDepth, const float *depth, const float *a1,
                                  const float *a2, const float *a3, float b1, float b2, float b3,
                                  const int size, const int rgbRows, const int rgbCols);

__global__ void depthDilation(float *depth, const int rgbRows, const int rgbCols,
                              const float maxDepth);

__global__ void correctDepthRange(float *depth, const int size, const float minDepth,
                                  const float maxDepth);

__global__ void depth2PointCloud(const float *depth, float *pc, const int rows, const int cols,
                                 const float fx, const float fy, const float s, const float cx,
                                 const float cy);

__global__ void depth2RgbPointCloud(const float *depth, const float *rgba, float *pc,
                                    const int rows, const int cols, const float fx, const float fy,
                                    const float s, const float cx, const float cy);

} // namespace simsense

#endif