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
#include "simsense/lrcheck.h"

namespace simsense {

__global__
void lrConsistencyCheck(float *d_leftDisp, const uint16_t *d_rightDisp, const int rows, const int cols, const int lrMaxDiff) {
    const int pos = blockIdx.x * blockDim.x + threadIdx.x;
    const int x = pos % cols;
    const int y = pos / cols;
    if (y >= rows) { return; }

    int ld = (int)round(d_leftDisp[pos]);
    if (ld < 0 || x-ld < 0 || abs(ld - d_rightDisp[y*cols + x-ld]) > lrMaxDiff) {
        d_leftDisp[pos] = -1;
    }
}

}