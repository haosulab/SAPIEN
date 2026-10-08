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
#include "simsense/cost.h"

namespace simsense {

__global__
void hammingCost(const uint32_t *censusL, const uint32_t *censusR, cost_t *d_cost, const int rows, const int cols, const int maxDisp) {
    const int x = blockIdx.x * maxDisp;
    const int y = blockIdx.y;
    const int thrId = threadIdx.x;

    extern __shared__ uint32_t mem[];
    uint32_t *left = mem; // Capacity is maxDisp
    uint32_t *right = mem + maxDisp; // Capacity is 2*maxDisp

    if (x+thrId < cols) {
        left[thrId] = censusL[y*cols + x+thrId];
        right[thrId+maxDisp] = censusR[y*cols + x+thrId];
    }
    right[thrId] = (x == 0) ? censusR[y*cols] : censusR[y*cols + x+thrId-maxDisp];
    __syncthreads();

    int imax = maxDisp;
    if (cols % maxDisp != 0 && blockIdx.x == gridDim.x-1) {
        imax = cols % maxDisp;
    }
    for (int i = 0; i < imax; i++) {
        const int base = left[i];
        const int match = right[i+maxDisp-thrId];
        d_cost[(y*cols + x+i)*maxDisp + thrId] = __popc(base^match);
    }
}

}