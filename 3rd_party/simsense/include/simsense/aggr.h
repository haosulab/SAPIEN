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
#ifndef SIMSENSE_AGGR_H
#define SIMSENSE_AGGR_H

#include <stdint.h>
#include "config.h"

namespace simsense {

__global__
void aggrLeft2Right(cost_t *d_cost, cost_t *d_L, const int p1, const int p2, const int rows, const int cols, const int maxDisp);

__global__
void aggrRight2Left(cost_t *d_cost, cost_t *d_L, const int p1, const int p2, const int rows, const int cols, const int maxDisp);

__global__
void aggrTop2Bottom(cost_t *d_cost, cost_t *d_L, const int p1, const int p2, const int rows, const int cols, const int maxDisp);

// Compute total aggregation cost of four paths
__global__
void aggrBottom2Top(cost_t *d_cost, cost_t *d_LAll, cost_t *d_L0, cost_t *d_L1, cost_t *d_L2,
                    const int p1, const int p2, const int rows, const int cols, const int maxDisp);
}

#endif