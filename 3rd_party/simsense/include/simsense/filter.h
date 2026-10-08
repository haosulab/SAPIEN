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
#ifndef SIMSENSE_FILTER_H
#define SIMSENSE_FILTER_H

#include <stdint.h>
#include "config.h"

namespace simsense {

__global__
void boxFilterHorizontal(const cost_t *in, cost_t *out, const int rows, const int cols, const int maxDisp, const int size);

__global__
void boxFilterVertical(const cost_t *in, cost_t *out, const int rows, const int cols, const int maxDisp, const int size);

__global__
void medianFilter(const float *in, float *out, const int rows, const int cols, const int size);

}

#endif