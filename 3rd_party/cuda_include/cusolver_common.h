/*
 * Copyright 2014 NVIDIA Corporation.  All rights reserved.
 *
 * NOTICE TO LICENSEE:
 *
 * This source code and/or documentation ("Licensed Deliverables") are
 * subject to NVIDIA intellectual property rights under U.S. and
 * international Copyright laws.
 *
 * These Licensed Deliverables contained herein is PROPRIETARY and
 * CONFIDENTIAL to NVIDIA and is being provided under the terms and
 * conditions of a form of NVIDIA software license agreement by and
 * between NVIDIA and Licensee ("License Agreement") or electronically
 * accepted by Licensee.  Notwithstanding any terms or conditions to
 * the contrary in the License Agreement, reproduction or disclosure
 * of the Licensed Deliverables to any third party without the express
 * written consent of NVIDIA is prohibited.
 *
 * NOTWITHSTANDING ANY TERMS OR CONDITIONS TO THE CONTRARY IN THE
 * LICENSE AGREEMENT, NVIDIA MAKES NO REPRESENTATION ABOUT THE
 * SUITABILITY OF THESE LICENSED DELIVERABLES FOR ANY PURPOSE.  IT IS
 * PROVIDED "AS IS" WITHOUT EXPRESS OR IMPLIED WARRANTY OF ANY KIND.
 * NVIDIA DISCLAIMS ALL WARRANTIES WITH REGARD TO THESE LICENSED
 * DELIVERABLES, INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY,
 * NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE.
 * NOTWITHSTANDING ANY TERMS OR CONDITIONS TO THE CONTRARY IN THE
 * LICENSE AGREEMENT, IN NO EVENT SHALL NVIDIA BE LIABLE FOR ANY
 * SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
 * WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS
 * ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THESE LICENSED DELIVERABLES.
 *
 * U.S. Government End Users.  These Licensed Deliverables are a
 * "commercial item" as that term is defined at 48 C.F.R. 2.101 (OCT
 * 1995), consisting of "commercial computer software" and "commercial
 * computer software documentation" as such terms are used in 48
 * C.F.R. 12.212 (SEPT 1995) and is provided to the U.S. Government
 * only as a commercial end item.  Consistent with 48 C.F.R.12.212 and
 * 48 C.F.R. 227.7202-1 through 227.7202-4 (JUNE 1995), all
 * U.S. Government End Users acquire the Licensed Deliverables with
 * only those rights set forth herein.
 *
 * Any use of the Licensed Deliverables in individual and commercial
 * software must include, in the user documentation and internal
 * comments to the code, the above Disclaimer and U.S. Government End
 * Users Notice.
 */

#if !defined(CUSOLVER_COMMON_H_)
#define CUSOLVER_COMMON_H_

#include "library_types.h"

#ifndef CUSOLVERAPI
#ifdef _WIN32
#define CUSOLVERAPI __stdcall
#else
#define CUSOLVERAPI 
#endif
#endif


#if defined(_MSC_VER)
typedef __int64 int64_t;
#else
#include <inttypes.h>
#endif



#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

#define CUSOLVER_VER_MAJOR 10
#define CUSOLVER_VER_MINOR 2
#define CUSOLVER_VER_PATCH 0
#define CUSOLVER_VER_BUILD 243
#define CUSOLVER_VERSION (CUSOLVER_VER_MAJOR * 1000 + \
                        CUSOLVER_VER_MINOR *  100 + \
                        CUSOLVER_VER_PATCH)

typedef enum{
    CUSOLVER_STATUS_SUCCESS=0,
    CUSOLVER_STATUS_NOT_INITIALIZED=1,
    CUSOLVER_STATUS_ALLOC_FAILED=2,
    CUSOLVER_STATUS_INVALID_VALUE=3,
    CUSOLVER_STATUS_ARCH_MISMATCH=4,
    CUSOLVER_STATUS_MAPPING_ERROR=5,
    CUSOLVER_STATUS_EXECUTION_FAILED=6,
    CUSOLVER_STATUS_INTERNAL_ERROR=7,
    CUSOLVER_STATUS_MATRIX_TYPE_NOT_SUPPORTED=8,
    CUSOLVER_STATUS_NOT_SUPPORTED = 9,
    CUSOLVER_STATUS_ZERO_PIVOT=10,
    CUSOLVER_STATUS_INVALID_LICENSE=11
} cusolverStatus_t;

typedef enum {
    CUSOLVER_EIG_TYPE_1=1,
    CUSOLVER_EIG_TYPE_2=2,
    CUSOLVER_EIG_TYPE_3=3
} cusolverEigType_t ;

typedef enum {
    CUSOLVER_EIG_MODE_NOVECTOR=0,
    CUSOLVER_EIG_MODE_VECTOR=1
} cusolverEigMode_t ;


typedef enum {
    CUSOLVER_EIG_RANGE_ALL=1001,
    CUSOLVER_EIG_RANGE_I=1002,
    CUSOLVER_EIG_RANGE_V=1003,
} cusolverEigRange_t ;


cusolverStatus_t CUSOLVERAPI cusolverGetProperty(
    libraryPropertyType type, 
    int *value);

cusolverStatus_t CUSOLVERAPI cusolverGetVersion(
    int *version);


#if defined(__cplusplus)
}
#endif /* __cplusplus */


#endif // CUSOLVER_COMMON_H_



