/*
 * Copyright 1993-2018 NVIDIA Corporation.  All rights reserved.
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
#pragma once

#ifndef CUBLASAPI
#ifdef __CUDACC__
#define CUBLASAPI __host__ __device__
#else
#define CUBLASAPI
#endif
#endif

#include <cublas_api.h>

#include <stdint.h>
#include <stddef.h>

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/** Opaque structure holding CUBLASLT context
 */
typedef struct cublasLtContext *cublasLtHandle_t;

cublasStatus_t CUBLASWINAPI
cublasLtCreate(cublasLtHandle_t *lightHandle);

cublasStatus_t CUBLASWINAPI
cublasLtDestroy(cublasLtHandle_t lightHandle);

size_t CUBLASWINAPI
cublasLtGetVersion(void);

size_t CUBLASWINAPI
cublasLtGetCudartVersion(void);

cublasStatus_t CUBLASWINAPI
cublasLtGetProperty(libraryPropertyType type, int *value);

/** Opaque descriptor for matrix memory layout
 */
typedef struct cublasLtMatrixLayoutStruct *cublasLtMatrixLayout_t;

/** Semi-opaque algorithm descriptor (to avoid complicated alloc/free schemes)
 *
 * This structure can be trivially serialized and later restored for use with the same version of cuBLAS library to save
 * on selecting the right configuration again.
 */
typedef struct {
    uint64_t data[8];
} cublasLtMatmulAlgo_t;

/** Opaque descriptor for cublasLtMatmul() operation details
 */
typedef struct cublasLtMatmulDescStruct *cublasLtMatmulDesc_t;

/** Opaque descriptor for cublasLtMatrixTransform() operation details
 */
typedef struct cublasLtMatrixTransformDescStruct *cublasLtMatrixTransformDesc_t;

/** Opaque descriptor for cublasLtMatmulAlgoGetHeuristic() configuration
 */
typedef struct cublasLtMatmulPreferenceStruct *cublasLtMatmulPreference_t;

/** Tile size (in C/D matrix Rows x Cols)
 *
 * General order of tile IDs is sort by size first by first dimension next.
 */
typedef enum {
    CUBLASLT_MATMUL_TILE_UNDEFINED = 0,
    CUBLASLT_MATMUL_TILE_8x8       = 1,
    CUBLASLT_MATMUL_TILE_8x16      = 2,
    CUBLASLT_MATMUL_TILE_16x8      = 3,
    CUBLASLT_MATMUL_TILE_8x32      = 4,
    CUBLASLT_MATMUL_TILE_16x16     = 5,
    CUBLASLT_MATMUL_TILE_32x8      = 6,
    CUBLASLT_MATMUL_TILE_8x64      = 7,
    CUBLASLT_MATMUL_TILE_16x32     = 8,
    CUBLASLT_MATMUL_TILE_32x16     = 9,
    CUBLASLT_MATMUL_TILE_64x8      = 10,
    CUBLASLT_MATMUL_TILE_32x32     = 11,
    CUBLASLT_MATMUL_TILE_32x64     = 12,
    CUBLASLT_MATMUL_TILE_64x32     = 13,
    CUBLASLT_MATMUL_TILE_32x128    = 14,
    CUBLASLT_MATMUL_TILE_64x64     = 15,
    CUBLASLT_MATMUL_TILE_128x32    = 16,
    CUBLASLT_MATMUL_TILE_64x128    = 17,
    CUBLASLT_MATMUL_TILE_128x64    = 18,
    CUBLASLT_MATMUL_TILE_64x256    = 19,
    CUBLASLT_MATMUL_TILE_128x128   = 20,
    CUBLASLT_MATMUL_TILE_256x64    = 21,
    CUBLASLT_MATMUL_TILE_64x512    = 22,
    CUBLASLT_MATMUL_TILE_128x256   = 23,
    CUBLASLT_MATMUL_TILE_256x128   = 24,
    CUBLASLT_MATMUL_TILE_512x64    = 25,
    CUBLASLT_MATMUL_TILE_END
} cublasLtMatmulTile_t;

/** Pointer mode to use for alpha/beta */
typedef enum {
    /** matches CUBLAS_POINTER_MODE_HOST, pointer targets a single value host memory */
    CUBLASLT_POINTER_MODE_HOST = CUBLAS_POINTER_MODE_HOST,
    /** matches CUBLAS_POINTER_MODE_DEVICE, pointer targets a single value device memory */
    CUBLASLT_POINTER_MODE_DEVICE = CUBLAS_POINTER_MODE_DEVICE,
    /** pointer targets an array in device memory */
    CUBLASLT_POINTER_MODE_DEVICE_VECTOR = 2,
    /** alpha pointer targets an array in device memory, beta is zero */
    CUBLASLT_POINTER_MODE_ALPHA_DEVICE_VECTOR_BETA_ZERO = 3,
} cublasLtPointerMode_t;

/** Mask to define and query pointer mode capability */
typedef enum {
    /** see CUBLASLT_POINTER_MODE_HOST */
    CUBLASLT_POINTER_MODE_MASK_HOST = 1,
    /** see CUBLASLT_POINTER_MODE_DEVICE */
    CUBLASLT_POINTER_MODE_MASK_DEVICE = 2,
    /** see CUBLASLT_POINTER_MODE_DEVICE_VECTOR */
    CUBLASLT_POINTER_MODE_MASK_DEVICE_VECTOR = 4,
    /** see CUBLASLT_POINTER_MODE_ALPHA_DEVICE_VECTOR_BETA_ZERO */
    CUBLASLT_POINTER_MODE_MASK_ALPHA_DEVICE_VECTOR_BETA_ZERO = 8,
} cublasLtPointerModeMask_t;

/** Execute matrix multiplication (D = alpha * op(A) * op(B) + beta * C).
 *
 * \retval     CUBLAS_STATUS_NOT_INITIALIZED   if cuBLASLt handle has not been initialized
 * \retval     CUBLAS_STATUS_INVALID_VALUE     if parameters are in conflict or in an impossible configuration; e.g.
 *                                             when workspaceSizeInBytes is less than workspace required by configured
 *                                             algo
 * \retval     CUBLAS_STATUS_NOT_SUPPORTED     if current implementation on selected device doesn't support configured
 *                                             operation
 * \retval     CUBLAS_STATUS_ARCH_MISMATCH     if configured operation cannot be run using selected device
 * \retval     CUBLAS_STATUS_EXECUTION_FAILED  if cuda reported execution error from the device
 * \retval     CUBLAS_STATUS_SUCCESS           if the operation completed successfully
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatmul(cublasLtHandle_t lightHandle,
               cublasLtMatmulDesc_t computeDesc,
               const void *alpha, /* host or device pointer */
               const void *A,
               cublasLtMatrixLayout_t Adesc,
               const void *B,
               cublasLtMatrixLayout_t Bdesc,
               const void *beta, /* host or device pointer */
               const void *C,
               cublasLtMatrixLayout_t Cdesc,
               void *D,
               cublasLtMatrixLayout_t Ddesc,
               const cublasLtMatmulAlgo_t *algo,
               void *workspace,
               size_t workspaceSizeInBytes,
               cudaStream_t stream);

/** Matrix layout conversion helper (C = alpha * op(A) + beta * op(B))
 *
 * Can be used to change memory order of data or to scale and shift the values.
 *
 * \retval     CUBLAS_STATUS_NOT_INITIALIZED   if cuBLASLt handle has not been initialized
 * \retval     CUBLAS_STATUS_INVALID_VALUE     if parameters are in conflict or in an impossible configuration; e.g.
 *                                             when A is not NULL, but Adesc is NULL
 * \retval     CUBLAS_STATUS_NOT_SUPPORTED     if current implementation on selected device doesn't support configured
 *                                             operation
 * \retval     CUBLAS_STATUS_ARCH_MISMATCH     if configured operation cannot be run using selected device
 * \retval     CUBLAS_STATUS_EXECUTION_FAILED  if cuda reported execution error from the device
 * \retval     CUBLAS_STATUS_SUCCESS           if the operation completed successfully
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatrixTransform(cublasLtHandle_t lightHandle,
                        cublasLtMatrixTransformDesc_t transformDesc,
                        const void *alpha, /* host or device pointer */
                        const void *A,
                        cublasLtMatrixLayout_t Adesc,
                        const void *beta, /* host or device pointer */
                        const void *B,
                        cublasLtMatrixLayout_t Bdesc,
                        void *C,
                        cublasLtMatrixLayout_t Cdesc,
                        cudaStream_t stream);

/* ---------------------------------------------------------------------------------------*/
/* Helper functions for cublasLtMatrixLayout_t */
/* ---------------------------------------------------------------------------------------*/

/** Enum for data ordering */
typedef enum {
    /** Column-major
     *
     * Leading dimension is the stride (in elements) to the beginning of next column in memory.
     */
    CUBLASLT_ORDER_COL = 0,
    /** Row major
     *
     * Leading dimension is the stride (in elements) to the beginning of next row in memory.
     */
    CUBLASLT_ORDER_ROW = 1,
    /** Column-major ordered tiles of 32 columns.
     *
     * Leading dimension is the stride (in elements) to the beginning of next group of 32-columns. E.g. if matrix has 33
     * columns and 2 rows, ld must be at least (32) * 2 = 64.
     */
    CUBLASLT_ORDER_COL32       = 2,
    /** Column-major ordered tiles of composite tiles with total 32 columns and 8 rows, tile composed of interleaved
     * inner tiles of 4 columns within 4 even or odd rows in an alternating pattern.
     *
     * Leading dimension is the stride (in elements) to the beginning of the first 32 column x 8 row tile for the next
     * 32-wide group of columns. E.g. if matrix has 33 columns and 1 row, ld must be at least (32 * 8) * 1 = 256.
     */
    CUBLASLT_ORDER_COL4_4R2_8C = 3, 
} cublasLtOrder_t;

/** Attributes of memory layout */
typedef enum {
    /** Data type, see cudaDataType.
     * 
     * uint32_t
     */
    CUBLASLT_MATRIX_LAYOUT_TYPE,

    /** Memory order of the data, see cublasLtOrder_t.
     * 
     * int32_t, default: CUBLASLT_ORDER_COL
     */
    CUBLASLT_MATRIX_LAYOUT_ORDER,

    /** Number of rows.
     * 
     * Usually only values that can be expressed as int32_t are supported.
     * 
     * uint64_t
     */
    CUBLASLT_MATRIX_LAYOUT_ROWS,                  

    /** Number of columns.
     * 
     * Usually only values that can be expressed as int32_t are supported.
     * 
     * uint64_t
     */
    CUBLASLT_MATRIX_LAYOUT_COLS,

    /** Matrix leading dimension.
     *
     * For CUBLASLT_ORDER_COL this is stride (in elements) of matrix column, for more details and documentation for
     * other memory orders see documentation for cublasLtOrder_t values.
     *
     * Currently only non-negative values are supported, must be large enough so that matrix memory locations are not
     * overlapping (e.g. greater or equal to CUBLASLT_MATRIX_LAYOUT_ROWS in case of CUBLASLT_ORDER_COL).
     *
     * int64_t;
     */
    CUBLASLT_MATRIX_LAYOUT_LD,

    /** Number of matmul operations to perform in the batch.
     * 
     * See also CUBLASLT_ALGO_CAP_STRIDED_BATCH_SUPPORT
     * 
     * int32_t, default: 1
     */
    CUBLASLT_MATRIX_LAYOUT_BATCH_COUNT,

    /** Stride (in elements) to the next matrix for strided batch operation.
     * 
     * int64_t, default: 0
     */
    CUBLASLT_MATRIX_LAYOUT_STRIDED_BATCH_OFFSET,

    /** Stride (in bytes) to the imaginary plane for planar complex layout.
     *
     * int64_t, default: 0 - 0 means that layout is regular (real and imaginary parts of complex numbers are interleaved
     * in memory in each element)
     */
    CUBLASLT_MATRIX_LAYOUT_PLANE_OFFSET,
} cublasLtMatrixLayoutAttribute_t;

/** Create new matrix layout descriptor.
 *
 * \retval     CUBLAS_STATUS_ALLOC_FAILED  if memory could not be allocated
 * \retval     CUBLAS_STATUS_SUCCESS       if desciptor was created successfully
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatrixLayoutCreate(  //
    cublasLtMatrixLayout_t *matLayout,
    cudaDataType type,
    uint64_t rows,
    uint64_t cols,
    int64_t ld);

/** Destroy matrix layout descriptor.
 *
 * \retval     CUBLAS_STATUS_SUCCESS  if operation was successful
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatrixLayoutDestroy(cublasLtMatrixLayout_t matLayout);

/** Set matrix layout descriptor attribute.
 *
 * \param[in]  matLayout    The descriptor
 * \param[in]  attr         The attribute
 * \param[in]  buf          memory address containing the new value
 * \param[in]  sizeInBytes  size of buf buffer for verification (in bytes)
 *
 * \retval     CUBLAS_STATUS_INVALID_VALUE  if buf is NULL or sizeInBytes doesn't match size of internal storage for
 *                                          selected attribute
 * \retval     CUBLAS_STATUS_SUCCESS        if attribute was set successfully
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatrixLayoutSetAttribute(  //
    cublasLtMatrixLayout_t matLayout,
    cublasLtMatrixLayoutAttribute_t attr,
    const void *buf,
    size_t sizeInBytes);

/** Get matrix layout descriptor attribute.
 *
 * \param[in]  matLayout    The descriptor
 * \param[in]  attr         The attribute
 * \param[out] buf          memory address containing the new value
 * \param[in]  sizeInBytes  size of buf buffer for verification (in bytes)
 * \param[out] sizeWritten  only valid when return value is CUBLAS_STATUS_SUCCESS. If sizeInBytes is non-zero: number of
 *                          bytes actually written, if sizeInBytes is 0: number of bytes needed to write full contents
 *
 * \retval     CUBLAS_STATUS_INVALID_VALUE  if sizeInBytes is 0 and sizeWritten is NULL, or if  sizeInBytes is non-zero
 *                                          and buf is NULL or sizeInBytes doesn't match size of internal storage for
 *                                          selected attribute
 * \retval     CUBLAS_STATUS_SUCCESS        if attribute's value was successfully written to user memory
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatrixLayoutGetAttribute(  //
    cublasLtMatrixLayout_t matLayout,
    cublasLtMatrixLayoutAttribute_t attr,
    void *buf,
    size_t sizeInBytes,
    size_t *sizeWritten);

/* ---------------------------------------------------------------------------------------*/
/* Helper functions for cublasLtMatmulDesc_t */
/* ---------------------------------------------------------------------------------------*/

/** Matmul descriptor attributes to define details of the operation. */
typedef enum {
    /** Compute type, see cudaDataType. Defines data type used for multiply and accumulate operations and the
     * accumulator during matrix multiplication.
     *
     * int32_t
     */
    CUBLASLT_MATMUL_DESC_COMPUTE_TYPE,

    /** Scale type, see cudaDataType. Defines data type of alpha and beta. Accumulator and value from matrix C are
     * typically converted to scale type before final scaling. Value is then converted from scale type to type of matrix
     * D before being stored in memory.
     *
     * int32_t, default: same as CUBLASLT_MATMUL_DESC_COMPUTE_TYPE
     */
    CUBLASLT_MATMUL_DESC_SCALE_TYPE,

    /** Pointer mode of alpha and beta, see cublasLtPointerMode_t. When CUBLASLT_POINTER_MODE_DEVICE_VECTOR is in use,
     * alpha/beta vector lenghts must match number of output matrix rows.
     *
     * int32_t, default: CUBLASLT_POINTER_MODE_HOST
     */
    CUBLASLT_MATMUL_DESC_POINTER_MODE,

    /** Transform of matrix A, see cublasOperation_t.
     *
     * int32_t, default: CUBLAS_OP_N
     */
    CUBLASLT_MATMUL_DESC_TRANSA,

    /** Transform of matrix B, see cublasOperation_t.
     *
     * int32_t, default: CUBLAS_OP_N
     */
    CUBLASLT_MATMUL_DESC_TRANSB,

    /** Transform of matrix C, see cublasOperation_t.
     *
     * Must be CUBLAS_OP_N if performing matrix multiplication in place (when C == D).
     *
     * int32_t, default: CUBLAS_OP_N
     */
    CUBLASLT_MATMUL_DESC_TRANSC,

    /** Matrix fill mode, see cublasFillMode_t.
     *
     * int32_t, default: CUBLAS_FILL_MODE_FULL
     */
    CUBLASLT_MATMUL_DESC_FILL_MODE,

    /** Epilogue function, see cublasLtEpilogue_t.
     *
     * uint32_t, default: CUBLASLT_EPILOGUE_DEFAULT
     */
    CUBLASLT_MATMUL_DESC_EPILOGUE,

    /** Bias vector pointer in the device memory, see CUBLASLT_EPILOGUE_BIAS. Bias vector elements are the same type as
     * alpha, beta (see CUBLASLT_MATMUL_DESC_SCALE_TYPE). Bias vector length must match matrix D rows count.
     *
     * const void *, default: NULL
     */
    CUBLASLT_MATMUL_DESC_BIAS_POINTER,
} cublasLtMatmulDescAttributes_t;

/** Create new matmul operation descriptor.
 *
 * \retval     CUBLAS_STATUS_ALLOC_FAILED  if memory could not be allocated
 * \retval     CUBLAS_STATUS_SUCCESS       if desciptor was created successfully
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatmulDescCreate(cublasLtMatmulDesc_t *matmulDesc, cudaDataType computeType);

/** Destroy matmul operation descriptor.
 *
 * \retval     CUBLAS_STATUS_SUCCESS  if operation was successful
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatmulDescDestroy(cublasLtMatmulDesc_t matmulDesc);

/** Set matmul operation descriptor attribute.
 *
 * \param[in]  matmulDesc   The descriptor
 * \param[in]  attr         The attribute
 * \param[in]  buf          memory address containing the new value
 * \param[in]  sizeInBytes  size of buf buffer for verification (in bytes)
 *
 * \retval     CUBLAS_STATUS_INVALID_VALUE  if buf is NULL or sizeInBytes doesn't match size of internal storage for
 *                                          selected attribute
 * \retval     CUBLAS_STATUS_SUCCESS        if attribute was set successfully
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatmulDescSetAttribute(  //
    cublasLtMatmulDesc_t matmulDesc,
    cublasLtMatmulDescAttributes_t attr,
    const void *buf,
    size_t sizeInBytes);

/** Get matmul operation descriptor attribute.
 *
 * \param[in]  matmulDesc   The descriptor
 * \param[in]  attr         The attribute
 * \param[out] buf          memory address containing the new value
 * \param[in]  sizeInBytes  size of buf buffer for verification (in bytes)
 * \param[out] sizeWritten  only valid when return value is CUBLAS_STATUS_SUCCESS. If sizeInBytes is non-zero: number of
 *                          bytes actually written, if sizeInBytes is 0: number of bytes needed to write full contents
 *
 * \retval     CUBLAS_STATUS_INVALID_VALUE  if sizeInBytes is 0 and sizeWritten is NULL, or if  sizeInBytes is non-zero
 *                                          and buf is NULL or sizeInBytes doesn't match size of internal storage for
 *                                          selected attribute
 * \retval     CUBLAS_STATUS_SUCCESS        if attribute's value was successfully written to user memory
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatmulDescGetAttribute(  //
    cublasLtMatmulDesc_t matmulDesc,
    cublasLtMatmulDescAttributes_t attr,
    void *buf,
    size_t sizeInBytes,
    size_t *sizeWritten);

/* ---------------------------------------------------------------------------------------*/
/* Helper functions for cublasLtMatrixTransformDesc_t */
/* ---------------------------------------------------------------------------------------*/

/** Matrix transform descriptor attributes to define details of the operation.
 */
typedef enum {
    /** Scale type, see cudaDataType. Inputs are converted to scale type for scaling and summation and results are then
     * converted to output type to store in memory.
     * 
     * int32_t
     */
    CUBLASLT_MATRIX_TRANSFORM_DESC_SCALE_TYPE,

    /** Pointer mode of alpha and beta, see cublasLtPointerMode_t.
     * 
     * int32_t, default: CUBLASLT_POINTER_MODE_HOST
     */
    CUBLASLT_MATRIX_TRANSFORM_DESC_POINTER_MODE,

    /** Transform of matrix A, see cublasOperation_t.
     * 
     * int32_t, default: CUBLAS_OP_N
     */
    CUBLASLT_MATRIX_TRANSFORM_DESC_TRANSA,

    /** Transform of matrix B, see cublasOperation_t.
     * 
     * int32_t, default: CUBLAS_OP_N
     */
    CUBLASLT_MATRIX_TRANSFORM_DESC_TRANSB,
} cublasLtMatrixTransformDescAttributes_t;

/** Create new matrix transform operation descriptor.
 *
 * \retval     CUBLAS_STATUS_ALLOC_FAILED  if memory could not be allocated
 * \retval     CUBLAS_STATUS_SUCCESS       if desciptor was created successfully
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatrixTransformDescCreate(cublasLtMatrixTransformDesc_t *transformDesc, cudaDataType scaleType);

/** Destroy matrix transform operation descriptor.
 *
 * \retval     CUBLAS_STATUS_SUCCESS  if operation was successful
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatrixTransformDescDestroy(cublasLtMatrixTransformDesc_t transformDesc);

/** Set matrix transform operation descriptor attribute.
 *
 * \param[in]  transformDesc  The descriptor
 * \param[in]  attr           The attribute
 * \param[in]  buf            memory address containing the new value
 * \param[in]  sizeInBytes    size of buf buffer for verification (in bytes)
 *
 * \retval     CUBLAS_STATUS_INVALID_VALUE  if buf is NULL or sizeInBytes doesn't match size of internal storage for
 *                                          selected attribute
 * \retval     CUBLAS_STATUS_SUCCESS        if attribute was set successfully
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatrixTransformDescSetAttribute(  //
    cublasLtMatrixTransformDesc_t transformDesc,
    cublasLtMatrixTransformDescAttributes_t attr,
    const void *buf,
    size_t sizeInBytes);

/** Get matrix transform operation descriptor attribute.
 *
 * \param[in]  transformDesc  The descriptor
 * \param[in]  attr           The attribute
 * \param[out] buf            memory address containing the new value
 * \param[in]  sizeInBytes    size of buf buffer for verification (in bytes)
 * \param[out] sizeWritten    only valid when return value is CUBLAS_STATUS_SUCCESS. If sizeInBytes is non-zero: number of
 *                            bytes actually written, if sizeInBytes is 0: number of bytes needed to write full contents
 *
 * \retval     CUBLAS_STATUS_INVALID_VALUE  if sizeInBytes is 0 and sizeWritten is NULL, or if  sizeInBytes is non-zero
 *                                          and buf is NULL or sizeInBytes doesn't match size of internal storage for
 *                                          selected attribute
 * \retval     CUBLAS_STATUS_SUCCESS        if attribute's value was successfully written to user memory
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatrixTransformDescGetAttribute(  //
    cublasLtMatrixTransformDesc_t transformDesc,
    cublasLtMatrixTransformDescAttributes_t attr,
    void *buf,
    size_t sizeInBytes,
    size_t *sizeWritten);

/** For computation with complex numbers, this enum allows to apply the Gauss Complexity reduction algorithm
 */
typedef enum {
    CUBLASLT_3M_MODE_DISALLOWED = 0,
    CUBLASLT_3M_MODE_ALLOWED    = 1,
} cublasLt3mMode_t;

/** Reduction scheme for portions of the dot-product calculated in parallel (a. k. a. "split - K").
 */
typedef enum {
    /** No reduction scheme, dot-product shall be performed in one sequence.
     */
    CUBLASLT_REDUCTION_SCHEME_NONE = 0,

    /** Reduction is performed "in place" - using the output buffer (and output data type) and counters (in workspace) to guarantee the
     * sequentiality.
     */
    CUBLASLT_REDUCTION_SCHEME_INPLACE = 1,

    /** Intermediate results are stored in compute type in the workspace and reduced in a separate step.
     */
    CUBLASLT_REDUCTION_SCHEME_COMPUTE_TYPE = 2,  

    /** Intermediate results are stored in output type in the workspace and reduced in a separate step.
     */
    CUBLASLT_REDUCTION_SCHEME_OUTPUT_TYPE  = 4,  

    CUBLASLT_REDUCTION_SCHEME_MASK         = 0x7,
} cublasLtReductionScheme_t;

/** Postprocessing options for the epilogue
 */
typedef enum {
    /** No special postprocessing, just scale and quantize results if necessary.
     */
    CUBLASLT_EPILOGUE_DEFAULT = 1,

    /** ReLu, apply ReLu point-wise transform to the results (x:=max(x, 0))
     */
    CUBLASLT_EPILOGUE_RELU = 2,

    /** Bias, apply (broadcasted) Bias from bias vector. Bias vector length must match matrix D rows, it must be packed
     * (stride between vector elements is 1). Bias vector is broadcasted to all columns and added before applying final
     * postprocessing.
     */
    CUBLASLT_EPILOGUE_BIAS = 4,

    /** ReLu and Bias, apply Bias and then ReLu transform
     */
    CUBLASLT_EPILOGUE_RELU_BIAS = (CUBLASLT_EPILOGUE_RELU | CUBLASLT_EPILOGUE_BIAS),
} cublasLtEpilogue_t;

/** Matmul heuristic search mode
 */
typedef enum {
    /** ask heuristics for best algo for given usecase
     */
    CUBLASLT_SEARCH_BEST_FIT = 0,
    /** only try to find best config for preconfigured algo id
     */
    CUBLASLT_SEARCH_LIMITED_BY_ALGO_ID = 1,
} cublasLtMatmulSearch_t;

/** Algo search preference to fine tune the heuristic function. */
typedef enum {
    /** Search mode, see cublasLtMatmulSearch_t.
     * 
     * uint32_t, default: CUBLASLT_SEARCH_BEST_FIT
     */
    CUBLASLT_MATMUL_PREF_SEARCH_MODE,          

    /** Maximum allowed workspace size in bytes.
     * 
     * uint64_t, default: 0 - no workspace allowed
     */
    CUBLASLT_MATMUL_PREF_MAX_WORKSPACE_BYTES,  

    /** Math mode mask, see cublasMath_t.
     * 
     * Only algorithms with CUBLASLT_ALGO_CAP_MATHMODE_IMPL that is not masked out by this attribute are allowed.
     * 
     * uint32_t, default: 1 (allows both default and tensor op math)
     */
    CUBLASLT_MATMUL_PREF_MATH_MODE_MASK,       

    /** Reduction scheme mask, see cublasLtReductionScheme_t. Filters heuristic result to only include algo configs that use one of the required modes.
     * 
     * E.g. mask value of 0x03 will allow only INPLACE and COMPUTE_TYPE reduction schemes.
     * 
     * uint32_t, default: CUBLASLT_REDUCTION_SCHEME_MASK (allows all reduction schemes)
     */
    CUBLASLT_MATMUL_PREF_REDUCTION_SCHEME_MASK,

    /** Gaussian mode mask, see cublasLt3mMode_t.
     * 
     * Only algorithms with CUBLASLT_ALGO_CAP_GAUSSIAN_IMPL that is not masked out by this attribute are allowed.
     * 
     * uint32_t, default: CUBLASLT_3M_MODE_ALLOWED (allows both gaussian and non-gaussian algorithms)
     */
    CUBLASLT_MATMUL_PREF_GAUSSIAN_MODE_MASK,   

    /** Minimum buffer alignment for matrix A (in bytes).
     * 
     * Selecting a smaller value will exclude algorithms that can not work with matrix A that is not as strictly aligned as they need.
     * 
     * uint32_t, default: 256
     */
    CUBLASLT_MATMUL_PREF_MIN_ALIGNMENT_A_BYTES,

    /** Minimum buffer alignment for matrix B (in bytes).
     * 
     * Selecting a smaller value will exclude algorithms that can not work with matrix B that is not as strictly aligned as they need.
     * 
     * uint32_t, default: 256
     */
    CUBLASLT_MATMUL_PREF_MIN_ALIGNMENT_B_BYTES,

    /** Minimum buffer alignment for matrix C (in bytes).
     * 
     * Selecting a smaller value will exclude algorithms that can not work with matrix C that is not as strictly aligned as they need.
     * 
     * uint32_t, default: 256
     */
    CUBLASLT_MATMUL_PREF_MIN_ALIGNMENT_C_BYTES,

    /** Minimum buffer alignment for matrix D (in bytes).
     * 
     * Selecting a smaller value will exclude algorithms that can not work with matrix D that is not as strictly aligned as they need.
     * 
     * uint32_t, default: 256
     */
    CUBLASLT_MATMUL_PREF_MIN_ALIGNMENT_D_BYTES,

    /** Maximum wave count.
     * 
     * See cublasLtMatmulHeuristicResult_t::wavesCount.
     * 
     * Selecting a non-zero value will exclude algorithms that report device utilization higher than specified.
     * 
     * float, default: 0.0f
     */
    CUBLASLT_MATMUL_PREF_MAX_WAVES_COUNT,

    /** Pointer mode mask, see cublasLtPointerModeMask_t. Filters heuristic result to only include algorithms that support all required modes.
     *
     * uint32_t, default: (CUBLASLT_POINTER_MODE_MASK_HOST | CUBLASLT_POINTER_MODE_MASK_DEVICE) (only allows algorithms that support both regular host and device pointers)
     */
    CUBLASLT_MATMUL_PREF_POINTER_MODE_MASK,

    /** Epilogue selector mask, see cublasLtEpilogue_t. Filters heuristic result to only include algorithms that support all required operations.
     *
     * uint32_t, default: CUBLASLT_EPILOGUE_DEFAULT (only allows algorithms that support default epilogue)
     */
    CUBLASLT_MATMUL_PREF_EPILOGUE_MASK,

} cublasLtMatmulPreferenceAttributes_t;

/** Create new matmul heuristic search preference descriptor.
 *
 * \retval     CUBLAS_STATUS_ALLOC_FAILED  if memory could not be allocated
 * \retval     CUBLAS_STATUS_SUCCESS       if desciptor was created successfully
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatmulPreferenceCreate(cublasLtMatmulPreference_t *pref);

/** Destroy matmul heuristic search preference descriptor.
 *
 * \retval     CUBLAS_STATUS_SUCCESS  if operation was successful
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatmulPreferenceDestroy(cublasLtMatmulPreference_t pref);

/** Set matmul heuristic search preference descriptor attribute.
 *
 * \param[in]  pref         The descriptor
 * \param[in]  attr         The attribute
 * \param[in]  buf          memory address containing the new value
 * \param[in]  sizeInBytes  size of buf buffer for verification (in bytes)
 *
 * \retval     CUBLAS_STATUS_INVALID_VALUE  if buf is NULL or sizeInBytes doesn't match size of internal storage for
 *                                          selected attribute
 * \retval     CUBLAS_STATUS_SUCCESS        if attribute was set successfully
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatmulPreferenceSetAttribute(  //
    cublasLtMatmulPreference_t pref,
    cublasLtMatmulPreferenceAttributes_t attr,
    const void *buf,
    size_t sizeInBytes);

/** Get matmul heuristic search preference descriptor attribute.
 *
 * \param[in]  pref         The descriptor
 * \param[in]  attr         The attribute
 * \param[out] buf          memory address containing the new value
 * \param[in]  sizeInBytes  size of buf buffer for verification (in bytes)
 * \param[out] sizeWritten  only valid when return value is CUBLAS_STATUS_SUCCESS. If sizeInBytes is non-zero: number of
 *                          bytes actually written, if sizeInBytes is 0: number of bytes needed to write full contents
 *
 * \retval     CUBLAS_STATUS_INVALID_VALUE  if sizeInBytes is 0 and sizeWritten is NULL, or if  sizeInBytes is non-zero
 *                                          and buf is NULL or sizeInBytes doesn't match size of internal storage for
 *                                          selected attribute
 * \retval     CUBLAS_STATUS_SUCCESS        if attribute's value was successfully written to user memory
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatmulPreferenceGetAttribute(  //
    cublasLtMatmulPreference_t pref,
    cublasLtMatmulPreferenceAttributes_t attr,
    void *buf,
    size_t sizeInBytes,
    size_t *sizeWritten);

/** Results structure used by cublasLtMatmulGetAlgo.
 *
 * Holds returned configured algo descriptor and its runtime properties.
 */
typedef struct {
    /** Matmul algorithm descriptor.
     *
     * Must be initialized with cublasLtMatmulAlgoInit() if preferences' CUBLASLT_MATMUL_PERF_SEARCH_MODE is set to
     * CUBLASLT_SEARCH_LIMITED_BY_ALGO_ID
     */
    cublasLtMatmulAlgo_t algo;

    /** Actual size of workspace memory required.
     */
    size_t workspaceSize;

    /** Result status, other fields are only valid if after call to cublasLtMatmulAlgoGetHeuristic() this member is set to CUBLAS_STATUS_SUCCESS.
     */
    cublasStatus_t state;

    /** Waves count - a device utilization metric.
     *
     * wavesCount value of 1.0f suggests that when kernel is launched it will fully occupy the GPU.
     */
    float wavesCount;

    int reserved[4];
} cublasLtMatmulHeuristicResult_t;

/** Query cublasLt heuristic for algorithm appropriate for given use case.
 *
 * \param[in]  requestedAlgoCount     size of heuristicResultsArray (in elements) and requested maximum number of
 *                                    algorithms to return
 * \param[out] heuristicResultsArray  output algorithms and associated runtime characteristics, ordered in increasing
 *                                    estimated compute time
 * \param[out] returnAlgoCount        number of heuristicResultsArray elements written
 *
 * \retval     CUBLAS_STATUS_INVALID_VALUE  if requestedAlgoCount is less or equal to zero
 * \retval     CUBLAS_STATUS_NOT_SUPPORTED  if no heuristic function available for current configuration
 * \retval     CUBLAS_STATUS_SUCCESS        if query was successful, inspect heuristicResultsArray[0 to (returnAlgoCount -
 *                                          1)].state for detail status of results
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatmulAlgoGetHeuristic(
    cublasLtHandle_t lightHandle,
    cublasLtMatmulDesc_t operationDesc,
    cublasLtMatrixLayout_t Adesc,
    cublasLtMatrixLayout_t Bdesc,
    cublasLtMatrixLayout_t Cdesc,
    cublasLtMatrixLayout_t Ddesc,
    cublasLtMatmulPreference_t preference,
    int requestedAlgoCount,
    cublasLtMatmulHeuristicResult_t heuristicResultsArray[],
    int *returnAlgoCount);


/* ---------------------------------------------------------------------------------------*/
/* Lower level API to be able to implement own Heuristic and Find routines                */
/* ---------------------------------------------------------------------------------------*/

/** Routine to get all algo IDs that can potentially run
 *
 * \param[in]  int              requestedAlgoCount requested number of algos (must be less or equal to size of algoIdsA (in
 *                              elements))
 * \param[out] algoIdsA         array to write algoIds to
 * \param[out] returnAlgoCount  number of algoIds actually written
 *
 * \retval     CUBLAS_STATUS_INVALID_VALUE  if requestedAlgoCount is less or equal to zero
 * \retval     CUBLAS_STATUS_SUCCESS        if query was successful, inspect returnAlgoCount to get actual number of IDs
 *                                          available
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatmulAlgoGetIds(
    cublasLtHandle_t lightHandle,
    cudaDataType_t computeType,
    cudaDataType_t scaleType,
    cudaDataType_t Atype,
    cudaDataType_t Btype,
    cudaDataType_t Ctype,
    cudaDataType_t Dtype,
    int requestedAlgoCount,
    int algoIdsArray[],
    int *returnAlgoCount);

/** Initialize algo structure
 *
 * \retval     CUBLAS_STATUS_INVALID_VALUE  if algo is NULL or algoId is outside of recognized range
 * \retval     CUBLAS_STATUS_NOT_SUPPORTED  if algoId is not supported for given combination of data types
 * \retval     CUBLAS_STATUS_SUCCESS        if the structure was successfully initialized
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatmulAlgoInit ( cublasLtHandle_t lightHandle,
                         cudaDataType_t computeType,
                         cudaDataType_t scaleType,
                         cudaDataType_t Atype,
                         cudaDataType_t Btype,
                         cudaDataType_t Ctype,
                         cudaDataType_t Dtype,
                         int algoId,
                         cublasLtMatmulAlgo_t *algo);

/** Check configured algo descriptor for correctness and support on current device.
 *
 * Result includes required workspace size and calculated wave count.
 *
 * CUBLAS_STATUS_SUCCESS doesn't fully guarantee algo will run (will fail if e.g. buffers are not correctly aligned);
 * but if cublasLtMatmulAlgoCheck fails, the algo will not run.
 *
 * \param[in]  algo    algo configuration to check
 * \param[out] result  result structure to report algo runtime characteristics; algo field is never updated
 *
 * \retval     CUBLAS_STATUS_INVALID_VALUE  if matrix layout descriptors or operation descriptor don't match algo
 *                                          descriptor
 * \retval     CUBLAS_STATUS_NOT_SUPPORTED  if algo configuration or data type combination is not currently supported on
 *                                          given device
 * \retval     CUBLAS_STATUS_ARCH_MISMATCH  if algo configuration cannot be run using the selected device
 * \retval     CUBLAS_STATUS_SUCCESS        if check was successful
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatmulAlgoCheck(  //
    cublasLtHandle_t lightHandle,
    cublasLtMatmulDesc_t operationDesc,
    cublasLtMatrixLayout_t Adesc,
    cublasLtMatrixLayout_t Bdesc,
    cublasLtMatrixLayout_t Cdesc,
    cublasLtMatrixLayout_t Ddesc,
    const cublasLtMatmulAlgo_t *algo, ///< may point to result->algo
    cublasLtMatmulHeuristicResult_t *result);

/** Capabilities Attributes that can be retrieved from an initialized Algo structure
 */
typedef enum {  
    /** support for split K, see CUBLASLT_ALGO_CONFIG_SPLITK_NUM
     *
     * int32_t, 0 means no support, supported otherwise
     */
    CUBLASLT_ALGO_CAP_SPLITK_SUPPORT,
    /** reduction scheme mask, see cublasLtReductionScheme_t; shows supported reduction schemes, if reduction scheme is not masked out it is supported.
     *
     * e.g. int isReductionSchemeComputeTypeSupported ? (reductionSchemeMask & CUBLASLT_REDUCTION_SCHEME_COMPUTE_TYPE) == CUBLASLT_REDUCTION_SCHEME_COMPUTE_TYPE ? 1 : 0;
     *
     * uint32_t
     */
    CUBLASLT_ALGO_CAP_REDUCTION_SCHEME_MASK,
    /** support for cta swizzling, see CUBLASLT_ALGO_CONFIG_CTA_SWIZZLING
     *
     * uint32_t, 0 means no support, 1 means supported value of 1, other values are reserved
     */
    CUBLASLT_ALGO_CAP_CTA_SWIZZLING_SUPPORT,
    /** support strided batch
     *
     * int32_t, 0 means no support, supported otherwise
     */
    CUBLASLT_ALGO_CAP_STRIDED_BATCH_SUPPORT,
    /** support results out of place (D != C in D = alpha.A.B + beta.C)
     *
     * int32_t, 0 means no support, supported otherwise
     */
    CUBLASLT_ALGO_CAP_OUT_OF_PLACE_RESULT_SUPPORT,
    /** syrk/herk support (on top of regular gemm)
     *
     * int32_t, 0 means no support, supported otherwise
     */
    CUBLASLT_ALGO_CAP_UPLO_SUPPORT,
    /** tile ids possible to use, see cublasLtMatmulTile_t; if no tile ids are supported use CUBLASLT_MATMUL_TILE_UNDEFINED
     *
     * use cublasLtMatmulAlgoCapGetAttribute() with sizeInBytes=0 to query actual count
     *
     * array of uint32_t
     */
    CUBLASLT_ALGO_CAP_TILE_IDS,
    /** custom option range is from 0 to CUBLASLT_ALGO_CAP_CUSTOM_OPTION_MAX (inclusive), see CUBLASLT_ALGO_CONFIG_CUSTOM_OPTION
     *
     * int32_t
     */
    CUBLASLT_ALGO_CAP_CUSTOM_OPTION_MAX,
    /** whether algorithm is using regular compute or tensor operations
     *
     * int32_t 0 means regular compute, 1 means tensor operations;
     */
    CUBLASLT_ALGO_CAP_MATHMODE_IMPL,
    /** whether algorithm implements gaussian optimization of complex matrix multiplication, see cublasMath_t
     *
     * int32_t 0 means regular compute, 1 means gaussian;
     */
    CUBLASLT_ALGO_CAP_GAUSSIAN_IMPL,
    /** whether algorithm supports custom (not COL or ROW memory order), see cublasLtOrder_t
     *
     * int32_t 0 means only COL and ROW memory order is allowed, 1 means that algo might have different requirements;
     */
    CUBLASLT_ALGO_CAP_CUSTOM_MEMORY_ORDER,

    /** bitmask enumerating pointer modes algorithm supports
     *
     * uint32_t, see cublasLtPointerModeMask_t
     */
    CUBLASLT_ALGO_CAP_POINTER_MODE_MASK,

    /** bitmask enumerating kinds of postprocessing algorithm supports in the epilogue
     *
     * uint32_t, see cublasLtEpilogue_t
     */
    CUBLASLT_ALGO_CAP_EPILOGUE_MASK,
} cublasLtMatmulAlgoCapAttributes_t;

/** Get algo capability attribute.
 * 
 * E.g. to get list of supported Tile IDs:
 *      cublasLtMatmulTile_t tiles[CUBLASLT_MATMUL_TILE_END];
 *      size_t num_tiles, size_written;
 *      if (cublasLtMatmulAlgoCapGetAttribute(algo, CUBLASLT_ALGO_CAP_TILE_IDS, tiles, sizeof(tiles), size_written) == CUBLAS_STATUS_SUCCESS) {
 *        num_tiles = size_written / sizeof(tiles[0]);
 *      }
 *
 * \param[in]  algo         The algo descriptor
 * \param[in]  attr         The attribute
 * \param[out] buf          memory address containing the new value
 * \param[in]  sizeInBytes  size of buf buffer for verification (in bytes)
 * \param[out] sizeWritten  only valid when return value is CUBLAS_STATUS_SUCCESS. If sizeInBytes is non-zero: number of
 *                          bytes actually written, if sizeInBytes is 0: number of bytes needed to write full contents
 *
 * \retval     CUBLAS_STATUS_INVALID_VALUE  if sizeInBytes is 0 and sizeWritten is NULL, or if  sizeInBytes is non-zero
 *                                          and buf is NULL or sizeInBytes doesn't match size of internal storage for
 *                                          selected attribute
 * \retval     CUBLAS_STATUS_SUCCESS        if attribute's value was successfully written to user memory
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatmulAlgoCapGetAttribute(
    const cublasLtMatmulAlgo_t *algo,
    cublasLtMatmulAlgoCapAttributes_t attr,
    void *buf,
    size_t sizeInBytes,
    size_t *sizeWritten);

/** Algo Configuration Attributes that can be set according to the Algo capabilities
 */
typedef enum {
    /** algorithm index, see cublasLtMatmulAlgoGetIds()
     *
     * readonly, set by cublasLtMatmulAlgoInit()
     * int32_t
     */
    CUBLASLT_ALGO_CONFIG_ID,
    /** tile id, see cublasLtMatmulTile_t
     *
     * uint32_t, default: CUBLASLT_MATMUL_TILE_UNDEFINED
     */
    CUBLASLT_ALGO_CONFIG_TILE_ID,
    /** number of K splits, if != 1, SPLITK_NUM parts of matrix multiplication will be computed in parallel,
     * and then results accumulated according to REDUCTION_SCHEME
     *
     * uint32_t, default: 1
     */
    CUBLASLT_ALGO_CONFIG_SPLITK_NUM,
    /** reduction scheme, see cublasLtReductionScheme_t
     *
     * uint32_t, default: CUBLASLT_REDUCTION_SCHEME_NONE
     */
    CUBLASLT_ALGO_CONFIG_REDUCTION_SCHEME,
    /** cta swizzling, change mapping from CUDA grid coordinates to parts of the matrices
     *
     * possible values: 0, 1, other values reserved
     *
     * uint32_t, default: 0
     */
    CUBLASLT_ALGO_CONFIG_CTA_SWIZZLING,
    /** custom option, each algorithm can support some custom options that don't fit description of the other config
     * attributes, see CUBLASLT_ALGO_CAP_CUSTOM_OPTION_MAX to get accepted range for any specific case
     *
     * uint32_t, default: 0
     */
    CUBLASLT_ALGO_CONFIG_CUSTOM_OPTION,
} cublasLtMatmulAlgoConfigAttributes_t;

/** Set algo configuration attribute.
 *
 * \param[in]  algo         The algo descriptor
 * \param[in]  attr         The attribute
 * \param[in]  buf          memory address containing the new value
 * \param[in]  sizeInBytes  size of buf buffer for verification (in bytes)
 *
 * \retval     CUBLAS_STATUS_INVALID_VALUE  if buf is NULL or sizeInBytes doesn't match size of internal storage for
 *                                          selected attribute
 * \retval     CUBLAS_STATUS_SUCCESS        if attribute was set successfully
 */
cublasStatus_t CUBLASWINAPI
cublasLtMatmulAlgoConfigSetAttribute(
    cublasLtMatmulAlgo_t *algo,
    cublasLtMatmulAlgoConfigAttributes_t attr,
    const void *buf,
    size_t sizeInBytes);
    
/** Get algo configuration attribute.
 *
 * \param[in]  algo         The algo descriptor
 * \param[in]  attr         The attribute
 * \param[out] buf          memory address containing the new value
 * \param[in]  sizeInBytes  size of buf buffer for verification (in bytes)
 * \param[out] sizeWritten  only valid when return value is CUBLAS_STATUS_SUCCESS. If sizeInBytes is non-zero: number of
 *                          bytes actually written, if sizeInBytes is 0: number of bytes needed to write full contents
 *
 * \retval     CUBLAS_STATUS_INVALID_VALUE  if sizeInBytes is 0 and sizeWritten is NULL, or if  sizeInBytes is non-zero
 *                                          and buf is NULL or sizeInBytes doesn't match size of internal storage for
 *                                          selected attribute
 * \retval     CUBLAS_STATUS_SUCCESS        if attribute's value was successfully written to user memory
 */
cublasStatus_t CUBLASWINAPI 
cublasLtMatmulAlgoConfigGetAttribute(
        const cublasLtMatmulAlgo_t *algo,
        cublasLtMatmulAlgoConfigAttributes_t attr,
        void *buf,
        size_t sizeInBytes,
        size_t *sizeWritten);


#if defined(__cplusplus)
}
#endif /* __cplusplus */
