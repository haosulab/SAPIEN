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
 
 /*   cuSolverDN : Dense Linear Algebra Library

 */
 
#if !defined(CUSOLVERDN_H_)
#define CUSOLVERDN_H_

#include "driver_types.h"
#include "cuComplex.h"   /* import complex data type */
#include "cublas_v2.h"
#include "cusolver_common.h"


#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

struct cusolverDnContext;
typedef struct cusolverDnContext *cusolverDnHandle_t;

struct syevjInfo;
typedef struct syevjInfo *syevjInfo_t;

struct gesvdjInfo;
typedef struct gesvdjInfo *gesvdjInfo_t;


cusolverStatus_t CUSOLVERAPI cusolverDnCreate(cusolverDnHandle_t *handle);
cusolverStatus_t CUSOLVERAPI cusolverDnDestroy(cusolverDnHandle_t handle);
cusolverStatus_t CUSOLVERAPI cusolverDnSetStream (cusolverDnHandle_t handle, cudaStream_t streamId);
cusolverStatus_t CUSOLVERAPI cusolverDnGetStream(cusolverDnHandle_t handle, cudaStream_t *streamId);


/* Cholesky factorization and its solver */
cusolverStatus_t CUSOLVERAPI cusolverDnSpotrf_bufferSize( 
    cusolverDnHandle_t handle, 
    cublasFillMode_t uplo, 
    int n, 
    float *A, 
    int lda, 
    int *Lwork );

cusolverStatus_t CUSOLVERAPI cusolverDnDpotrf_bufferSize( 
    cusolverDnHandle_t handle, 
    cublasFillMode_t uplo, 
    int n, 
    double *A, 
    int lda, 
    int *Lwork );

cusolverStatus_t CUSOLVERAPI cusolverDnCpotrf_bufferSize( 
    cusolverDnHandle_t handle, 
    cublasFillMode_t uplo, 
    int n, 
    cuComplex *A, 
    int lda, 
    int *Lwork );

cusolverStatus_t CUSOLVERAPI cusolverDnZpotrf_bufferSize( 
    cusolverDnHandle_t handle, 
    cublasFillMode_t uplo, 
    int n, 
    cuDoubleComplex *A, 
    int lda, 
    int *Lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnSpotrf( 
    cusolverDnHandle_t handle, 
    cublasFillMode_t uplo, 
    int n, 
    float *A, 
    int lda,  
    float *Workspace, 
    int Lwork, 
    int *devInfo );

cusolverStatus_t CUSOLVERAPI cusolverDnDpotrf( 
    cusolverDnHandle_t handle, 
    cublasFillMode_t uplo, 
    int n, 
    double *A, 
    int lda, 
    double *Workspace, 
    int Lwork, 
    int *devInfo );



cusolverStatus_t CUSOLVERAPI cusolverDnCpotrf( 
    cusolverDnHandle_t handle, 
    cublasFillMode_t uplo, 
    int n, 
    cuComplex *A, 
    int lda, 
    cuComplex *Workspace, 
    int Lwork, 
    int *devInfo );

cusolverStatus_t CUSOLVERAPI cusolverDnZpotrf( 
    cusolverDnHandle_t handle, 
    cublasFillMode_t uplo, 
    int n, 
    cuDoubleComplex *A, 
    int lda, 
    cuDoubleComplex *Workspace, 
    int Lwork, 
    int *devInfo );


cusolverStatus_t CUSOLVERAPI cusolverDnSpotrs(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    int nrhs,
    const float *A,
    int lda,
    float *B,
    int ldb,
    int *devInfo);

cusolverStatus_t CUSOLVERAPI cusolverDnDpotrs(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    int nrhs,
    const double *A,
    int lda,
    double *B,
    int ldb,
    int *devInfo);

cusolverStatus_t CUSOLVERAPI cusolverDnCpotrs(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    int nrhs,
    const cuComplex *A,
    int lda,
    cuComplex *B,
    int ldb,
    int *devInfo);

cusolverStatus_t CUSOLVERAPI cusolverDnZpotrs(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    int nrhs,
    const cuDoubleComplex *A,
    int lda,
    cuDoubleComplex *B,
    int ldb,
    int *devInfo);

/* batched Cholesky factorization and its solver */
cusolverStatus_t CUSOLVERAPI cusolverDnSpotrfBatched(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    float *Aarray[],
    int lda,
    int *infoArray,
    int batchSize);

cusolverStatus_t CUSOLVERAPI cusolverDnDpotrfBatched(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    double *Aarray[],
    int lda,
    int *infoArray,
    int batchSize);

cusolverStatus_t CUSOLVERAPI cusolverDnCpotrfBatched(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    cuComplex *Aarray[],
    int lda,
    int *infoArray,
    int batchSize);

cusolverStatus_t CUSOLVERAPI cusolverDnZpotrfBatched(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    cuDoubleComplex *Aarray[],
    int lda,
    int *infoArray,
    int batchSize);

cusolverStatus_t CUSOLVERAPI cusolverDnSpotrsBatched(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    int nrhs, /* only support rhs = 1*/
    float *A[],
    int lda,
    float *B[],
    int ldb,
    int *d_info,
    int batchSize);

cusolverStatus_t CUSOLVERAPI cusolverDnDpotrsBatched(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    int nrhs, /* only support rhs = 1*/
    double *A[],
    int lda,
    double *B[],
    int ldb,
    int *d_info,
    int batchSize);

cusolverStatus_t CUSOLVERAPI cusolverDnCpotrsBatched(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    int nrhs, /* only support rhs = 1*/
    cuComplex *A[],
    int lda,
    cuComplex *B[],
    int ldb,
    int *d_info,
    int batchSize);

cusolverStatus_t CUSOLVERAPI cusolverDnZpotrsBatched(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    int nrhs, /* only support rhs = 1*/
    cuDoubleComplex *A[],
    int lda,
    cuDoubleComplex *B[],
    int ldb,
    int *d_info,
    int batchSize);

/* s.p.d. matrix inversion (POTRI) and auxiliary routines (TRTRI and LAUUM)  */
cusolverStatus_t CUSOLVERAPI cusolverDnSpotri_bufferSize(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    float *A,
    int lda,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnDpotri_bufferSize(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    double *A,
    int lda,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnCpotri_bufferSize(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    cuComplex *A,
    int lda,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnZpotri_bufferSize(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    cuDoubleComplex *A,
    int lda,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnSpotri(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    float *A,
    int lda,
    float *work,
    int lwork,
    int *devInfo);

cusolverStatus_t CUSOLVERAPI cusolverDnDpotri(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    double *A,
    int lda,
    double *work,
    int lwork,
    int *devInfo);

cusolverStatus_t CUSOLVERAPI cusolverDnCpotri(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    cuComplex *A,
    int lda,
    cuComplex *work,
    int lwork,
    int *devInfo);

cusolverStatus_t CUSOLVERAPI cusolverDnZpotri(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    cuDoubleComplex *A,
    int lda,
    cuDoubleComplex *work,
    int lwork,
    int *devInfo);

cusolverStatus_t CUSOLVERAPI cusolverDnStrtri_bufferSize(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    cublasDiagType_t diag,
    int n,
    float *A,
    int lda,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnDtrtri_bufferSize(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    cublasDiagType_t diag,
    int n,
    double *A,
    int lda,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnCtrtri_bufferSize(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    cublasDiagType_t diag,
    int n,
    cuComplex *A,
    int lda,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnZtrtri_bufferSize(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    cublasDiagType_t diag,
    int n,
    cuDoubleComplex *A,
    int lda,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnStrtri(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    cublasDiagType_t diag,
    int n,
    float *A,
    int lda,
    float *work,
    int lwork,
    int *devInfo);

cusolverStatus_t CUSOLVERAPI cusolverDnDtrtri(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    cublasDiagType_t diag,
    int n,
    double *A,
    int lda,
    double *work,
    int lwork,
    int *devInfo);

cusolverStatus_t CUSOLVERAPI cusolverDnCtrtri(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    cublasDiagType_t diag,
    int n,
    cuComplex *A,
    int lda,
    cuComplex *work,
    int lwork,
    int *devInfo);

cusolverStatus_t CUSOLVERAPI cusolverDnZtrtri(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    cublasDiagType_t diag,
    int n,
    cuDoubleComplex *A,
    int lda,
    cuDoubleComplex *work,
    int lwork,
    int *devInfo);

/* lauum, auxiliar routine for s.p.d matrix inversion */
cusolverStatus_t CUSOLVERAPI cusolverDnSlauum_bufferSize(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    float *A,
    int lda,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnDlauum_bufferSize(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    double *A,
    int lda,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnClauum_bufferSize(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    cuComplex *A,
    int lda,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnZlauum_bufferSize(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    cuDoubleComplex *A,
    int lda,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnSlauum(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    float *A,
    int lda,
    float *work,
    int lwork,
    int *devInfo);

cusolverStatus_t CUSOLVERAPI cusolverDnDlauum(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    double *A,
    int lda,
    double *work,
    int lwork,
    int *devInfo);

cusolverStatus_t CUSOLVERAPI cusolverDnClauum(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    cuComplex *A,
    int lda,
    cuComplex *work,
    int lwork,
    int *devInfo);

cusolverStatus_t CUSOLVERAPI cusolverDnZlauum(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    cuDoubleComplex *A,
    int lda,
    cuDoubleComplex *work,
    int lwork,
    int *devInfo);



/* LU Factorization */
cusolverStatus_t CUSOLVERAPI cusolverDnSgetrf_bufferSize(
    cusolverDnHandle_t handle,
    int m,
    int n,
    float *A,
    int lda,
    int *Lwork );

cusolverStatus_t CUSOLVERAPI cusolverDnDgetrf_bufferSize(
    cusolverDnHandle_t handle,
    int m,
    int n,
    double *A,
    int lda,
    int *Lwork );

cusolverStatus_t CUSOLVERAPI cusolverDnCgetrf_bufferSize(
    cusolverDnHandle_t handle,
    int m,
    int n,
    cuComplex *A,
    int lda,
    int *Lwork );

cusolverStatus_t CUSOLVERAPI cusolverDnZgetrf_bufferSize(
    cusolverDnHandle_t handle,
    int m,
    int n,
    cuDoubleComplex *A,
    int lda,
    int *Lwork );


cusolverStatus_t CUSOLVERAPI cusolverDnSgetrf( 
    cusolverDnHandle_t handle, 
    int m, 
    int n, 
    float *A, 
    int lda, 
    float *Workspace, 
    int *devIpiv, 
    int *devInfo );

cusolverStatus_t CUSOLVERAPI cusolverDnDgetrf( 
    cusolverDnHandle_t handle, 
    int m, 
    int n, 
    double *A, 
    int lda, 
    double *Workspace, 
    int *devIpiv, 
    int *devInfo );

cusolverStatus_t CUSOLVERAPI cusolverDnCgetrf( 
    cusolverDnHandle_t handle, 
    int m, 
    int n, 
    cuComplex *A, 
    int lda, 
    cuComplex *Workspace, 
    int *devIpiv, 
    int *devInfo );

cusolverStatus_t CUSOLVERAPI cusolverDnZgetrf( 
    cusolverDnHandle_t handle, 
    int m, 
    int n, 
    cuDoubleComplex *A, 
    int lda, 
    cuDoubleComplex *Workspace, 
    int *devIpiv, 
    int *devInfo );

/* Row pivoting */
cusolverStatus_t CUSOLVERAPI cusolverDnSlaswp( 
    cusolverDnHandle_t handle, 
    int n, 
    float *A, 
    int lda, 
    int k1, 
    int k2, 
    const int *devIpiv, 
    int incx);

cusolverStatus_t CUSOLVERAPI cusolverDnDlaswp( 
    cusolverDnHandle_t handle, 
    int n, 
    double *A, 
    int lda, 
    int k1, 
    int k2, 
    const int *devIpiv, 
    int incx);

cusolverStatus_t CUSOLVERAPI cusolverDnClaswp( 
    cusolverDnHandle_t handle, 
    int n, 
    cuComplex *A, 
    int lda, 
    int k1, 
    int k2, 
    const int *devIpiv, 
    int incx);

cusolverStatus_t CUSOLVERAPI cusolverDnZlaswp( 
    cusolverDnHandle_t handle, 
    int n, 
    cuDoubleComplex *A, 
    int lda, 
    int k1, 
    int k2, 
    const int *devIpiv, 
    int incx);

/* LU solve */
cusolverStatus_t CUSOLVERAPI cusolverDnSgetrs( 
    cusolverDnHandle_t handle, 
    cublasOperation_t trans, 
    int n, 
    int nrhs, 
    const float *A, 
    int lda, 
    const int *devIpiv, 
    float *B, 
    int ldb, 
    int *devInfo );

cusolverStatus_t CUSOLVERAPI cusolverDnDgetrs( 
    cusolverDnHandle_t handle, 
    cublasOperation_t trans, 
    int n, 
    int nrhs, 
    const double *A, 
    int lda, 
    const int *devIpiv, 
    double *B, 
    int ldb, 
    int *devInfo );

cusolverStatus_t CUSOLVERAPI cusolverDnCgetrs( 
    cusolverDnHandle_t handle, 
    cublasOperation_t trans, 
    int n, 
    int nrhs, 
    const cuComplex *A, 
    int lda, 
    const int *devIpiv, 
    cuComplex *B, 
    int ldb, 
    int *devInfo );

cusolverStatus_t CUSOLVERAPI cusolverDnZgetrs( 
    cusolverDnHandle_t handle, 
    cublasOperation_t trans, 
    int n, 
    int nrhs, 
    const cuDoubleComplex *A, 
    int lda, 
    const int *devIpiv, 
    cuDoubleComplex *B, 
    int ldb, 
    int *devInfo );


/* QR factorization */
cusolverStatus_t CUSOLVERAPI cusolverDnSgeqrf_bufferSize(
    cusolverDnHandle_t handle,
    int m,
    int n,
    float *A,
    int lda,
    int *lwork );

cusolverStatus_t CUSOLVERAPI cusolverDnDgeqrf_bufferSize(
    cusolverDnHandle_t handle,
    int m,
    int n,
    double *A,
    int lda,
    int *lwork );

cusolverStatus_t CUSOLVERAPI cusolverDnCgeqrf_bufferSize(
    cusolverDnHandle_t handle,
    int m,
    int n,
    cuComplex *A,
    int lda,
    int *lwork );

cusolverStatus_t CUSOLVERAPI cusolverDnZgeqrf_bufferSize(
    cusolverDnHandle_t handle,
    int m,
    int n,
    cuDoubleComplex *A,
    int lda,
    int *lwork );

cusolverStatus_t CUSOLVERAPI cusolverDnSgeqrf( 
    cusolverDnHandle_t handle, 
    int m, 
    int n, 
    float *A,  
    int lda, 
    float *TAU,  
    float *Workspace,  
    int Lwork, 
    int *devInfo );

cusolverStatus_t CUSOLVERAPI cusolverDnDgeqrf( 
    cusolverDnHandle_t handle, 
    int m, 
    int n, 
    double *A, 
    int lda, 
    double *TAU, 
    double *Workspace, 
    int Lwork, 
    int *devInfo );

cusolverStatus_t CUSOLVERAPI cusolverDnCgeqrf( 
    cusolverDnHandle_t handle, 
    int m, 
    int n, 
    cuComplex *A, 
    int lda, 
    cuComplex *TAU, 
    cuComplex *Workspace, 
    int Lwork, 
    int *devInfo );

cusolverStatus_t CUSOLVERAPI cusolverDnZgeqrf( 
    cusolverDnHandle_t handle, 
    int m, 
    int n, 
    cuDoubleComplex *A, 
    int lda, 
    cuDoubleComplex *TAU, 
    cuDoubleComplex *Workspace, 
    int Lwork, 
    int *devInfo );


/* generate unitary matrix Q from QR factorization */
cusolverStatus_t CUSOLVERAPI cusolverDnSorgqr_bufferSize(
    cusolverDnHandle_t handle,
    int m,
    int n,
    int k,
    const float *A,
    int lda,
    const float *tau,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnDorgqr_bufferSize(
    cusolverDnHandle_t handle,
    int m,
    int n,
    int k,
    const double *A,
    int lda,
    const double *tau,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnCungqr_bufferSize(
    cusolverDnHandle_t handle,
    int m,
    int n,
    int k,
    const cuComplex *A,
    int lda,
    const cuComplex *tau,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnZungqr_bufferSize(
    cusolverDnHandle_t handle,
    int m,
    int n,
    int k,
    const cuDoubleComplex *A,
    int lda,
    const cuDoubleComplex *tau,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnSorgqr(
    cusolverDnHandle_t handle,
    int m,
    int n,
    int k,
    float *A,
    int lda,
    const float *tau,
    float *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnDorgqr(
    cusolverDnHandle_t handle,
    int m,
    int n,
    int k,
    double *A,
    int lda,
    const double *tau,
    double *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnCungqr(
    cusolverDnHandle_t handle,
    int m,
    int n,
    int k,
    cuComplex *A,
    int lda,
    const cuComplex *tau,
    cuComplex *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnZungqr(
    cusolverDnHandle_t handle,
    int m,
    int n,
    int k,
    cuDoubleComplex *A,
    int lda,
    const cuDoubleComplex *tau,
    cuDoubleComplex *work,
    int lwork,
    int *info);



/* compute Q**T*b in solve min||A*x = b|| */
cusolverStatus_t CUSOLVERAPI cusolverDnSormqr_bufferSize(
    cusolverDnHandle_t handle,
    cublasSideMode_t side,
    cublasOperation_t trans,
    int m,
    int n,
    int k,
    const float *A,
    int lda,
    const float *tau,
    const float *C,
    int ldc,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnDormqr_bufferSize(
    cusolverDnHandle_t handle,
    cublasSideMode_t side,
    cublasOperation_t trans,
    int m,
    int n,
    int k,
    const double *A,
    int lda,
    const double *tau,
    const double *C,
    int ldc,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnCunmqr_bufferSize(
    cusolverDnHandle_t handle,
    cublasSideMode_t side,
    cublasOperation_t trans,
    int m,
    int n,
    int k,
    const cuComplex *A,
    int lda,
    const cuComplex *tau,
    const cuComplex *C,
    int ldc,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnZunmqr_bufferSize(
    cusolverDnHandle_t handle,
    cublasSideMode_t side,
    cublasOperation_t trans,
    int m,
    int n,
    int k,
    const cuDoubleComplex *A,
    int lda,
    const cuDoubleComplex *tau,
    const cuDoubleComplex *C,
    int ldc,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnSormqr(
    cusolverDnHandle_t handle,
    cublasSideMode_t side,
    cublasOperation_t trans,
    int m,
    int n,
    int k,
    const float *A,
    int lda,
    const float *tau,
    float *C,
    int ldc,
    float *work,
    int lwork,
    int *devInfo);

cusolverStatus_t CUSOLVERAPI cusolverDnDormqr(
    cusolverDnHandle_t handle,
    cublasSideMode_t side,
    cublasOperation_t trans,
    int m,
    int n,
    int k,
    const double *A,
    int lda,
    const double *tau,
    double *C,
    int ldc,
    double *work,
    int lwork,
    int *devInfo);

cusolverStatus_t CUSOLVERAPI cusolverDnCunmqr(
    cusolverDnHandle_t handle,
    cublasSideMode_t side,
    cublasOperation_t trans,
    int m,
    int n,
    int k,
    const cuComplex *A,
    int lda,
    const cuComplex *tau,
    cuComplex *C,
    int ldc,
    cuComplex *work,
    int lwork,
    int *devInfo);

cusolverStatus_t CUSOLVERAPI cusolverDnZunmqr(
    cusolverDnHandle_t handle,
    cublasSideMode_t side,
    cublasOperation_t trans,
    int m,
    int n,
    int k,
    const cuDoubleComplex *A,
    int lda,
    const cuDoubleComplex *tau,
    cuDoubleComplex *C,
    int ldc,
    cuDoubleComplex *work,
    int lwork,
    int *devInfo);


/* L*D*L**T,U*D*U**T factorization */
cusolverStatus_t CUSOLVERAPI cusolverDnSsytrf_bufferSize(
    cusolverDnHandle_t handle,
    int n,
    float *A,
    int lda,
    int *lwork );

cusolverStatus_t CUSOLVERAPI cusolverDnDsytrf_bufferSize(
    cusolverDnHandle_t handle,
    int n,
    double *A,
    int lda,
    int *lwork );

cusolverStatus_t CUSOLVERAPI cusolverDnCsytrf_bufferSize(
    cusolverDnHandle_t handle,
    int n,
    cuComplex *A,
    int lda,
    int *lwork );

cusolverStatus_t CUSOLVERAPI cusolverDnZsytrf_bufferSize(
    cusolverDnHandle_t handle,
    int n,
    cuDoubleComplex *A,
    int lda,
    int *lwork );

cusolverStatus_t CUSOLVERAPI cusolverDnSsytrf(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    float *A,
    int lda,
    int *ipiv,
    float *work,
    int lwork,
    int *info );

cusolverStatus_t CUSOLVERAPI cusolverDnDsytrf(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    double *A,
    int lda,
    int *ipiv,
    double *work,
    int lwork,
    int *info );

cusolverStatus_t CUSOLVERAPI cusolverDnCsytrf(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    cuComplex *A,
    int lda,
    int *ipiv,
    cuComplex *work,
    int lwork,
    int *info );

cusolverStatus_t CUSOLVERAPI cusolverDnZsytrf(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    cuDoubleComplex *A,
    int lda,
    int *ipiv,
    cuDoubleComplex *work,
    int lwork,
    int *info );

/* Symmetric indefinite solve (SYTRS) */
cusolverStatus_t CUSOLVERAPI cusolverDnSsytrs_bufferSize(
        cusolverDnHandle_t handle,
        cublasFillMode_t uplo,
        int n,
        int nrhs,
        const float *A,
        int lda,
        const int *ipiv,
        float *B,
        int ldb,
        int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnDsytrs_bufferSize(
        cusolverDnHandle_t handle,
        cublasFillMode_t uplo,
        int n,
        int nrhs,
        const double *A,
        int lda,
        const int *ipiv,
        double *B,
        int ldb,
        int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnCsytrs_bufferSize(
        cusolverDnHandle_t handle,
        cublasFillMode_t uplo,
        int n,
        int nrhs,
        const cuComplex *A,
        int lda,
        const int *ipiv,
        cuComplex *B,
        int ldb,
        int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnZsytrs_bufferSize(
        cusolverDnHandle_t handle,
        cublasFillMode_t uplo,
        int n,
        int nrhs,
        const cuDoubleComplex *A,
        int lda,
        const int *ipiv,
        cuDoubleComplex *B,
        int ldb,
        int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnSsytrs(
        cusolverDnHandle_t handle,
        cublasFillMode_t uplo,
        int n,
        int nrhs,
        const float *A,
        int lda,
        const int *ipiv,
        float *B,
        int ldb,
        float *work,
        int lwork,
        int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnDsytrs(
        cusolverDnHandle_t handle,
        cublasFillMode_t uplo,
        int n,
        int nrhs,
        const double *A,
        int lda,
        const int *ipiv,
        double *B,
        int ldb,
        double *work,
        int lwork,
        int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnCsytrs(
        cusolverDnHandle_t handle,
        cublasFillMode_t uplo,
        int n,
        int nrhs,
        const cuComplex *A,
        int lda,
        const int *ipiv,
        cuComplex *B,
        int ldb,
        cuComplex *work,
        int lwork,
        int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnZsytrs(
        cusolverDnHandle_t handle,
        cublasFillMode_t uplo,
        int n,
        int nrhs,
        const cuDoubleComplex *A,
        int lda,
        const int *ipiv,
        cuDoubleComplex *B,
        int ldb,
        cuDoubleComplex *work,
        int lwork,
        int *info);

/* Symmetric indefinite inversion (sytri) */
cusolverStatus_t CUSOLVERAPI cusolverDnSsytri_bufferSize(
        cusolverDnHandle_t handle,
        cublasFillMode_t uplo,
        int n,
        float *A,
        int lda,
        const int *ipiv,
        int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnDsytri_bufferSize(
        cusolverDnHandle_t handle,
        cublasFillMode_t uplo,
        int n,
        double *A,
        int lda,
        const int *ipiv,
        int *lwork);
;
cusolverStatus_t CUSOLVERAPI cusolverDnCsytri_bufferSize(
        cusolverDnHandle_t handle,
        cublasFillMode_t uplo,
        int n,
        cuComplex *A,
        int lda,
        const int *ipiv,
        int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnZsytri_bufferSize(
        cusolverDnHandle_t handle,
        cublasFillMode_t uplo,
        int n,
        cuDoubleComplex *A,
        int lda,
        const int *ipiv,
        int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnSsytri(
        cusolverDnHandle_t handle,
        cublasFillMode_t uplo,
        int n,
        float *A,
        int lda,
        const int *ipiv,
        float *work,
        int lwork,
        int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnDsytri(
        cusolverDnHandle_t handle,
        cublasFillMode_t uplo,
        int n,
        double *A,
        int lda,
        const int *ipiv,
        double *work,
        int lwork,
        int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnCsytri(
        cusolverDnHandle_t handle,
        cublasFillMode_t uplo,
        int n,
        cuComplex *A,
        int lda,
        const int *ipiv,
        cuComplex *work,
        int lwork,
        int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnZsytri(
        cusolverDnHandle_t handle,
        cublasFillMode_t uplo,
        int n,
        cuDoubleComplex *A,
        int lda,
        const int *ipiv,
        cuDoubleComplex *work,
        int lwork,
        int *info);


/* bidiagonal factorization */
cusolverStatus_t CUSOLVERAPI cusolverDnSgebrd_bufferSize(
    cusolverDnHandle_t handle,
    int m,
    int n,
    int *Lwork );

cusolverStatus_t CUSOLVERAPI cusolverDnDgebrd_bufferSize(
    cusolverDnHandle_t handle,
    int m,
    int n,
    int *Lwork );

cusolverStatus_t CUSOLVERAPI cusolverDnCgebrd_bufferSize(
    cusolverDnHandle_t handle,
    int m,
    int n,
    int *Lwork );

cusolverStatus_t CUSOLVERAPI cusolverDnZgebrd_bufferSize(
    cusolverDnHandle_t handle,
    int m,
    int n,
    int *Lwork );

cusolverStatus_t CUSOLVERAPI cusolverDnSgebrd( 
    cusolverDnHandle_t handle, 
    int m, 
    int n, 
    float *A,  
    int lda,
    float *D, 
    float *E, 
    float *TAUQ,  
    float *TAUP, 
    float *Work,
    int Lwork, 
    int *devInfo );

cusolverStatus_t CUSOLVERAPI cusolverDnDgebrd( 
    cusolverDnHandle_t handle, 
    int m, 
    int n, 
    double *A, 
    int lda,
    double *D, 
    double *E, 
    double *TAUQ, 
    double *TAUP, 
    double *Work,
    int Lwork, 
    int *devInfo );

cusolverStatus_t CUSOLVERAPI cusolverDnCgebrd( 
    cusolverDnHandle_t handle, 
    int m, 
    int n, 
    cuComplex *A, 
    int lda, 
    float *D, 
    float *E, 
    cuComplex *TAUQ, 
    cuComplex *TAUP,
    cuComplex *Work, 
    int Lwork, 
    int *devInfo );

cusolverStatus_t CUSOLVERAPI cusolverDnZgebrd( 
    cusolverDnHandle_t handle, 
    int m, 
    int n, 
    cuDoubleComplex *A,
    int lda, 
    double *D, 
    double *E, 
    cuDoubleComplex *TAUQ,
    cuDoubleComplex *TAUP, 
    cuDoubleComplex *Work, 
    int Lwork, 
    int *devInfo );

/* generates one of the unitary matrices Q or P**T determined by GEBRD*/
cusolverStatus_t CUSOLVERAPI cusolverDnSorgbr_bufferSize(
    cusolverDnHandle_t handle,
    cublasSideMode_t side, 
    int m,
    int n,
    int k,
    const float *A,
    int lda,
    const float *tau,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnDorgbr_bufferSize(
    cusolverDnHandle_t handle,
    cublasSideMode_t side, 
    int m,
    int n,
    int k,
    const double *A,
    int lda,
    const double *tau,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnCungbr_bufferSize(
    cusolverDnHandle_t handle,
    cublasSideMode_t side, 
    int m,
    int n,
    int k,
    const cuComplex *A,
    int lda,
    const cuComplex *tau,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnZungbr_bufferSize(
    cusolverDnHandle_t handle,
    cublasSideMode_t side, 
    int m,
    int n,
    int k,
    const cuDoubleComplex *A,
    int lda,
    const cuDoubleComplex *tau,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnSorgbr(
    cusolverDnHandle_t handle,
    cublasSideMode_t side, 
    int m,
    int n,
    int k,
    float *A,
    int lda,
    const float *tau,
    float *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnDorgbr(
    cusolverDnHandle_t handle,
    cublasSideMode_t side, 
    int m,
    int n,
    int k,
    double *A,
    int lda,
    const double *tau,
    double *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnCungbr(
    cusolverDnHandle_t handle,
    cublasSideMode_t side, 
    int m,
    int n,
    int k,
    cuComplex *A,
    int lda,
    const cuComplex *tau,
    cuComplex *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnZungbr(
    cusolverDnHandle_t handle,
    cublasSideMode_t side, 
    int m,
    int n,
    int k,
    cuDoubleComplex *A,
    int lda,
    const cuDoubleComplex *tau,
    cuDoubleComplex *work,
    int lwork,
    int *info);


/* tridiagonal factorization */
cusolverStatus_t CUSOLVERAPI cusolverDnSsytrd_bufferSize(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    const float *A,
    int lda,
    const float *d,
    const float *e,
    const float *tau,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnDsytrd_bufferSize(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    const double *A,
    int lda,
    const double *d,
    const double *e,
    const double *tau,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnChetrd_bufferSize(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    const cuComplex *A,
    int lda,
    const float *d,
    const float *e,
    const cuComplex *tau,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnZhetrd_bufferSize(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    const cuDoubleComplex *A,
    int lda,
    const double *d,
    const double *e,
    const cuDoubleComplex *tau,
    int *lwork);


cusolverStatus_t CUSOLVERAPI cusolverDnSsytrd(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    float *A,
    int lda,
    float *d,
    float *e,
    float *tau,
    float *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnDsytrd(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    double *A,
    int lda,
    double *d,
    double *e,
    double *tau,
    double *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnChetrd(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    cuComplex *A,
    int lda,
    float *d,
    float *e,
    cuComplex *tau,
    cuComplex *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnZhetrd(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    cuDoubleComplex *A,
    int lda,
    double *d,
    double *e,
    cuDoubleComplex *tau,
    cuDoubleComplex *work,
    int lwork,
    int *info);



/* generate unitary Q comes from sytrd */
cusolverStatus_t CUSOLVERAPI cusolverDnSorgtr_bufferSize(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo, 
    int n,
    const float *A,
    int lda,
    const float *tau,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnDorgtr_bufferSize(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo,
    int n,
    const double *A,
    int lda,
    const double *tau,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnCungtr_bufferSize(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo, 
    int n,
    const cuComplex *A,
    int lda,
    const cuComplex *tau,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnZungtr_bufferSize(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo, 
    int n,
    const cuDoubleComplex *A,
    int lda,
    const cuDoubleComplex *tau,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnSorgtr(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo, 
    int n,
    float *A,
    int lda,
    const float *tau,
    float *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnDorgtr(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo, 
    int n,
    double *A,
    int lda,
    const double *tau,
    double *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnCungtr(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo, 
    int n,
    cuComplex *A,
    int lda,
    const cuComplex *tau,
    cuComplex *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnZungtr(
    cusolverDnHandle_t handle,
    cublasFillMode_t uplo, 
    int n,
    cuDoubleComplex *A,
    int lda,
    const cuDoubleComplex *tau,
    cuDoubleComplex *work,
    int lwork,
    int *info);



/* compute op(Q)*C or C*op(Q) where Q comes from sytrd */
cusolverStatus_t CUSOLVERAPI cusolverDnSormtr_bufferSize(
    cusolverDnHandle_t handle,
    cublasSideMode_t side,
    cublasFillMode_t uplo,
    cublasOperation_t trans,
    int m,
    int n,
    const float *A,
    int lda,
    const float *tau,
    const float *C,
    int ldc,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnDormtr_bufferSize(
    cusolverDnHandle_t handle,
    cublasSideMode_t side,
    cublasFillMode_t uplo,
    cublasOperation_t trans,
    int m,
    int n,
    const double *A,
    int lda,
    const double *tau,
    const double *C,
    int ldc,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnCunmtr_bufferSize(
    cusolverDnHandle_t handle,
    cublasSideMode_t side,
    cublasFillMode_t uplo,
    cublasOperation_t trans,
    int m,
    int n,
    const cuComplex *A,
    int lda,
    const cuComplex *tau,
    const cuComplex *C,
    int ldc,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnZunmtr_bufferSize(
    cusolverDnHandle_t handle,
    cublasSideMode_t side,
    cublasFillMode_t uplo,
    cublasOperation_t trans,
    int m,
    int n,
    const cuDoubleComplex *A,
    int lda,
    const cuDoubleComplex *tau,
    const cuDoubleComplex *C,
    int ldc,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnSormtr(
    cusolverDnHandle_t handle,
    cublasSideMode_t side,
    cublasFillMode_t uplo,
    cublasOperation_t trans,
    int m,
    int n,
    float *A,
    int lda,
    float *tau,
    float *C,
    int ldc,
    float *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnDormtr(
    cusolverDnHandle_t handle,
    cublasSideMode_t side,
    cublasFillMode_t uplo,
    cublasOperation_t trans,
    int m,
    int n,
    double *A,
    int lda,
    double *tau,
    double *C,
    int ldc,
    double *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnCunmtr(
    cusolverDnHandle_t handle,
    cublasSideMode_t side,
    cublasFillMode_t uplo,
    cublasOperation_t trans,
    int m,
    int n,
    cuComplex *A,
    int lda,
    cuComplex *tau,
    cuComplex *C,
    int ldc,
    cuComplex *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnZunmtr(
    cusolverDnHandle_t handle,
    cublasSideMode_t side,
    cublasFillMode_t uplo,
    cublasOperation_t trans,
    int m,
    int n,
    cuDoubleComplex *A,
    int lda,
    cuDoubleComplex *tau,
    cuDoubleComplex *C,
    int ldc,
    cuDoubleComplex *work,
    int lwork,
    int *info);



/* singular value decomposition, A = U * Sigma * V^H */
cusolverStatus_t CUSOLVERAPI cusolverDnSgesvd_bufferSize(
    cusolverDnHandle_t handle,
    int m,
    int n,
    int *lwork );

cusolverStatus_t CUSOLVERAPI cusolverDnDgesvd_bufferSize(
    cusolverDnHandle_t handle,
    int m,
    int n,
    int *lwork );

cusolverStatus_t CUSOLVERAPI cusolverDnCgesvd_bufferSize(
    cusolverDnHandle_t handle,
    int m,
    int n,
    int *lwork );

cusolverStatus_t CUSOLVERAPI cusolverDnZgesvd_bufferSize(
    cusolverDnHandle_t handle,
    int m,
    int n,
    int *lwork );

cusolverStatus_t CUSOLVERAPI cusolverDnSgesvd (
    cusolverDnHandle_t handle, 
    signed char jobu, 
    signed char jobvt, 
    int m, 
    int n, 
    float *A, 
    int lda, 
    float *S, 
    float *U, 
    int ldu, 
    float *VT, 
    int ldvt, 
    float *work, 
    int lwork, 
    float *rwork, 
    int  *info );

cusolverStatus_t CUSOLVERAPI cusolverDnDgesvd (
    cusolverDnHandle_t handle, 
    signed char jobu, 
    signed char jobvt, 
    int m, 
    int n, 
    double *A, 
    int lda, 
    double *S, 
    double *U, 
    int ldu, 
    double *VT, 
    int ldvt, 
    double *work,
    int lwork, 
    double *rwork, 
    int *info );

cusolverStatus_t CUSOLVERAPI cusolverDnCgesvd (
    cusolverDnHandle_t handle, 
    signed char jobu, 
    signed char jobvt, 
    int m, 
    int n, 
    cuComplex *A,
    int lda, 
    float *S, 
    cuComplex *U, 
    int ldu, 
    cuComplex *VT, 
    int ldvt,
    cuComplex *work, 
    int lwork, 
    float *rwork, 
    int *info );

cusolverStatus_t CUSOLVERAPI cusolverDnZgesvd (
    cusolverDnHandle_t handle, 
    signed char jobu, 
    signed char jobvt, 
    int m, 
    int n, 
    cuDoubleComplex *A, 
    int lda, 
    double *S, 
    cuDoubleComplex *U, 
    int ldu, 
    cuDoubleComplex *VT, 
    int ldvt, 
    cuDoubleComplex *work, 
    int lwork, 
    double *rwork, 
    int *info );


/* standard symmetric eigenvalue solver, A*x = lambda*x, by divide-and-conquer  */
cusolverStatus_t CUSOLVERAPI cusolverDnSsyevd_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cublasFillMode_t uplo, 
    int n,
    const float *A,
    int lda,
    const float *W,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnDsyevd_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cublasFillMode_t uplo, 
    int n,
    const double *A,
    int lda,
    const double *W,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnCheevd_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cublasFillMode_t uplo, 
    int n,
    const cuComplex *A,
    int lda,
    const float *W,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnZheevd_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cublasFillMode_t uplo, 
    int n,
    const cuDoubleComplex *A,
    int lda,
    const double *W,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnSsyevd(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cublasFillMode_t uplo, 
    int n,
    float *A,
    int lda,
    float *W, 
    float *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnDsyevd(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cublasFillMode_t uplo, 
    int n,
    double *A,
    int lda,
    double *W, 
    double *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnCheevd(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cublasFillMode_t uplo, 
    int n,
    cuComplex *A,
    int lda,
    float *W, 
    cuComplex *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnZheevd(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cublasFillMode_t uplo, 
    int n,
    cuDoubleComplex *A,
    int lda,
    double *W, 
    cuDoubleComplex *work,
    int lwork,
    int *info);

/* standard selective symmetric eigenvalue solver, A*x = lambda*x, by divide-and-conquer  */
cusolverStatus_t CUSOLVERAPI cusolverDnSsyevdx_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cusolverEigRange_t range,
    cublasFillMode_t uplo, 
    int n,
    const float *A,
    int lda,
    float vl,
    float vu,
    int il,
    int iu,
    int *meig,
    const float *W,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnDsyevdx_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cusolverEigRange_t range,
    cublasFillMode_t uplo, 
    int n,
    const double *A,
    int lda,
    double vl,
    double vu,
    int il,
    int iu,
    int *meig,
    const double *W,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnCheevdx_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cusolverEigRange_t range,
    cublasFillMode_t uplo, 
    int n,
    const cuComplex *A,
    int lda,
    float vl,
    float vu,
    int il,
    int iu,
    int *meig,
    const float *W,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnZheevdx_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cusolverEigRange_t range,
    cublasFillMode_t uplo, 
    int n,
    const cuDoubleComplex *A,
    int lda,
    double vl,
    double vu,
    int il,
    int iu,
    int *meig,
    const double *W,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnSsyevdx(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cusolverEigRange_t range,
    cublasFillMode_t uplo, 
    int n,
    float *A,
    int lda,
    float vl,
    float vu,
    int il,
    int iu,
    int *meig,
    float *W, 
    float *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnDsyevdx(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cusolverEigRange_t range,
    cublasFillMode_t uplo, 
    int n,
    double *A,
    int lda,
    double vl,
    double vu,
    int il,
    int iu,
    int *meig,
    double *W, 
    double *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnCheevdx(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cusolverEigRange_t range,
    cublasFillMode_t uplo, 
    int n,
    cuComplex *A,
    int lda,
    float vl,
    float vu,
    int il,
    int iu,
    int *meig,
    float *W, 
    cuComplex *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnZheevdx(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cusolverEigRange_t range,
    cublasFillMode_t uplo, 
    int n,
    cuDoubleComplex *A,
    int lda,
    double vl,
    double vu,
    int il,
    int iu,
    int *meig,
    double *W, 
    cuDoubleComplex *work,
    int lwork,
    int *info);

/* selective generalized symmetric eigenvalue solver, A*x = lambda*B*x, by divide-and-conquer  */
cusolverStatus_t CUSOLVERAPI cusolverDnSsygvdx_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype,
    cusolverEigMode_t jobz,
    cusolverEigRange_t range,
    cublasFillMode_t uplo, 
    int n,
    const float *A, 
    int lda,
    const float *B, 
    int ldb,
    float vl,
    float vu,
    int il,
    int iu,
    int *meig,
    const float *W,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnDsygvdx_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype, 
    cusolverEigMode_t jobz,  
    cusolverEigRange_t range,
    cublasFillMode_t uplo,  
    int n,
    const double *A, 
    int lda,
    const double *B, 
    int ldb,
    double vl,
    double vu,
    int il,
    int iu,
    int *meig,
    const double *W,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnChegvdx_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype, 
    cusolverEigMode_t jobz,  
    cusolverEigRange_t range,
    cublasFillMode_t uplo,  
    int n,
    const cuComplex *A, 
    int lda,
    const cuComplex *B, 
    int ldb,
    float vl,
    float vu,
    int il,
    int iu,
    int *meig,
    const float *W,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnZhegvdx_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype,   
    cusolverEigMode_t jobz, 
    cusolverEigRange_t range,
    cublasFillMode_t uplo,  
    int n,
    const cuDoubleComplex *A,
    int lda,
    const cuDoubleComplex *B, 
    int ldb,
    double vl,
    double vu,
    int il,
    int iu,
    int *meig,
    const double *W,
    int *lwork);


cusolverStatus_t CUSOLVERAPI cusolverDnSsygvdx(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype,   
    cusolverEigMode_t jobz,  
    cusolverEigRange_t range,
    cublasFillMode_t uplo,  
    int n,
    float *A, 
    int lda,
    float *B, 
    int ldb,
    float vl,
    float vu,
    int il,
    int iu,
    int *meig,
    float *W, 
    float *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnDsygvdx(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype,  
    cusolverEigMode_t jobz,  
    cusolverEigRange_t range,
    cublasFillMode_t uplo,  
    int n,
    double *A, 
    int lda,
    double *B, 
    int ldb,
    double vl,
    double vu,
    int il,
    int iu,
    int *meig,
    double *W, 
    double *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnChegvdx(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype,   
    cusolverEigMode_t jobz,  
    cusolverEigRange_t range,
    cublasFillMode_t uplo,  
    int n,
    cuComplex *A,
    int lda,
    cuComplex *B, 
    int ldb,
    float vl,
    float vu,
    int il,
    int iu,
    int *meig,
    float *W, 
    cuComplex *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnZhegvdx(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype,   
    cusolverEigMode_t jobz,  
    cusolverEigRange_t range,
    cublasFillMode_t uplo,  
    int n,
    cuDoubleComplex *A, 
    int lda,
    cuDoubleComplex *B, 
    int ldb,
    double vl,
    double vu,
    int il,
    int iu,
    int *meig,
    double *W, 
    cuDoubleComplex *work,
    int lwork,
    int *info);


/* generalized symmetric eigenvalue solver, A*x = lambda*B*x, by divide-and-conquer  */
cusolverStatus_t CUSOLVERAPI cusolverDnSsygvd_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype,
    cusolverEigMode_t jobz,
    cublasFillMode_t uplo, 
    int n,
    const float *A, 
    int lda,
    const float *B, 
    int ldb,
    const float *W,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnDsygvd_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype, 
    cusolverEigMode_t jobz,  
    cublasFillMode_t uplo,  
    int n,
    const double *A, 
    int lda,
    const double *B, 
    int ldb,
    const double *W,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnChegvd_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype, 
    cusolverEigMode_t jobz,  
    cublasFillMode_t uplo,  
    int n,
    const cuComplex *A, 
    int lda,
    const cuComplex *B, 
    int ldb,
    const float *W,
    int *lwork);

cusolverStatus_t CUSOLVERAPI cusolverDnZhegvd_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype,   
    cusolverEigMode_t jobz, 
    cublasFillMode_t uplo,  
    int n,
    const cuDoubleComplex *A,
    int lda,
    const cuDoubleComplex *B, 
    int ldb,
    const double *W,
    int *lwork);


cusolverStatus_t CUSOLVERAPI cusolverDnSsygvd(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype,   
    cusolverEigMode_t jobz,  
    cublasFillMode_t uplo,  
    int n,
    float *A, 
    int lda,
    float *B, 
    int ldb,
    float *W, 
    float *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnDsygvd(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype,  
    cusolverEigMode_t jobz,  
    cublasFillMode_t uplo,  
    int n,
    double *A, 
    int lda,
    double *B, 
    int ldb,
    double *W, 
    double *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnChegvd(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype,   
    cusolverEigMode_t jobz,  
    cublasFillMode_t uplo,  
    int n,
    cuComplex *A,
    int lda,
    cuComplex *B, 
    int ldb,
    float *W, 
    cuComplex *work,
    int lwork,
    int *info);

cusolverStatus_t CUSOLVERAPI cusolverDnZhegvd(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype,   
    cusolverEigMode_t jobz,  
    cublasFillMode_t uplo,  
    int n,
    cuDoubleComplex *A, 
    int lda,
    cuDoubleComplex *B, 
    int ldb,
    double *W, 
    cuDoubleComplex *work,
    int lwork,
    int *info);


cusolverStatus_t CUSOLVERAPI cusolverDnCreateSyevjInfo(
    syevjInfo_t *info);

cusolverStatus_t CUSOLVERAPI cusolverDnDestroySyevjInfo(
    syevjInfo_t info);

cusolverStatus_t CUSOLVERAPI cusolverDnXsyevjSetTolerance(
    syevjInfo_t info,
    double tolerance);

cusolverStatus_t CUSOLVERAPI cusolverDnXsyevjSetMaxSweeps(
    syevjInfo_t info,
    int max_sweeps);

cusolverStatus_t CUSOLVERAPI cusolverDnXsyevjSetSortEig(
    syevjInfo_t info,
    int sort_eig);

cusolverStatus_t CUSOLVERAPI cusolverDnXsyevjGetResidual(
    cusolverDnHandle_t handle,
    syevjInfo_t info,
    double *residual);

cusolverStatus_t CUSOLVERAPI cusolverDnXsyevjGetSweeps(
    cusolverDnHandle_t handle,
    syevjInfo_t info,
    int *executed_sweeps);


cusolverStatus_t CUSOLVERAPI cusolverDnSsyevjBatched_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cublasFillMode_t uplo, 
    int n,
    const float *A,
    int lda,
    const float *W,
    int *lwork,
    syevjInfo_t params,
    int batchSize
    );

cusolverStatus_t CUSOLVERAPI cusolverDnDsyevjBatched_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cublasFillMode_t uplo, 
    int n,
    const double *A, 
    int lda,
    const double *W,
    int *lwork,
    syevjInfo_t params,
    int batchSize
    );

cusolverStatus_t CUSOLVERAPI cusolverDnCheevjBatched_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cublasFillMode_t uplo, 
    int n,
    const cuComplex *A, 
    int lda,
    const float *W,
    int *lwork,
    syevjInfo_t params,
    int batchSize
    );

cusolverStatus_t CUSOLVERAPI cusolverDnZheevjBatched_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cublasFillMode_t uplo, 
    int n,
    const cuDoubleComplex *A, 
    int lda,
    const double *W,
    int *lwork,
    syevjInfo_t params,
    int batchSize
    );


cusolverStatus_t CUSOLVERAPI cusolverDnSsyevjBatched(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz,
    cublasFillMode_t uplo,
    int n,   
    float *A,
    int lda,
    float *W, 
    float *work,
    int lwork,
    int *info, 
    syevjInfo_t params,
    int batchSize
    );

cusolverStatus_t CUSOLVERAPI cusolverDnDsyevjBatched(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz,
    cublasFillMode_t uplo, 
    int n,
    double *A,
    int lda,
    double *W,
    double *work,
    int lwork,
    int *info,
    syevjInfo_t params,
    int batchSize
    );

cusolverStatus_t CUSOLVERAPI cusolverDnCheevjBatched(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz,
    cublasFillMode_t uplo, 
    int n,
    cuComplex *A,
    int lda,
    float *W,
    cuComplex *work,
    int lwork,
    int *info,
    syevjInfo_t params,
    int batchSize
    );

cusolverStatus_t CUSOLVERAPI cusolverDnZheevjBatched(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cublasFillMode_t uplo, 
    int n,
    cuDoubleComplex *A,
    int lda,
    double *W,
    cuDoubleComplex *work,
    int lwork,
    int *info,
    syevjInfo_t params,
    int batchSize
    );


cusolverStatus_t CUSOLVERAPI cusolverDnSsyevj_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz,
    cublasFillMode_t uplo,
    int n,
    const float *A,
    int lda,
    const float *W,
    int *lwork,
    syevjInfo_t params);


cusolverStatus_t CUSOLVERAPI cusolverDnDsyevj_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz,
    cublasFillMode_t uplo,
    int n,
    const double *A,
    int lda,
    const double *W,
    int *lwork,
    syevjInfo_t params);


cusolverStatus_t CUSOLVERAPI cusolverDnCheevj_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz,
    cublasFillMode_t uplo,
    int n,
    const cuComplex *A,
    int lda,
    const float *W,
    int *lwork,
    syevjInfo_t params);


cusolverStatus_t CUSOLVERAPI cusolverDnZheevj_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cublasFillMode_t uplo,
    int n,
    const cuDoubleComplex *A,
    int lda,
    const double *W,
    int *lwork,
    syevjInfo_t params);


cusolverStatus_t CUSOLVERAPI cusolverDnSsyevj(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cublasFillMode_t uplo, 
    int n,
    float *A,
    int lda,
    float *W,
    float *work,
    int lwork,
    int *info,
    syevjInfo_t params);


cusolverStatus_t CUSOLVERAPI cusolverDnDsyevj(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cublasFillMode_t uplo,
    int n,
    double *A,
    int lda,
    double *W, 
    double *work,
    int lwork,
    int *info,
    syevjInfo_t params);


cusolverStatus_t CUSOLVERAPI cusolverDnCheevj(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    cublasFillMode_t uplo,
    int n,
    cuComplex *A,
    int lda,
    float *W, 
    cuComplex *work,
    int lwork,
    int *info,
    syevjInfo_t params);


cusolverStatus_t CUSOLVERAPI cusolverDnZheevj(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz,
    cublasFillMode_t uplo,
    int n,
    cuDoubleComplex *A,
    int lda,
    double *W, 
    cuDoubleComplex *work,
    int lwork,
    int *info,
    syevjInfo_t params);


cusolverStatus_t CUSOLVERAPI cusolverDnSsygvj_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype, 
    cusolverEigMode_t jobz, 
    cublasFillMode_t uplo,
    int n,
    const float *A, 
    int lda,
    const float *B, 
    int ldb,
    const float *W,
    int *lwork,
    syevjInfo_t params);

cusolverStatus_t CUSOLVERAPI cusolverDnDsygvj_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype, 
    cusolverEigMode_t jobz, 
    cublasFillMode_t uplo,
    int n,
    const double *A, 
    int lda,
    const double *B,
    int ldb,
    const double *W,
    int *lwork,
    syevjInfo_t params);

cusolverStatus_t CUSOLVERAPI cusolverDnChegvj_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype,
    cusolverEigMode_t jobz,
    cublasFillMode_t uplo,
    int n,
    const cuComplex *A, 
    int lda,
    const cuComplex *B, 
    int ldb,
    const float *W,
    int *lwork,
    syevjInfo_t params);

cusolverStatus_t CUSOLVERAPI cusolverDnZhegvj_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype,
    cusolverEigMode_t jobz,
    cublasFillMode_t uplo,
    int n,
    const cuDoubleComplex *A, 
    int lda,
    const cuDoubleComplex *B, 
    int ldb,
    const double *W,
    int *lwork,
    syevjInfo_t params);

cusolverStatus_t CUSOLVERAPI cusolverDnSsygvj(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype, 
    cusolverEigMode_t jobz,
    cublasFillMode_t uplo, 
    int n,
    float *A, 
    int lda,
    float *B, 
    int ldb,
    float *W,
    float *work,
    int lwork,
    int *info,
    syevjInfo_t params);

cusolverStatus_t CUSOLVERAPI cusolverDnDsygvj(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype, 
    cusolverEigMode_t jobz,
    cublasFillMode_t uplo, 
    int n,
    double *A, 
    int lda,
    double *B,
    int ldb,
    double *W, 
    double *work,
    int lwork,
    int *info,
    syevjInfo_t params);

cusolverStatus_t CUSOLVERAPI cusolverDnChegvj(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype, 
    cusolverEigMode_t jobz, 
    cublasFillMode_t uplo,
    int n,
    cuComplex *A, 
    int lda,
    cuComplex *B, 
    int ldb,
    float *W,
    cuComplex *work,
    int lwork,
    int *info,
    syevjInfo_t params);

cusolverStatus_t CUSOLVERAPI cusolverDnZhegvj(
    cusolverDnHandle_t handle,
    cusolverEigType_t itype, 
    cusolverEigMode_t jobz,
    cublasFillMode_t uplo,  
    int n,
    cuDoubleComplex *A, 
    int lda,
    cuDoubleComplex *B, 
    int ldb,
    double *W, 
    cuDoubleComplex *work,
    int lwork,
    int *info,
    syevjInfo_t params);


cusolverStatus_t CUSOLVERAPI cusolverDnCreateGesvdjInfo(
    gesvdjInfo_t *info);

cusolverStatus_t CUSOLVERAPI cusolverDnDestroyGesvdjInfo(
    gesvdjInfo_t info);

cusolverStatus_t CUSOLVERAPI cusolverDnXgesvdjSetTolerance(
    gesvdjInfo_t info,
    double tolerance);

cusolverStatus_t CUSOLVERAPI cusolverDnXgesvdjSetMaxSweeps(
    gesvdjInfo_t info,
    int max_sweeps);

cusolverStatus_t CUSOLVERAPI cusolverDnXgesvdjSetSortEig(
    gesvdjInfo_t info,
    int sort_svd);

cusolverStatus_t CUSOLVERAPI cusolverDnXgesvdjGetResidual(
    cusolverDnHandle_t handle,
    gesvdjInfo_t info,
    double *residual);

cusolverStatus_t CUSOLVERAPI cusolverDnXgesvdjGetSweeps(
    cusolverDnHandle_t handle,
    gesvdjInfo_t info,
    int *executed_sweeps);

cusolverStatus_t CUSOLVERAPI cusolverDnSgesvdjBatched_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz,
    int m,                
    int n,                
    const float *A,    
    int lda,           
    const float *S, 
    const float *U,   
    int ldu, 
    const float *V,
    int ldv,  
    int *lwork,
    gesvdjInfo_t params,
    int batchSize);

cusolverStatus_t CUSOLVERAPI cusolverDnDgesvdjBatched_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz,
    int m,
    int n,
    const double *A, 
    int lda,
    const double *S,
    const double *U,
    int ldu,
    const double *V,
    int ldv,
    int *lwork,
    gesvdjInfo_t params,
    int batchSize);

cusolverStatus_t CUSOLVERAPI cusolverDnCgesvdjBatched_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz,
    int m,
    int n,
    const cuComplex *A,
    int lda,
    const float *S,
    const cuComplex *U,
    int ldu,
    const cuComplex *V,
    int ldv,
    int *lwork,
    gesvdjInfo_t params,
    int batchSize);

cusolverStatus_t CUSOLVERAPI cusolverDnZgesvdjBatched_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    int m, 
    int n, 
    const cuDoubleComplex *A,
    int lda,
    const double *S,
    const cuDoubleComplex *U,
    int ldu, 
    const cuDoubleComplex *V,
    int ldv,
    int *lwork,
    gesvdjInfo_t params,
    int batchSize);

cusolverStatus_t CUSOLVERAPI cusolverDnSgesvdjBatched(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    int m, 
    int n, 
    float *A, 
    int lda, 
    float *S, 
    float *U,
    int ldu,
    float *V,
    int ldv, 
    float *work,
    int lwork,
    int *info,
    gesvdjInfo_t params,
    int batchSize);

cusolverStatus_t CUSOLVERAPI cusolverDnDgesvdjBatched(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz,
    int m,
    int n,
    double *A,
    int lda,
    double *S,
    double *U,
    int ldu,
    double *V,
    int ldv, 
    double *work,
    int lwork,
    int *info, 
    gesvdjInfo_t params,
    int batchSize);

cusolverStatus_t CUSOLVERAPI cusolverDnCgesvdjBatched(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz,
    int m, 
    int n,
    cuComplex *A,
    int lda,
    float *S,
    cuComplex *U,
    int ldu,
    cuComplex *V,
    int ldv,
    cuComplex *work,
    int lwork,
    int *info,
    gesvdjInfo_t params,
    int batchSize);

cusolverStatus_t CUSOLVERAPI cusolverDnZgesvdjBatched(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz,
    int m,
    int n,
    cuDoubleComplex *A,
    int lda, 
    double *S, 
    cuDoubleComplex *U,
    int ldu,
    cuDoubleComplex *V,
    int ldv,
    cuDoubleComplex *work,
    int lwork,
    int *info,
    gesvdjInfo_t params,
    int batchSize);

cusolverStatus_t CUSOLVERAPI cusolverDnSgesvdj_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    int econ,
    int m,
    int n, 
    const float *A,
    int lda,
    const float *S,
    const float *U,
    int ldu, 
    const float *V,
    int ldv,
    int *lwork,
    gesvdjInfo_t params);

cusolverStatus_t CUSOLVERAPI cusolverDnDgesvdj_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    int econ,
    int m,
    int n,
    const double *A, 
    int lda,
    const double *S,
    const double *U,
    int ldu,
    const double *V,
    int ldv,
    int *lwork,
    gesvdjInfo_t params);

cusolverStatus_t CUSOLVERAPI cusolverDnCgesvdj_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    int econ,
    int m,
    int n,
    const cuComplex *A,
    int lda,
    const float *S,
    const cuComplex *U,
    int ldu,
    const cuComplex *V,
    int ldv,
    int *lwork,
    gesvdjInfo_t params);

cusolverStatus_t CUSOLVERAPI cusolverDnZgesvdj_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz,
    int econ,
    int m,
    int n,
    const cuDoubleComplex *A,
    int lda,
    const double *S,
    const cuDoubleComplex *U,
    int ldu,
    const cuDoubleComplex *V,
    int ldv, 
    int *lwork,
    gesvdjInfo_t params);

cusolverStatus_t CUSOLVERAPI cusolverDnSgesvdj(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz,
    int econ,
    int m,
    int n,
    float *A, 
    int lda,
    float *S,
    float *U,
    int ldu,
    float *V,
    int ldv,
    float *work,
    int lwork,
    int *info,
    gesvdjInfo_t params);

cusolverStatus_t CUSOLVERAPI cusolverDnDgesvdj(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz,
    int econ, 
    int m, 
    int n, 
    double *A, 
    int lda,
    double *S,
    double *U,
    int ldu,
    double *V,
    int ldv,
    double *work,
    int lwork,
    int *info,
    gesvdjInfo_t params);

cusolverStatus_t CUSOLVERAPI cusolverDnCgesvdj(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz,
    int econ,
    int m,
    int n,
    cuComplex *A,
    int lda,
    float *S,
    cuComplex *U,
    int ldu,
    cuComplex *V,
    int ldv,
    cuComplex *work,
    int lwork,
    int *info,
    gesvdjInfo_t params);

cusolverStatus_t CUSOLVERAPI cusolverDnZgesvdj(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz,
    int econ,
    int m,
    int n,
    cuDoubleComplex *A,
    int lda,
    double *S,
    cuDoubleComplex *U, 
    int ldu, 
    cuDoubleComplex *V,
    int ldv,
    cuDoubleComplex *work,
    int lwork,
    int *info,
    gesvdjInfo_t params);


/* batched approximate SVD */

cusolverStatus_t CUSOLVERAPI cusolverDnSgesvdaStridedBatched_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz,
    int rank,
    int m,
    int n,
    const float *d_A, 
    int lda,
    long long int strideA, 
    const float *d_S, 
    long long int strideS, 
    const float *d_U, 
    int ldu,
    long long int strideU, 
    const float *d_V, 
    int ldv,
    long long int strideV,
    int *lwork,
    int batchSize
    );


cusolverStatus_t CUSOLVERAPI cusolverDnDgesvdaStridedBatched_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz,
    int rank,
    int m,
    int n,
    const double *d_A, 
    int lda,
    long long int strideA, 
    const double *d_S,   
    long long int strideS, 
    const double *d_U,  
    int ldu,
    long long int strideU, 
    const double *d_V,
    int ldv,
    long long int strideV, 
    int *lwork,
    int batchSize
    );


cusolverStatus_t CUSOLVERAPI cusolverDnCgesvdaStridedBatched_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz,
    int rank,
    int m,
    int n,
    const cuComplex *d_A, 
    int lda,
    long long int strideA, 
    const float *d_S, 
    long long int strideS, 
    const cuComplex *d_U,
    int ldu,
    long long int strideU, 
    const cuComplex *d_V, 
    int ldv,
    long long int strideV, 
    int *lwork,
    int batchSize
    );

cusolverStatus_t CUSOLVERAPI cusolverDnZgesvdaStridedBatched_bufferSize(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz,
    int rank,
    int m,
    int n,
    const cuDoubleComplex *d_A,
    int lda,
    long long int strideA,
    const double *d_S, 
    long long int strideS, 
    const cuDoubleComplex *d_U, 
    int ldu,
    long long int strideU,
    const cuDoubleComplex *d_V,
    int ldv,
    long long int strideV, 
    int *lwork,
    int batchSize
    );


cusolverStatus_t CUSOLVERAPI cusolverDnSgesvdaStridedBatched(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    int rank, 
    int m,   
    int n,  
    const float *d_A, 
    int lda, 
    long long int strideA,
    float *d_S, 
    long long int strideS, 
    float *d_U, 
    int ldu, 
    long long int strideU,
    float *d_V, 
    int ldv,    
    long long int strideV, 
    float *d_work,
    int lwork,
    int *d_info,
    double *h_R_nrmF,
    int batchSize);


cusolverStatus_t CUSOLVERAPI cusolverDnDgesvdaStridedBatched(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    int rank,
    int m, 
    int n, 
    const double *d_A,
    int lda,  
    long long int strideA, 
    double *d_S, 
    long long int strideS,
    double *d_U, 
    int ldu, 
    long long int strideU, 
    double *d_V, 
    int ldv, 
    long long int strideV,
    double *d_work,
    int lwork,
    int *d_info,
    double *h_R_nrmF, 
    int batchSize);


cusolverStatus_t CUSOLVERAPI cusolverDnCgesvdaStridedBatched(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    int rank,  
    int m, 
    int n, 
    const cuComplex *d_A, 
    int lda,
    long long int strideA,
    float *d_S,
    long long int strideS,
    cuComplex *d_U, 
    int ldu,   
    long long int strideU,  
    cuComplex *d_V, 
    int ldv, 
    long long int strideV,
    cuComplex *d_work,
    int lwork,
    int *d_info,
    double *h_R_nrmF, 
    int batchSize);


cusolverStatus_t CUSOLVERAPI cusolverDnZgesvdaStridedBatched(
    cusolverDnHandle_t handle,
    cusolverEigMode_t jobz, 
    int rank, 
    int m,   
    int n,  
    const cuDoubleComplex *d_A, 
    int lda,    
    long long int strideA,
    double *d_S,
    long long int strideS,
    cuDoubleComplex *d_U, 
    int ldu,   
    long long int strideU, 
    cuDoubleComplex *d_V,
    int ldv, 
    long long int strideV, 
    cuDoubleComplex *d_work,
    int lwork,
    int *d_info,
    double *h_R_nrmF,
    int batchSize);




#if defined(__cplusplus)
}
#endif /* __cplusplus */


#endif /* !defined(CUDENSE_H_) */
