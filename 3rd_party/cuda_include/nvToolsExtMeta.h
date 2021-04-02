/*
* Copyright 2009-2017 NVIDIA Corporation.  All rights reserved.
*
* NOTICE TO USER:
*
* This source code is subject to NVIDIA ownership rights under U.S. and
* international Copyright laws.
*
* This software and the information contained herein is PROPRIETARY and
* CONFIDENTIAL to NVIDIA and is being provided under the terms and conditions
* of a form of NVIDIA software license agreement.
*
* NVIDIA MAKES NO REPRESENTATION ABOUT THE SUITABILITY OF THIS SOURCE
* CODE FOR ANY PURPOSE.  IT IS PROVIDED "AS IS" WITHOUT EXPRESS OR
* IMPLIED WARRANTY OF ANY KIND.  NVIDIA DISCLAIMS ALL WARRANTIES WITH
* REGARD TO THIS SOURCE CODE, INCLUDING ALL IMPLIED WARRANTIES OF
* MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE.
* IN NO EVENT SHALL NVIDIA BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL,
* OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS
* OF USE, DATA OR PROFITS,  WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
* OR OTHER TORTIOUS ACTION,  ARISING OUT OF OR IN CONNECTION WITH THE USE
* OR PERFORMANCE OF THIS SOURCE CODE.
*
* U.S. Government End Users.   This source code is a "commercial item" as
* that term is defined at  48 C.F.R. 2.101 (OCT 1995), consisting  of
* "commercial computer  software"  and "commercial computer software
* documentation" as such terms are  used in 48 C.F.R. 12.212 (SEPT 1995)
* and is provided to the U.S. Government only as a commercial end item.
* Consistent with 48 C.F.R.12.212 and 48 C.F.R. 227.7202-1 through
* 227.7202-4 (JUNE 1995), all U.S. Government End Users acquire the
* source code with only those rights set forth herein.
*
* Any use of this source code in individual and commercial software must
* include, in the user documentation and internal comments to the code,
* the above Disclaimer and U.S. Government End Users Notice.
*/

/** \mainpage
 * \section Introduction
 * The NVIDIA Tools Extension library is a set of functions that a
 * developer can use to provide additional information to tools.
 * The additional information is used by the tool to improve
 * analysis and visualization of data.
 *
 * The library introduces close to zero overhead if no tool is
 * attached to the application.  The overhead when a tool is
 * attached is specific to the tool.
 */

#ifndef NVTOOLSEXT_META_H_
#define NVTOOLSEXT_META_H_



#ifdef _MSC_VER
#  define NVTX_PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#elif (defined(__GNUC__) || defined(__clang__))
#  define NVTX_PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif


/* Structs defining parameters for NVTX API functions */

typedef struct NvtxMarkEx{ const nvtxEventAttributes_t* eventAttrib; } NvtxMarkEx;
NVTX_PACK(struct NvtxDomainMarkEx { nvtxDomainHandle_t domain; NvtxMarkEx core; });
typedef struct NvtxDomainMarkEx NvtxDomainMarkEx;

typedef struct NvtxMarkA        { const char* message; } NvtxMarkA;
typedef struct NvtxMarkW        { const wchar_t* message; } NvtxMarkW;
typedef struct NvtxRangeStartEx { const nvtxEventAttributes_t* eventAttrib; } NvtxRangeStartEx;

NVTX_PACK(struct NvtxDomainRangeStartEx { nvtxDomainHandle_t domain; NvtxRangeStartEx core; });
typedef struct NvtxDomainRangeStartEx NvtxDomainRangeStartEx;

typedef struct NvtxRangeStartA  { const char* message; } NvtxRangeStartA;
typedef struct NvtxRangeStartW  { const wchar_t* message; } NvtxRangeStartW;
typedef struct NvtxRangeEnd     { nvtxRangeId_t id; } NvtxRangeEnd;

NVTX_PACK(struct NvtxDomainRangeEnd { nvtxDomainHandle_t domain; NvtxRangeEnd core; });
typedef struct NvtxDomainRangeEnd NvtxDomainRangeEnd;

typedef struct NvtxRangePushEx  { const nvtxEventAttributes_t* eventAttrib; } NvtxRangePushEx;

NVTX_PACK(struct NvtxDomainRangePushEx { nvtxDomainHandle_t domain; NvtxRangePushEx core; });
typedef struct NvtxDomainRangePushEx NvtxDomainRangePushEx;

typedef struct NvtxRangePushA   { const char* message; } NvtxRangePushA;
typedef struct NvtxRangePushW   { const wchar_t* message; } NvtxRangePushW;
typedef struct NvtxDomainRangePop   { nvtxDomainHandle_t domain; } NvtxDomainRangePop;
/*     NvtxRangePop     - no parameters, params will be NULL. */
typedef struct NvtxDomainResourceCreate  { nvtxDomainHandle_t domain; const nvtxResourceAttributes_t* attribs; } NvtxDomainResourceCreate;
typedef struct NvtxDomainResourceDestroy  { nvtxResourceHandle_t handle; } NvtxDomainResourceDestroy;
typedef struct NvtxDomainRegisterString  { nvtxDomainHandle_t domain; const void* str; } NvtxDomainRegisterString;
typedef struct NvtxDomainCreate  { const void* name; } NvtxDomainCreate;
typedef struct NvtxDomainDestroy  { nvtxDomainHandle_t domain; } NvtxDomainDestroy;


#ifdef NVTOOLSEXT_SYNC_H_
typedef struct NvtxSyncUserCommon  { nvtxSyncUser_t handle; } NvtxSyncUserCommon;
typedef struct NvtxSyncUserCreate  { nvtxDomainHandle_t domain; const nvtxSyncUserAttributes_t* attribs; } NvtxSyncUserCreate;
#endif

/* All other NVTX API functions are for naming resources. 
 * A generic params struct is used for all such functions,
 * passing all resource handles as a uint64_t.
 */
typedef struct NvtxNameResourceA
{
    uint64_t resourceHandle;
    const char* name;
} NvtxNameResourceA;

typedef struct NvtxNameResourceW
{
    uint64_t resourceHandle;
    const wchar_t* name;
} NvtxNameResourceW;

NVTX_PACK(struct NvtxDomainNameResourceA { nvtxDomainHandle_t domain; NvtxNameResourceA core; });
typedef struct NvtxDomainNameResourceA NvtxDomainNameResourceA;
NVTX_PACK(struct NvtxDomainNameResourceW { nvtxDomainHandle_t domain; NvtxNameResourceW core; });
typedef struct NvtxDomainNameResourceW NvtxDomainNameResourceW;


#endif /* NVTOOLSEXT_META_H_ */
