/*
 * NVIDIA_COPYRIGHT_BEGIN
 *
 * Copyright (c) 2010-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 *
 * NVIDIA_COPYRIGHT_END
 */

#ifndef fatbinaryctl_INCLUDED
#define fatbinaryctl_INCLUDED

#ifndef __CUDA_INTERNAL_COMPILATION__
#include <stddef.h> /* for size_t */
#endif
#include "fatbinary.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 
 * These are routines for controlling the fat binary.
 * An opaque handle is used.
 */
typedef struct fatBinaryCtlHandle *fatBinaryCtl_t;
typedef const struct fatBinaryCtlHandle *fatBinaryCtl_ct;
// !!! until driver sources are changed, do not require opaque type

typedef enum {
  FBCTL_ERROR_NONE = 0,
  FBCTL_ERROR_NULL,                      /* null pointer */
  FBCTL_ERROR_UNRECOGNIZED,              /* unrecognized kind */
  FBCTL_ERROR_NO_CANDIDATE,              /* no candidate found */
  FBCTL_ERROR_COMPILE_FAILED,            /* no candidate found */
  FBCTL_ERROR_INTERNAL,                  /* unexpected internal error */
  FBCTL_ERROR_COMPILER_LOAD_FAILED,      /* loading compiler library failed */
} fatBinaryCtlError_t;
extern const char* fatBinaryCtl_Errmsg (fatBinaryCtlError_t e);

/* Cannot change directly to opaque handle without causing warnings,
 * so add new CreateHandle routine and eventually switch everyone to it. */
extern fatBinaryCtlError_t fatBinaryCtl_Create (void **vhandle);
extern fatBinaryCtlError_t fatBinaryCtl_CreateHandle (fatBinaryCtl_t *handle);

extern void fatBinaryCtl_Delete (fatBinaryCtl_t handle);

/* Set (fatbin or elf) binary that we will search */
extern fatBinaryCtlError_t fatBinaryCtl_SetBinary (fatBinaryCtl_t handle, 
                                                   const void* binary);

/* Set target SM that we are looking for */
extern fatBinaryCtlError_t fatBinaryCtl_SetTargetSM (fatBinaryCtl_t handle,
                                                     unsigned int arch);

typedef enum {
  fatBinary_PreferBestCode,  /* default */
  fatBinary_AvoidPTX,        /* use sass if possible for compile-time savings */
  fatBinary_ForcePTX,        /* use ptx (mainly for testing) */
  fatBinary_JITIfNotMatch,   /* use ptx if arch doesn't match */
  fatBinary_PreferIr,        /* choose IR when available */
  fatBinary_LinkCompatible,  /* use sass if link-compatible */
} fatBinary_CompilationPolicy;
/* Set policy for how we handle JIT compiles */
extern fatBinaryCtlError_t fatBinaryCtl_SetPolicy(fatBinaryCtl_t handle,
                                            fatBinary_CompilationPolicy policy);

/* Set ptxas options for JIT compiles */
extern fatBinaryCtlError_t fatBinaryCtl_SetPtxasOptions(fatBinaryCtl_t handle,
                                                        const char *options);

/* Set flags for fatbinary */
extern fatBinaryCtlError_t fatBinaryCtl_SetFlags (fatBinaryCtl_t handle,
                                                  long long flags);

/* Return identifier string for fatbinary */
extern fatBinaryCtlError_t fatBinaryCtl_GetIdentifier(fatBinaryCtl_ct handle,
                                                      const char **id);

/* Return ptxas options for fatbinary */
extern fatBinaryCtlError_t fatBinaryCtl_GetPtxasOptions(fatBinaryCtl_ct handle,
                                                        const char **options);

/* Return cicc options for fatbinary */
extern fatBinaryCtlError_t fatBinaryCtl_GetCiccOptions(fatBinaryCtl_ct handle,
                                                       const char **options);

/* Return whether fatbin has debug code (1 == true, 0 == false) */
extern fatBinaryCtlError_t fatBinaryCtl_HasDebug(fatBinaryCtl_ct handle,
                                                 int *debug);

/* Using the input values, pick the best candidate */
extern fatBinaryCtlError_t fatBinaryCtl_PickCandidate (fatBinaryCtl_t handle);

/* 
 * Using the previously chosen candidate, compile the code to elf,
 * returning elf image and size.
 * Note that because elf is allocated inside fatBinaryCtl, 
 * it will be freed when _Delete routine is called.
 */
extern fatBinaryCtlError_t fatBinaryCtl_Compile (fatBinaryCtl_t handle, 
                                                 void* *elf, size_t *esize);

/* Return the candidate found */
extern fatBinaryCtlError_t fatBinaryCtl_GetCandidate(fatBinaryCtl_ct handle, 
                                                     void **binary,
                                                     fatBinaryCodeKind *kind,
                                                     size_t *size);

/**** old APIs; remove once everyone has switched ****/
// !!! Below routines are deprecated, will be removed once driver is updated
extern fatBinaryCtlError_t fatBinaryCtl (void *vhandle, int request, ...);

/* defined requests */
#define FBCTL_SET_BINARY        1  /* void* (e.g. fatbin, elf or ptx object) */
#define FBCTL_SET_TARGETSM      2  /* int (use values from nvelf.h) */
#define FBCTL_SET_FLAGS         3  /* longlong */
#define FBCTL_SET_CMDOPTIONS    4  /* char* */
#define FBCTL_SET_POLICY        5  /* fatBinary_CompilationPolicy */
/* get calls return value in arg, thus are all by reference */
#define FBCTL_GET_CANDIDATE     10 /* void** binary,
                                    * fatBinaryCodeKind* kind,
                                    * size_t* size */
#define FBCTL_GET_IDENTIFIER    11 /* char* * */
#define FBCTL_HAS_DEBUG         12 /* Bool * */
#define FBCTL_GET_PTXAS_OPTIONS 13 /* char** */
#define FBCTL_GET_CICC_OPTIONS  14 /* char** */
/**** end old APIs ****/

/*
 * These defines are for the fatbin.c runtime wrapper
 * This code will be removed once everyone switches to fatbinary_section.h
 */
#ifndef fatbinary_section_INCLUDED
#define fatbinary_section_INCLUDED

#define FATBINC_MAGIC   0x466243B1
#define FATBINC_VERSION 1
#define FATBINC_LINK_VERSION 2

typedef struct {
  int magic;
  int version;
  const unsigned long long* data;
  void *filename_or_fatbins;  /* version 1: offline filename,
                               * version 2: array of prelinked fatbins */
} __fatBinC_Wrapper_t;

/*
 * The section that contains the fatbin control structure
 */
#ifdef STD_OS_Darwin
/* mach-o sections limited to 15 chars, and want __ prefix else strip complains, * so use a different name */
#define FATBIN_CONTROL_SECTION_NAME     "__fatbin"
#define FATBIN_DATA_SECTION_NAME        "__nv_fatbin"
/* only need segment name for mach-o */
#define FATBIN_SEGMENT_NAME             "__NV_CUDA"
#else
#define FATBIN_CONTROL_SECTION_NAME     ".nvFatBinSegment"
/*
 * The section that contains the fatbin data itself
 * (put in separate section so easy to find)
 */
#define FATBIN_DATA_SECTION_NAME        ".nv_fatbin"
#endif
/* section for pre-linked relocatable fatbin data */
#define FATBIN_PRELINK_DATA_SECTION_NAME "__nv_relfatbin"

#endif /* fatbinary_section_INCLUDED */

#ifdef __cplusplus
}
#endif

#endif /* fatbinaryctl_INCLUDED */
