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

#ifndef fatbinary_INCLUDED
#define fatbinary_INCLUDED

/* 
 * This is the fat binary header structure. 
 * Because all layout information is contained in all the structures, 
 * it is both forward and backward compatible. 
 * A new driver can interpret an old binary 
 * as it will not address fields that are present in the current version. 
 * An old driver can, for minor version differences, 
 * still interpret a new binary, 
 * as the new features in the binary will be ignored by the driver.
 *
 * This is the top level type for the binary format. 
 * It points to a fatBinaryHeader structure. 
 * It is followed by a number of code binaries.
 * The structures must be 8-byte aligned, 
 * and are the same on both 32bit and 64bit platforms.
 *
 * The details of the format for the binaries that follow the header
 * are in a separate internal header.
 */

typedef struct fatBinaryHeader * computeFatBinaryFormat_t;
typedef const struct fatBinaryHeader * computeFatBinaryFormat_ct;

/* ensure 8-byte alignment */
#if defined(__GNUC__)
#define fatbinary_ALIGN_(n) __attribute__((aligned(n)))
#elif defined(_WIN32)
#define fatbinary_ALIGN_(n) __declspec(align(n))
#else
#error !! UNSUPPORTED COMPILER !!
#endif

/* Magic numbers */
#define FATBIN_MAGIC 0xBA55ED50U
#define OLD_STYLE_FATBIN_MAGIC 0x1EE55A01U

#define FATBIN_VERSION 0x0001U

/*
 * This is the fat binary header structure. 
 * The 'magic' field holds the magic number. 
 * A magic of OLD_STYLE_FATBIN_MAGIC indicates an old style fat binary. 
 * Because old style binaries are in little endian, we can just read 
 * the magic in a 32 bit container for both 32 and 64 bit platforms. 
 * The 'version' fields holds the fatbin version.
 * It should be the goal to never bump this version. 
 * The headerSize holds the size of the header (must be multiple of 8).
 * The 'fatSize' fields holds the size of the entire fat binary, 
 * excluding this header. It must be a multiple of 8.
 */
struct fatbinary_ALIGN_(8) fatBinaryHeader
{
  unsigned int           magic;
  unsigned short         version;
  unsigned short         headerSize;
  unsigned long long int fatSize;
};

/* Code kinds supported by the driver */
typedef enum {
  FATBIN_KIND_PTX      = 0x0001,
  FATBIN_KIND_ELF      = 0x0002,
  FATBIN_KIND_OLDCUBIN = 0x0004, /* old format no longer generated */
  FATBIN_KIND_IR       = 0x0008, /* NVVM IR */
} fatBinaryCodeKind;

#endif /* fatbinary_INCLUDED */
