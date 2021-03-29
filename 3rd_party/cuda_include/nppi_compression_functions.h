 /* Copyright 2009-2019 NVIDIA Corporation.  All rights reserved. 
  * 
  * NOTICE TO LICENSEE: 
  * 
  * The source code and/or documentation ("Licensed Deliverables") are 
  * subject to NVIDIA intellectual property rights under U.S. and 
  * international Copyright laws. 
  * 
  * The Licensed Deliverables contained herein are PROPRIETARY and 
  * CONFIDENTIAL to NVIDIA and are being provided under the terms and 
  * conditions of a form of NVIDIA software license agreement by and 
  * between NVIDIA and Licensee ("License Agreement") or electronically 
  * accepted by Licensee.  Notwithstanding any terms or conditions to 
  * the contrary in the License Agreement, reproduction or disclosure 
  * of the Licensed Deliverables to any third party without the express 
  * written consent of NVIDIA is prohibited. 
  * 
  * NOTWITHSTANDING ANY TERMS OR CONDITIONS TO THE CONTRARY IN THE 
  * LICENSE AGREEMENT, NVIDIA MAKES NO REPRESENTATION ABOUT THE 
  * SUITABILITY OF THESE LICENSED DELIVERABLES FOR ANY PURPOSE.  THEY ARE 
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
  * C.F.R. 12.212 (SEPT 1995) and are provided to the U.S. Government 
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
#ifndef NV_NPPI_COMPRESSION_FUNCTIONS_H
#define NV_NPPI_COMPRESSION_FUNCTIONS_H
 
/**
 * \file nppi_compression_functions.h
 * NPP Image Processing Functionality.
 */
 
#include "nppdefs.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup image_compression Compression
 *  @ingroup nppi
 *
 * Image compression primitives.
 *
 * The JPEG standard defines a flow of level shift, DCT and quantization for
 * forward JPEG transform and inverse level shift, IDCT and de-quantization
 * for inverse JPEG transform. This group has the functions for both forward
 * and inverse functions. 
 *
 * @{
 *
 * These functions can be found in the nppicom library. Linking to only the sub-libraries that you use can significantly
 * save link time, application load time, and CUDA runtime startup time when using dynamic libraries.
 *
 */

/** @defgroup image_quantization Quantization Functions
 * The set of quantization functions available in the library.
 * @{
 *
 */

/**
 * Apply quality factor to raw 8-bit quantization table.
 *
 * This is effectively and in-place method that modifies a given raw
 * quantization table based on a quality factor.
 * Note that this method is a host method and that the pointer to the
 * raw quantization table is a host pointer.
 *
 * \param hpQuantRawTable Raw quantization table.
 * \param nQualityFactor Quality factor for the table. Range is [1:100].
 * \return Error code:
 *      ::NPP_NULL_POINTER_ERROR is returned if hpQuantRawTable is 0.
 */
NppStatus 
nppiQuantFwdRawTableInit_JPEG_8u(Npp8u * hpQuantRawTable, int nQualityFactor);

/**
 * Initializes a quantization table for nppiDCTQuantFwd8x8LS_JPEG_8u16s_C1R().
 *    The method creates a 16-bit version of the raw table and converts the 
 * data order from zigzag layout to original row-order layout since raw
 * quantization tables are typically stored in zigzag format.
 *
 * This method is a host method. It consumes and produces host data. I.e. the pointers
 * passed to this function must be host pointers. The resulting table needs to be
 * transferred to device memory in order to be used with nppiDCTQuantFwd8x8LS_JPEG_8u16s_C1R()
 * function.
 *
 * \param hpQuantRawTable Host pointer to raw quantization table as returned by 
 *      nppiQuantFwdRawTableInit_JPEG_8u(). The raw quantization table is assumed to be in
 *      zigzag order.
 * \param hpQuantFwdRawTable Forward quantization table for use with nppiDCTQuantFwd8x8LS_JPEG_8u16s_C1R().
 * \return Error code:
 *      ::NPP_NULL_POINTER_ERROR pQuantRawTable is 0.
 */
NppStatus 
nppiQuantFwdTableInit_JPEG_8u16u(const Npp8u * hpQuantRawTable, Npp16u * hpQuantFwdRawTable);

/**
 * Initializes a quantization table for nppiDCTQuantInv8x8LS_JPEG_16s8u_C1R().
 *      The nppiDCTQuantFwd8x8LS_JPEG_8u16s_C1R() method uses a quantization table
 * in a 16-bit format allowing for faster processing. In addition it converts the 
 * data order from zigzag layout to original row-order layout. Typically raw
 * quantization tables are stored in zigzag format.
 *
 * This method is a host method and consumes and produces host data. I.e. the pointers
 * passed to this function must be host pointers. The resulting table needs to be
 * transferred to device memory in order to be used with nppiDCTQuantFwd8x8LS_JPEG_8u16s_C1R()
 * function.
 *
 * \param hpQuantRawTable Raw quantization table.
 * \param hpQuantFwdRawTable Inverse quantization table.
 * \return ::NPP_NULL_POINTER_ERROR pQuantRawTable or pQuantFwdRawTable is0.
 */
NppStatus 
nppiQuantInvTableInit_JPEG_8u16u(const Npp8u * hpQuantRawTable, Npp16u * hpQuantFwdRawTable);


/**
 * Forward DCT, quantization and level shift part of the JPEG encoding.
 * Input is expected in 8x8 macro blocks and output is expected to be in 64x1
 * macro blocks.
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param pQuantFwdTable Forward quantization tables for JPEG encoding created
 *          using nppiQuantInvTableInit_JPEG_8u16u().
 * \param oSizeROI \ref roi_specification.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return Error codes:
 *         - ::NPP_SIZE_ERROR For negative input height/width or not a multiple of
 *           8 width/height.
 *         - ::NPP_STEP_ERROR If input image width is not multiple of 8 or does not
 *           match ROI.
 *         - ::NPP_NULL_POINTER_ERROR If the destination pointer is 0.
 */
NppStatus 
nppiDCTQuantFwd8x8LS_JPEG_8u16s_C1R_Ctx(const Npp8u  * pSrc, int nSrcStep, 
                                              Npp16s * pDst, int nDstStep, 
                                        const Npp16u * pQuantFwdTable, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiDCTQuantFwd8x8LS_JPEG_8u16s_C1R(const Npp8u  * pSrc, int nSrcStep, 
                                          Npp16s * pDst, int nDstStep, 
                                    const Npp16u * pQuantFwdTable, NppiSize oSizeROI);

/**
 * Inverse DCT, de-quantization and level shift part of the JPEG decoding.
 * Input is expected in 64x1 macro blocks and output is expected to be in 8x8
 * macro blocks.
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep Image width in pixels x 8 x sizeof(Npp16s).
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep Image width in pixels x 8 x sizeof(Npp16s).
 * \param pQuantInvTable Inverse quantization tables for JPEG decoding created
 *           using nppiQuantInvTableInit_JPEG_8u16u().
 * \param oSizeROI \ref roi_specification.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return Error codes:
 *         - ::NPP_SIZE_ERROR For negative input height/width or not a multiple of
 *           8 width/height.
 *         - ::NPP_STEP_ERROR If input image width is not multiple of 8 or does not
 *           match ROI.
 *         - ::NPP_NULL_POINTER_ERROR If the destination pointer is 0.
 */
NppStatus 
nppiDCTQuantInv8x8LS_JPEG_16s8u_C1R_Ctx(const Npp16s * pSrc, int nSrcStep, 
                                              Npp8u  * pDst, int nDstStep, 
                                        const Npp16u * pQuantInvTable, NppiSize oSizeROI, NppStreamContext nppStreamCtx);
   
NppStatus 
nppiDCTQuantInv8x8LS_JPEG_16s8u_C1R(const Npp16s * pSrc, int nSrcStep, 
                                          Npp8u  * pDst, int nDstStep, 
                                    const Npp16u * pQuantInvTable, NppiSize oSizeROI);
   

#if defined (__cplusplus)
struct NppiDCTState;
#else
typedef struct NppiDCTState NppiDCTState;
#endif


/**
 * Initializes DCT state structure and allocates additional resources.
 *
 * \see nppiDCTQuantFwd8x8LS_JPEG_8u16s_C1R_NEW(), nppiDCTQuantInv8x8LS_JPEG_16s8u_C1R_NEW.
 * 
 * \param ppState Pointer to pointer to DCT state structure. 
 * \param nppStreamCtx \ref application_managed_stream_context. 
 *
 * \return NPP_SUCCESS Indicates no error. Any other value indicates an error
 *         or a warning
 * \return NPP_SIZE_ERROR Indicates an error condition if any image dimension
 *         has zero or negative value
 * \return NPP_NULL_POINTER_ERROR Indicates an error condition if pBufSize
 *         pointer is NULL
 */
NppStatus nppiDCTInitAlloc_Ctx(NppiDCTState** ppState, NppStreamContext nppStreamCtx);

NppStatus nppiDCTInitAlloc(NppiDCTState** ppState);

/**
 * Frees the additional resources of the DCT state structure.
 *
 * \see nppiDCTInitAlloc
 * 
 * \param pState Pointer to DCT state structure. 
 *
 * \return NPP_SUCCESS Indicates no error. Any other value indicates an error
 *         or a warning
 * \return NPP_SIZE_ERROR Indicates an error condition if any image dimension
 *         has zero or negative value
 * \return NPP_NULL_POINTER_ERROR Indicates an error condition if pState
 *         pointer is NULL
 */
NppStatus nppiDCTFree(NppiDCTState* pState);

/**
 * Forward DCT, quantization and level shift part of the JPEG encoding.
 * Input is expected in 8x8 macro blocks and output is expected to be in 64x1
 * macro blocks. The new version of the primitive takes the ROI in image pixel size and
 * works with DCT coefficients that are in zig-zag order.
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep Image width in pixels x 8 x sizeof(Npp16s).
 * \param pQuantizationTable Quantization Table in zig-zag order.
 * \param oSizeROI \ref roi_specification.
 * \param pState Pointer to DCT state structure. This structure must be
 *          initialized allocated and initialized using nppiDCTInitAlloc(). 
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return Error codes:
 *         - ::NPP_SIZE_ERROR For negative input height/width or not a multiple of
 *           8 width/height.
 *         - ::NPP_STEP_ERROR If input image width is not multiple of 8 or does not
 *           match ROI.
 *         - ::NPP_NULL_POINTER_ERROR If the destination pointer is 0.
 */
NppStatus 
nppiDCTQuantFwd8x8LS_JPEG_8u16s_C1R_NEW_Ctx(const Npp8u  * pSrc, int nSrcStep, 
                                                  Npp16s * pDst, int nDstStep, 
                                            const Npp8u * pQuantizationTable, NppiSize oSizeROI,
                                            NppiDCTState* pState, NppStreamContext nppStreamCtx);

NppStatus 
nppiDCTQuantFwd8x8LS_JPEG_8u16s_C1R_NEW(const Npp8u  * pSrc, int nSrcStep, 
                                              Npp16s * pDst, int nDstStep, 
                                        const Npp8u * pQuantizationTable, NppiSize oSizeROI,
                                        NppiDCTState* pState);

/**
 * Inverse DCT, de-quantization and level shift part of the JPEG decoding.
 * Input is expected in 64x1 macro blocks and output is expected to be in 8x8
 * macro blocks. The new version of the primitive takes the ROI in image pixel size and
 * works with DCT coefficients that are in zig-zag order.
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep Image width in pixels x 8 x sizeof(Npp16s).
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param pQuantizationTable Quantization Table in zig-zag order.
 * \param oSizeROI \ref roi_specification.
 * \param pState Pointer to DCT state structure. This structure must be
 *          initialized allocated and initialized using nppiDCTInitAlloc().  
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return Error codes:
 *         - ::NPP_SIZE_ERROR For negative input height/width or not a multiple of
 *           8 width/height.
 *         - ::NPP_STEP_ERROR If input image width is not multiple of 8 or does not
 *           match ROI.
 *         - ::NPP_NULL_POINTER_ERROR If the destination pointer is 0.
 */
NppStatus 
nppiDCTQuantInv8x8LS_JPEG_16s8u_C1R_NEW_Ctx(const Npp16s * pSrc, int nSrcStep, 
                                                  Npp8u  * pDst, int nDstStep, 
                                            const Npp8u * pQuantizationTable, NppiSize oSizeROI,
                                            NppiDCTState* pState, NppStreamContext nppStreamCtx);
                                    
NppStatus 
nppiDCTQuantInv8x8LS_JPEG_16s8u_C1R_NEW(const Npp16s * pSrc, int nSrcStep, 
                                              Npp8u  * pDst, int nDstStep, 
                                        const Npp8u * pQuantizationTable, NppiSize oSizeROI,
                                        NppiDCTState* pState);
                                    
/**
 * Forward DCT, quantization and level shift part of the JPEG encoding, 16-bit short integer.
 * Input is expected in 8x8 macro blocks and output is expected to be in 64x1
 * macro blocks. The new version of the primitive takes the ROI in image pixel size and
 * works with DCT coefficients that are in zig-zag order.
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep Image width in pixels x 8 x sizeof(Npp16s).
 * \param pQuantizationTable Quantization Table in zig-zag order.
 * \param oSizeROI \ref roi_specification.
 * \param pState Pointer to DCT state structure. This structure must be
 *          initialized allocated and initialized using nppiDCTInitAlloc(). 
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return Error codes:
 *         - ::NPP_SIZE_ERROR For negative input height/width or not a multiple of
 *           8 width/height.
 *         - ::NPP_STEP_ERROR If input image width is not multiple of 8 or does not
 *           match ROI.
 *         - ::NPP_NULL_POINTER_ERROR If the destination pointer is 0.
 */
NppStatus
nppiDCTQuant16Fwd8x8LS_JPEG_8u16s_C1R_NEW_Ctx(const Npp8u  * pSrc, int nSrcStep,
                                                    Npp16s * pDst, int nDstStep,
                                              const Npp16u * pQuantizationTable, NppiSize oSizeROI,
                                              NppiDCTState* pState, NppStreamContext nppStreamCtx);

NppStatus
nppiDCTQuant16Fwd8x8LS_JPEG_8u16s_C1R_NEW(const Npp8u  * pSrc, int nSrcStep,
                                                Npp16s * pDst, int nDstStep,
                                          const Npp16u * pQuantizationTable, NppiSize oSizeROI,
                                          NppiDCTState* pState);

/**
 * Inverse DCT, de-quantization and level shift part of the JPEG decoding, 16-bit short integer.
 * Input is expected in 64x1 macro blocks and output is expected to be in 8x8
 * macro blocks. The new version of the primitive takes the ROI in image pixel size and
 * works with DCT coefficients that are in zig-zag order.
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep Image width in pixels x 8 x sizeof(Npp16s).
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param pQuantizationTable Quantization Table in zig-zag order.
 * \param oSizeROI \ref roi_specification.
 * \param pState Pointer to DCT state structure. This structure must be
 *          initialized allocated and initialized using nppiDCTInitAlloc().  
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return Error codes:
 *         - ::NPP_SIZE_ERROR For negative input height/width or not a multiple of
 *           8 width/height.
 *         - ::NPP_STEP_ERROR If input image width is not multiple of 8 or does not
 *           match ROI.
 *         - ::NPP_NULL_POINTER_ERROR If the destination pointer is 0.
 */
NppStatus
nppiDCTQuant16Inv8x8LS_JPEG_16s8u_C1R_NEW_Ctx(const Npp16s * pSrc, int nSrcStep,
                                                    Npp8u  * pDst, int nDstStep,
                                              const Npp16u * pQuantizationTable, NppiSize oSizeROI,
                                              NppiDCTState* pState, NppStreamContext nppStreamCtx);
                                          
NppStatus
nppiDCTQuant16Inv8x8LS_JPEG_16s8u_C1R_NEW(const Npp16s * pSrc, int nSrcStep,
                                                Npp8u  * pDst, int nDstStep,
                                          const Npp16u * pQuantizationTable, NppiSize oSizeROI,
                                          NppiDCTState* pState);
                                          
/** @} image_quantization */

#if defined (__cplusplus)
struct NppiDecodeHuffmanSpec;
#else
typedef struct NppiDecodeHuffmanSpec NppiDecodeHuffmanSpec;
#endif

/**
 * Returns the length of the NppiDecodeHuffmanSpec structure.
 * \param pSize Pointer to a variable that will receive the length of the NppiDecodeHuffmanSpec structure.
 * \return Error codes:
 *         - ::NPP_NULL_POINTER_ERROR If one of the pointers is 0.
**/
NppStatus
nppiDecodeHuffmanSpecGetBufSize_JPEG(int* pSize);

/**
 * Creates a Huffman table in a format that is suitable for the decoder on the host.
 * \param pRawHuffmanTable Huffman table formated as specified in the JPEG standard.
 * \param eTableType Enum specifying type of table (nppiDCTable or nppiACTable).
 * \param pHuffmanSpec Pointer to the Huffman table for the decoder
 * \return Error codes:
 *         - ::NPP_NULL_POINTER_ERROR If one of the pointers is 0.
**/
NppStatus
nppiDecodeHuffmanSpecInitHost_JPEG(const Npp8u* pRawHuffmanTable, NppiHuffmanTableType eTableType, NppiDecodeHuffmanSpec  *pHuffmanSpec);

/**
 * Allocates memory and creates a Huffman table in a format that is suitable for the decoder on the host.
 * \param pRawHuffmanTable Huffman table formated as specified in the JPEG standard.
 * \param eTableType Enum specifying type of table (nppiDCTable or nppiACTable).
 * \param ppHuffmanSpec Pointer to returned pointer to the Huffman table for the decoder
 * \return Error codes:
 *         - ::NPP_NULL_POINTER_ERROR If one of the pointers is 0.
**/
NppStatus
nppiDecodeHuffmanSpecInitAllocHost_JPEG(const Npp8u* pRawHuffmanTable, NppiHuffmanTableType eTableType, NppiDecodeHuffmanSpec  **ppHuffmanSpec);

/**
 * Frees the host memory allocated by nppiDecodeHuffmanSpecInitAllocHost_JPEG.
 * \param pHuffmanSpec Pointer to the Huffman table for the decoder

**/
NppStatus
nppiDecodeHuffmanSpecFreeHost_JPEG(NppiDecodeHuffmanSpec  *pHuffmanSpec);

/**
 * Huffman Decoding of the JPEG decoding on the host.
 * Input is expected in byte stuffed huffman encoded JPEG scan and output is expected to be 64x1 macro blocks.
 *
 * \param pSrc Byte-stuffed huffman encoded JPEG scan.
 * \param nLength Byte length of the input.
 * \param restartInterval Restart Interval, see JPEG standard.
 * \param Ss Start Coefficient, see JPEG standard.
 * \param Se End Coefficient, see JPEG standard.
 * \param Ah Bit Approximation High, see JPEG standard.
 * \param Al Bit Approximation Low, see JPEG standard.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param pHuffmanTableDC DC Huffman table.
 * \param pHuffmanTableAC AC Huffman table.
 * \param oSizeROI \ref roi_specification.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return Error codes:
 *         - ::NPP_SIZE_ERROR For negative input height/width or not a multiple of
 *           8 width/height.
 *         - ::NPP_STEP_ERROR If input image width is not multiple of 8 or does not
 *           match ROI.
 *         - ::NPP_NULL_POINTER_ERROR If the destination pointer is 0.
 */
NppStatus
nppiDecodeHuffmanScanHost_JPEG_8u16s_P1R_Ctx(const Npp8u  * pSrc, Npp32s nLength,
                                             Npp32s restartInterval, Npp32s Ss, Npp32s Se, Npp32s Ah, Npp32s Al,
                                             Npp16s * pDst, Npp32s nDstStep,
                                             NppiDecodeHuffmanSpec  * pHuffmanTableDC, 
                                             NppiDecodeHuffmanSpec  * pHuffmanTableAC, 
                                             NppiSize oSizeROI, NppStreamContext nppStreamCtx); 

NppStatus
nppiDecodeHuffmanScanHost_JPEG_8u16s_P1R(const Npp8u  * pSrc, Npp32s nLength,
                                         Npp32s restartInterval, Npp32s Ss, Npp32s Se, Npp32s Ah, Npp32s Al,
                                         Npp16s * pDst, Npp32s nDstStep,
                                         NppiDecodeHuffmanSpec  * pHuffmanTableDC, 
                                         NppiDecodeHuffmanSpec  * pHuffmanTableAC, 
                                         NppiSize oSizeROI); 

/**
 * Huffman Decoding of the JPEG decoding on the host.
 * Input is expected in byte stuffed huffman encoded JPEG scan and output is expected to be 64x1 macro blocks.
 *
 * \param pSrc Byte-stuffed huffman encoded JPEG scan.
 * \param nLength Byte length of the input.
 * \param nRestartInterval Restart Interval, see JPEG standard. 
 * \param nSs Start Coefficient, see JPEG standard.
 * \param nSe End Coefficient, see JPEG standard.
 * \param nAh Bit Approximation High, see JPEG standard.
 * \param nAl Bit Approximation Low, see JPEG standard.
 * \param apDst \ref destination_image_pointer.
 * \param aDstStep \ref destination_image_line_step.
 * \param apHuffmanDCTable DC Huffman tables.
 * \param apHuffmanACTable AC Huffman tables.
 * \param aSizeROI \ref roi_specification.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return Error codes:
 *         - ::NPP_SIZE_ERROR For negative input height/width or not a multiple of
 *           8 width/height.
 *         - ::NPP_STEP_ERROR If input image width is not multiple of 8 or does not
 *           match ROI.
 *         - ::NPP_NULL_POINTER_ERROR If the destination pointer is 0.
 */
 NppStatus
 nppiDecodeHuffmanScanHost_JPEG_8u16s_P3R_Ctx(const Npp8u * pSrc, Npp32s nLength,
                                              Npp32s nRestartInterval, Npp32s nSs, Npp32s nSe, Npp32s nAh, Npp32s nAl,
                                              Npp16s * apDst[3], Npp32s aDstStep[3],
                                              NppiDecodeHuffmanSpec * apHuffmanDCTable[3], 
                                              NppiDecodeHuffmanSpec * apHuffmanACTable[3], 
                                              NppiSize aSizeROI[3], NppStreamContext nppStreamCtx);

 NppStatus
 nppiDecodeHuffmanScanHost_JPEG_8u16s_P3R(const Npp8u * pSrc, Npp32s nLength,
				                          Npp32s nRestartInterval, Npp32s nSs, Npp32s nSe, Npp32s nAh, Npp32s nAl,
					                      Npp16s * apDst[3], Npp32s aDstStep[3],
					                      NppiDecodeHuffmanSpec * apHuffmanDCTable[3], 
					                      NppiDecodeHuffmanSpec * apHuffmanACTable[3], 
					                      NppiSize aSizeROI[3]);

#if defined (__cplusplus)
struct NppiEncodeHuffmanSpec;
#else
typedef struct NppiEncodeHuffmanSpec NppiEncodeHuffmanSpec;
#endif


/**
 * Returns the length of the NppiEncodeHuffmanSpec structure.
 * \param pSize Pointer to a variable that will receive the length of the NppiEncodeHuffmanSpec structure.
 * \return Error codes:
 *         - ::NPP_NULL_POINTER_ERROR If one of the pointers is 0.
**/
NppStatus
nppiEncodeHuffmanSpecGetBufSize_JPEG(int* pSize);

/**
 * Creates a Huffman table in a format that is suitable for the encoder.
 * \param pRawHuffmanTable Huffman table formated as specified in the JPEG standard.
 * \param eTableType Enum specifying type of table (nppiDCTable or nppiACTable).
 * \param pHuffmanSpec Pointer to the Huffman table for the decoder
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return Error codes:
 *         - ::NPP_NULL_POINTER_ERROR If one of the pointers is 0.
**/
NppStatus
nppiEncodeHuffmanSpecInit_JPEG_Ctx(const Npp8u* pRawHuffmanTable, NppiHuffmanTableType eTableType, NppiEncodeHuffmanSpec  *pHuffmanSpec, NppStreamContext nppStreamCtx);

NppStatus
nppiEncodeHuffmanSpecInit_JPEG(const Npp8u* pRawHuffmanTable, NppiHuffmanTableType eTableType, NppiEncodeHuffmanSpec  *pHuffmanSpec);

/**
 * Allocates memory and creates a Huffman table in a format that is suitable for the encoder.
 * \param pRawHuffmanTable Huffman table formated as specified in the JPEG standard.
 * \param eTableType Enum specifying type of table (nppiDCTable or nppiACTable).
 * \param ppHuffmanSpec Pointer to returned pointer to the Huffman table for the encoder
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return Error codes:
 *         - ::NPP_NULL_POINTER_ERROR If one of the pointers is 0.
**/
NppStatus
nppiEncodeHuffmanSpecInitAlloc_JPEG_Ctx(const Npp8u* pRawHuffmanTable, NppiHuffmanTableType eTableType, NppiEncodeHuffmanSpec  **ppHuffmanSpec, NppStreamContext nppStreamCtx);

NppStatus
nppiEncodeHuffmanSpecInitAlloc_JPEG(const Npp8u* pRawHuffmanTable, NppiHuffmanTableType eTableType, NppiEncodeHuffmanSpec  **ppHuffmanSpec);

/**
 * Frees the memory allocated by nppiEncodeHuffmanSpecInitAlloc_JPEG.
 * \param pHuffmanSpec Pointer to the Huffman table for the encoder

**/
NppStatus
nppiEncodeHuffmanSpecFree_JPEG(NppiEncodeHuffmanSpec  *pHuffmanSpec);

/**
 * Huffman Encoding of the JPEG Encoding.
 * Input is expected to be 64x1 macro blocks and output is expected as byte stuffed huffman encoded JPEG scan.
 *
 * \param pSrc \ref destination_image_pointer.
 * \param nSrcStep \ref destination_image_line_step.
 * \param nRestartInterval Restart Interval, see JPEG standard. Currently only values <=0 are supported.
 * \param nSs Start Coefficient, see JPEG standard.
 * \param nSe End Coefficient, see JPEG standard.
 * \param nAh Bit Approximation High, see JPEG standard.
 * \param nAl Bit Approximation Low, see JPEG standard.
 * \param pDst Byte-stuffed huffman encoded JPEG scan.
 * \param nLength Byte length of the huffman encoded JPEG scan.
 * \param pHuffmanTableDC DC Huffman table.
 * \param pHuffmanTableAC AC Huffman table.
 * \param oSizeROI \ref roi_specification.
 * \param pTempStorage Temporary storage.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return Error codes:
 *         - ::NPP_SIZE_ERROR For negative input height/width or not a multiple of
 *           8 width/height.
 *         - ::NPP_STEP_ERROR If input image width is not multiple of 8 or does not
 *           match ROI.
 *         - ::NPP_NULL_POINTER_ERROR If the destination pointer is 0.
 *         - ::NPP_NOT_SUFFICIENT_COMPUTE_CAPABILITY If the device has compute capability < 2.0. 
 */
NppStatus
nppiEncodeHuffmanScan_JPEG_8u16s_P1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep,
                                         Npp32s nRestartInterval, Npp32s nSs, Npp32s nSe, Npp32s nAh, Npp32s nAl,
                                         Npp8u * pDst, Npp32s* nLength,
                                         NppiEncodeHuffmanSpec  * pHuffmanTableDC, 
                                         NppiEncodeHuffmanSpec  * pHuffmanTableAC, 
                                         NppiSize oSizeROI,
                                         Npp8u* pTempStorage, NppStreamContext nppStreamCtx); 

NppStatus
nppiEncodeHuffmanScan_JPEG_8u16s_P1R(const Npp16s * pSrc, Npp32s nSrcStep,
                                     Npp32s nRestartInterval, Npp32s nSs, Npp32s nSe, Npp32s nAh, Npp32s nAl,
                                     Npp8u * pDst, Npp32s* nLength,
                                     NppiEncodeHuffmanSpec  * pHuffmanTableDC, 
                                     NppiEncodeHuffmanSpec  * pHuffmanTableAC, 
                                     NppiSize oSizeROI,
                                     Npp8u* pTempStorage); 

/**
 * Huffman Encoding of the JPEG Encoding.
 * Input is expected to be 64x1 macro blocks and output is expected as byte stuffed huffman encoded JPEG scan.
 *
 * \param apSrc \ref destination_image_pointer.
 * \param aSrcStep \ref destination_image_line_step.
 * \param nRestartInterval Restart Interval, see JPEG standard. Currently only values <=0 are supported.
 * \param nSs Start Coefficient, see JPEG standard.
 * \param nSe End Coefficient, see JPEG standard.
 * \param nAh Bit Approximation High, see JPEG standard.
 * \param nAl Bit Approximation Low, see JPEG standard.
 * \param pDst Byte-stuffed huffman encoded JPEG scan.
 * \param nLength Byte length of the huffman encoded JPEG scan.
 * \param apHuffmanDCTable DC Huffman tables.
 * \param apHuffmanACTable AC Huffman tables.
 * \param aSizeROI \ref roi_specification.
 * \param pTempStorage Temporary storage.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return Error codes:
 *         - ::NPP_SIZE_ERROR For negative input height/width or not a multiple of
 *           8 width/height.
 *         - ::NPP_STEP_ERROR If input image width is not multiple of 8 or does not
 *           match ROI.
 *         - ::NPP_NULL_POINTER_ERROR If the destination pointer is 0.
 *         - ::NPP_NOT_SUFFICIENT_COMPUTE_CAPABILITY If the device has compute capability < 2.0.
 */
 NppStatus
 nppiEncodeHuffmanScan_JPEG_8u16s_P3R_Ctx(Npp16s * apSrc[3], Npp32s aSrcStep[3],
                                          Npp32s nRestartInterval, Npp32s nSs, Npp32s nSe, Npp32s nAh, Npp32s nAl,
                                          Npp8u  * pDst, Npp32s* nLength,
                                          NppiEncodeHuffmanSpec * apHuffmanDCTable[3], 
                                          NppiEncodeHuffmanSpec * apHuffmanACTable[3], 
                                          NppiSize aSizeROI[3],
                                          Npp8u* pTempStorage, NppStreamContext nppStreamCtx);

NppStatus
 nppiEncodeHuffmanScan_JPEG_8u16s_P3R(Npp16s * apSrc[3], Npp32s aSrcStep[3],
                                      Npp32s nRestartInterval, Npp32s nSs, Npp32s nSe, Npp32s nAh, Npp32s nAl,
                                      Npp8u  * pDst, Npp32s* nLength,
                                      NppiEncodeHuffmanSpec * apHuffmanDCTable[3], 
                                      NppiEncodeHuffmanSpec * apHuffmanACTable[3], 
                                      NppiSize aSizeROI[3],
                                      Npp8u* pTempStorage);

/**
 * Optimize Huffman Encoding of the JPEG Encoding.
 * Input is expected to be 64x1 macro blocks and output is expected as byte stuffed huffman encoded JPEG scan.
 *
 * \param pSrc \ref destination_image_pointer.
 * \param nSrcStep \ref destination_image_line_step.
 * \param nRestartInterval Restart Interval, see JPEG standard. Currently only values <=0 are supported.
 * \param nSs Start Coefficient, see JPEG standard.
 * \param nSe End Coefficient, see JPEG standard.
 * \param nAh Bit Approximation High, see JPEG standard.
 * \param nAl Bit Approximation Low, see JPEG standard.
 * \param pDst Byte-stuffed huffman encoded JPEG scan.
 * \param pLength Pointer to the byte length of the huffman encoded JPEG scan.
 * \param hpCodesDC Host pointer to the code of the huffman tree for DC component.
 * \param hpTableDC Host pointer to the table of the huffman tree for DC component.
 * \param hpCodesAC Host pointer to the code of the huffman tree for AC component.
 * \param hpTableAC Host pointer to the table of the huffman tree for AC component.
 * \param pHuffmanDCTable DC Huffman table.
 * \param pHuffmanACTable AC Huffman table.
 * \param oSizeROI \ref roi_specification.
 * \param pTempStorage Temporary storage.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return Error codes:
 *         - ::NPP_SIZE_ERROR For negative input height/width or not a multiple of
 *           8 width/height.
 *         - ::NPP_STEP_ERROR If input image width is not multiple of 8 or does not
 *           match ROI.
 *         - ::NPP_NULL_POINTER_ERROR If the destination pointer is 0.
 *         - ::NPP_NOT_SUFFICIENT_COMPUTE_CAPABILITY If the device has compute capability < 2.0. 
 */
NppStatus
nppiEncodeOptimizeHuffmanScan_JPEG_8u16s_P1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep,
                                                 Npp32s nRestartInterval, Npp32s nSs, 
                                                 Npp32s nSe, Npp32s nAh, Npp32s nAl,
                                                 Npp8u * pDst, Npp32s * pLength,
                                                 Npp8u * hpCodesDC, Npp8u * hpTableDC,
                                                 Npp8u * hpCodesAC, Npp8u * hpTableAC,
                                                 NppiEncodeHuffmanSpec * pHuffmanDCTable, 
                                                 NppiEncodeHuffmanSpec * pHuffmanACTable, 
                                                 NppiSize oSizeROI, Npp8u * pTempStorage, NppStreamContext nppStreamCtx);

NppStatus
nppiEncodeOptimizeHuffmanScan_JPEG_8u16s_P1R(const Npp16s * pSrc, Npp32s nSrcStep,
                                             Npp32s nRestartInterval, Npp32s nSs, 
                                             Npp32s nSe, Npp32s nAh, Npp32s nAl,
                                             Npp8u * pDst, Npp32s * pLength,
                                             Npp8u * hpCodesDC, Npp8u * hpTableDC,
                                             Npp8u * hpCodesAC, Npp8u * hpTableAC,
                                             NppiEncodeHuffmanSpec * pHuffmanDCTable, 
                                             NppiEncodeHuffmanSpec * pHuffmanACTable, 
                                             NppiSize oSizeROI, Npp8u * pTempStorage);

/**
 * Optimize Huffman Encoding of the JPEG Encoding.
 * Input is expected to be 64x1 macro blocks and output is expected as byte stuffed huffman encoded JPEG scan.
 *
 * \param apSrc \ref destination_image_pointer.
 * \param aSrcStep \ref destination_image_line_step.
 * \param nRestartInterval Restart Interval, see JPEG standard. Currently only values <=0 are supported.
 * \param nSs Start Coefficient, see JPEG standard.
 * \param nSe End Coefficient, see JPEG standard.
 * \param nAh Bit Approximation High, see JPEG standard.
 * \param nAl Bit Approximation Low, see JPEG standard.
 * \param pDst Byte-stuffed huffman encoded JPEG scan.
 * \param pLength Pointer to the byte length of the huffman encoded JPEG scan.
 * \param hpCodesDC Host pointer to the code of the huffman tree for DC component.
 * \param hpTableDC Host pointer to the table of the huffman tree for DC component.
 * \param hpCodesAC Host pointer to the code of the huffman tree for AC component.
 * \param hpTableAC Host pointer to the table of the huffman tree for AC component.
 * \param apHuffmanDCTable DC Huffman tables.
 * \param apHuffmanACTable AC Huffman tables.
 * \param oSizeROI \ref roi_specification.
 * \param pTempStorage Temporary storage.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return Error codes:
 *         - ::NPP_SIZE_ERROR For negative input height/width or not a multiple of
 *           8 width/height.
 *         - ::NPP_STEP_ERROR If input image width is not multiple of 8 or does not
 *           match ROI.
 *         - ::NPP_NULL_POINTER_ERROR If the destination pointer is 0.
 *         - ::NPP_NOT_SUFFICIENT_COMPUTE_CAPABILITY If the device has compute capability < 2.0.
 */
NppStatus
nppiEncodeOptimizeHuffmanScan_JPEG_8u16s_P3R_Ctx(Npp16s * apSrc[3], Npp32s aSrcStep[3],
                                                 Npp32s nRestartInterval, Npp32s nSs, 
                                                 Npp32s nSe, Npp32s nAh, Npp32s nAl,
                                                 Npp8u * pDst, Npp32s * pLength,
                                                 Npp8u * hpCodesDC[3], Npp8u * hpTableDC[3],
                                                 Npp8u * hpCodesAC[3], Npp8u * hpTableAC[3],
                                                 NppiEncodeHuffmanSpec * apHuffmanDCTable[3], 
                                                 NppiEncodeHuffmanSpec * apHuffmanACTable[3], 
                                                 NppiSize oSizeROI[3], Npp8u * pTempStorage, NppStreamContext nppStreamCtx);

NppStatus
nppiEncodeOptimizeHuffmanScan_JPEG_8u16s_P3R(Npp16s * apSrc[3], Npp32s aSrcStep[3],
                                             Npp32s nRestartInterval, Npp32s nSs, 
                                             Npp32s nSe, Npp32s nAh, Npp32s nAl,
                                             Npp8u * pDst, Npp32s * pLength,
                                             Npp8u * hpCodesDC[3], Npp8u * hpTableDC[3],
                                             Npp8u * hpCodesAC[3], Npp8u * hpTableAC[3],
                                             NppiEncodeHuffmanSpec * apHuffmanDCTable[3], 
                                             NppiEncodeHuffmanSpec * apHuffmanACTable[3], 
                                             NppiSize oSizeROI[3], Npp8u * pTempStorage);

/**
 * Calculates the size of the temporary buffer for baseline Huffman encoding.
 *
 * \see nppiEncodeHuffmanScan_JPEG_8u16s_P1R(), nppiEncodeHuffmanScan_JPEG_8u16s_P3R().
 * 
 * \param oSize Image Dimension.
 * \param nChannels Number of channels in the image.
 * \param pBufSize Pointer to variable that returns the size of the
 *        temporary buffer. 
 *
 * \return NPP_SUCCESS Indicates no error. Any other value indicates an error
 *         or a warning
 * \return NPP_SIZE_ERROR Indicates an error condition if any image dimension
 *         has zero or negative value
 * \return NPP_NULL_POINTER_ERROR Indicates an error condition if pBufSize
 *         pointer is NULL
 */
NppStatus nppiEncodeHuffmanGetSize(NppiSize oSize, int nChannels, size_t * pBufSize);

/**
 * Calculates the size of the temporary buffer for optimize Huffman coding.
 *
 * 
 * \param oSize Image Dimension.
 * \param nChannels Number of channels in the image.
 * \param pBufSize Pointer to variable that returns the size of the
 *        temporary buffer. 
 *
 * \return NPP_SUCCESS Indicates no error. Any other value indicates an error
 *         or a warning
 * \return NPP_SIZE_ERROR Indicates an error condition if any image dimension
 *         has zero or negative value
 * \return NPP_NULL_POINTER_ERROR Indicates an error condition if pBufSize
 *         pointer is NULL
 */
NppStatus nppiEncodeOptimizeHuffmanGetSize(NppiSize oSize, int nChannels, int * pBufSize);


/**
 * @name Hybrid CPU+GPU JPEG Huffman decoding
 *
 * These functions and structs are used for Huffman decoding part of JPEG decode pipeline.
 * It uses hybrid CPU + GPU approach.
 *
 * See \ref nppiJpegDecodeJob for more documentation and example
 */
/*@{*/

/**
 * JPEG frame descriptor.
 *
 * Can hold from 1 to 4 components.
 */
typedef struct {
    Npp8u nComponents; /**< Number of components in frame */
    NppiSize oSizeInBlocks; /**< Size of component with 1x1 subsampling (usually luma) in DCT blocks. */
    NppiSize aComponentSubsampling[4]; /**< Subsampling factors of component, as described in frame header */
    Npp16s * apComponentBuffer[4]; /**<
        Buffer containing DCT coefficients. Use \ref nppiJpegDecodeGetDCTBufferSize to
        determine size of this buffer. After decoding, coefficients will be stored in
        zig-zag order, block by block. So the c-th coeffient of block `(x, y)` will
        be stored at `buffer[64 * (y * interleavedComponentWidthInBlocks + x) + c]`.
    */
} NppiJpegFrameDescr; 

/// JPEG scan descriptor
typedef struct {
    Npp8u nComponents; /**< Number of components present in scan */
    Npp8u aComponentIdx[4]; /**< Frame-indexes of components.
        These values will be used to index arrays in \ref NppiJpegFrameDescr */
    Npp8u aComponentDcHtSel[4]; /**< DC Huffman table selector per component */
    Npp8u aComponentAcHtSel[4]; /**< AC Huffman table selector per component */
    const Npp8u * apRawDcHtTable[4]; /**< Pointers to DC Huffman table description in the raw format
        (the same format as used in JPEG header).
        This array will be indexed by \ref aComponentDcHtSel. Pointers for
        tables unused in scan may be set to NULL. */
    const Npp8u * apRawAcHtTable[4]; /**< See \ref apRawDcHtTable */
    Npp8u nSs; /**< Start of spectral selection (index of first coefficient), 0-63 */
    Npp8u nSe; /**< End of spectral selection (index of first coefficient), 0-63 */
    Npp8u nAh; /**< Successive approximation bit position high */
    Npp8u nAl; /**< Successive approximation bit position low */
    Npp32s restartInterval; /**< Restart interval in MCUs. Use 0 or -1 when none */
    Npp32s length; /**< Length of compressed (encoded) scan data */
} NppiJpegScanDescr;

/**
 * Type of job to execute. Usually you will need just SIMPLE
 * for each scan, one MEMZERO at the beginning and FINALIZE at the end.
 * See the example in \ref nppiJpegDecodeJob
 *
 * SIMPLE can be split into multiple jobs: PRE, CPU & GPU.
 * Please note that if you don't use SIMPLE,
 * you man need to add some memcopies and synchronizes as
 * described in \ref nppiJpegDecodeJob.
 *
 * \sa nppiJpegDecodeJob
 */
enum NppiJpegDecodeJobKind {
    NPPI_JPEG_DECODE_SIMPLE, /**< Decode whole scan using a single job */

    // SIMPLE can be split into:
    /*@{*/
    NPPI_JPEG_DECODE_PRE, /**< Preprocessing scan on GPU */
    NPPI_JPEG_DECODE_CPU, /**< Part of decoding run on CPU */
    NPPI_JPEG_DECODE_GPU, /**< Part of decoding run on GPU */
    /*@}*/

    NPPI_JPEG_DECODE_MEMZERO, /**< Zeroing memory before decoding */
    NPPI_JPEG_DECODE_FINALIZE /**< Change memory representation of DCT coefficients to final */
};

/**
 * JPEG decode job used by \ref nppiJpegDecodeJob (see that for more documentation)
 *
 * The job describes piece of computation to be done.
 */
typedef struct {
    NppiJpegFrameDescr * pFrame; /**< This field and its contents are never written */
    NppiJpegScanDescr * pScan; /**< This field is never written. `*pScan` is written
                                 only by ...Create... functions */
    enum NppiJpegDecodeJobKind eKind;
} NppiJpegDecodeJob;

/** Number of additional buffers that may be used by JPEG decode jobs.
 * This number may change in the future, but it remain small.
 *
 * \sa NppiJpegDecodeJobMemory
 */
#define NPPI_JPEG_DECODE_N_BUFFERS 3

/**
 * Memory buffers used by one decode job.
 *
 * \sa nppiJpegDecodeJobMemorySize
 * \sa nppiJpegDecodeJob
 */
typedef struct {
    const Npp8u * pCpuScan;
    /**< Pointer to host memory containing compressed scan data.
     * Should be allocated with additional \ref nppiJpegDecodeGetScanDeadzoneSize
     * bytes of usable memory after the end of compressed scan data.
     * Should be filled by caller. */

    Npp8u * pGpuScan;
    /**< Pointer to device memory used for compressed scan data.
     * Should be allocated with additional \ref nppiJpegDecodeGetScanDeadzoneSize
     * bytes of usable memory after the end of compressed scan data.
     * Should be filled by caller.
     * This buffer may be overwritten by the decoder.
     * Could be NULL for \ref NPPI_JPEG_DECODE_CPU. */

    void * apCpuBuffer[NPPI_JPEG_DECODE_N_BUFFERS];
    /**< Pointers to additional host buffers used by job. Call \ref nppiJpegDecodeJobMemorySize
     * to query sizes of these buffers. `apCpuBuffer[i]` should point to
     * at least `aSize[i]` bytes. If `aSize[i] == 0`, the pointer should be set to NULL. */

    void * apGpuBuffer[NPPI_JPEG_DECODE_N_BUFFERS];
    /**< Pointers to additional device buffers used by job.
     * Minimal sizes of buffers should be the same as the sizes of \ref apCpuBuffer. */
} NppiJpegDecodeJobMemory;

/**
 * Calculates sizes of additional buffers used by the job.
 *
 * \param pJob has to point to properly initialized job
 * \param[out] aSize will be filled with \ref NPPI_JPEG_DECODE_N_BUFFERS sizes,
 * `aSize[i]` telling how much memory has to be allocated for `apCpuBuffer[i]` and `apGpuBuffer[i]`.
 *
 * \sa NppiJpegDecodeJobMemory
 *
 * \returns
 * * \ref NPP_SUCCESS
 * * \ref NPP_BAD_ARGUMENT_ERROR when the scan doesn't represent valid JPEG scan
 * * \ref NPP_NULL_POINTER_ERROR when one of necessary arguments is NULL
 */
NppStatus nppiJpegDecodeJobMemorySize(const NppiJpegDecodeJob * pJob, size_t * aSize);

/**
 * Executes a job -- part of decoding.
 *
 * \param pJob has to be initialized by \ref nppiJpegDecodeJobCreateMemzero
 * or \ref nppiJpegDecodeJobCreateFinalize or manually.
 * \param pMemory has to point to valid structure, except for MEMZERO and FINALIZE
 * scans, for which it can be NULL.
 *
 * \sa nppiJpegDecodeJobMemorySize
 * \sa nppiJpegDecodeJobCreateMemzero
 * \sa nppiJpegDecodeJobCreateFinalize
 *
 * This function can be used in two ways, depending on how much control do you need:
 * 1. Decode whole scan in one job. That is accomplished when the job.eKind is
 *    set to \ref NPPI_JPEG_DECODE_SIMPLE. Nppi function handless all in-scan synchronization
 *    (if needed).
 *    This is described in example.
 * 2. Split decoding of scan into multiple jobs. In this case, caller is responsible
 *    necessary synchronizations. If multiple jobs
 *    from the same scan claim to use the same additional buffer, it means that
 *    the jobs are exchaning information throught this buffer and the buffer should not be
 *    reused or reallocated meanwhile, additionaly if the buffer is used first by PRE job,
 *    and then by CPU job, caller has to call cudaStreamSynchronize before CPU job.
 *    \ref NppiJpegDecodeJobKind for more information.
 *
 * Example (pseudo)code for decoding JPEG:
 * (Error handling code omitted for brevity. Each function which may return
 * an error should be checked)
 * @code
 // Allocate DCT buffers (using nppiJpegDecodeGetDCTBufferSize)
 
 NppiJpegFrameDescr frame;
 // Fill frame info...
 NppiJpegScanDescr scan;
 NppiJpegDecodeJob job;
 job.pFrame = frame;
 job.pScan = scan;
 
 nppiJpegDecodeJobCreateMemzero(&job);
 nppiJpegDecodeJob(&job, NULL);
 for (int i = 0; i < nScans; ++i)
 {
     // Fill scan info...
     job.eKind = NPPI_JPEG_DECODE_SIMPLE;
 
     size_t sizes[NPPI_JPEG_DECODE_N_BUFFERS];
     nppiJpegDecodeJobMemorySize(&job, sizes);
 
     NppiJpegJobMemory memory;
     // Allocate and fill scan buffers (using nppiJpegDecodeGetScanBufferSize)...
     // Fill the memory struct according to sizes...
 
     nppiJpegDecodeJob(&job, &memory);
 
     // Synchronization is needed only if you reuse buffers between scans. 
     cudaStreamSynchronize(nppGetStream()); or CudaStreamSynchronize(nppStreamCtx.hStream);
 }
 
 nppiJpegDecodeJobCreateFinalize(&job);
 nppiJpegDecodeJob(&job, NULL);
 
 // Perform further steps of decoding (iDCT etc.)
 * @endcode
 *
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \returns
 * * \ref NPP_SUCCESS
 * * \ref NPP_BAD_ARGUMENT_ERROR when the scan doesn't represent valid JPEG scan
 * * \ref NPP_NULL_POINTER_ERROR when one of necessary arguments of buffers is NULL
 * * \ref NPP_ERROR when encountered CUDA error
 */
NppStatus nppiJpegDecodeJob_Ctx(const NppiJpegDecodeJob * pJob, const NppiJpegDecodeJobMemory * pMemory, NppStreamContext nppStreamCtx);

NppStatus nppiJpegDecodeJob(const NppiJpegDecodeJob * pJob, const NppiJpegDecodeJobMemory * pMemory);

/**
 * Initializes a job that has to be called at the beginning of decoding.
 *
 * \param pJob:
 * `pJob.pFrame` should point to valid frame description.
 * `*pJob.pScan` will be overwritten.
 *
 * If the caller had manually zeroed the memory for DCT buffers,
 * (note: whole \ref nppiJpegDecodeGetDCTBufferSize has to be zeroed).
 * this job doesn't have to be executed.
 */
NppStatus nppiJpegDecodeJobCreateMemzero(NppiJpegDecodeJob * pJob);

/**
 * Initializes a job that has to be called at the end of decoding,
 * in order to convert temporary representation of DCT coefficients
 * to the final one.
 *
 * \param pJob:
 * pJob.pFrame should point to valid frame description
 * *pJob.pScan will be overwritten.
 */
NppStatus nppiJpegDecodeJobCreateFinalize(NppiJpegDecodeJob * pJob);

/**
 * This function returns how much additional memory has to be available
 * after the end of compressed scan data.
 *
 * The following buffers: `pCpuScan` and `pGpuScan` in \ref NppiJpegDecodeJobMemory
 * should have size at least `pScan->length + nppiJpegDecodeGetScanDeadzoneSize()`.
 *
 * The additional memory is needed because the decoder may perform
 * some speculative reads after the end of compressed scan data.
 */
size_t nppiJpegDecodeGetScanDeadzoneSize(void);

/**
 * Returns how much memory has to be allocated for DCT coefficient buffers
 * declared in \ref NppiJpegDecodeJobMemory. The returned value may be bigger than
 * simply `number of blocks * 64 * sizeof (short)`, because decoder
 * may use slightly bigger temporary representation of data.
 *
 * \param oBlocks Size of the interleaved component in blocks.
 * (That means that the size of component in blocks has to be aligned according
 * to subsampling of this component and frame).
 * 
 * This function assumes no ununsed space between rows,
 * so at the end stride = `oBlocks.width * 64 * sizeof (short)`.
 */
size_t nppiJpegDecodeGetDCTBufferSize(NppiSize oBlocks);

/*@}*/

/**
 * Inverse DCT in WebP decoding. Input is the bitstream that contains the coefficients of 16x16 blocks.
 * These coefficients are based on a 4x4 sub-block unit, e.g., 
 * Coeffs in 0th 4x4 block, 1st 4x4 block 2nd 4x4 block, etc.
 * Output is the coefficients after inverse DCT transform.
 * The output is put in an image format (i.e. raster scan order), different from the input order.
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return Error codes:
 *         - ::NPP_SIZE_ERROR For negative input height/width or not a multiple of
 *           8 width/height.
 *         - ::NPP_STEP_ERROR If input image width is not multiple of 8 or does not
 *           match ROI.
 *         - ::NPP_NULL_POINTER_ERROR If the destination pointer is 0.
 */
NppStatus nppiDCTInv4x4_WebP_16s_C1R_Ctx(const Npp16s * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus nppiDCTInv4x4_WebP_16s_C1R(const Npp16s * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI);

/** @} image_compression */


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* NV_NPPI_COMPRESSION_FUNCTIONS_H */
