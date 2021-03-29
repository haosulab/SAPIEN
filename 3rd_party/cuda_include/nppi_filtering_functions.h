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
#ifndef NV_NPPI_FILTERING_FUNCTIONS_H
#define NV_NPPI_FILTERING_FUNCTIONS_H
 
/**
 * \file nppi_filtering_functions.h
 * NPP Image Processing Functionality.
 */
 
#include "nppdefs.h"


#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup image_filtering_functions Filtering Functions
 *  @ingroup nppi
 *
 * Linear and non-linear image filtering functions.
 *
 * Filtering functions are classified as \ref neighborhood_operations. It is the user's 
 * responsibility to avoid \ref sampling_beyond_image_boundaries. 
 *
 * @{
 *
 * These functions can be found in the nppif library. Linking to only the sub-libraries that you use can significantly
 * save link time, application load time, and CUDA runtime startup time when using dynamic libraries.
 *
 */

/** @defgroup image_1D_linear_filter 1D Linear Filter
 * The set of 1D linear filtering functions available in the library.
 * @{
 *
 */

/** @defgroup image_filter_column FilterColumn
 * Apply convolution filter with user specified 1D column of weights.
 *  
 * Result pixel is equal to the sum of the products between the kernel
 * coefficients (pKernel array) and corresponding neighboring column pixel
 * values in the source image defined by nKernelDim and nAnchorY, divided by
 * nDivisor. 
 * 
 * <h3><a name="CommonFilterColumnParameters">Common parameters for nppiFilterColumn functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oROI \ref roi_specification.
 * \param pKernel Pointer to the start address of the kernel coefficient array.
 *                Coefficients are expected to be stored in reverse order.
 * \param nMaskSize Length of the linear kernel array.
 * \param nAnchor Y offset of the kernel origin frame of reference relative to the
 *                 source pixel.
 * \param nDivisor The factor by which the convolved summation from the Filter
 *                 operation should be divided.  If equal to the sum of
 *                 coefficients, this will keep the maximum result value within
 *                 full scale.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */


/**
 * 8-bit unsigned single-channel 1D column convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnParameters">Common parameters for nppiFilterColumn functions</a>.
 */
NppStatus 
nppiFilterColumn_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                            const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                        const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 8-bit unsigned three-channel 1D column convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnParameters">Common parameters for nppiFilterColumn functions</a>.
 */
NppStatus 
nppiFilterColumn_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                            const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                        const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 8-bit unsigned four-channel 1D column convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnParameters">Common parameters for nppiFilterColumn functions</a>.
 */
NppStatus 
nppiFilterColumn_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                            const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                        const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 8-bit unsigned four-channel 1D column convolution ignoring alpha-channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnParameters">Common parameters for nppiFilterColumn functions</a>.
 */
NppStatus 
nppiFilterColumn_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                             const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                         const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 16-bit unsigned single-channel 1D column convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnParameters">Common parameters for nppiFilterColumn functions</a>.
 */
NppStatus 
nppiFilterColumn_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                             const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                         const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 16-bit unsigned three-channel 1D column convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnParameters">Common parameters for nppiFilterColumn functions</a>.
 */
NppStatus 
nppiFilterColumn_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                             const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                         const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 16-bit unsigned four-channel 1D column convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnParameters">Common parameters for nppiFilterColumn functions</a>.
 */
NppStatus 
nppiFilterColumn_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                             const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                         const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 16-bit unsigned four-channel 1D column convolution ignoring alpha-channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnParameters">Common parameters for nppiFilterColumn functions</a>.
 */
NppStatus 
nppiFilterColumn_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                              const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                          const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 16-bit single-channel 1D column convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnParameters">Common parameters for nppiFilterColumn functions</a>.
 */
NppStatus 
nppiFilterColumn_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                             const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                         const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 16-bit three-channel 1D column convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnParameters">Common parameters for nppiFilterColumn functions</a>.
 */
NppStatus 
nppiFilterColumn_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                             const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                         const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 16-bit four-channel 1D column convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnParameters">Common parameters for nppiFilterColumn functions</a>.
 */
NppStatus 
nppiFilterColumn_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                             const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                         const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 16-bit four-channel 1D column convolution ignoring alpha-channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnParameters">Common parameters for nppiFilterColumn functions</a>.
 */
NppStatus 
nppiFilterColumn_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                              const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                          const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 32-bit float single-channel 1D column convolution.
 *
 * For common parameter descriptions, see <a href="#CommonFilterColumnParameters">Common parameters for nppiFilterColumn functions</a>.
 */
NppStatus 
nppiFilterColumn_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                             const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                         const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 32-bit float three-channel 1D column convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnParameters">Common parameters for nppiFilterColumn functions</a>.
 */
NppStatus 
nppiFilterColumn_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                             const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                         const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 32-bit float four-channel 1D column convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnParameters">Common parameters for nppiFilterColumn functions</a>.
 */
NppStatus 
nppiFilterColumn_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                             const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                         const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 32-bit float four-channel 1D column convolution ignoring alpha-channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnParameters">Common parameters for nppiFilterColumn functions</a>.
 */
NppStatus 
nppiFilterColumn_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                              const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                          const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 64-bit float single-channel 1D column convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnParameters">Common parameters for nppiFilterColumn functions</a>.
 */
NppStatus 
nppiFilterColumn_64f_C1R_Ctx(const Npp64f * pSrc, Npp32s nSrcStep, Npp64f * pDst, Npp32s nDstStep, NppiSize oROI, 
                             const Npp64f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn_64f_C1R(const Npp64f * pSrc, Npp32s nSrcStep, Npp64f * pDst, Npp32s nDstStep, NppiSize oROI, 
                         const Npp64f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);


/** @} image_filter_column */

/** @defgroup image_filter_column_border FilterColumnBorder
 * General purpose 1D convolution column filter with border control.
 *
 * Pixels under the mask are multiplied by the respective weights in the mask
 * and the results are summed. Before writing the result pixel the sum is scaled
 * back via division by nDivisor. If any portion of the mask overlaps the source
 * image boundary the requested border type operation is applied to all mask pixels
 * which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.
 *
 * <h3><a name="CommonFilterColumnBorderParameters">Common parameters for nppiFilterColumnBorder functions include:</a></h3>
 *
 * \param pSrc  \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param pKernel Pointer to the start address of the kernel coefficient array.
 *        Coeffcients are expected to be stored in reverse order.
 * \param nMaskSize Width of the kernel.
 * \param nAnchor X offset of the kernel origin frame of reference
 *        relative to the source pixel.
 * \param nDivisor The factor by which the convolved summation from the Filter
 *        operation should be divided.  If equal to the sum of coefficients,
 *        this will keep the maximum result value within full scale.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned 1D column convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorderParameters">Common parameters for nppiFilterColumnBorder functions</a>.
 */
NppStatus 
nppiFilterColumnBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                  const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
  
NppStatus 
nppiFilterColumnBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                              const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);
  
/**
 * Three channel 8-bit unsigned 1D column convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorderParameters">Common parameters for nppiFilterColumnBorder functions</a>.
 */
NppStatus 
nppiFilterColumnBorder_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                  const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
                 
NppStatus 
nppiFilterColumnBorder_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                              const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);
                 
/**
 * Four channel channel 8-bit unsigned 1D column convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorderParameters">Common parameters for nppiFilterColumnBorder functions</a>.
 */
NppStatus 
nppiFilterColumnBorder_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                  const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumnBorder_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                              const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned convolution 1D column filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorderParameters">Common parameters for nppiFilterColumnBorder functions</a>.
 */
NppStatus 
nppiFilterColumnBorder_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                   const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumnBorder_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);

/**
 * Single channel 16-bit unsigned convolution 1D column filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorderParameters">Common parameters for nppiFilterColumnBorder functions</a>.
 */
NppStatus 
nppiFilterColumnBorder_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                   const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
  
NppStatus 
nppiFilterColumnBorder_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);
  
/**
 * Three channel 16-bit unsigned 1D column convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorderParameters">Common parameters for nppiFilterColumnBorder functions</a>.
 */
NppStatus 
nppiFilterColumnBorder_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                   const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
                 
NppStatus 
nppiFilterColumnBorder_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);
                 
/**
 * Four channel channel 16-bit 1D column unsigned convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorderParameters">Common parameters for nppiFilterColumnBorder functions</a>.
 */
NppStatus 
nppiFilterColumnBorder_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                   const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumnBorder_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned 1D column convolution filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorderParameters">Common parameters for nppiFilterColumnBorder functions</a>.
 */
NppStatus 
nppiFilterColumnBorder_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                    const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumnBorder_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);

/**
 * Single channel 16-bit 1D column convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorderParameters">Common parameters for nppiFilterColumnBorder functions</a>.
 */
NppStatus 
nppiFilterColumnBorder_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                   const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
  
NppStatus 
nppiFilterColumnBorder_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);
  
/**
 * Three channel 16-bit 1D column convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorderParameters">Common parameters for nppiFilterColumnBorder functions</a>.
 */
NppStatus 
nppiFilterColumnBorder_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                   const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
                 
NppStatus 
nppiFilterColumnBorder_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);
                 
/**
 * Four channel channel 16-bit 1D column convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorderParameters">Common parameters for nppiFilterColumnBorder functions</a>.
 */
NppStatus 
nppiFilterColumnBorder_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                   const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumnBorder_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit 1D column convolution filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorderParameters">Common parameters for nppiFilterColumnBorder functions</a>.
 */
NppStatus 
nppiFilterColumnBorder_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                    const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
 
NppStatus 
nppiFilterColumnBorder_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);
 
/**
 * Single channel 32-bit float 1D column convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorderParameters">Common parameters for nppiFilterColumnBorder functions</a>.
 */
NppStatus
nppiFilterColumnBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                   const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterColumnBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Three channel 32-bit float 1D column convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorderParameters">Common parameters for nppiFilterColumnBorder functions</a>.
 */
NppStatus
nppiFilterColumnBorder_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                   const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterColumnBorder_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Four channel 32-bit float 1D column convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorderParameters">Common parameters for nppiFilterColumnBorder functions</a>.
 */
NppStatus
nppiFilterColumnBorder_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                   const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterColumnBorder_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Four channel 32-bit float 1D column convolution filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorderParameters">Common parameters for nppiFilterColumnBorder functions</a>.
 */
NppStatus
nppiFilterColumnBorder_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                    const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterColumnBorder_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/** @} image_filter_column_border */

/** @defgroup image_filter_column_32f FilterColumn32f
 * 
 * FilterColumn using floating-point weights.
 * 
 * <h3><a name="CommonFilterColumn32fParameters">Common parameters for nppiFilterColumn32f functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oROI \ref roi_specification.
 * \param pKernel Pointer to the start address of the kernel coefficient array.
 *                Coefficients are expected to be stored in reverse order.
 * \param nMaskSize Length of the linear kernel array.
 * \param nAnchor Y offset of the kernel origin frame of reference relative to the
 *                 source pixel.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * 8-bit unsigned single-channel 1D column convolution.
 *
 * For common parameter descriptions, see <a href="#CommonFilterColumn32fParameters">Common parameters for nppiFilterColumn32f functions</a>.
 * 
 */
NppStatus 
nppiFilterColumn32f_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                               const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn32f_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                           const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 8-bit unsigned three-channel 1D column convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumn32fParameters">Common parameters for nppiFilterColumn32f functions</a>.
 * 
 */
NppStatus 
nppiFilterColumn32f_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                               const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn32f_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                           const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 8-bit unsigned four-channel 1D column convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumn32fParameters">Common parameters for nppiFilterColumn32f functions</a>.
 * 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 */
NppStatus 
nppiFilterColumn32f_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                               const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn32f_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                           const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 8-bit unsigned four-channel 1D column convolution ignoring alpha-channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumn32fParameters">Common parameters for nppiFilterColumn32f functions</a>.
 * 
 */
NppStatus 
nppiFilterColumn32f_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                                const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn32f_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                            const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 16-bit unsigned single-channel 1D column convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumn32fParameters">Common parameters for nppiFilterColumn32f functions</a>.
 * 
 */
NppStatus 
nppiFilterColumn32f_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                                const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn32f_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                            const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 16-bit unsigned three-channel 1D column convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumn32fParameters">Common parameters for nppiFilterColumn32f functions</a>.
 * 
 */
NppStatus 
nppiFilterColumn32f_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                                const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn32f_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                            const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 16-bit unsigned four-channel 1D column convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumn32fParameters">Common parameters for nppiFilterColumn32f functions</a>.
 * 
 */
NppStatus 
nppiFilterColumn32f_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                                const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn32f_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                            const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 16-bit unsigned four-channel 1D column convolution ignoring alpha-channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumn32fParameters">Common parameters for nppiFilterColumn32f functions</a>.
 * 
 */
NppStatus 
nppiFilterColumn32f_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                                 const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn32f_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                             const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 16-bit single-channel 1D column convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumn32fParameters">Common parameters for nppiFilterColumn32f functions</a>.
 * 
 */
NppStatus 
nppiFilterColumn32f_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                                const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn32f_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                            const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 16-bit three-channel 1D column convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumn32fParameters">Common parameters for nppiFilterColumn32f functions</a>.
 * 
 */
NppStatus 
nppiFilterColumn32f_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                                const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn32f_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                            const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 16-bit four-channel 1D column convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumn32fParameters">Common parameters for nppiFilterColumn32f functions</a>.
 * 
 */
NppStatus 
nppiFilterColumn32f_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                                const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn32f_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                            const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 16-bit four-channel 1D column convolution ignoring alpha-channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumn32fParameters">Common parameters for nppiFilterColumn32f functions</a>.
 * 
 */
NppStatus 
nppiFilterColumn32f_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                                 const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterColumn32f_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                             const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/** @} image_filter_column_32f */

/** @defgroup image_filter_column_border_32f FilterColumnBorder32f
 * General purpose 1D column convolution filter using floating-point weights with border control.
 *
 * Pixels under the mask are multiplied by the respective weights in the mask
 * and the results are summed.  If any portion of the mask overlaps the source
 * image boundary the requested border type operation is applied to all mask pixels
 * which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.
 *
 * <h3><a name="CommonFilterColumnBorder32fParameters">Common parameters for nppiFilterColumnBorder32f functions include:</a></h3>
 *
 * \param pSrc  \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param pKernel Pointer to the start address of the kernel coefficient array.
 *        Coeffcients are expected to be stored in reverse order.
 * \param nMaskSize Width of the kernel.
 * \param nAnchor X offset of the kernel origin frame of reference
 *        relative to the source pixel.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */


/**
 * Single channel 8-bit unsigned 1D column convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorder32fParameters">Common parameters for nppiFilterColumnBorder32f functions</a>.
 * 
 */
NppStatus
nppiFilterColumnBorder32f_8u_C1R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                                     const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
        
NppStatus
nppiFilterColumnBorder32f_8u_C1R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                                 const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);
        
/**
 * Three channel 8-bit unsigned 1D column convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorder32fParameters">Common parameters for nppiFilterColumnBorder32f functions</a>.
 * 
 */
NppStatus
nppiFilterColumnBorder32f_8u_C3R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                                     const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
        
NppStatus
nppiFilterColumnBorder32f_8u_C3R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                                 const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);
        
/**
 * Four channel 8-bit unsigned 1D column convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorder32fParameters">Common parameters for nppiFilterColumnBorder32f functions</a>.
 * 
 */
NppStatus
nppiFilterColumnBorder32f_8u_C4R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                                     const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
        
NppStatus
nppiFilterColumnBorder32f_8u_C4R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                                 const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);
        
/**
 * Four channel 8-bit unsigned 1D column convolution filter with border control, ignorint alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorder32fParameters">Common parameters for nppiFilterColumnBorder32f functions</a>.
 * 
 */
NppStatus
nppiFilterColumnBorder32f_8u_AC4R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                                      const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterColumnBorder32f_8u_AC4R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                                  const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Single channel 16-bit unsigned 1D column convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorder32fParameters">Common parameters for nppiFilterColumnBorder32f functions</a>.
 * 
 */
NppStatus
nppiFilterColumnBorder32f_16u_C1R_Ctx(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                                      const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterColumnBorder32f_16u_C1R(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                                  const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Three channel 16-bit unsigned 1D column convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorder32fParameters">Common parameters for nppiFilterColumnBorder32f functions</a>.
 * 
 */
NppStatus
nppiFilterColumnBorder32f_16u_C3R_Ctx(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                                      const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterColumnBorder32f_16u_C3R(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                                  const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned 1D column convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorder32fParameters">Common parameters for nppiFilterColumnBorder32f functions</a>.
 * 
 */
NppStatus
nppiFilterColumnBorder32f_16u_C4R_Ctx(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                                      const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterColumnBorder32f_16u_C4R(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                                  const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned 1D column convolution filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorder32fParameters">Common parameters for nppiFilterColumnBorder32f functions</a>.
 * 
 */
NppStatus
nppiFilterColumnBorder32f_16u_AC4R_Ctx(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                                       const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterColumnBorder32f_16u_AC4R(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                                   const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Single channel 16-bit 1D column convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorder32fParameters">Common parameters for nppiFilterColumnBorder32f functions</a>.
 * 
 */
NppStatus
nppiFilterColumnBorder32f_16s_C1R_Ctx(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                      const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterColumnBorder32f_16s_C1R(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                  const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Three channel 16-bit 1D column convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorder32fParameters">Common parameters for nppiFilterColumnBorder32f functions</a>.
 * 
 */
NppStatus
nppiFilterColumnBorder32f_16s_C3R_Ctx(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                      const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterColumnBorder32f_16s_C3R(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                  const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit 1D column convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorder32fParameters">Common parameters for nppiFilterColumnBorder32f functions</a>.
 * 
 */
NppStatus
nppiFilterColumnBorder32f_16s_C4R_Ctx(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                      const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterColumnBorder32f_16s_C4R(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                  const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit 1D column convolution filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterColumnBorder32fParameters">Common parameters for nppiFilterColumnBorder32f functions</a>.
 * 
 */
NppStatus
nppiFilterColumnBorder32f_16s_AC4R_Ctx(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                       const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterColumnBorder32f_16s_AC4R(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                   const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/** @} image_filter_column_border_32f */

/** @defgroup image_filter_row FilterRow
 * Apply convolution filter with user specified 1D row of weights.
 *  
 * Result pixel is equal to the sum of the products between the kernel
 * coefficients (pKernel array) and corresponding neighboring row pixel
 * values in the source image defined by nKernelDim and nAnchorX, divided by
 * nDivisor. 
 * 
 * <h3><a name="CommonFilterRowParameters">Common parameters for nppiFilterRow functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oROI \ref roi_specification.
 * \param pKernel Pointer to the start address of the kernel coefficient array.
 *                Coefficients are expected to be stored in reverse order.
 * \param nMaskSize Length of the linear kernel array.
 * \param nAnchor X offset of the kernel origin frame of reference relative to the
 *                 source pixel.
 * \param nDivisor The factor by which the convolved summation from the Filter
 *                 operation should be divided.  If equal to the sum of
 *                 coefficients, this will keep the maximum result value within
 *                 full scale.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * 8-bit unsigned single-channel 1D row convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowParameters">Common parameters for nppiFilterRow functions</a>.
 *
 */
NppStatus 
nppiFilterRow_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                         const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                     const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 8-bit unsigned three-channel 1D row convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowParameters">Common parameters for nppiFilterRow functions</a>.
 *
 */
NppStatus 
nppiFilterRow_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                         const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                     const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 8-bit unsigned four-channel 1D row convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowParameters">Common parameters for nppiFilterRow functions</a>.
 *
 */
NppStatus 
nppiFilterRow_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                         const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                     const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 8-bit unsigned four-channel 1D row convolution ignoring alpha-channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowParameters">Common parameters for nppiFilterRow functions</a>.
 *
 */
NppStatus 
nppiFilterRow_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                          const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                      const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 16-bit unsigned single-channel 1D row convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowParameters">Common parameters for nppiFilterRow functions</a>.
 *
 */
NppStatus 
nppiFilterRow_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                          const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                      const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 16-bit unsigned three-channel 1D row convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowParameters">Common parameters for nppiFilterRow functions</a>.
 *
 */
NppStatus 
nppiFilterRow_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                          const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                      const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 16-bit unsigned four-channel 1D row convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowParameters">Common parameters for nppiFilterRow functions</a>.
 *
 */
NppStatus
nppiFilterRow_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                          const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterRow_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                      const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 16-bit unsigned four-channel 1D row convolution ignoring alpha-channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowParameters">Common parameters for nppiFilterRow functions</a>.
 *
 */
NppStatus 
nppiFilterRow_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                           const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                       const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 16-bit single-channel 1D row convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowParameters">Common parameters for nppiFilterRow functions</a>.
 *
 */
NppStatus 
nppiFilterRow_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                          const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                      const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 16-bit three-channel 1D row convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowParameters">Common parameters for nppiFilterRow functions</a>.
 *
 */
NppStatus 
nppiFilterRow_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                          const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                      const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 16-bit four-channel 1D row convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowParameters">Common parameters for nppiFilterRow functions</a>.
 *
 */
NppStatus 
nppiFilterRow_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                          const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                      const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 16-bit four-channel 1D row convolution ignoring alpha-channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowParameters">Common parameters for nppiFilterRow functions</a>.
 *
 */
NppStatus 
nppiFilterRow_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                           const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                       const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor);

/**
 * 32-bit float single-channel 1D row convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowParameters">Common parameters for nppiFilterRow functions</a>.
 *
 */
NppStatus 
nppiFilterRow_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                          const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                      const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 32-bit float three-channel 1D row convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowParameters">Common parameters for nppiFilterRow functions</a>.
 *
 */
NppStatus 
nppiFilterRow_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                          const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                      const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 32-bit float four-channel 1D row convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowParameters">Common parameters for nppiFilterRow functions</a>.
 *
 */
NppStatus 
nppiFilterRow_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                          const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                      const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 32-bit float four-channel 1D row convolution ignoring alpha-channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowParameters">Common parameters for nppiFilterRow functions</a>.
 *
 */
NppStatus 
nppiFilterRow_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                           const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                       const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 64-bit float single-channel 1D row convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowParameters">Common parameters for nppiFilterRow functions</a>.
 *
 */
NppStatus 
nppiFilterRow_64f_C1R_Ctx(const Npp64f * pSrc, Npp32s nSrcStep, Npp64f * pDst, Npp32s nDstStep, NppiSize oROI, 
                          const Npp64f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow_64f_C1R(const Npp64f * pSrc, Npp32s nSrcStep, Npp64f * pDst, Npp32s nDstStep, NppiSize oROI, 
                      const Npp64f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);


/** @} image_filter_row */

/** @defgroup image_filter_row_border FilterRowBorder
 * General purpose 1D convolution row filter with border control.
 *
 * Pixels under the mask are multiplied by the respective weights in the mask
 * and the results are summed. Before writing the result pixel the sum is scaled
 * back via division by nDivisor. If any portion of the mask overlaps the source
 * image boundary the requested border type operation is applied to all mask pixels
 * which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.
 *
 * <h3><a name="CommonFilterRowBorderParameters">Common parameters for nppiFilterRowBorder functions include:</a></h3>
 *
 * \param pSrc  \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param pKernel Pointer to the start address of the kernel coefficient array.
 *        Coeffcients are expected to be stored in reverse order.
 * \param nMaskSize Width of the kernel.
 * \param nAnchor X offset of the kernel origin frame of reference
 *        relative to the source pixel.
 * \param nDivisor The factor by which the convolved summation from the Filter
 *        operation should be divided.  If equal to the sum of coefficients,
 *        this will keep the maximum result value within full scale.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned 1D row convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorderParameters">Common parameters for nppiFilterRowBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRowBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
  
NppStatus 
nppiFilterRowBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                           const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);
  
/**
 * Three channel 8-bit unsigned 1D row convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorderParameters">Common parameters for nppiFilterRowBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRowBorder_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
                 
NppStatus 
nppiFilterRowBorder_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                           const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);
                 
/**
 * Four channel channel 8-bit unsigned 1D row convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorderParameters">Common parameters for nppiFilterRowBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRowBorder_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRowBorder_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                           const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned convolution 1D row filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorderParameters">Common parameters for nppiFilterRowBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRowBorder_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRowBorder_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);

/**
 * Single channel 16-bit unsigned convolution 1D row filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorderParameters">Common parameters for nppiFilterRowBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRowBorder_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
  
NppStatus 
nppiFilterRowBorder_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);
  
/**
 * Three channel 16-bit unsigned 1D row convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorderParameters">Common parameters for nppiFilterRowBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRowBorder_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
                 
NppStatus 
nppiFilterRowBorder_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);
                 
/**
 * Four channel channel 16-bit 1D row unsigned convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorderParameters">Common parameters for nppiFilterRowBorder functions</a>.
 *
 */
NppStatus
nppiFilterRowBorder_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterRowBorder_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned 1D row convolution filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorderParameters">Common parameters for nppiFilterRowBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRowBorder_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                 const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRowBorder_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);

/**
 * Single channel 16-bit 1D row convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorderParameters">Common parameters for nppiFilterRowBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRowBorder_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
  
NppStatus 
nppiFilterRowBorder_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);
  
/**
 * Three channel 16-bit 1D row convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorderParameters">Common parameters for nppiFilterRowBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRowBorder_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
                 
NppStatus 
nppiFilterRowBorder_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);
                 
/**
 * Four channel channel 16-bit 1D row convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorderParameters">Common parameters for nppiFilterRowBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRowBorder_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRowBorder_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit 1D row convolution filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorderParameters">Common parameters for nppiFilterRowBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRowBorder_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                 const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
 
NppStatus 
nppiFilterRowBorder_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             const Npp32s * pKernel, Npp32s nMaskSize, Npp32s nAnchor, Npp32s nDivisor, NppiBorderType eBorderType);
 
/**
 * Single channel 32-bit float 1D row convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorderParameters">Common parameters for nppiFilterRowBorder functions</a>.
 *
 */
NppStatus
nppiFilterRowBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterRowBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Three channel 32-bit float 1D row convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorderParameters">Common parameters for nppiFilterRowBorder functions</a>.
 *
 */
NppStatus
nppiFilterRowBorder_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterRowBorder_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Four channel 32-bit float 1D row convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorderParameters">Common parameters for nppiFilterRowBorder functions</a>.
 *
 */
NppStatus
nppiFilterRowBorder_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterRowBorder_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Four channel 32-bit float 1D row convolution filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorderParameters">Common parameters for nppiFilterRowBorder functions</a>.
 *
 */
NppStatus
nppiFilterRowBorder_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                 const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterRowBorder_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/** @} image_filter_row_border */

/** @defgroup image_filter_row_32f FilterRow32f
 * 
 * FilterRow using floating-point weights.
 * 
 * <h3><a name="CommonFilterRow32fParameters">Common parameters for nppiFilterRow32f functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oROI \ref roi_specification.
 * \param pKernel Pointer to the start address of the kernel coefficient array.
 *                Coefficients are expected to be stored in reverse order.
 * \param nMaskSize Length of the linear kernel array.
 * \param nAnchor X offset of the kernel origin frame of reference relative to the
 *                 source pixel.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * 8-bit unsigned single-channel 1D row convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRow32fParameters">Common parameters for nppiFilterRow32f functions</a>.
 *
 */
NppStatus 
nppiFilterRow32f_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                            const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow32f_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                        const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 8-bit unsigned three-channel 1D row convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRow32fParameters">Common parameters for nppiFilterRow32f functions</a>.
 *
 */
NppStatus 
nppiFilterRow32f_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                            const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow32f_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                        const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 8-bit unsigned four-channel 1D row convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRow32fParameters">Common parameters for nppiFilterRow32f functions</a>.
 *
 */
NppStatus 
nppiFilterRow32f_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                            const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow32f_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                        const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 8-bit unsigned four-channel 1D row convolution ignoring alpha-channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRow32fParameters">Common parameters for nppiFilterRow32f functions</a>.
 *
 */
NppStatus 
nppiFilterRow32f_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                             const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow32f_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oROI, 
                         const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 16-bit unsigned single-channel 1D row convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRow32fParameters">Common parameters for nppiFilterRow32f functions</a>.
 *
 */
NppStatus 
nppiFilterRow32f_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                             const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow32f_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                         const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 16-bit unsigned three-channel 1D row convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRow32fParameters">Common parameters for nppiFilterRow32f functions</a>.
 *
 */
NppStatus 
nppiFilterRow32f_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                             const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow32f_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                         const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 16-bit unsigned four-channel 1D row convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRow32fParameters">Common parameters for nppiFilterRow32f functions</a>.
 *
 */
NppStatus 
nppiFilterRow32f_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                             const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow32f_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                         const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 16-bit unsigned four-channel 1D row convolution ignoring alpha-channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRow32fParameters">Common parameters for nppiFilterRow32f functions</a>.
 *
 */
NppStatus 
nppiFilterRow32f_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                              const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow32f_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oROI, 
                          const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 16-bit single-channel 1D row convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRow32fParameters">Common parameters for nppiFilterRow32f functions</a>.
 *
 */
NppStatus 
nppiFilterRow32f_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                             const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow32f_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                         const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 16-bit three-channel 1D row convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRow32fParameters">Common parameters for nppiFilterRow32f functions</a>.
 *
 */
NppStatus 
nppiFilterRow32f_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                             const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow32f_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                         const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 16-bit four-channel 1D row convolution.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRow32fParameters">Common parameters for nppiFilterRow32f functions</a>.
 *
 */
NppStatus 
nppiFilterRow32f_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                             const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow32f_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                         const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * 16-bit four-channel 1D row convolution ignoring alpha-channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRow32fParameters">Common parameters for nppiFilterRow32f functions</a>.
 *
 */
NppStatus 
nppiFilterRow32f_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                              const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRow32f_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oROI, 
                          const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor);

/** @} image_filter_row_32f */

/** @defgroup image_filter_row_border_32f FilterRowBorder32f
 * General purpose 1D row convolution filter using floating-point weights with border control.
 *
 * Pixels under the mask are multiplied by the respective weights in the mask
 * and the results are summed.  If any portion of the mask overlaps the source
 * image boundary the requested border type operation is applied to all mask pixels
 * which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.
 *
 * <h3><a name="CommonFilterRowBorder32fParameters">Common parameters for nppiFilterRowBorder32f functions include:</a></h3>
 *
 * \param pSrc  \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param pKernel Pointer to the start address of the kernel coefficient array.
 *        Coeffcients are expected to be stored in reverse order.
 * \param nMaskSize Width of the kernel.
 * \param nAnchor X offset of the kernel origin frame of reference
 *        relative to the source pixel.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */


/**
 * Single channel 8-bit unsigned 1D row convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorder32fParameters">Common parameters for nppiFilterRowBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterRowBorder32f_8u_C1R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                                  const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
        
NppStatus
nppiFilterRowBorder32f_8u_C1R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                              const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);
        
/**
 * Three channel 8-bit unsigned 1D row convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorder32fParameters">Common parameters for nppiFilterRowBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterRowBorder32f_8u_C3R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                                  const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
        
NppStatus
nppiFilterRowBorder32f_8u_C3R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                              const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);
        
/**
 * Four channel 8-bit unsigned 1D row convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorder32fParameters">Common parameters for nppiFilterRowBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterRowBorder32f_8u_C4R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                                  const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
        
NppStatus
nppiFilterRowBorder32f_8u_C4R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                              const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);
        
/**
 * Four channel 8-bit unsigned 1D row convolution filter with border control, ignorint alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorder32fParameters">Common parameters for nppiFilterRowBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterRowBorder32f_8u_AC4R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                                   const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterRowBorder32f_8u_AC4R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                               const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Single channel 16-bit unsigned 1D row convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorder32fParameters">Common parameters for nppiFilterRowBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterRowBorder32f_16u_C1R_Ctx(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                                   const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterRowBorder32f_16u_C1R(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                               const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Three channel 16-bit unsigned 1D row convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorder32fParameters">Common parameters for nppiFilterRowBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterRowBorder32f_16u_C3R_Ctx(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                                   const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterRowBorder32f_16u_C3R(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                               const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned 1D row convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorder32fParameters">Common parameters for nppiFilterRowBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterRowBorder32f_16u_C4R_Ctx(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                                   const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterRowBorder32f_16u_C4R(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                               const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned 1D row convolution filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorder32fParameters">Common parameters for nppiFilterRowBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterRowBorder32f_16u_AC4R_Ctx(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                                    const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterRowBorder32f_16u_AC4R(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                                const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Single channel 16-bit 1D row convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorder32fParameters">Common parameters for nppiFilterRowBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterRowBorder32f_16s_C1R_Ctx(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                   const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterRowBorder32f_16s_C1R(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                               const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Three channel 16-bit 1D row convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorder32fParameters">Common parameters for nppiFilterRowBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterRowBorder32f_16s_C3R_Ctx(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                   const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterRowBorder32f_16s_C3R(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                               const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit 1D row convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorder32fParameters">Common parameters for nppiFilterRowBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterRowBorder32f_16s_C4R_Ctx(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                   const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterRowBorder32f_16s_C4R(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                               const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit 1D row convolution filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterRowBorder32fParameters">Common parameters for nppiFilterRowBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterRowBorder32f_16s_AC4R_Ctx(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                    const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterRowBorder32f_16s_AC4R(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                const Npp32f * pKernel, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/** @} image_filter_row_border_32f */

/** @} image_1D_linear_filter */

/** @defgroup image_1D_window_sum 1D Window Sum
 * The set of 1D window sum functions available in the library.
 * @{
 *
 */

/** @defgroup image_1D_window_column_sum 1D Window Column Sum
 *  1D mask Window Column Sum for 8 and 16 bit images.
 *
 * <h3><a name="CommonFilterSumWindowColumnParameters">Common parameters for nppiSumWindowColumn functions include:</a></h3>
 *
 * \param pSrc  \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oROI \ref roi_specification.
 * \param nMaskSize Length of the linear kernel array.
 * \param nAnchor Y offset of the kernel origin frame of reference relative to the
 *        source pixel.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * One channel 8-bit unsigned 1D (column) sum to 32f.
 *
 * Apply Column Window Summation filter over a 1D mask region around each
 * source pixel for 1-channel 8 bit/pixel input images with 32-bit floating point
 * output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring column pixel values in a mask region of the source image defined by
 * nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowColumnParameters">Common parameters for nppiSumWindowColumn functions</a>.
 *
 */
NppStatus nppiSumWindowColumn_8u32f_C1R_Ctx(const Npp8u  * pSrc, Npp32s nSrcStep, 
                                                  Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                            Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus nppiSumWindowColumn_8u32f_C1R(const Npp8u  * pSrc, Npp32s nSrcStep, 
                                              Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                        Npp32s nMaskSize, Npp32s nAnchor);


/**
 * Three channel 8-bit unsigned 1D (column) sum to 32f.
 *
 * Apply Column Window Summation filter over a 1D mask region around each
 * source pixel for 3-channel 8 bit/pixel input images with 32-bit floating point
 * output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring column pixel values in a mask region of the source image defined by
 * nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowColumnParameters">Common parameters for nppiSumWindowColumn functions</a>.
 *
 */
NppStatus nppiSumWindowColumn_8u32f_C3R_Ctx(const Npp8u  * pSrc, Npp32s nSrcStep, 
                                                  Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                            Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus nppiSumWindowColumn_8u32f_C3R(const Npp8u  * pSrc, Npp32s nSrcStep, 
                                              Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                        Npp32s nMaskSize, Npp32s nAnchor);


/**
 * Four channel 8-bit unsigned 1D (column) sum to 32f.
 *
 * Apply Column Window Summation filter over a 1D mask region around each
 * source pixel for 4-channel 8 bit/pixel input images with 32-bit floating point
 * output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring column pixel values in a mask region of the source image defined by
 * nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowColumnParameters">Common parameters for nppiSumWindowColumn functions</a>.
 *
 */
NppStatus nppiSumWindowColumn_8u32f_C4R_Ctx(const Npp8u  * pSrc, Npp32s nSrcStep, 
                                                  Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                            Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus nppiSumWindowColumn_8u32f_C4R(const Npp8u  * pSrc, Npp32s nSrcStep, 
                                              Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                        Npp32s nMaskSize, Npp32s nAnchor);

/**
 * One channel 16-bit unsigned 1D (column) sum to 32f.
 *
 * Apply Column Window Summation filter over a 1D mask region around each
 * source pixel for 1-channel 16 bit/pixel input images with 32-bit floating point
 * output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring column pixel values in a mask region of the source image defined by
 * nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowColumnParameters">Common parameters for nppiSumWindowColumn functions</a>.
 *
 */
NppStatus nppiSumWindowColumn_16u32f_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, 
                                                   Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                             Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus nppiSumWindowColumn_16u32f_C1R(const Npp16u * pSrc, Npp32s nSrcStep, 
                                               Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                         Npp32s nMaskSize, Npp32s nAnchor);

/**
 * Three channel 16-bit unsigned 1D (column) sum to 32f.
 *
 * Apply Column Window Summation filter over a 1D mask region around each
 * source pixel for 3-channel 16 bit/pixel input images with 32-bit floating point
 * output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring column pixel values in a mask region of the source image defined by
 * nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowColumnParameters">Common parameters for nppiSumWindowColumn functions</a>.
 *
 */
NppStatus nppiSumWindowColumn_16u32f_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, 
                                                   Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                             Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus nppiSumWindowColumn_16u32f_C3R(const Npp16u * pSrc, Npp32s nSrcStep, 
                                               Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                         Npp32s nMaskSize, Npp32s nAnchor);

/**
 * Four channel 16-bit unsigned 1D (column) sum to 32f.
 *
 * Apply Column Window Summation filter over a 1D mask region around each
 * source pixel for 4-channel 16 bit/pixel input images with 32-bit floating point
 * output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring column pixel values in a mask region of the source image defined by
 * nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowColumnParameters">Common parameters for nppiSumWindowColumn functions</a>.
 *
 */
NppStatus nppiSumWindowColumn_16u32f_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, 
                                                   Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                             Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus nppiSumWindowColumn_16u32f_C4R(const Npp16u * pSrc, Npp32s nSrcStep, 
                                               Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                         Npp32s nMaskSize, Npp32s nAnchor);

/**
 * One channel 16-bit signed 1D (column) sum to 32f.
 *
 * Apply Column Window Summation filter over a 1D mask region around each
 * source pixel for 1-channel 16 bit/pixel input images with 32-bit floating point
 * output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring column pixel values in a mask region of the source image defined by
 * nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowColumnParameters">Common parameters for nppiSumWindowColumn functions</a>.
 *
 */
NppStatus nppiSumWindowColumn_16s32f_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, 
                                                   Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                             Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus nppiSumWindowColumn_16s32f_C1R(const Npp16s * pSrc, Npp32s nSrcStep, 
                                               Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                         Npp32s nMaskSize, Npp32s nAnchor);

/**
 * Three channel 16-bit signed 1D (column) sum to 32f.
 *
 * Apply Column Window Summation filter over a 1D mask region around each
 * source pixel for 1-channel 16 bit/pixel input images with 32-bit floating point
 * output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring column pixel values in a mask region of the source image defined by
 * nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowColumnParameters">Common parameters for nppiSumWindowColumn functions</a>.
 *
 */
NppStatus nppiSumWindowColumn_16s32f_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, 
                                                   Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                             Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus nppiSumWindowColumn_16s32f_C3R(const Npp16s * pSrc, Npp32s nSrcStep, 
                                               Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                         Npp32s nMaskSize, Npp32s nAnchor);

/**
 * Four channel 16-bit signed 1D (column) sum to 32f.
 *
 * Apply Column Window Summation filter over a 1D mask region around each
 * source pixel for 4-channel 16 bit/pixel input images with 32-bit floating point
 * output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring column pixel values in a mask region of the source image defined by
 * nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowColumnParameters">Common parameters for nppiSumWindowColumn functions</a>.
 *
 */
NppStatus nppiSumWindowColumn_16s32f_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, 
                                                   Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                             Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus nppiSumWindowColumn_16s32f_C4R(const Npp16s * pSrc, Npp32s nSrcStep, 
                                               Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                         Npp32s nMaskSize, Npp32s nAnchor);

/** @} image_filter_1D_window_column_sum */

/** @defgroup image_filter_1D_window_row_sum 1D Window Row Sum
 *  1D mask Window Row Sum for 8 and 16 bit images.
 *
 * <h3><a name="CommonFilterSumWindowRowParameters">Common parameters for nppiSumWindowRow functions include:</a></h3>
 *
 * \param pSrc  \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oROI \ref roi_specification.
 * \param nMaskSize Length of the linear kernel array.
 * \param nAnchor X offset of the kernel origin frame of reference relative to the
 *        source pixel.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * One channel 8-bit unsigned 1D (row) sum to 32f.
 *
 * Apply Row Window Summation filter over a 1D mask region around each source
 * pixel for 1-channel 8-bit pixel input images with 32-bit floating point output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring row pixel values in a mask region of the source image defined
 * by nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowRowParameters">Common parameters for nppiSumWindowRow functions</a>.
 *
 */
NppStatus 
nppiSumWindowRow_8u32f_C1R_Ctx(const Npp8u  * pSrc, Npp32s nSrcStep, 
                                     Npp32f * pDst, Npp32s nDstStep, 
                               NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiSumWindowRow_8u32f_C1R(const Npp8u  * pSrc, Npp32s nSrcStep, 
                                 Npp32f * pDst, Npp32s nDstStep, 
                           NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * Three channel 8-bit unsigned 1D (row) sum to 32f.
 *
 * Apply Row Window Summation filter over a 1D mask region around each source
 * pixel for 3-channel 8-bit pixel input images with 32-bit floating point output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring row pixel values in a mask region of the source image defined
 * by nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowRowParameters">Common parameters for nppiSumWindowRow functions</a>.
 *
 */
NppStatus 
nppiSumWindowRow_8u32f_C3R_Ctx(const Npp8u  * pSrc, Npp32s nSrcStep, 
                                     Npp32f * pDst, Npp32s nDstStep, 
                               NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiSumWindowRow_8u32f_C3R(const Npp8u  * pSrc, Npp32s nSrcStep, 
                                 Npp32f * pDst, Npp32s nDstStep, 
                           NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * Four channel 8-bit unsigned 1D (row) sum to 32f.
 *
 * Apply Row Window Summation filter over a 1D mask region around each source
 * pixel for 4-channel 8-bit pixel input images with 32-bit floating point output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring row pixel values in a mask region of the source image defined
 * by nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowRowParameters">Common parameters for nppiSumWindowRow functions</a>.
 *
 */
NppStatus 
nppiSumWindowRow_8u32f_C4R_Ctx(const Npp8u  * pSrc, Npp32s nSrcStep, 
                                     Npp32f * pDst, Npp32s nDstStep, 
                               NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiSumWindowRow_8u32f_C4R(const Npp8u  * pSrc, Npp32s nSrcStep, 
                                 Npp32f * pDst, Npp32s nDstStep, 
                           NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * One channel 16-bit unsigned 1D (row) sum to 32f.
 *
 * Apply Row Window Summation filter over a 1D mask region around each source
 * pixel for 1-channel 16-bit pixel input images with 32-bit floating point output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring row pixel values in a mask region of the source image defined
 * by nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowRowParameters">Common parameters for nppiSumWindowRow functions</a>.
 *
 */
NppStatus 
nppiSumWindowRow_16u32f_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, 
                                      Npp32f * pDst, Npp32s nDstStep, 
                                NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiSumWindowRow_16u32f_C1R(const Npp16u * pSrc, Npp32s nSrcStep, 
                                  Npp32f * pDst, Npp32s nDstStep, 
                            NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * Three channel 16-bit unsigned 1D (row) sum to 32f.
 *
 * Apply Row Window Summation filter over a 1D mask region around each source
 * pixel for 3-channel 16-bit pixel input images with 32-bit floating point output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring row pixel values in a mask region of the source image defined
 * by nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowRowParameters">Common parameters for nppiSumWindowRow functions</a>.
 *
 */
NppStatus 
nppiSumWindowRow_16u32f_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, 
                                      Npp32f * pDst, Npp32s nDstStep, 
                                NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiSumWindowRow_16u32f_C3R(const Npp16u * pSrc, Npp32s nSrcStep, 
                                  Npp32f * pDst, Npp32s nDstStep, 
                            NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * Four channel 16-bit unsigned 1D (row) sum to 32f.
 *
 * Apply Row Window Summation filter over a 1D mask region around each source
 * pixel for 4-channel 16-bit pixel input images with 32-bit floating point output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring row pixel values in a mask region of the source image defined
 * by nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowRowParameters">Common parameters for nppiSumWindowRow functions</a>.
 *
 */
NppStatus 
nppiSumWindowRow_16u32f_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, 
                                      Npp32f * pDst, Npp32s nDstStep, 
                                NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiSumWindowRow_16u32f_C4R(const Npp16u * pSrc, Npp32s nSrcStep, 
                                  Npp32f * pDst, Npp32s nDstStep, 
                            NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * One channel 16-bit signed 1D (row) sum to 32f.
 *
 * Apply Row Window Summation filter over a 1D mask region around each source
 * pixel for 1-channel 16-bit pixel input images with 32-bit floating point output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring row pixel values in a mask region of the source image defined
 * by nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowRowParameters">Common parameters for nppiSumWindowRow functions</a>.
 *
 */
NppStatus 
nppiSumWindowRow_16s32f_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, 
                                      Npp32f * pDst, Npp32s nDstStep, 
                                NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiSumWindowRow_16s32f_C1R(const Npp16s * pSrc, Npp32s nSrcStep, 
                                  Npp32f * pDst, Npp32s nDstStep, 
                            NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * Three channel 16-bit signed 1D (row) sum to 32f.
 *
 * Apply Row Window Summation filter over a 1D mask region around each source
 * pixel for 3-channel 16-bit pixel input images with 32-bit floating point output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring row pixel values in a mask region of the source image defined
 * by nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowRowParameters">Common parameters for nppiSumWindowRow functions</a>.
 *
 */
NppStatus 
nppiSumWindowRow_16s32f_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, 
                                      Npp32f * pDst, Npp32s nDstStep, 
                                NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiSumWindowRow_16s32f_C3R(const Npp16s * pSrc, Npp32s nSrcStep, 
                                  Npp32f * pDst, Npp32s nDstStep, 
                            NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor);

/**
 * Four channel 16-bit signed 1D (row) sum to 32f.
 *
 * Apply Row Window Summation filter over a 1D mask region around each source
 * pixel for 4-channel 16-bit pixel input images with 32-bit floating point output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring row pixel values in a mask region of the source image defined
 * by nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowRowParameters">Common parameters for nppiSumWindowRow functions</a>.
 *
 */
NppStatus 
nppiSumWindowRow_16s32f_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, 
                                      Npp32f * pDst, Npp32s nDstStep, 
                                NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiSumWindowRow_16s32f_C4R(const Npp16s * pSrc, Npp32s nSrcStep, 
                                  Npp32f * pDst, Npp32s nDstStep, 
                            NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor);

/** @} image_filter_1D_window_row_sum */

/** @} image_1D_window_sum */

/** @defgroup image_1D_window_sum_border 1D Window Sum with Border Control
 * The set of 1D window sum functions with border control available in the library.
 * @{
 *
 */

/** @defgroup image_filter_1D_window_column_sum_border 1D Window Column Sum Border
 * 1D mask Window Column Sum for 8 and 16 bit images with border control.
 * 
 * If any portion of the mask overlaps the source image boundary the requested border type operation 
 * is applied to all mask pixels which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.
 *
 * <h3><a name="CommonFilterSumWindowColumnBorderParameters">Common parameters for nppiSumWindowColumnBorder functions include:</a></h3>
 *
 * \param pSrc  \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oROI \ref roi_specification.
 * \param nMaskSize Length of the linear kernel array.
 * \param nAnchor Y offset of the kernel origin frame of reference relative to the
 *        source pixel.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * One channel 8-bit unsigned 1D (column) sum to 32f with border control.
 *
 * Apply Column Window Summation filter over a 1D mask region around each
 * source pixel for 1-channel 8 bit/pixel input images with 32-bit floating point
 * output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring column pixel values in a mask region of the source image defined by
 * nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowColumnBorderParameters">Common parameters for nppiSumWindowColumnBorder functions</a>.
 *
 */
NppStatus nppiSumWindowColumnBorder_8u32f_C1R_Ctx(const Npp8u  * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                        Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                                        Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus nppiSumWindowColumnBorder_8u32f_C1R(const Npp8u  * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                    Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                                    Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);


/**
 * Three channel 8-bit unsigned 1D (column) sum to 32f with border control.
 *
 * Apply Column Window Summation filter over a 1D mask region around each
 * source pixel for 3-channel 8 bit/pixel input images with 32-bit floating point
 * output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring column pixel values in a mask region of the source image defined by
 * nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowColumnBorderParameters">Common parameters for nppiSumWindowColumnBorder functions</a>.
 *
 */
NppStatus nppiSumWindowColumnBorder_8u32f_C3R_Ctx(const Npp8u  * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                        Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                                        Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus nppiSumWindowColumnBorder_8u32f_C3R(const Npp8u  * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                    Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                                    Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);


/**
 * Four channel 8-bit unsigned 1D (column) sum to 32f with border control.
 *
 * Apply Column Window Summation filter over a 1D mask region around each
 * source pixel for 4-channel 8 bit/pixel input images with 32-bit floating point
 * output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring column pixel values in a mask region of the source image defined by
 * nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowColumnBorderParameters">Common parameters for nppiSumWindowColumnBorder functions</a>.
 *
 */
NppStatus nppiSumWindowColumnBorder_8u32f_C4R_Ctx(const Npp8u  * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                        Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                                        Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus nppiSumWindowColumnBorder_8u32f_C4R(const Npp8u  * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                    Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                                    Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * One channel 16-bit unsigned 1D (column) sum to 32f with border control.
 *
 * Apply Column Window Summation filter over a 1D mask region around each
 * source pixel for 1-channel 16 bit/pixel input images with 32-bit floating point
 * output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring column pixel values in a mask region of the source image defined by
 * nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowColumnBorderParameters">Common parameters for nppiSumWindowColumnBorder functions</a>.
 *
 */
NppStatus nppiSumWindowColumnBorder_16u32f_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                         Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                                         Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus nppiSumWindowColumnBorder_16u32f_C1R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                     Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                                     Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Three channel 16-bit unsigned 1D (column) sum to 32f with border control.
 *
 * Apply Column Window Summation filter over a 1D mask region around each
 * source pixel for 3-channel 16 bit/pixel input images with 32-bit floating point
 * output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring column pixel values in a mask region of the source image defined by
 * nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowColumnBorderParameters">Common parameters for nppiSumWindowColumnBorder functions</a>.
 *
 */
NppStatus nppiSumWindowColumnBorder_16u32f_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                         Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                                         Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus nppiSumWindowColumnBorder_16u32f_C3R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                     Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                                     Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned 1D (column) sum to 32f with border control.
 *
 * Apply Column Window Summation filter over a 1D mask region around each
 * source pixel for 4-channel 16 bit/pixel input images with 32-bit floating point
 * output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring column pixel values in a mask region of the source image defined by
 * nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowColumnBorderParameters">Common parameters for nppiSumWindowColumnBorder functions</a>.
 *
 */
NppStatus nppiSumWindowColumnBorder_16u32f_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                         Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                                         Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus nppiSumWindowColumnBorder_16u32f_C4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                     Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                                     Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * One channel 16-bit signed 1D (column) sum to 32f with border control.
 *
 * Apply Column Window Summation filter over a 1D mask region around each
 * source pixel for 1-channel 16 bit/pixel input images with 32-bit floating point
 * output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring column pixel values in a mask region of the source image defined by
 * nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowColumnBorderParameters">Common parameters for nppiSumWindowColumnBorder functions</a>.
 *
 */
NppStatus nppiSumWindowColumnBorder_16s32f_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                         Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                                         Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus nppiSumWindowColumnBorder_16s32f_C1R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                     Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                                     Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Three channel 16-bit signed 1D (column) sum to 32f with border control.
 *
 * Apply Column Window Summation filter over a 1D mask region around each
 * source pixel for 1-channel 16 bit/pixel input images with 32-bit floating point
 * output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring column pixel values in a mask region of the source image defined by
 * nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowColumnBorderParameters">Common parameters for nppiSumWindowColumnBorder functions</a>.
 *
 */
NppStatus nppiSumWindowColumnBorder_16s32f_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                         Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                                         Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus nppiSumWindowColumnBorder_16s32f_C3R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                     Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                                     Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed 1D (column) sum to 32f with border control.
 *
 * Apply Column Window Summation filter over a 1D mask region around each
 * source pixel for 4-channel 16 bit/pixel input images with 32-bit floating point
 * output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring column pixel values in a mask region of the source image defined by
 * nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowColumnBorderParameters">Common parameters for nppiSumWindowColumnBorder functions</a>.
 *
 */
NppStatus nppiSumWindowColumnBorder_16s32f_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                         Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                                         Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus nppiSumWindowColumnBorder_16s32f_C4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                     Npp32f * pDst, Npp32s nDstStep, NppiSize oROI, 
                                                     Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/** @} image_filter_1D_window_column_sum_border */

/** @defgroup image_filter_1D_window_row_sum_border 1D Window Row Sum Border
 * 1D mask Window Row Sum for 8 and 16 bit images with border control.
 *
 * <h3><a name="CommonFilterSumWindowRowBorderParameters">Common parameters for nppiSumWindowRowBorder functions include:</a></h3>
 *
 * \param pSrc  \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oROI \ref roi_specification.
 * \param nMaskSize Length of the linear kernel array.
 * \param nAnchor X offset of the kernel origin frame of reference relative to the
 *        source pixel.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * One channel 8-bit unsigned 1D (row) sum to 32f with border control.
 *
 * Apply Row Window Summation filter over a 1D mask region around each source
 * pixel for 1-channel 8-bit pixel input images with 32-bit floating point output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring row pixel values in a mask region of the source image defined
 * by nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowRowBorderParameters">Common parameters for nppiSumWindowRowBorder functions</a>.
 *
 */
NppStatus 
nppiSumWindowRowBorder_8u32f_C1R_Ctx(const Npp8u  * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                           Npp32f * pDst, Npp32s nDstStep, 
                                           NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiSumWindowRowBorder_8u32f_C1R(const Npp8u  * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                       Npp32f * pDst, Npp32s nDstStep, 
                                       NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Three channel 8-bit unsigned 1D (row) sum to 32f with border control.
 *
 * Apply Row Window Summation filter over a 1D mask region around each source
 * pixel for 3-channel 8-bit pixel input images with 32-bit floating point output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring row pixel values in a mask region of the source image defined
 * by nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowRowBorderParameters">Common parameters for nppiSumWindowRowBorder functions</a>.
 *
 */
NppStatus 
nppiSumWindowRowBorder_8u32f_C3R_Ctx(const Npp8u  * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                           Npp32f * pDst, Npp32s nDstStep, 
                                           NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiSumWindowRowBorder_8u32f_C3R(const Npp8u  * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                       Npp32f * pDst, Npp32s nDstStep, 
                                       NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned 1D (row) sum to 32f with border control.
 *
 * Apply Row Window Summation filter over a 1D mask region around each source
 * pixel for 4-channel 8-bit pixel input images with 32-bit floating point output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring row pixel values in a mask region of the source image defined
 * by nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowRowBorderParameters">Common parameters for nppiSumWindowRowBorder functions</a>.
 *
 */
NppStatus 
nppiSumWindowRowBorder_8u32f_C4R_Ctx(const Npp8u  * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                           Npp32f * pDst, Npp32s nDstStep, 
                                           NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiSumWindowRowBorder_8u32f_C4R(const Npp8u  * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                       Npp32f * pDst, Npp32s nDstStep, 
                                       NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * One channel 16-bit unsigned 1D (row) sum to 32f with border control.
 *
 * Apply Row Window Summation filter over a 1D mask region around each source
 * pixel for 1-channel 16-bit pixel input images with 32-bit floating point output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring row pixel values in a mask region of the source image defined
 * by nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowRowBorderParameters">Common parameters for nppiSumWindowRowBorder functions</a>.
 *
 */
NppStatus 
nppiSumWindowRowBorder_16u32f_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                            Npp32f * pDst, Npp32s nDstStep, 
                                            NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiSumWindowRowBorder_16u32f_C1R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                        Npp32f * pDst, Npp32s nDstStep, 
                                        NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Three channel 16-bit unsigned 1D (row) sum to 32f with border control.
 *
 * Apply Row Window Summation filter over a 1D mask region around each source
 * pixel for 3-channel 16-bit pixel input images with 32-bit floating point output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring row pixel values in a mask region of the source image defined
 * by nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowRowBorderParameters">Common parameters for nppiSumWindowRowBorder functions</a>.
 *
 */
NppStatus 
nppiSumWindowRowBorder_16u32f_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                            Npp32f * pDst, Npp32s nDstStep, 
                                            NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiSumWindowRowBorder_16u32f_C3R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                        Npp32f * pDst, Npp32s nDstStep, 
                                        NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned 1D (row) sum to 32f with border control.
 *
 * Apply Row Window Summation filter over a 1D mask region around each source
 * pixel for 4-channel 16-bit pixel input images with 32-bit floating point output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring row pixel values in a mask region of the source image defined
 * by nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowRowBorderParameters">Common parameters for nppiSumWindowRowBorder functions</a>.
 *
 */
NppStatus 
nppiSumWindowRowBorder_16u32f_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                            Npp32f * pDst, Npp32s nDstStep, 
                                            NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiSumWindowRowBorder_16u32f_C4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                        Npp32f * pDst, Npp32s nDstStep, 
                                        NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * One channel 16-bit signed 1D (row) sum to 32f with border control.
 *
 * Apply Row Window Summation filter over a 1D mask region around each source
 * pixel for 1-channel 16-bit pixel input images with 32-bit floating point output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring row pixel values in a mask region of the source image defined
 * by nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowRowBorderParameters">Common parameters for nppiSumWindowRowBorder functions</a>.
 *
 */
NppStatus 
nppiSumWindowRowBorder_16s32f_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                            Npp32f * pDst, Npp32s nDstStep, 
                                            NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiSumWindowRowBorder_16s32f_C1R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                        Npp32f * pDst, Npp32s nDstStep, 
                                        NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Three channel 16-bit signed 1D (row) sum to 32f with border control.
 *
 * Apply Row Window Summation filter over a 1D mask region around each source
 * pixel for 3-channel 16-bit pixel input images with 32-bit floating point output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring row pixel values in a mask region of the source image defined
 * by nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowRowBorderParameters">Common parameters for nppiSumWindowRowBorder functions</a>.
 *
 */
NppStatus 
nppiSumWindowRowBorder_16s32f_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                            Npp32f * pDst, Npp32s nDstStep, 
                                            NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiSumWindowRowBorder_16s32f_C3R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                        Npp32f * pDst, Npp32s nDstStep, 
                                        NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed 1D (row) sum to 32f with border control.
 *
 * Apply Row Window Summation filter over a 1D mask region around each source
 * pixel for 4-channel 16-bit pixel input images with 32-bit floating point output.  
 * Result 32-bit floating point pixel is equal to the sum of the corresponding and
 * neighboring row pixel values in a mask region of the source image defined
 * by nMaskSize and nAnchor. 
 *
 * For common parameter descriptions, see <a href="#CommonFilterSumWindowRowBorderParameters">Common parameters for nppiSumWindowRowBorder functions</a>.
 *
 */
NppStatus 
nppiSumWindowRowBorder_16s32f_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                            Npp32f * pDst, Npp32s nDstStep, 
                                            NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiSumWindowRowBorder_16s32f_C4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                        Npp32f * pDst, Npp32s nDstStep, 
                                        NppiSize oROI, Npp32s nMaskSize, Npp32s nAnchor, NppiBorderType eBorderType);

/** @} image_filter_1D_window_row_sum_border */

/** @} image_1D_window_sum_border */

/** @defgroup image_convolution Convolution
 * The set convolution functions available in the library.
 * @{
 *
 */

/** @defgroup image_filter Filter
 * General purpose 2D convolution filter.
 *
 * Pixels under the mask are multiplied by the respective weights in the mask
 * and the results are summed. Before writing the result pixel the sum is scaled
 * back via division by nDivisor.
 *
 * <h3><a name="CommonFilterParameters">Common parameters for nppiFilter functions include:</a></h3>
 *
 * \param pSrc  \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param pKernel Pointer to the start address of the kernel coefficient array.
 *        Coeffcients are expected to be stored in reverse order.
 * \param oKernelSize Width and Height of the rectangular kernel.
 * \param oAnchor X and Y offsets of the kernel origin frame of reference
 *        relative to the source pixel.
 * \param nDivisor The factor by which the convolved summation from the Filter
 *        operation should be divided.  If equal to the sum of coefficients,
 *        this will keep the maximum result value within full scale.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned convolution filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterParameters">Common parameters for nppiFilter functions</a>.
 *
 */
NppStatus 
nppiFilter_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);
  
NppStatus 
nppiFilter_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                  const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor);
  
/**
 * Three channel 8-bit unsigned convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterParameters">Common parameters for nppiFilter functions</a>.
 *
 */
NppStatus 
nppiFilter_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);
                 
NppStatus 
nppiFilter_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                  const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor);
                 
/**
 * Four channel channel 8-bit unsigned convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterParameters">Common parameters for nppiFilter functions</a>.
 *
 */
NppStatus 
nppiFilter_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilter_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                  const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor);

/**
 * Four channel 8-bit unsigned convolution filter, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterParameters">Common parameters for nppiFilter functions</a>.
 *
 */
NppStatus 
nppiFilter_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                       const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilter_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                   const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor);

/**
 * Single channel 16-bit unsigned convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterParameters">Common parameters for nppiFilter functions</a>.
 *
 */
NppStatus 
nppiFilter_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                       const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);
  
NppStatus 
nppiFilter_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                   const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor);
  
/**
 * Three channel 16-bit unsigned convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterParameters">Common parameters for nppiFilter functions</a>.
 *
 */
NppStatus 
nppiFilter_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                       const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);
                 
NppStatus 
nppiFilter_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                   const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor);
                 
/**
 * Four channel channel 16-bit unsigned convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterParameters">Common parameters for nppiFilter functions</a>.
 *
 */
NppStatus 
nppiFilter_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                       const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilter_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                   const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor);

/**
 * Four channel 16-bit unsigned convolution filter, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterParameters">Common parameters for nppiFilter functions</a>.
 *
 */
NppStatus 
nppiFilter_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                        const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilter_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                   const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor);

/**
 * Single channel 16-bit convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterParameters">Common parameters for nppiFilter functions</a>.
 *
 */
NppStatus 
nppiFilter_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                       const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);
  
NppStatus 
nppiFilter_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                   const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor);
  
/**
 * Three channel 16-bit convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterParameters">Common parameters for nppiFilter functions</a>.
 *
 */
NppStatus 
nppiFilter_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                       const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);
                 
NppStatus 
nppiFilter_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                   const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor);
                 
/**
 * Four channel channel 16-bit convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterParameters">Common parameters for nppiFilter functions</a>.
 *
 */
NppStatus 
nppiFilter_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                       const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilter_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                   const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor);

/**
 * Four channel 16-bit convolution filter, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterParameters">Common parameters for nppiFilter functions</a>.
 *
 */
NppStatus 
nppiFilter_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                        const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppStreamContext nppStreamCtx);
 
NppStatus 
nppiFilter_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                    const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor);
 
/**
 * Single channel 32-bit float convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterParameters">Common parameters for nppiFilter functions</a>.
 *
 */
NppStatus
nppiFilter_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                       const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                   const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Two channel 32-bit float convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterParameters">Common parameters for nppiFilter functions</a>.
 *
 */
NppStatus
nppiFilter_32f_C2R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                       const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter_32f_C2R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                   const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Three channel 32-bit float convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterParameters">Common parameters for nppiFilter functions</a>.
 *
 */
NppStatus
nppiFilter_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                       const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                   const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Four channel 32-bit float convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterParameters">Common parameters for nppiFilter functions</a>.
 *
 */
NppStatus
nppiFilter_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                       const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                   const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Four channel 32-bit float convolution filter, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterParameters">Common parameters for nppiFilter functions</a>.
 *
 */
NppStatus
nppiFilter_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                        const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                    const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Single channel 64-bit float convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterParameters">Common parameters for nppiFilter functions</a>.
 *
 */
NppStatus
nppiFilter_64f_C1R_Ctx(const Npp64f * pSrc, Npp32s nSrcStep, Npp64f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                       const Npp64f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter_64f_C1R(const Npp64f * pSrc, Npp32s nSrcStep, Npp64f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                   const Npp64f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);


/** @} image_filter */

/** @defgroup image_filter_32f Filter32f
 * General purpose 2D convolution filter using floating point weights.
 *
 * Pixels under the mask are multiplied by the respective weights in the mask
 * and the results are summed. 
 *
 * <h3><a name="CommonFilter32fParameters">Common parameters for nppiFilter32f functions include:</a></h3>
 *
 * \param pSrc  \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param pKernel Pointer to the start address of the kernel coefficient array.
 *        Coeffcients are expected to be stored in reverse order.
 * \param oKernelSize Width and Height of the rectangular kernel.
 * \param oAnchor X and Y offsets of the kernel origin frame of reference
 *        relative to the source pixel.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */


/**
 * Single channel 8-bit unsigned convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_8u_C1R_Ctx(const Npp8u * pSrc, int nSrcStep, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                         const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);
        
NppStatus
nppiFilter32f_8u_C1R(const Npp8u * pSrc, int nSrcStep, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                     const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);
        
/**
 * Two channel 8-bit unsigned convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_8u_C2R_Ctx(const Npp8u * pSrc, int nSrcStep, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                         const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);
        
NppStatus
nppiFilter32f_8u_C2R(const Npp8u * pSrc, int nSrcStep, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                     const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);
        
/**
 * Three channel 8-bit unsigned convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_8u_C3R_Ctx(const Npp8u * pSrc, int nSrcStep, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                         const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);
        
NppStatus
nppiFilter32f_8u_C3R(const Npp8u * pSrc, int nSrcStep, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                     const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);
        
/**
 * Four channel 8-bit unsigned convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_8u_C4R_Ctx(const Npp8u * pSrc, int nSrcStep, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                         const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);
        
NppStatus
nppiFilter32f_8u_C4R(const Npp8u * pSrc, int nSrcStep, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                     const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);
        
/**
 * Four channel 8-bit unsigned convolution filter, ignorint alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_8u_AC4R_Ctx(const Npp8u * pSrc, int nSrcStep, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                          const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_8u_AC4R(const Npp8u * pSrc, int nSrcStep, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                      const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Single channel 8-bit signed convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_8s_C1R_Ctx(const Npp8s * pSrc, int nSrcStep, Npp8s * pDst, int nDstStep, NppiSize oSizeROI, 
                         const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_8s_C1R(const Npp8s * pSrc, int nSrcStep, Npp8s * pDst, int nDstStep, NppiSize oSizeROI, 
                     const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Two channel 8-bit signed convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_8s_C2R_Ctx(const Npp8s * pSrc, int nSrcStep, Npp8s * pDst, int nDstStep, NppiSize oSizeROI, 
                         const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_8s_C2R(const Npp8s * pSrc, int nSrcStep, Npp8s * pDst, int nDstStep, NppiSize oSizeROI, 
                     const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Three channel 8-bit signed convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_8s_C3R_Ctx(const Npp8s * pSrc, int nSrcStep, Npp8s * pDst, int nDstStep, NppiSize oSizeROI, 
                         const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_8s_C3R(const Npp8s * pSrc, int nSrcStep, Npp8s * pDst, int nDstStep, NppiSize oSizeROI, 
                     const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Four channel 8-bit signed convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_8s_C4R_Ctx(const Npp8s * pSrc, int nSrcStep, Npp8s * pDst, int nDstStep, NppiSize oSizeROI, 
                         const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_8s_C4R(const Npp8s * pSrc, int nSrcStep, Npp8s * pDst, int nDstStep, NppiSize oSizeROI, 
                     const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Four channel 8-bit signed convolution filter, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_8s_AC4R_Ctx(const Npp8s * pSrc, int nSrcStep, Npp8s * pDst, int nDstStep, NppiSize oSizeROI, 
                          const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_8s_AC4R(const Npp8s * pSrc, int nSrcStep, Npp8s * pDst, int nDstStep, NppiSize oSizeROI, 
                      const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Single channel 16-bit unsigned convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_16u_C1R_Ctx(const Npp16u * pSrc, int nSrcStep, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                          const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_16u_C1R(const Npp16u * pSrc, int nSrcStep, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                      const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Three channel 16-bit unsigned convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_16u_C3R_Ctx(const Npp16u * pSrc, int nSrcStep, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                          const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_16u_C3R(const Npp16u * pSrc, int nSrcStep, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                      const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Four channel 16-bit unsigned convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_16u_C4R_Ctx(const Npp16u * pSrc, int nSrcStep, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                          const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_16u_C4R(const Npp16u * pSrc, int nSrcStep, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                      const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Four channel 16-bit unsigned convolution filter, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_16u_AC4R_Ctx(const Npp16u * pSrc, int nSrcStep, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                           const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_16u_AC4R(const Npp16u * pSrc, int nSrcStep, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                       const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Single channel 16-bit convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_16s_C1R_Ctx(const Npp16s * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                          const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_16s_C1R(const Npp16s * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                      const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Three channel 16-bit convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_16s_C3R_Ctx(const Npp16s * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                          const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_16s_C3R(const Npp16s * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                      const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Four channel 16-bit convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_16s_C4R_Ctx(const Npp16s * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                          const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_16s_C4R(const Npp16s * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                      const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Four channel 16-bit convolution filter, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_16s_AC4R_Ctx(const Npp16s * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                           const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_16s_AC4R(const Npp16s * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                       const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);


/**
 * Single channel 32-bit convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_32s_C1R_Ctx(const Npp32s * pSrc, int nSrcStep, Npp32s * pDst, int nDstStep, NppiSize oSizeROI, 
                          const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_32s_C1R(const Npp32s * pSrc, int nSrcStep, Npp32s * pDst, int nDstStep, NppiSize oSizeROI, 
                      const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Three channel 32-bit convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_32s_C3R_Ctx(const Npp32s * pSrc, int nSrcStep, Npp32s * pDst, int nDstStep, NppiSize oSizeROI, 
                          const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_32s_C3R(const Npp32s * pSrc, int nSrcStep, Npp32s * pDst, int nDstStep, NppiSize oSizeROI, 
                      const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Four channel 32-bit convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_32s_C4R_Ctx(const Npp32s * pSrc, int nSrcStep, Npp32s * pDst, int nDstStep, NppiSize oSizeROI, 
                          const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_32s_C4R(const Npp32s * pSrc, int nSrcStep, Npp32s * pDst, int nDstStep, NppiSize oSizeROI, 
                      const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Four channel 32-bit convolution filter, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_32s_AC4R_Ctx(const Npp32s * pSrc, int nSrcStep, Npp32s * pDst, int nDstStep, NppiSize oSizeROI, 
                           const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_32s_AC4R(const Npp32s * pSrc, int nSrcStep, Npp32s * pDst, int nDstStep, NppiSize oSizeROI, 
                       const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Single channel 16-bit floating point convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_16f_C1R_Ctx(const Npp16f * pSrc, int nSrcStep, Npp16f * pDst, int nDstStep, NppiSize oSizeROI, 
                          const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_16f_C1R(const Npp16f * pSrc, int nSrcStep, Npp16f * pDst, int nDstStep, NppiSize oSizeROI, 
                      const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Three channel 16-bit floating point convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_16f_C3R_Ctx(const Npp16f * pSrc, int nSrcStep, Npp16f * pDst, int nDstStep, NppiSize oSizeROI, 
                          const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_16f_C3R(const Npp16f * pSrc, int nSrcStep, Npp16f * pDst, int nDstStep, NppiSize oSizeROI, 
                      const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Four channel 16-bit floating point convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_16f_C4R_Ctx(const Npp16f * pSrc, int nSrcStep, Npp16f * pDst, int nDstStep, NppiSize oSizeROI, 
                          const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_16f_C4R(const Npp16f * pSrc, int nSrcStep, Npp16f * pDst, int nDstStep, NppiSize oSizeROI, 
                      const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Single channel 8-bit unsigned to 16-bit signed convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_8u16s_C1R_Ctx(const Npp8u * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                            const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_8u16s_C1R(const Npp8u * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                        const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Three channel 8-bit unsigned to 16-bit signed convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_8u16s_C3R_Ctx(const Npp8u * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                            const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_8u16s_C3R(const Npp8u * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                        const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Four channel 8-bit unsigned to 16-bit signed convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_8u16s_C4R_Ctx(const Npp8u * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                            const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_8u16s_C4R(const Npp8u * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                        const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Four channel 8-bit unsigned to 16-bit signed convolution filter, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_8u16s_AC4R_Ctx(const Npp8u * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                             const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_8u16s_AC4R(const Npp8u * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                         const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Single channel 8-bit to 16-bit signed convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_8s16s_C1R_Ctx(const Npp8s * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                            const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_8s16s_C1R(const Npp8s * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                        const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Three channel 8-bit to 16-bit signed convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_8s16s_C3R_Ctx(const Npp8s * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                            const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_8s16s_C3R(const Npp8s * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                        const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Four channel 8-bit to 16-bit signed convolution filter.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_8s16s_C4R_Ctx(const Npp8s * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                            const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_8s16s_C4R(const Npp8s * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                        const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);

/**
 * Four channel 8-bit to 16-bit signed convolution filter, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilter32fParameters">Common parameters for nppiFilter32f functions</a>.
 *
 */
NppStatus
nppiFilter32f_8s16s_AC4R_Ctx(const Npp8s * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                             const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilter32f_8s16s_AC4R(const Npp8s * pSrc, int nSrcStep, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                         const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor);


/** @} image_filter_32f */

/** @defgroup image_filter_border FilterBorder
 * General purpose 2D convolution filter with border control.
 *
 * Pixels under the mask are multiplied by the respective weights in the mask
 * and the results are summed. Before writing the result pixel the sum is scaled
 * back via division by nDivisor. If any portion of the mask overlaps the source
 * image boundary the requested border type operation is applied to all mask pixels
 * which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.
 *
 * <h3><a name="CommonFilterBorderParameters">Common parameters for nppiFilterBorder functions include:</a></h3>
 *
 * \param pSrc  \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param pKernel Pointer to the start address of the kernel coefficient array.
 *        Coeffcients are expected to be stored in reverse order.
 * \param oKernelSize Width and Height of the rectangular kernel.
 * \param oAnchor X and Y offsets of the kernel origin frame of reference
 *        relative to the source pixel.
 * \param nDivisor The factor by which the convolved summation from the Filter
 *        operation should be divided.  If equal to the sum of coefficients,
 *        this will keep the maximum result value within full scale.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorderParameters">Common parameters for nppiFilterBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
  
NppStatus 
nppiFilterBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                        const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType);
  
/**
 * Three channel 8-bit unsigned convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorderParameters">Common parameters for nppiFilterBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBorder_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
                 
NppStatus 
nppiFilterBorder_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                        const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType);
                 
/**
 * Four channel channel 8-bit unsigned convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorderParameters">Common parameters for nppiFilterBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBorder_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBorder_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                        const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned convolution filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorderParameters">Common parameters for nppiFilterBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBorder_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBorder_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType);

/**
 * Single channel 16-bit unsigned convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorderParameters">Common parameters for nppiFilterBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBorder_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
  
NppStatus 
nppiFilterBorder_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType);
  
/**
 * Three channel 16-bit unsigned convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorderParameters">Common parameters for nppiFilterBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBorder_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
                 
NppStatus 
nppiFilterBorder_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType);
                 
/**
 * Four channel channel 16-bit unsigned convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorderParameters">Common parameters for nppiFilterBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBorder_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBorder_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned convolution filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorderParameters">Common parameters for nppiFilterBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBorder_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                              const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBorder_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType);

/**
 * Single channel 16-bit convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorderParameters">Common parameters for nppiFilterBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBorder_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
  
NppStatus 
nppiFilterBorder_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType);
  
/**
 * Three channel 16-bit convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorderParameters">Common parameters for nppiFilterBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBorder_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
                 
NppStatus 
nppiFilterBorder_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType);
                 
/**
 * Four channel channel 16-bit convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorderParameters">Common parameters for nppiFilterBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBorder_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBorder_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit convolution filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBorderParameters">Common parameters for nppiFilterBorder functions</a>.
 *
 */ 
NppStatus 
nppiFilterBorder_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                              const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
 
NppStatus 
nppiFilterBorder_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          const Npp32s * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, Npp32s nDivisor, NppiBorderType eBorderType);
 
/**
 * Single channel 32-bit float convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorderParameters">Common parameters for nppiFilterBorder functions</a>.
 *
 */
NppStatus
nppiFilterBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Two channel 32-bit float convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorderParameters">Common parameters for nppiFilterBorder functions</a>.
 *
 */
NppStatus
nppiFilterBorder_32f_C2R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder_32f_C2R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Three channel 32-bit float convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorderParameters">Common parameters for nppiFilterBorder functions</a>.
 *
 */
NppStatus
nppiFilterBorder_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 32-bit float convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorderParameters">Common parameters for nppiFilterBorder functions</a>.
 *
 */
NppStatus
nppiFilterBorder_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 32-bit float convolution filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorderParameters">Common parameters for nppiFilterBorder functions</a>.
 *
 */
NppStatus
nppiFilterBorder_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                              const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/** @} image_filter_border */

/** @defgroup image_filter_border_32f FilterBorder32f
 * General purpose 2D convolution filter using floating-point weights with border control.
 *
 * Pixels under the mask are multiplied by the respective weights in the mask
 * and the results are summed. Before writing the result pixel the sum is scaled
 * back via division by nDivisor. If any portion of the mask overlaps the source
 * image boundary the requested border type operation is applied to all mask pixels
 * which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.
 *
 * <h3><a name="CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions include:</a></h3>
 *
 * \param pSrc  \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param pKernel Pointer to the start address of the kernel coefficient array.
 *        Coeffcients are expected to be stored in reverse order.
 * \param oKernelSize Width and Height of the rectangular kernel.
 * \param oAnchor X and Y offsets of the kernel origin frame of reference
 *        relative to the source pixel.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */


/**
 * Single channel 8-bit unsigned convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_8u_C1R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                               const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
        
NppStatus
nppiFilterBorder32f_8u_C1R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                           const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);
        
/**
 * Two channel 8-bit unsigned convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_8u_C2R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                               const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
        
NppStatus
nppiFilterBorder32f_8u_C2R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                           const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);
        
/**
 * Three channel 8-bit unsigned convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_8u_C3R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                               const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
        
NppStatus
nppiFilterBorder32f_8u_C3R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                           const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);
        
/**
 * Four channel 8-bit unsigned convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_8u_C4R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                               const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
        
NppStatus
nppiFilterBorder32f_8u_C4R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                           const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);
        
/**
 * Four channel 8-bit unsigned convolution filter with border control, ignorint alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */ 
NppStatus
nppiFilterBorder32f_8u_AC4R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                                const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_8u_AC4R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, 
                            const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Single channel 8-bit signed convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_8s_C1R_Ctx(const Npp8s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8s * pDst, int nDstStep, NppiSize oSizeROI, 
                               const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_8s_C1R(const Npp8s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8s * pDst, int nDstStep, NppiSize oSizeROI, 
                           const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Two channel 8-bit signed convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_8s_C2R_Ctx(const Npp8s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8s * pDst, int nDstStep, NppiSize oSizeROI, 
                               const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_8s_C2R(const Npp8s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8s * pDst, int nDstStep, NppiSize oSizeROI, 
                           const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Three channel 8-bit signed convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_8s_C3R_Ctx(const Npp8s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8s * pDst, int nDstStep, NppiSize oSizeROI, 
                               const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_8s_C3R(const Npp8s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8s * pDst, int nDstStep, NppiSize oSizeROI, 
                           const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 8-bit signed convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_8s_C4R_Ctx(const Npp8s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8s * pDst, int nDstStep, NppiSize oSizeROI, 
                               const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_8s_C4R(const Npp8s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8s * pDst, int nDstStep, NppiSize oSizeROI, 
                           const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 8-bit signed convolution filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_8s_AC4R_Ctx(const Npp8s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8s * pDst, int nDstStep, NppiSize oSizeROI, 
                                const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_8s_AC4R(const Npp8s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8s * pDst, int nDstStep, NppiSize oSizeROI, 
                            const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Single channel 16-bit unsigned convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_16u_C1R_Ctx(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                                const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_16u_C1R(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                            const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Three channel 16-bit unsigned convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_16u_C3R_Ctx(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                                const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_16u_C3R(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                            const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_16u_C4R_Ctx(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                                const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_16u_C4R(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                            const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned convolution filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_16u_AC4R_Ctx(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                                 const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_16u_AC4R(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, 
                             const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Single channel 16-bit convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_16s_C1R_Ctx(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_16s_C1R(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                            const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Three channel 16-bit convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_16s_C3R_Ctx(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_16s_C3R(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                            const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_16s_C4R_Ctx(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_16s_C4R(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                            const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit convolution filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_16s_AC4R_Ctx(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                 const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_16s_AC4R(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                             const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Single channel 32-bit convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_32s_C1R_Ctx(const Npp32s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32s * pDst, int nDstStep, NppiSize oSizeROI, 
                                const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_32s_C1R(const Npp32s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32s * pDst, int nDstStep, NppiSize oSizeROI, 
                            const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Three channel 32-bit convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_32s_C3R_Ctx(const Npp32s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32s * pDst, int nDstStep, NppiSize oSizeROI, 
                                const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_32s_C3R(const Npp32s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32s * pDst, int nDstStep, NppiSize oSizeROI, 
                            const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 32-bit convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_32s_C4R_Ctx(const Npp32s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32s * pDst, int nDstStep, NppiSize oSizeROI, 
                                const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_32s_C4R(const Npp32s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32s * pDst, int nDstStep, NppiSize oSizeROI, 
                            const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 32-bit convolution filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_32s_AC4R_Ctx(const Npp32s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32s * pDst, int nDstStep, NppiSize oSizeROI, 
                                 const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_32s_AC4R(const Npp32s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32s * pDst, int nDstStep, NppiSize oSizeROI, 
                             const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Single channel 16-bit floating point convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_16f_C1R_Ctx(const Npp16f * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16f * pDst, int nDstStep, NppiSize oSizeROI, 
                                const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_16f_C1R(const Npp16f * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16f * pDst, int nDstStep, NppiSize oSizeROI, 
                            const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Three channel 16-bit floating point convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_16f_C3R_Ctx(const Npp16f * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16f * pDst, int nDstStep, NppiSize oSizeROI, 
                                const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_16f_C3R(const Npp16f * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16f * pDst, int nDstStep, NppiSize oSizeROI, 
                            const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit floating point convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_16f_C4R_Ctx(const Npp16f * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16f * pDst, int nDstStep, NppiSize oSizeROI, 
                                const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_16f_C4R(const Npp16f * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16f * pDst, int nDstStep, NppiSize oSizeROI, 
                            const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Single channel 8-bit unsigned to 16-bit signed convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_8u16s_C1R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                  const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_8u16s_C1R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                              const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Three channel 8-bit unsigned to 16-bit signed convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_8u16s_C3R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                  const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_8u16s_C3R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                              const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned to 16-bit signed convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_8u16s_C4R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                  const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_8u16s_C4R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                              const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned to 16-bit signed convolution filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_8u16s_AC4R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                   const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_8u16s_AC4R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                               const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Single channel 8-bit to 16-bit signed convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_8s16s_C1R_Ctx(const Npp8s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                  const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_8s16s_C1R(const Npp8s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                              const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Three channel 8-bit to 16-bit signed convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_8s16s_C3R_Ctx(const Npp8s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                  const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_8s16s_C3R(const Npp8s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                              const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 8-bit to 16-bit signed convolution filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_8s16s_C4R_Ctx(const Npp8s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                  const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_8s16s_C4R(const Npp8s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                              const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 8-bit to 16-bit signed convolution filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterBorder32fParameters">Common parameters for nppiFilterBorder32f functions</a>.
 *
 */
NppStatus
nppiFilterBorder32f_8s16s_AC4R_Ctx(const Npp8s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                                   const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterBorder32f_8s16s_AC4R(const Npp8s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, int nDstStep, NppiSize oSizeROI, 
                               const Npp32f * pKernel, NppiSize oKernelSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/** @} image_filter_border_32f */

/** @} image_convolution */

/** @defgroup image_2D_fixed_linear_filters 2D Fixed Linear Filters
 * The set of 2D fixed linear filtering functions available in the library.
 * @{
 *
 */

/** @defgroup image_filter_box FilterBox
 * Computes the average pixel values of the pixels under a rectangular mask.
 *
 * <h3><a name="CommonFilterBoxParameters">Common parameters for nppiFilterBox functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param oMaskSize Width and Height of the neighborhood region for the local
 *        Avg operation.
 * \param oAnchor X and Y offsets of the kernel origin frame of reference relative to
 *        the source pixel.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned box filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxParameters">Common parameters for nppiFilterBox functions</a>.
 *
 */
NppStatus 
nppiFilterBox_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBox_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                     NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Three channel 8-bit unsigned box filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxParameters">Common parameters for nppiFilterBox functions</a>.
 *
 */
NppStatus 
nppiFilterBox_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBox_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                     NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 8-bit unsigned box filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxParameters">Common parameters for nppiFilterBox functions</a>.
 *
 */
NppStatus 
nppiFilterBox_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBox_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                     NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 8-bit unsigned box filter, ignorting alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxParameters">Common parameters for nppiFilterBox functions</a>.
 *
 */
NppStatus 
nppiFilterBox_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBox_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Single channel 16-bit unsigned box filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxParameters">Common parameters for nppiFilterBox functions</a>.
 *
 */
NppStatus 
nppiFilterBox_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBox_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Three channel 16-bit unsigned box filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxParameters">Common parameters for nppiFilterBox functions</a>.
 *
 */
NppStatus 
nppiFilterBox_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBox_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 16-bit unsigned box filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxParameters">Common parameters for nppiFilterBox functions</a>.
 *
 */
NppStatus 
nppiFilterBox_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBox_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 16-bit unsigned box filter, ignorting alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxParameters">Common parameters for nppiFilterBox functions</a>.
 *
 */
NppStatus 
nppiFilterBox_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                           NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBox_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                       NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Single channel 16-bit box filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxParameters">Common parameters for nppiFilterBox functions</a>.
 *
 */
NppStatus 
nppiFilterBox_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBox_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Three channel 16-bit box filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxParameters">Common parameters for nppiFilterBox functions</a>.
 *
 */
NppStatus 
nppiFilterBox_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBox_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 16-bit box filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxParameters">Common parameters for nppiFilterBox functions</a>.
 *
 */
NppStatus 
nppiFilterBox_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBox_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 16-bit box filter, ignorting alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxParameters">Common parameters for nppiFilterBox functions</a>.
 *
 */
NppStatus 
nppiFilterBox_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                           NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBox_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                       NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Single channel 32-bit floating-point box filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxParameters">Common parameters for nppiFilterBox functions</a>.
 *
 */
NppStatus 
nppiFilterBox_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBox_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Three channel 32-bit floating-point box filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxParameters">Common parameters for nppiFilterBox functions</a>.
 *
 */
NppStatus 
nppiFilterBox_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBox_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 32-bit floating-point box filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxParameters">Common parameters for nppiFilterBox functions</a>.
 *
 */
NppStatus 
nppiFilterBox_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBox_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 32-bit floating-point box filter, ignorting alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxParameters">Common parameters for nppiFilterBox functions</a>.
 *
 */
NppStatus 
nppiFilterBox_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                           NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBox_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                       NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Single channel 64-bit floating-point box filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxParameters">Common parameters for nppiFilterBox functions</a>.
 *
 */
NppStatus 
nppiFilterBox_64f_C1R_Ctx(const Npp64f * pSrc, Npp32s nSrcStep, Npp64f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBox_64f_C1R(const Npp64f * pSrc, Npp32s nSrcStep, Npp64f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/** @} image_filter_box */

/** @defgroup image_filter_box_border FilterBoxBorder
 *
 * Computes the average pixel values of the pixels under a rectangular mask with border control.
 * If any portion of the mask overlaps the source image boundary the requested 
 * border type operation is applied to all mask pixels which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported. *
 *
 * <h3><a name="CommonFilterBoxBorderParameters">Common parameters for nppiFilterBoxBorder functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param oMaskSize Width and Height of the neighborhood region for the local
 *        Avg operation.
 * \param oAnchor X and Y offsets of the kernel origin frame of reference relative to
 *        the source pixel.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned box filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxBorderParameters">Common parameters for nppiFilterBoxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBoxBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBoxBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                           NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Three channel 8-bit unsigned box filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxBorderParameters">Common parameters for nppiFilterBoxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBoxBorder_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBoxBorder_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                           NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned box filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxBorderParameters">Common parameters for nppiFilterBoxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBoxBorder_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBoxBorder_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                           NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned box filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxBorderParameters">Common parameters for nppiFilterBoxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBoxBorder_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBoxBorder_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Single channel 16-bit unsigned box filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxBorderParameters">Common parameters for nppiFilterBoxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBoxBorder_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBoxBorder_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Three channel 16-bit unsigned box filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxBorderParameters">Common parameters for nppiFilterBoxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBoxBorder_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBoxBorder_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned box filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxBorderParameters">Common parameters for nppiFilterBoxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBoxBorder_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBoxBorder_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned box filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxBorderParameters">Common parameters for nppiFilterBoxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBoxBorder_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                 NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBoxBorder_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Single channel 16-bit box filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxBorderParameters">Common parameters for nppiFilterBoxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBoxBorder_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBoxBorder_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Three channel 16-bit box filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxBorderParameters">Common parameters for nppiFilterBoxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBoxBorder_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBoxBorder_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit box filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxBorderParameters">Common parameters for nppiFilterBoxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBoxBorder_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBoxBorder_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit box filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxBorderParameters">Common parameters for nppiFilterBoxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBoxBorder_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                 NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBoxBorder_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point box filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxBorderParameters">Common parameters for nppiFilterBoxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBoxBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBoxBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Three channel 32-bit floating-point box filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxBorderParameters">Common parameters for nppiFilterBoxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBoxBorder_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBoxBorder_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point box filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxBorderParameters">Common parameters for nppiFilterBoxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBoxBorder_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBoxBorder_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point box filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBoxBorderParameters">Common parameters for nppiFilterBoxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBoxBorder_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                 NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBoxBorder_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/** @} image_filter_box_border */

/** @defgroup image_filter_threshold_adaptive_box_border FilterThresholdAdaptiveBoxBorder
 *
 * Computes the average pixel values of the pixels under a square mask with border control.
 * If any portion of the mask overlaps the source image boundary the requested 
 * border type operation is applied to all mask pixels which fall outside of the source image.
 * Once the neighborhood average around a source pixel is determined the souce pixel is compared to the average - nDelta
 * and if the source pixel is greater than that average the corresponding destination pixel is set to nValGT, otherwise nValLE.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param oMaskSize Width and Height of the neighborhood region for the local
 *        Avg operation, Width and Height must be equal and odd.
 * \param nDelta Neighborhood average adjustment value
 * \param nValGT Destination output value if source pixel is greater than average.
 * \param nValLE Destination output value if source pixel is less than or equal to average.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned threshold adaptive box filter with border control.
 *
 */
NppStatus 
nppiFilterThresholdAdaptiveBoxBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                                NppiSize oMaskSize, Npp32f nDelta, Npp8u nValGT, Npp8u nValLE, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterThresholdAdaptiveBoxBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                            NppiSize oMaskSize, Npp32f nDelta, Npp8u nValGT, Npp8u nValLE, NppiBorderType eBorderType);

/** @} image_filter_threshold_adaptive_box_border */

/** @} image_2D_fixed_linear_filters */

/** @defgroup image_rank_filters Rank Filters
 * The set of functions providing min/max/median values for rectangular mask region with/without border available in the library.
 * @{
 *
 */

/** @defgroup image_filter_max FilterMax
 * Result pixel value is the maximum of pixel values under the rectangular mask region.
 *
 * <h3><a name="CommonFilterMaxParameters">Common parameters for nppiFilterMax functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param oMaskSize Width and Height of the neighborhood region for the local
 *        Max operation.
 * \param oAnchor X and Y offsets of the kernel origin frame of reference
 *        relative to the source pixel.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned maximum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxParameters">Common parameters for nppiFilterMax functions</a>.
 *
 */
NppStatus 
nppiFilterMax_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMax_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                     NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Three channel 8-bit unsigned maximum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxParameters">Common parameters for nppiFilterMax functions</a>.
 *
 */
NppStatus 
nppiFilterMax_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMax_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                     NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 8-bit unsigned maximum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxParameters">Common parameters for nppiFilterMax functions</a>.
 *
 */
NppStatus
nppiFilterMax_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMax_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                     NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 8-bit unsigned maximum filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxParameters">Common parameters for nppiFilterMax functions</a>.
 *
 */
NppStatus
nppiFilterMax_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMax_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                     NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Single channel 16-bit unsigned maximum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxParameters">Common parameters for nppiFilterMax functions</a>.
 *
 */
NppStatus 
nppiFilterMax_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMax_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Three channel 16-bit unsigned maximum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxParameters">Common parameters for nppiFilterMax functions</a>.
 *
 */
NppStatus 
nppiFilterMax_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMax_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 16-bit unsigned maximum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxParameters">Common parameters for nppiFilterMax functions</a>.
 *
 */
NppStatus
nppiFilterMax_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMax_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 16-bit unsigned maximum filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxParameters">Common parameters for nppiFilterMax functions</a>.
 *
 */
NppStatus
nppiFilterMax_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                           NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMax_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                       NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Single channel 16-bit signed maximum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxParameters">Common parameters for nppiFilterMax functions</a>.
 *
 */
NppStatus 
nppiFilterMax_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMax_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Three channel 16-bit signed maximum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxParameters">Common parameters for nppiFilterMax functions</a>.
 *
 */
NppStatus 
nppiFilterMax_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMax_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 16-bit signed maximum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxParameters">Common parameters for nppiFilterMax functions</a>.
 *
 */
NppStatus
nppiFilterMax_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMax_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 16-bit signed maximum filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxParameters">Common parameters for nppiFilterMax functions</a>.
 *
 */
NppStatus
nppiFilterMax_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                           NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMax_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                       NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Single channel 32-bit floating-point maximum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxParameters">Common parameters for nppiFilterMax functions</a>.
 *
 */
NppStatus 
nppiFilterMax_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMax_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Three channel 32-bit floating-point maximum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxParameters">Common parameters for nppiFilterMax functions</a>.
 *
 */
NppStatus 
nppiFilterMax_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMax_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 32-bit floating-point maximum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxParameters">Common parameters for nppiFilterMax functions</a>.
 *
 */
NppStatus
nppiFilterMax_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMax_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 32-bit floating-point maximum filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxParameters">Common parameters for nppiFilterMax functions</a>.
 *
 */
NppStatus
nppiFilterMax_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                           NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMax_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                       NppiSize oMaskSize, NppiPoint oAnchor);

/** @} image_filter_max */

/** @defgroup image_filter_max_border FilterMaxBorder
 * Result pixel value is the maximum of pixel values under the rectangular mask region with border control.
 * 
 * If any portion of the mask overlaps the source
 * image boundary the requested border type operation is applied to all mask pixels
 * which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.
 *
 * <h3><a name="CommonFilterMaxBorderParameters">Common parameters for nppiFilterMaxBorder functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param oMaskSize Width and Height of the neighborhood region for the local
 *        Max operation.
 * \param oAnchor X and Y offsets of the kernel origin frame of reference
 *        relative to the source pixel.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned maximum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxBorderParameters">Common parameters for nppiFilterMaxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterMaxBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMaxBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                           NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Three channel 8-bit unsigned maximum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxBorderParameters">Common parameters for nppiFilterMaxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterMaxBorder_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMaxBorder_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                           NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned maximum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxBorderParameters">Common parameters for nppiFilterMaxBorder functions</a>.
 *
 */
NppStatus
nppiFilterMaxBorder_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMaxBorder_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                           NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned maximum filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxBorderParameters">Common parameters for nppiFilterMaxBorder functions</a>.
 *
 */
NppStatus
nppiFilterMaxBorder_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMaxBorder_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Single channel 16-bit unsigned maximum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxBorderParameters">Common parameters for nppiFilterMaxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterMaxBorder_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMaxBorder_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Three channel 16-bit unsigned maximum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxBorderParameters">Common parameters for nppiFilterMaxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterMaxBorder_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMaxBorder_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned maximum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxBorderParameters">Common parameters for nppiFilterMaxBorder functions</a>.
 *
 */
NppStatus
nppiFilterMaxBorder_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMaxBorder_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned maximum filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxBorderParameters">Common parameters for nppiFilterMaxBorder functions</a>.
 *
 */
NppStatus
nppiFilterMaxBorder_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                 NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMaxBorder_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Single channel 16-bit signed maximum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxBorderParameters">Common parameters for nppiFilterMaxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterMaxBorder_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMaxBorder_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Three channel 16-bit signed maximum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxBorderParameters">Common parameters for nppiFilterMaxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterMaxBorder_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMaxBorder_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed maximum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxBorderParameters">Common parameters for nppiFilterMaxBorder functions</a>.
 *
 */
NppStatus
nppiFilterMaxBorder_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMaxBorder_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed maximum filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxBorderParameters">Common parameters for nppiFilterMaxBorder functions</a>.
 *
 */
NppStatus
nppiFilterMaxBorder_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                 NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMaxBorder_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point maximum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxBorderParameters">Common parameters for nppiFilterMaxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterMaxBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMaxBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Three channel 32-bit floating-point maximum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxBorderParameters">Common parameters for nppiFilterMaxBorder functions</a>.
 *
 */
NppStatus 
nppiFilterMaxBorder_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMaxBorder_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point maximum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxBorderParameters">Common parameters for nppiFilterMaxBorder functions</a>.
 *
 */
NppStatus
nppiFilterMaxBorder_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMaxBorder_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point maximum filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMaxBorderParameters">Common parameters for nppiFilterMaxBorder functions</a>.
 *
 */
NppStatus
nppiFilterMaxBorder_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                 NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMaxBorder_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/** @} image_filter_max_border */

/** @defgroup image_filter_min FilterMin
 * Result pixel value is the minimum of pixel values under the rectangular mask region.
 *
 * <h3><a name="CommonFilterMinParameters">Common parameters for nppiFilterMin functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param oMaskSize Width and Height of the neighborhood region for the local
 *        Min operation.
 * \param oAnchor X and Y offsets of the kernel origin frame of reference
 *        relative to the source pixel.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned minimum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinParameters">Common parameters for nppiFilterMin functions</a>.
 *
 */
NppStatus 
nppiFilterMin_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMin_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                     NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Three channel 8-bit unsigned minimum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinParameters">Common parameters for nppiFilterMin functions</a>.
 *
 */
NppStatus 
nppiFilterMin_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMin_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                     NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 8-bit unsigned minimum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinParameters">Common parameters for nppiFilterMin functions</a>.
 *
 */
NppStatus 
nppiFilterMin_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMin_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                     NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 8-bit unsigned minimum filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinParameters">Common parameters for nppiFilterMin functions</a>.
 *
 */
NppStatus 
nppiFilterMin_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMin_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Single channel 16-bit unsigned minimum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinParameters">Common parameters for nppiFilterMin functions</a>.
 *
 */
NppStatus 
nppiFilterMin_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMin_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Three channel 16-bit unsigned minimum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinParameters">Common parameters for nppiFilterMin functions</a>.
 *
 */
NppStatus 
nppiFilterMin_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMin_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 16-bit unsigned minimum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinParameters">Common parameters for nppiFilterMin functions</a>.
 *
 */
NppStatus 
nppiFilterMin_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMin_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 16-bit unsigned minimum filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinParameters">Common parameters for nppiFilterMin functions</a>.
 *
 */
NppStatus 
nppiFilterMin_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                           NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMin_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                       NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Single channel 16-bit signed minimum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinParameters">Common parameters for nppiFilterMin functions</a>.
 *
 */
NppStatus 
nppiFilterMin_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMin_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Three channel 16-bit signed minimum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinParameters">Common parameters for nppiFilterMin functions</a>.
 *
 */
NppStatus 
nppiFilterMin_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMin_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 16-bit signed minimum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinParameters">Common parameters for nppiFilterMin functions</a>.
 *
 */
NppStatus 
nppiFilterMin_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMin_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 16-bit signed minimum filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinParameters">Common parameters for nppiFilterMin functions</a>.
 *
 */
NppStatus 
nppiFilterMin_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                           NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMin_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                       NppiSize oMaskSize, NppiPoint oAnchor);


/**
 * Single channel 32-bit floating-point minimum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinParameters">Common parameters for nppiFilterMin functions</a>.
 *
 */
NppStatus 
nppiFilterMin_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMin_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Three channel 32-bit floating-point minimum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinParameters">Common parameters for nppiFilterMin functions</a>.
 *
 */
NppStatus 
nppiFilterMin_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMin_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 32-bit floating-point minimum filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinParameters">Common parameters for nppiFilterMin functions</a>.
 *
 */
NppStatus 
nppiFilterMin_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMin_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                      NppiSize oMaskSize, NppiPoint oAnchor);

/**
 * Four channel 32-bit floating-point minimum filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinParameters">Common parameters for nppiFilterMin functions</a>.
 *
 */
NppStatus 
nppiFilterMin_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                           NppiSize oMaskSize, NppiPoint oAnchor, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMin_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                       NppiSize oMaskSize, NppiPoint oAnchor);

/** @} image_filter_min */

/** @defgroup image_filter_min_border FilterMinBorder
 * Result pixel value is the minimum of pixel values under the rectangular mask region with border control. 
 *
 * If any portion of the mask overlaps the source
 * image boundary the requested border type operation is applied to all mask pixels
 * which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.
 *
 * <h3><a name="CommonFilterMinBorderParameters">Common parameters for nppiFilterMinBorder functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param oMaskSize Width and Height of the neighborhood region for the local
 *        Min operation.
 * \param oAnchor X and Y offsets of the kernel origin frame of reference
 *        relative to the source pixel.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned minimum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinBorderParameters">Common parameters for nppiFilterMinBorder functions</a>.
 *
 */
NppStatus 
nppiFilterMinBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMinBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                           NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Three channel 8-bit unsigned minimum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinBorderParameters">Common parameters for nppiFilterMinBorder functions</a>.
 *
 */
NppStatus 
nppiFilterMinBorder_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMinBorder_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                           NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned minimum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinBorderParameters">Common parameters for nppiFilterMinBorder functions</a>.
 *
 */
NppStatus
nppiFilterMinBorder_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMinBorder_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                           NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned minimum filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinBorderParameters">Common parameters for nppiFilterMinBorder functions</a>.
 *
 */
NppStatus
nppiFilterMinBorder_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMinBorder_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Single channel 16-bit unsigned minimum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinBorderParameters">Common parameters for nppiFilterMinBorder functions</a>.
 *
 */
NppStatus 
nppiFilterMinBorder_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMinBorder_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Three channel 16-bit unsigned minimum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinBorderParameters">Common parameters for nppiFilterMinBorder functions</a>.
 *
 */
NppStatus 
nppiFilterMinBorder_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMinBorder_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned minimum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinBorderParameters">Common parameters for nppiFilterMinBorder functions</a>.
 *
 */
NppStatus
nppiFilterMinBorder_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMinBorder_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned minimum filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinBorderParameters">Common parameters for nppiFilterMinBorder functions</a>.
 *
 */
NppStatus
nppiFilterMinBorder_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                 NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMinBorder_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Single channel 16-bit signed minimum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinBorderParameters">Common parameters for nppiFilterMinBorder functions</a>.
 *
 */
NppStatus 
nppiFilterMinBorder_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMinBorder_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Three channel 16-bit signed minimum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinBorderParameters">Common parameters for nppiFilterMinBorder functions</a>.
 *
 */
NppStatus 
nppiFilterMinBorder_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMinBorder_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed minimum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinBorderParameters">Common parameters for nppiFilterMinBorder functions</a>.
 *
 */
NppStatus
nppiFilterMinBorder_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMinBorder_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed minimum filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinBorderParameters">Common parameters for nppiFilterMinBorder functions</a>.
 *
 */
NppStatus
nppiFilterMinBorder_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                 NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMinBorder_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point minimum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinBorderParameters">Common parameters for nppiFilterMinBorder functions</a>.
 *
 */
NppStatus 
nppiFilterMinBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMinBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Three channel 32-bit floating-point minimum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinBorderParameters">Common parameters for nppiFilterMinBorder functions</a>.
 *
 */
NppStatus 
nppiFilterMinBorder_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMinBorder_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point minimum filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinBorderParameters">Common parameters for nppiFilterMinBorder functions</a>.
 *
 */
NppStatus
nppiFilterMinBorder_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMinBorder_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point minimum filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMinBorderParameters">Common parameters for nppiFilterMinBorder functions</a>.
 *
 */
NppStatus
nppiFilterMinBorder_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                 NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterMinBorder_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             NppiSize oMaskSize, NppiPoint oAnchor, NppiBorderType eBorderType);

/** @} image_filter_min_border */

/** @defgroup image_filter_median FilterMedian
 * Result pixel value is the median of pixel values under the rectangular mask region.
 *
 * <h3><a name="CommonFilterMedianParameters">Common parameters for nppiFilterMedian functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param oMaskSize Width and Height of the neighborhood region for the local
 *        Median operation.
 * \param oAnchor X and Y offsets of the kernel origin frame of reference
 *        relative to the source pixel.
 * \param pBuffer Pointer to the user-allocated scratch buffer required for the Median operation.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * <h3><a name="CommonFilterMedianGetBufferSizeParameters">Common parameters for nppiFilterMedianGetBufferSize functions include:</a></h3>
 *
 * \param oSizeROI \ref roi_specification.
 * \param oMaskSize Width and Height of the neighborhood region for the local Median operation.
 * \param nBufferSize Pointer to the size of the scratch buffer required for the Median operation.
 * \return \ref image_data_error_codes
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned median filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianParameters">Common parameters for nppiFilterMedian functions</a>.
 *
 */
NppStatus 
nppiFilterMedian_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedian_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                        NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer);

/**
 * Three channel 8-bit unsigned median filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianParameters">Common parameters for nppiFilterMedian functions</a>.
 *
 */
NppStatus 
nppiFilterMedian_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedian_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                        NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer);

/**
 * Four channel 8-bit unsigned median filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianParameters">Common parameters for nppiFilterMedian functions</a>.
 *
 */
NppStatus 
nppiFilterMedian_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                            NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedian_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                        NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer);

/**
 * Four channel 8-bit unsigned median filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianParameters">Common parameters for nppiFilterMedian functions</a>.
 *
 */
NppStatus 
nppiFilterMedian_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedian_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer);

/**
 * Single channel 16-bit unsigned median filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianParameters">Common parameters for nppiFilterMedian functions</a>.
 *
 */
NppStatus 
nppiFilterMedian_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedian_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer);

/**
 * Three channel 16-bit unsigned median filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianParameters">Common parameters for nppiFilterMedian functions</a>.
 *
 */
NppStatus 
nppiFilterMedian_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedian_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer);

/**
 * Four channel 16-bit unsigned median filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianParameters">Common parameters for nppiFilterMedian functions</a>.
 *
 */
NppStatus 
nppiFilterMedian_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedian_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer);

/**
 * Four channel 16-bit unsigned median filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianParameters">Common parameters for nppiFilterMedian functions</a>.
 *
 */
NppStatus 
nppiFilterMedian_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                              NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedian_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer);

/**
 * Single channel 16-bit signed median filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianParameters">Common parameters for nppiFilterMedian functions</a>.
 *
 */
NppStatus 
nppiFilterMedian_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedian_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer);

/**
 * Three channel 16-bit signed median filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianParameters">Common parameters for nppiFilterMedian functions</a>.
 *
 */
NppStatus 
nppiFilterMedian_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedian_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer);

/**
 * Four channel 16-bit signed median filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianParameters">Common parameters for nppiFilterMedian functions</a>.
 *
 */
NppStatus 
nppiFilterMedian_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedian_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer);

/**
 * Four channel 16-bit signed median filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianParameters">Common parameters for nppiFilterMedian functions</a>.
 *
 */
NppStatus 
nppiFilterMedian_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                              NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedian_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer);


/**
 * Single channel 32-bit floating-point median filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianParameters">Common parameters for nppiFilterMedian functions</a>.
 *
 */
NppStatus 
nppiFilterMedian_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedian_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer);

/**
 * Three channel 32-bit floating-point median filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianParameters">Common parameters for nppiFilterMedian functions</a>.
 *
 */
NppStatus 
nppiFilterMedian_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedian_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer);

/**
 * Four channel 32-bit floating-point median filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianParameters">Common parameters for nppiFilterMedian functions</a>.
 *
 */
NppStatus 
nppiFilterMedian_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                             NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedian_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                         NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer);

/**
 * Four channel 32-bit floating-point median filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianParameters">Common parameters for nppiFilterMedian functions</a>.
 *
 */
NppStatus 
nppiFilterMedian_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                              NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer, NppStreamContext nppStreamCtx);
                       
NppStatus 
nppiFilterMedian_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                          NppiSize oMaskSize, NppiPoint oAnchor, Npp8u * pBuffer);
                       


/**
 * Single channel 8-bit unsigned median filter scratch memory size.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianGetBufferSizeParameters">Common parameters for nppiFilterMedianGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterMedianGetBufferSize_8u_C1R_Ctx(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedianGetBufferSize_8u_C1R(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize);

/**
 * Three channel 8-bit unsigned median filter scratch memory size.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterMedianGetBufferSizeParameters">Common parameters for nppiFilterMedianGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterMedianGetBufferSize_8u_C3R_Ctx(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedianGetBufferSize_8u_C3R(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize);

/**
 * Four channel 8-bit unsigned median filter scratch memory size.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianGetBufferSizeParameters">Common parameters for nppiFilterMedianGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterMedianGetBufferSize_8u_C4R_Ctx(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedianGetBufferSize_8u_C4R(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize);

/**
 * Four channel 8-bit unsigned median filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianGetBufferSizeParameters">Common parameters for nppiFilterMedianGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterMedianGetBufferSize_8u_AC4R_Ctx(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedianGetBufferSize_8u_AC4R(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize);

/**
 * Single channel 16-bit unsigned median filter scratch memory size.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianGetBufferSizeParameters">Common parameters for nppiFilterMedianGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterMedianGetBufferSize_16u_C1R_Ctx(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedianGetBufferSize_16u_C1R(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize);

/**
 * Three channel 16-bit unsigned median filter scratch memory size.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianGetBufferSizeParameters">Common parameters for nppiFilterMedianGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterMedianGetBufferSize_16u_C3R_Ctx(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedianGetBufferSize_16u_C3R(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize);

/**
 * Four channel 16-bit unsigned median filter scratch memory size.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianGetBufferSizeParameters">Common parameters for nppiFilterMedianGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterMedianGetBufferSize_16u_C4R_Ctx(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedianGetBufferSize_16u_C4R(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize);

/**
 * Four channel 16-bit unsigned median filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianGetBufferSizeParameters">Common parameters for nppiFilterMedianGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterMedianGetBufferSize_16u_AC4R_Ctx(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedianGetBufferSize_16u_AC4R(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize);

/**
 * Single channel 16-bit signed median filter scratch memory size.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianGetBufferSizeParameters">Common parameters for nppiFilterMedianGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterMedianGetBufferSize_16s_C1R_Ctx(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedianGetBufferSize_16s_C1R(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize);

/**
 * Three channel 16-bit signed median filter scratch memory size.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianGetBufferSizeParameters">Common parameters for nppiFilterMedianGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterMedianGetBufferSize_16s_C3R_Ctx(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedianGetBufferSize_16s_C3R(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize);

/**
 * Four channel 16-bit signed median filter scratch memory size.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianGetBufferSizeParameters">Common parameters for nppiFilterMedianGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterMedianGetBufferSize_16s_C4R_Ctx(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedianGetBufferSize_16s_C4R(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize);

/**
 * Four channel 16-bit signed median filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianGetBufferSizeParameters">Common parameters for nppiFilterMedianGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterMedianGetBufferSize_16s_AC4R_Ctx(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedianGetBufferSize_16s_AC4R(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize);


/**
 * Single channel 32-bit floating-point median filter scratch memory size.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianGetBufferSizeParameters">Common parameters for nppiFilterMedianGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterMedianGetBufferSize_32f_C1R_Ctx(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedianGetBufferSize_32f_C1R(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize);

/**
 * Three channel 32-bit floating-point median filter scratch memory size.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianGetBufferSizeParameters">Common parameters for nppiFilterMedianGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterMedianGetBufferSize_32f_C3R_Ctx(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedianGetBufferSize_32f_C3R(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize);

/**
 * Four channel 32-bit floating-point median filter scratch memory size.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianGetBufferSizeParameters">Common parameters for nppiFilterMedianGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterMedianGetBufferSize_32f_C4R_Ctx(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedianGetBufferSize_32f_C4R(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize);

/**
 * Four channel 32-bit floating-point median filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterMedianGetBufferSizeParameters">Common parameters for nppiFilterMedianGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterMedianGetBufferSize_32f_AC4R_Ctx(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterMedianGetBufferSize_32f_AC4R(NppiSize oSizeROI, NppiSize oMaskSize, Npp32u * nBufferSize);

/** @} image_filter_median */

/** @} image_rank_filters */

/** @defgroup fixed_filters Fixed Filters
 *
 * Fixed filters perform linear filtering operations (such as convolutions) with predefined kernels
 * of fixed sizes.  Note that this section also contains a few dynamic kernel filters, namely GaussAdvanced and Bilateral.
 * 
 * Some of the fixed filters have versions with border control.   For these functions, if any portion 
 * of the mask overlaps the source image boundary the requested border type operation is applied to 
 * all mask pixels which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported for these functions.
 *
 * @{
 *
 */

/** @defgroup image_filter_prewitt FilterPrewitt 
 * Filters the image using a Prewitt filter kernel.
 *
 * <h3><a name="CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/** @name FilterPrewittHoriz 
 *
 * Filters the image using a horizontal Prewitt filter kernel:
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *    1 &  1 &  1 \\
 *    0 &  0 &  0 \\
 *   -1 & -1 & -1 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned horizontal Prewitt filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHoriz_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHoriz_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Three channel 8-bit unsigned horizontal Prewitt filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHoriz_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHoriz_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 8-bit unsigned horizontal Prewitt filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHoriz_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHoriz_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 8-bit unsigned horizontal Prewitt filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHoriz_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHoriz_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Single channel 16-bit signed horizontal Prewitt filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHoriz_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHoriz_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Three channel 16-bit signed horizontal Prewitt filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHoriz_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHoriz_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 16-bit signed horizontal Prewitt filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHoriz_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHoriz_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 16-bit signed horizontal Prewitt filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHoriz_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHoriz_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Single channel 32-bit floating-point horizontal Prewitt filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHoriz_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHoriz_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Three channel 32-bit floating-point horizontal Prewitt filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHoriz_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHoriz_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 32-bit floating-point horizontal Prewitt filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHoriz_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHoriz_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 32-bit floating-point horizontal Prewitt filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHoriz_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHoriz_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/** @} FilterPrewittHoriz */

/** @name FilterPrewittVert 
 *
 * Filters the image using a vertical Prewitt filter kernel:
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *   -1 & 0 & 1 \\
 *   -1 & 0 & 1 \\
 *   -1 & 0 & 1 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned vertical Prewitt filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVert_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVert_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Three channel 8-bit unsigned vertical Prewitt filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVert_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVert_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 8-bit unsigned vertical Prewitt filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVert_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVert_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 8-bit unsigned vertical Prewitt filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVert_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVert_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Single channel 16-bit signed vertical Prewitt filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVert_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVert_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Three channel 16-bit signed vertical Prewitt filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVert_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVert_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 16-bit signed vertical Prewitt filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVert_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVert_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 16-bit signed vertical Prewitt filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVert_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVert_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Single channel 32-bit floating-point vertical Prewitt filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVert_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVert_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Three channel 32-bit floating-point vertical Prewitt filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVert_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVert_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 32-bit floating-point vertical Prewitt filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVert_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVert_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 32-bit floating-point vertical Prewitt filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittParameters">Common parameters for nppiFilterPrewitt functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVert_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVert_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/** @} FilterPrewittVert */

/** @} image_filter_prewitt */

/** @defgroup image_filter_prewitt_border FilterPrewittBorder 
 * Filters the image using a Prewitt filter kernel with border control.
 *
 * <h3><a name="CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/** @name FilterPrewittHorizBorder 
 *
 * Filters the image using a horizontal Prewitt filter kernel with border control. If any portion of the mask overlaps the source
 * image boundary the requested border type operation is applied to all mask pixels
 * which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *    1 &  1 &  1 \\
 *    0 &  0 &  0 \\
 *   -1 & -1 & -1 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned horizontal Prewitt filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHorizBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHorizBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Three channel 8-bit unsigned horizontal Prewitt filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHorizBorder_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHorizBorder_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned horizontal Prewitt filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHorizBorder_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHorizBorder_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned horizontal Prewitt filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHorizBorder_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHorizBorder_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Single channel 16-bit signed horizontal Prewitt filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHorizBorder_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHorizBorder_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Three channel 16-bit signed horizontal Prewitt filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHorizBorder_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHorizBorder_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed horizontal Prewitt filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHorizBorder_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHorizBorder_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed horizontal Prewitt filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHorizBorder_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHorizBorder_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point horizontal Prewitt filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHorizBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHorizBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Three channel 32-bit floating-point horizontal Prewitt filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHorizBorder_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHorizBorder_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point horizontal Prewitt filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHorizBorder_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHorizBorder_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point horizontal Prewitt filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittHorizBorder_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittHorizBorder_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/** @} FilterPrewittHorizBorder */

/** @name FilterPrewittVertBorder 
 *
 * Filters the image using a vertical Prewitt filter kernel with border control. If any portion of the mask overlaps the source
 * image boundary the requested border type operation is applied to all mask pixels
 * which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *   -1 & 0 & 1 \\
 *   -1 & 0 & 1 \\
 *   -1 & 0 & 1 \\
 *  \end{array} \right);
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned vertical Prewitt filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVertBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVertBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Three channel 8-bit unsigned vertical Prewitt filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVertBorder_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVertBorder_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned vertical Prewitt filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVertBorder_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVertBorder_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned vertical Prewitt filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVertBorder_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVertBorder_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Single channel 16-bit signed vertical Prewitt filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVertBorder_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVertBorder_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Three channel 16-bit signed vertical Prewitt filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVertBorder_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVertBorder_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed vertical Prewitt filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVertBorder_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVertBorder_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed vertical Prewitt filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVertBorder_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVertBorder_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point vertical Prewitt filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVertBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVertBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Three channel 32-bit floating-point vertical Prewitt filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVertBorder_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVertBorder_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point vertical Prewitt filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVertBorder_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVertBorder_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point vertical Prewitt filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterPrewittBorderParameters">Common parameters for nppiFilterPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiFilterPrewittVertBorder_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterPrewittVertBorder_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/** @} FilterPrewittVertBorder */

/** @} image_filter_prewitt_border */

/** @defgroup image_filter_scharr FilterScharr 
 * Filters the image using a Scharr filter kernel.
 *
 * <h3><a name="CommonFilterScharrParameters">Common parameters for nppiFilterScharr functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/** @name FilterScharrHoriz 
 *
 * Filters the image using a horizontal Scharr filter kernel:
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *    3 &  10 &  3 \\
 *    0 &   0 &  0 \\
 *   -3 & -10 & -3 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned to 16-bit signed horizontal Scharr filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterScharrParameters">Common parameters for nppiFilterScharr functions</a>.
 *
 */
NppStatus 
nppiFilterScharrHoriz_8u16s_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterScharrHoriz_8u16s_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Single channel 8-bit signed to 16-bit signed horizontal Scharr filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterScharrParameters">Common parameters for nppiFilterScharr functions</a>.
 *
 */
NppStatus 
nppiFilterScharrHoriz_8s16s_C1R_Ctx(const Npp8s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterScharrHoriz_8s16s_C1R(const Npp8s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Single channel 32-bit floating-point horizontal Scharr filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterScharrParameters">Common parameters for nppiFilterScharr functions</a>.
 *
 */
NppStatus 
nppiFilterScharrHoriz_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterScharrHoriz_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/** @} FilterScharrHoriz */

/** @name FilterScharrVert 
 *
 * Filters the image using a vertical Scharr filter kernel:
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *     3 &   0 &  -3 \\
 *    10 &   0 & -10 \\
 *     3 &   0 &  -3 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned to 16-bit signed vertical Scharr filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterScharrParameters">Common parameters for nppiFilterScharr functions</a>.
 *
 */
NppStatus 
nppiFilterScharrVert_8u16s_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterScharrVert_8u16s_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Single channel 8-bit signed to 16-bit signed vertical Scharr filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterScharrParameters">Common parameters for nppiFilterScharr functions</a>.
 *
 */
NppStatus 
nppiFilterScharrVert_8s16s_C1R_Ctx(const Npp8s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterScharrVert_8s16s_C1R(const Npp8s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Single channel 32-bit floating-point vertical Scharr filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterScharrParameters">Common parameters for nppiFilterScharr functions</a>.
 *
 */
NppStatus 
nppiFilterScharrVert_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterScharrVert_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/** @} FilterScharrVert */

/** @} image_filter_scharr */

/** @defgroup image_filter_scharr_border FilterScharrBorder 
 * Filters the image using a Scharr filter kernel with border control.
 *
 * <h3><a name="CommonFilterScharrBorderParameters">Common parameters for nppiFilterScharrBorder functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/** @name FilterScharrHorizBorder 
 *
 * Filters the image using a horizontal Scharr filter kernel with border control:
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *    3 &  10 &  3 \\
 *    0 &   0 &  0 \\
 *   -3 & -10 & -3 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned to 16-bit signed horizontal Scharr filter kernel with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterScharrBorderParameters">Common parameters for nppiFilterScharrBorder functions</a>.
 *
 */
NppStatus 
nppiFilterScharrHorizBorder_8u16s_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterScharrHorizBorder_8u16s_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Single channel 8-bit signed to 16-bit signed horizontal Scharr filter kernel with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterScharrBorderParameters">Common parameters for nppiFilterScharrBorder functions</a>.
 *
 */
NppStatus 
nppiFilterScharrHorizBorder_8s16s_C1R_Ctx(const Npp8s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterScharrHorizBorder_8s16s_C1R(const Npp8s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point horizontal Scharr filter kernel with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterScharrBorderParameters">Common parameters for nppiFilterScharrBorder functions</a>.
 *
 */
NppStatus 
nppiFilterScharrHorizBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterScharrHorizBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/** @} FilterScharrHorizBorder */

/** @name FilterScharrVertBorder 
 *
 * Filters the image using a vertical Scharr filter kernel kernel with border control:
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *     3 &   0 &  -3 \\
 *    10 &   0 & -10 \\
 *     3 &   0 &  -3 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned to 16-bit signed vertical Scharr filter kernel with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterScharrBorderParameters">Common parameters for nppiFilterScharrBorder functions</a>.
 *
 */
NppStatus 
nppiFilterScharrVertBorder_8u16s_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterScharrVertBorder_8u16s_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Single channel 8-bit signed to 16-bit signed vertical Scharr filter kernel with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterScharrBorderParameters">Common parameters for nppiFilterScharrBorder functions</a>.
 *
 */
NppStatus 
nppiFilterScharrVertBorder_8s16s_C1R_Ctx(const Npp8s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterScharrVertBorder_8s16s_C1R(const Npp8s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point vertical Scharr filter kernel with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterScharrBorderParameters">Common parameters for nppiFilterScharrBorder functions</a>.
 *
 */
NppStatus 
nppiFilterScharrVertBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterScharrVertBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/** @} FilterScharrVertBorder */

/** @} image_filter_scharr_border */

/** @defgroup image_filter_sobel FilterSobel 
 * Filters the image using a Sobel filter kernel.
 *
 * <h3><a name="CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param eMaskSize Enumeration value specifying the mask size.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/** @name FilterSobelHoriz 
 *              
 * Filters the image using a horizontal Sobel filter kernel:
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *    1 &  2 &  1 \\
 *    0 &  0 &  0 \\
 *   -1 & -2 & -1 \\
 *  \end{array} \right)
 *  \left( \begin{array}{rrrrr}
 *    1  &  4 &   6 &  4 &  1 \\
 *    2  &  8 &  12 &  8 &  2 \\
 *    0  &  0 &   0 &  0 &  0 \\
 *    -2 & -8 & -12 & -8 & -2 \\
 *    -1 & -4 &  -6 & -4 & -1 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned horizontal Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHoriz_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHoriz_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Three channel 8-bit unsigned horizontal Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHoriz_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHoriz_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 8-bit unsigned horizontal Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHoriz_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHoriz_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 16-bit signed horizontal Sobel filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHoriz_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);
NppStatus 
nppiFilterSobelHoriz_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Single channel 16-bit signed horizontal Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHoriz_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHoriz_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Three channel 16-bit signed horizontal Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHoriz_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHoriz_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 16-bit signed horizontal Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHoriz_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHoriz_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 8-bit unsigned horizontal Sobel filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHoriz_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHoriz_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Single channel 32-bit floating-point horizontal Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHoriz_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHoriz_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Three channel 32-bit floating-point horizontal Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHoriz_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHoriz_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 32-bit floating-point horizontal Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHoriz_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHoriz_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 32-bit floating-point horizontal Sobel filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHoriz_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHoriz_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Single channel 8-bit unsigned to 16-bit signed horizontal Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHoriz_8u16s_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                   NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHoriz_8u16s_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize);

/**
 * Single channel 8-bit signed to 16-bit signed horizontal Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHoriz_8s16s_C1R_Ctx(const Npp8s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                   NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHoriz_8s16s_C1R(const Npp8s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize);

/**
 * Single channel 32-bit floating-point horizontal Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHorizMask_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHorizMask_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 NppiMaskSize eMaskSize);


/** @} FilterSobelHoriz */

/** @name FilterSobelVert 
 *
 * Filters the image using a vertical Sobel filter kernel:
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *    -1 & 0 & 1 \\
 *    -2 & 0 & 2 \\
 *    -1 & 0 & 1 \\
 *  \end{array} \right)
 *  \left( \begin{array}{rrrrr}
 *    -1 &  -2 & 0 &  2 & 1 \\
 *    -4 &  -8 & 0 &  8 & 4 \\
 *    -6 & -12 & 0 & 12 & 6 \\
 *    -4 &  -8 & 0 &  8 & 4 \\
 *    -1 &  -2 & 0 &  2 & 1 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned vertical Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVert_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVert_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Three channel 8-bit unsigned vertical Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVert_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVert_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 8-bit unsigned vertical Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVert_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVert_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 16-bit signed vertical Sobel filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVert_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVert_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Single channel 16-bit signed vertical Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVert_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVert_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Three channel 16-bit signed vertical Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVert_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVert_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 16-bit signed vertical Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVert_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVert_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 8-bit unsigned vertical Sobel filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVert_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVert_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Single channel 32-bit floating-point vertical Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVert_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVert_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Three channel 32-bit floating-point vertical Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVert_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVert_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 32-bit floating-point vertical Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVert_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVert_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 32-bit floating-point vertical Sobel filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVert_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVert_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Single channel 8-bit unsigned to 16-bit signed vertical Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelMaskParameters">Common parameters for nppiFilterSobel functions with masks</a>.
 *
 */
NppStatus 
nppiFilterSobelVert_8u16s_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                  NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVert_8u16s_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize);

/**
 * Single channel 8-bit signed to 16-bit signed vertical Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVert_8s16s_C1R_Ctx(const Npp8s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                  NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVert_8s16s_C1R(const Npp8s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize);

/**
 * Single channel 32-bit floating-point vertical Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVertMask_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVertMask_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize);

/** @} FilterSobelVert */

/** @name FilterSobelHorizSecond
 *
 * Filters the image using a second derivative, horizontal Sobel filter kernel:
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *     1 &   2 &   1 \\
 *    -2 &  -4 &  -2 \\
 *     1 &   2 &   1 \\
 *  \end{array} \right)
 *  \left( \begin{array}{rrrrr}
 *     1  &  4 &   6 &  4 &  1 \\
 *     0  &  0 &   0 &  0 &  0 \\
 *    -2  & -8 & -12 & -8 & -2 \\
 *     0  &  0 &   0 &  0 &  0 \\
 *     1  &  4 &   6 &  4 &  1 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned to 16-bit signed second derivative, horizontal Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHorizSecond_8u16s_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                         NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHorizSecond_8u16s_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     NppiMaskSize eMaskSize);

/**
 * Single channel 8-bit signed to 16-bit signed second derivative, horizontal Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHorizSecond_8s16s_C1R_Ctx(const Npp8s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                         NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHorizSecond_8s16s_C1R(const Npp8s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     NppiMaskSize eMaskSize);

/**
 * Single channel 32-bit floating-point second derivative, horizontal Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHorizSecond_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                       NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHorizSecond_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                   NppiMaskSize eMaskSize);

/** @} FilterSobelHorizSecond */

/** @name FilterSobelVertSecond
 *
 * Filters the image using a second derivative, vertical Sobel filter kernel:
 *
 * \f[
 *  \left( \begin{array}{rrr}
     *    1 & -2 & 1 \\
     *    2 & -4 & 2 \\
     *    1 & -2 & 1 \\
 *  \end{array} \right)
 *  \left( \begin{array}{rrrrr}
 *     1 & 0 &  -2 & 0 & 1 \\
 *     4 & 0 &  -8 & 0 & 4 \\
 *     6 & 0 & -12 & 0 & 6 \\
 *     4 & 0 &  -8 & 0 & 4 \\
 *     1 & 0 &  -2 & 0 & 1 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned to 16-bit signed second derivative, vertical Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVertSecond_8u16s_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                        NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVertSecond_8u16s_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize);

/**
 * Single channel 8-bit signed to 16-bit signed second derivative, vertical Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVertSecond_8s16s_C1R_Ctx(const Npp8s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                        NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVertSecond_8s16s_C1R(const Npp8s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize);

/**
 * Single channel 32-bit floating-point second derivative, vertical Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVertSecond_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                      NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVertSecond_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                  NppiMaskSize eMaskSize);

/** @} FilterSobelVertSecond */

/** @name FilterSobelCross
 *
 * Filters the image using a second cross derivative Sobel filter kernel:
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *    -1 & 0 &  1 \\
 *     0 & 0 &  0 \\
 *     1 & 0 & -1 \\
 *  \end{array} \right)
 *  \left( \begin{array}{rrrrr}
 *    -1 & -2 & 0 &  2 &  1 \\
 *    -2 & -4 & 0 &  4 &  2 \\
 *     0 &  0 & 0 &  0 &  0 \\
 *     2 &  4 & 0 & -4 & -2 \\
 *     1 &  2 & 0 & -2 & -1 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned to 16-bit signed second cross derivative Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelCross_8u16s_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                   NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelCross_8u16s_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize);

/**
 * Single channel 8-bit signed to 16-bit signed second cross derivative Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelCross_8s16s_C1R_Ctx(const Npp8s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                   NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelCross_8s16s_C1R(const Npp8s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize);

/**
 * Single channel 32-bit floating-point second cross derivative Sobel filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelParameters">Common parameters for nppiFilterSobel functions</a>.
 *
 */
NppStatus 
nppiFilterSobelCross_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelCross_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                             NppiMaskSize eMaskSize);

/** @} FilterSobelCross */

/** @} image_filter_sobel */

/** @defgroup image_filter_sobel_border FilterSobelBorder 
 * Filters the image using a Sobel filter kernel with border control.
 *
 * <h3><a name="CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param eMaskSize Enumeration value specifying the mask size.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/** @name FilterSobelHorizBorder 
 *
 * Filters the image using a horizontal Sobel filter kernel with border control:
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *    1 &  2 &  1 \\
 *    0 &  0 &  0 \\
 *   -1 & -2 & -1 \\
 *  \end{array} \right)
 *  \left( \begin{array}{rrrrr}
 *    1  &  4 &   6 &  4 &  1 \\
 *    2  &  8 &  12 &  8 &  2 \\
 *    0  &  0 &   0 &  0 &  0 \\
 *    -2 & -8 & -12 & -8 & -2 \\
 *    -1 & -4 &  -6 & -4 & -1 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned horizontal Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHorizBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHorizBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Three channel 8-bit unsigned horizontal Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHorizBorder_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHorizBorder_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned horizontal Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHorizBorder_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHorizBorder_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed horizontal Sobel filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHorizBorder_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHorizBorder_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Single channel 16-bit signed horizontal Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHorizBorder_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHorizBorder_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Three channel 16-bit signed horizontal Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHorizBorder_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHorizBorder_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed horizontal Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHorizBorder_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHorizBorder_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned horizontal Sobel filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHorizBorder_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHorizBorder_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point horizontal Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHorizBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHorizBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Three channel 32-bit floating-point horizontal Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHorizBorder_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHorizBorder_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point horizontal Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHorizBorder_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHorizBorder_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point horizontal Sobel filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHorizBorder_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHorizBorder_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Single channel 8-bit unsigned to 16-bit signed horizontal Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHorizBorder_8u16s_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                         NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHorizBorder_8u16s_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Single channel 8-bit signed to 16-bit signed horizontal Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHorizBorder_8s16s_C1R_Ctx(const Npp8s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                         NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHorizBorder_8s16s_C1R(const Npp8s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point horizontal Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHorizMaskBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                           NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHorizMaskBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                       NppiMaskSize eMaskSize, NppiBorderType eBorderType);


/** @} FilterSobelHorizBorder */

/** @name FilterSobelVertBorder 
 *
 * Filters the image using a vertical Sobel filter kernel with border control:
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *    -1 & 0 & 1 \\
 *    -2 & 0 & 2 \\
 *    -1 & 0 & 1 \\
 *  \end{array} \right)
 *  \left( \begin{array}{rrrrr}
 *    -1 &  -2 & 0 &  2 & 1 \\
 *    -4 &  -8 & 0 &  8 & 4 \\
 *    -6 & -12 & 0 & 12 & 6 \\
 *    -4 &  -8 & 0 &  8 & 4 \\
 *    -1 &  -2 & 0 &  2 & 1 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned vertical Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVertBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVertBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Three channel 8-bit unsigned vertical Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVertBorder_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVertBorder_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned vertical Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVertBorder_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVertBorder_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed vertical Sobel filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVertBorder_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVertBorder_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Single channel 16-bit signed vertical Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVertBorder_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVertBorder_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Three channel 16-bit signed vertical Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVertBorder_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVertBorder_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed vertical Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVertBorder_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVertBorder_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned vertical Sobel filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVertBorder_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVertBorder_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point vertical Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVertBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVertBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Three channel 32-bit floating-point vertical Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVertBorder_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVertBorder_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point vertical Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVertBorder_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVertBorder_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point vertical Sobel filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVertBorder_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVertBorder_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Single channel 8-bit unsigned to 16-bit signed vertical Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVertBorder_8u16s_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                        NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVertBorder_8u16s_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Single channel 8-bit signed to 16-bit signed vertical Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVertBorder_8s16s_C1R_Ctx(const Npp8s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                        NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVertBorder_8s16s_C1R(const Npp8s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point vertical Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVertMaskBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                          NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVertMaskBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                      NppiMaskSize eMaskSize, NppiBorderType eBorderType);


/** @} FilterSobelVertBorder */

/** @name FilterSobelHorizSecondBorder
 *
 * Filters the image using a second derivative, horizontal Sobel filter kernel with border control:
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *     1 &   2 &   1 \\
 *    -2 &  -4 &  -2 \\
 *     1 &   2 &   1 \\
 *  \end{array} \right)
 *  \left( \begin{array}{rrrrr}
 *     1  &  4 &   6 &  4 &  1 \\
 *     0  &  0 &   0 &  0 &  0 \\
 *    -2  & -8 & -12 & -8 & -2 \\
 *     0  &  0 &   0 &  0 &  0 \\
 *     1  &  4 &   6 &  4 &  1 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned to 16-bit signed second derivative, horizontal Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHorizSecondBorder_8u16s_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                               NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHorizSecondBorder_8u16s_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                           NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Single channel 8-bit signed to 16-bit signed second derivative, horizontal Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHorizSecondBorder_8s16s_C1R_Ctx(const Npp8s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                               NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHorizSecondBorder_8s16s_C1R(const Npp8s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                           NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point second derivative, horizontal Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelHorizSecondBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                             NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelHorizSecondBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                         NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/** @} FilterSobelHorizSecondBorder */

/** @name FilterSobelVertSecondBorder
 *
 * Filters the image using a second derivative, vertical Sobel filter kernel with border control:
 *
 * \f[
 *  \left( \begin{array}{rrr}
     *    1 & -2 & 1 \\
     *    2 & -4 & 2 \\
     *    1 & -2 & 1 \\
 *  \end{array} \right)
 *  \left( \begin{array}{rrrrr}
 *     1 & 0 &  -2 & 0 & 1 \\
 *     4 & 0 &  -8 & 0 & 4 \\
 *     6 & 0 & -12 & 0 & 6 \\
 *     4 & 0 &  -8 & 0 & 4 \\
 *     1 & 0 &  -2 & 0 & 1 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned to 16-bit signed second derivative, vertical Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVertSecondBorder_8u16s_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                              NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVertSecondBorder_8u16s_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                          NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Single channel 8-bit signed to 16-bit signed second derivative, vertical Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVertSecondBorder_8s16s_C1R_Ctx(const Npp8s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                              NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVertSecondBorder_8s16s_C1R(const Npp8s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                          NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point second derivative, vertical Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelVertSecondBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                            NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelVertSecondBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                        NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/** @} FilterSobelVertSecondBorder */

/** @name FilterSobelCrossBorder
 *
 * Filters the image using a second cross derivative Sobel filter kernel with border control:
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *    -1 & 0 &  1 \\
 *     0 & 0 &  0 \\
 *     1 & 0 & -1 \\
 *  \end{array} \right)
 *  \left( \begin{array}{rrrrr}
 *    -1 & -2 & 0 &  2 &  1 \\
 *    -2 & -4 & 0 &  4 &  2 \\
 *     0 &  0 & 0 &  0 &  0 \\
 *     2 &  4 & 0 & -4 & -2 \\
 *     1 &  2 & 0 & -2 & -1 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned to 16-bit signed second cross derivative Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelCrossBorder_8u16s_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                         NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelCrossBorder_8u16s_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Single channel 8-bit signed to 16-bit signed second cross derivative Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelCrossBorder_8s16s_C1R_Ctx(const Npp8s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                         NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelCrossBorder_8s16s_C1R(const Npp8s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point second cross derivative Sobel filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSobelBorderParameters">Common parameters for nppiFilterSobelBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSobelCrossBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                       NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSobelCrossBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                   NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/** @} FilterSobelCrossBorder */

/** @} image_filter_sobel_border */

/** @defgroup image_filter_roberts FilterRoberts 
 * Filters the image using a Roberts filter kernel.
 *
 * <h3><a name="CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/** @name FilterRobertsDown
 *
 * Filters the image using a horizontal Roberts filter kernel:
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *   0 & 0 &  0 \\
 *   0 & 1 &  0 \\
 *   0 & 0 & -1 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned horizontal Roberts filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDown_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDown_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Three channel 8-bit unsigned horizontal Roberts filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDown_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDown_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 8-bit unsigned horizontal Roberts filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDown_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDown_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 8-bit unsigned horizontal Roberts filter, ignoring alpha-channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDown_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDown_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Single channel 16-bit signed horizontal Roberts filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDown_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDown_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Three channel 16-bit signed horizontal Roberts filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDown_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDown_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 16-bit signed horizontal Roberts filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDown_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDown_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 16-bit signed horizontal Roberts filter, ignoring alpha-channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDown_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDown_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Single channel 32-bit floating-point horizontal Roberts filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDown_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDown_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Three channel 32-bit floating-point horizontal Roberts filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDown_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDown_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 32-bit floating-point horizontal Roberts filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDown_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDown_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 32-bit floating-point horizontal Roberts filter, ignoring alpha-channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDown_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDown_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/** @} FilterRobertsDown */

/** @name FilterRobertsUp
 *
 * Filters the image using a vertical Roberts filter kernel:
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *   0 & 0 &  0 \\
 *   0 & 1 &  0 \\
 *  -1 & 0 &  0 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned vertical Roberts filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUp_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUp_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Three channel 8-bit unsigned vertical Roberts filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUp_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUp_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 8-bit unsigned vertical Roberts filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUp_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUp_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 8-bit unsigned vertical Roberts filter, ignoring alpha-channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUp_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUp_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Single channel 16-bit signed vertical Roberts filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUp_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUp_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Three channel 16-bit signed vertical Roberts filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUp_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUp_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 16-bit signed vertical Roberts filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUp_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUp_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 16-bit signed vertical Roberts filter, ignoring alpha-channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUp_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUp_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Single channel 32-bit floating-point vertical Roberts filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUp_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUp_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Three channel 32-bit floating-point vertical Roberts filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUp_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUp_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 32-bit floating-point vertical Roberts filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUp_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUp_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 32-bit floating-point vertical Roberts filter, ignoring alpha-channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsParameters">Common parameters for nppiFilterRoberts functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUp_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUp_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/** @} FilterRobertsUp */

/** @} image_filter_roberts */

/** @defgroup image_filter_roberts_border FilterRobertsBorder 
 * Filters the image using a Roberts filter kernel with border control.
 *
 * <h3><a name="CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/** @name FilterRobertsDownBorder
 *
 * Filters the image using a horizontal Roberts filter kernel with border control.  If any portion of the mask overlaps the source
 * image boundary the requested border type operation is applied to all mask pixels
 * which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *   0 & 0 &  0 \\
 *   0 & 1 &  0 \\
 *   0 & 0 & -1 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned horizontal Roberts filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDownBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDownBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Three channel 8-bit unsigned horizontal Roberts filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDownBorder_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDownBorder_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned horizontal Roberts filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDownBorder_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDownBorder_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned horizontal Roberts filter with border control, ignoring alpha-channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDownBorder_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDownBorder_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Single channel 16-bit signed horizontal Roberts filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDownBorder_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDownBorder_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Three channel 16-bit signed horizontal Roberts filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDownBorder_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDownBorder_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed horizontal Roberts filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDownBorder_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDownBorder_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed horizontal Roberts filter with border control, ignoring alpha-channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDownBorder_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDownBorder_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point horizontal Roberts filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDownBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDownBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Three channel 32-bit floating-point horizontal Roberts filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDownBorder_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDownBorder_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point horizontal Roberts filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDownBorder_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDownBorder_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point horizontal Roberts filter with border control, ignoring alpha-channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsDownBorder_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsDownBorder_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/** @} FilterRobertsDownBorder */

/** @name FilterRobertsUpBorder
 *
 * Filters the image using a vertical Roberts filter kernel with border control.  If any portion of the mask overlaps the source
 * image boundary the requested border type operation is applied to all mask pixels
 * which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *   0 & 0 &  0 \\
 *   0 & 1 &  0 \\
 *  -1 & 0 &  0 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned vertical Roberts filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUpBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUpBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Three channel 8-bit unsigned vertical Roberts filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUpBorder_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUpBorder_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned vertical Roberts filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUpBorder_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUpBorder_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned vertical Roberts filter with border control, ignoring alpha-channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUpBorder_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUpBorder_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Single channel 16-bit signed vertical Roberts filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUpBorder_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUpBorder_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Three channel 16-bit signed vertical Roberts filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUpBorder_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUpBorder_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed vertical Roberts filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUpBorder_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUpBorder_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed vertical Roberts filter with border control, ignoring alpha-channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUpBorder_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUpBorder_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point vertical Roberts filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUpBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUpBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Three channel 32-bit floating-point vertical Roberts filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUpBorder_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUpBorder_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point vertical Roberts filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUpBorder_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUpBorder_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point vertical Roberts filter with border control, ignoring alpha-channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterRobertsBorderParameters">Common parameters for nppiFilterRobertsBorder functions</a>.
 *
 */
NppStatus 
nppiFilterRobertsUpBorder_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterRobertsUpBorder_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/** @} FilterRobertsUpBorder */

/** @} image_filter_roberts_border */

/** @defgroup image_filter_laplace FilterLaplace 
 * Filters the image using a Laplacian filter kernel.
 *
 * <h3><a name="CommonFilterLaplaceParameters">Common parameters for nppiFilterLaplace functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param eMaskSize Enumeration value specifying the mask size.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *   -1 & -1 & -1 \\
 *   -1 &  8 & -1 \\
 *   -1 & -1 & -1 \\
 *  \end{array} \right)
  *  \left( \begin{array}{rrrrr}
 *   -1 & -3 & -4 & -3 & -1 \\
 *   -3 &  0 &  6 &  0 & -3 \\
 *   -4 &  6 & 20 &  6 & -4 \\
 *   -3 &  0 &  6 &  0 & -3 \\
 *   -1 & -3 & -4 & -3 & -1 \\
 *  \end{array} \right)

 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned Laplace filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceParameters">Common parameters for nppiFilterlaplace functions</a>.
 *
 */
NppStatus 
nppiFilterLaplace_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                             NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplace_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                         NppiMaskSize eMaskSize);

/**
 * Three channel 8-bit unsigned Laplace filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceParameters">Common parameters for nppiFilterLaplace functions</a>.
 *
 */
NppStatus 
nppiFilterLaplace_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                             NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplace_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                         NppiMaskSize eMaskSize);

/**
 * Four channel 8-bit unsigned Laplace filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceParameters">Common parameters for nppiFilterLaplace functions</a>.
 *
 */
NppStatus 
nppiFilterLaplace_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                             NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplace_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                         NppiMaskSize eMaskSize);

/**
 * Four channel 8-bit unsigned Laplace filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceParameters">Common parameters for nppiFilterLaplace functions</a>.
 *
 */
NppStatus 
nppiFilterLaplace_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplace_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                          NppiMaskSize eMaskSize);

/**
 * Single channel 16-bit signed Laplace filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceParameters">Common parameters for nppiFilterLaplace functions</a>.
 *
 */
NppStatus 
nppiFilterLaplace_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplace_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                          NppiMaskSize eMaskSize);

/**
 * Three channel 16-bit signed Laplace filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceParameters">Common parameters for nppiFilterLaplace functions</a>.
 *
 */
NppStatus 
nppiFilterLaplace_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplace_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                          NppiMaskSize eMaskSize);

/**
 * Four channel 16-bit signed Laplace filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceParameters">Common parameters for nppiFilterLaplace functions</a>.
 *
 */
NppStatus 
nppiFilterLaplace_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplace_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                          NppiMaskSize eMaskSize);

/**
 * Four channel 16-bit signed Laplace filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceParameters">Common parameters for nppiFilterLaplace functions</a>.
 *
 */
NppStatus 
nppiFilterLaplace_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplace_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                           NppiMaskSize eMaskSize);

/**
 * Single channel 32-bit floating-point Laplace filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceParameters">Common parameters for nppiFilterLaplace functions</a>.
 *
 */
NppStatus 
nppiFilterLaplace_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplace_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                          NppiMaskSize eMaskSize);

/**
 * Three channel 32-bit floating-point Laplace filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceParameters">Common parameters for nppiFilterLaplace functions</a>.
 *
 */
NppStatus 
nppiFilterLaplace_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplace_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                          NppiMaskSize eMaskSize);

/**
 * Four channel 32-bit floating-point Laplace filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceParameters">Common parameters for nppiFilterLaplace functions</a>.
 *
 */
NppStatus 
nppiFilterLaplace_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplace_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                          NppiMaskSize eMaskSize);

/**
 * Four channel 32-bit floating-point Laplace filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceParameters">Common parameters for nppiFilterLaplace functions</a>.
 *
 */
NppStatus 
nppiFilterLaplace_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplace_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                           NppiMaskSize eMaskSize);

/**
 * Single channel 8-bit unsigned to 16-bit signed Laplace filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceParameters">Common parameters for nppiFilterLaplace functions</a>.
 *
 */
NppStatus 
nppiFilterLaplace_8u16s_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplace_8u16s_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                            NppiMaskSize eMaskSize);

/**
 * Single channel 8-bit signed to 16-bit signed Laplace filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceParameters">Common parameters for nppiFilterLaplace functions</a>.
 *
 */
NppStatus 
nppiFilterLaplace_8s16s_C1R_Ctx(const Npp8s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplace_8s16s_C1R(const Npp8s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                            NppiMaskSize eMaskSize);

/** @} image_filter_laplace */

/** @defgroup image_filter_laplace_border FilterLaplaceBorder 
 * Filters the image using a Laplacian filter kernel with border control.
 *
 * If any portion of the mask overlaps the source
 * image boundary the requested border type operation is applied to all mask pixels
 * which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.
 *
 * <h3><a name="CommonFilterLaplaceBorderParameters">Common parameters for nppiFilterLaplaceBorder functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param eMaskSize Enumeration value specifying the mask size.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *   -1 & -1 & -1 \\
 *   -1 &  8 & -1 \\
 *   -1 & -1 & -1 \\
 *  \end{array} \right)
  *  \left( \begin{array}{rrrrr}
 *   -1 & -3 & -4 & -3 & -1 \\
 *   -3 &  0 &  6 &  0 & -3 \\
 *   -4 &  6 & 20 &  6 & -4 \\
 *   -3 &  0 &  6 &  0 & -3 \\
 *   -1 & -3 & -4 & -3 & -1 \\
 *  \end{array} \right)

 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned Laplace filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceBorderParameters">Common parameters for nppiFilterLaplaceBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLaplaceBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                   NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplaceBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Three channel 8-bit unsigned Laplace filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceBorderParameters">Common parameters for nppiFilterLaplaceBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLaplaceBorder_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                   NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplaceBorder_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned Laplace filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceBorderParameters">Common parameters for nppiFilterLaplaceBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLaplaceBorder_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                   NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplaceBorder_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned Laplace filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceBorderParameters">Common parameters for nppiFilterLaplaceBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLaplaceBorder_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplaceBorder_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Single channel 16-bit signed Laplace filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceBorderParameters">Common parameters for nppiFilterLaplaceBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLaplaceBorder_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplaceBorder_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Three channel 16-bit signed Laplace filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceBorderParameters">Common parameters for nppiFilterLaplaceBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLaplaceBorder_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplaceBorder_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed Laplace filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceBorderParameters">Common parameters for nppiFilterLaplaceBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLaplaceBorder_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplaceBorder_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed Laplace filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceBorderParameters">Common parameters for nppiFilterLaplaceBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLaplaceBorder_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplaceBorder_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point Laplace filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceBorderParameters">Common parameters for nppiFilterLaplaceBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLaplaceBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplaceBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Three channel 32-bit floating-point Laplace filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceBorderParameters">Common parameters for nppiFilterLaplaceBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLaplaceBorder_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplaceBorder_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point Laplace filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceBorderParameters">Common parameters for nppiFilterLaplaceBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLaplaceBorder_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplaceBorder_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point Laplace filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceBorderParameters">Common parameters for nppiFilterLaplaceBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLaplaceBorder_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplaceBorder_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Single channel 8-bit unsigned to 16-bit signed Laplace filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceBorderParameters">Common parameters for nppiFilterLaplaceBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLaplaceBorder_8u16s_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                      NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplaceBorder_8u16s_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                  NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Single channel 8-bit signed to 16-bit signed Laplace filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLaplaceBorderParameters">Common parameters for nppiFilterLaplaceBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLaplaceBorder_8s16s_C1R_Ctx(const Npp8s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                      NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLaplaceBorder_8s16s_C1R(const Npp8s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                  NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/** @} FilterLaplaceBorder */

/** @defgroup image_filter_gauss FilterGauss 
 * Filters the image using a Gaussian filter kernel.  Use FilterGaussAdvanced if you want to supply your own filter coefficients.
 *
 * Note that all FilterGauss functions currently support mask sizes up to 15x15. Filter kernels for these functions are calculated
 * using a sigma value of 0.4F + (mask width / 2) * 0.6F.
 *
 * <h3><a name="CommonFilterGaussParameters">Common parameters for nppiFilterGauss functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param eMaskSize Enumeration value specifying the mask size.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussParameters">Common parameters for nppiFilterGauss functions</a>.
 *
 */
NppStatus 
nppiFilterGauss_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                           NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGauss_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                       NppiMaskSize eMaskSize);

/**
 * Three channel 8-bit unsigned Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussParameters">Common parameters for nppiFilterGauss functions</a>.
 *
 */
NppStatus 
nppiFilterGauss_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                           NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGauss_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                       NppiMaskSize eMaskSize);

/**
 * Four channel 8-bit unsigned Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussParameters">Common parameters for nppiFilterGauss functions</a>.
 *
 */
NppStatus 
nppiFilterGauss_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                           NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGauss_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                       NppiMaskSize eMaskSize);

/**
 * Four channel 8-bit unsigned Gauss filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussParameters">Common parameters for nppiFilterGauss functions</a>.
 *
 */
NppStatus 
nppiFilterGauss_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                            NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGauss_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                        NppiMaskSize eMaskSize);

/**
 * Single channel 16-bit unsigned Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussParameters">Common parameters for nppiFilterGauss functions</a>.
 *
 */
NppStatus 
nppiFilterGauss_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                            NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGauss_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                        NppiMaskSize eMaskSize);

/**
 * Three channel 16-bit unsigned Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussParameters">Common parameters for nppiFilterGauss functions</a>.
 *
 */
NppStatus 
nppiFilterGauss_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                            NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGauss_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                        NppiMaskSize eMaskSize);

/**
 * Four channel 16-bit unsigned Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussParameters">Common parameters for nppiFilterGauss functions</a>.
 *
 */
NppStatus 
nppiFilterGauss_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                            NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGauss_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                        NppiMaskSize eMaskSize);

/**
 * Four channel 16-bit unsigned Gauss filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussParameters">Common parameters for nppiFilterGauss functions</a>.
 *
 */
NppStatus 
nppiFilterGauss_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                             NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGauss_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                         NppiMaskSize eMaskSize);

/**
 * Single channel 16-bit signed Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussParameters">Common parameters for nppiFilterGauss functions</a>.
 *
 */
NppStatus 
nppiFilterGauss_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                            NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGauss_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                        NppiMaskSize eMaskSize);

/**
 * Three channel 16-bit signed Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussParameters">Common parameters for nppiFilterGauss functions</a>.
 *
 */
NppStatus 
nppiFilterGauss_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                            NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGauss_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                        NppiMaskSize eMaskSize);

/**
 * Four channel 16-bit signed Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussParameters">Common parameters for nppiFilterGauss functions</a>.
 *
 */
NppStatus 
nppiFilterGauss_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                            NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGauss_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                        NppiMaskSize eMaskSize);

/**
 * Four channel 16-bit signed Gauss filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussParameters">Common parameters for nppiFilterGauss functions</a>.
 *
 */
NppStatus 
nppiFilterGauss_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                             NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGauss_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                         NppiMaskSize eMaskSize);

/**
 * Single channel 32-bit floating-point Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussParameters">Common parameters for nppiFilterGauss functions</a>.
 *
 */
NppStatus 
nppiFilterGauss_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                            NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGauss_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                        NppiMaskSize eMaskSize);

/**
 * Three channel 32-bit floating-point Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussParameters">Common parameters for nppiFilterGauss functions</a>.
 *
 */
NppStatus 
nppiFilterGauss_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                            NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGauss_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                        NppiMaskSize eMaskSize);

/**
 * Four channel 32-bit floating-point Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussParameters">Common parameters for nppiFilterGauss functions</a>.
 *
 */
NppStatus 
nppiFilterGauss_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                            NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGauss_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                        NppiMaskSize eMaskSize);

/**
 * Four channel 32-bit floating-point Gauss filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussParameters">Common parameters for nppiFilterGauss functions</a>.
 *
 */
NppStatus 
nppiFilterGauss_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                             NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGauss_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                         NppiMaskSize eMaskSize);

/** @} image_filter_gauss */

/** @defgroup image_filter_gauss_advanced FilterGaussAdvanced 
 * Filters the image using a separable Gaussian filter kernel with user supplied floating point coefficients:
 *
 * <h3><a name="CommonFilterGaussAdvancedParameters">Common parameters for nppiFilterGaussAdvanced functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nFilterTaps The number of filter taps where nFilterTaps =  2 * ((int)((float)ceil(radius) + 0.5F) ) + 1.
 * \param pKernel Pointer to an array of nFilterTaps kernel coefficients which sum to 1.0F. 
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */                                                  

/**
 * Single channel 8-bit unsigned Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedParameters">Common parameters for nppiFilterGaussAdvanced functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvanced_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                   const int nFilterTaps, const Npp32f * pKernel, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvanced_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               const int nFilterTaps, const Npp32f * pKernel);

/**
 * Three channel 8-bit unsigned Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedParameters">Common parameters for nppiFilterGaussAdvanced functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvanced_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                   const int nFilterTaps, const Npp32f * pKernel, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvanced_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               const int nFilterTaps, const Npp32f * pKernel);

/**
 * Four channel 8-bit unsigned Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedParameters">Common parameters for nppiFilterGaussAdvanced functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvanced_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                   const int nFilterTaps, const Npp32f * pKernel, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvanced_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               const int nFilterTaps, const Npp32f * pKernel);

/**
 * Four channel 8-bit unsigned Gauss filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedParameters">Common parameters for nppiFilterGaussAdvanced functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvanced_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    const int nFilterTaps, const Npp32f * pKernel, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvanced_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                const int nFilterTaps, const Npp32f * pKernel);

/**
 * Single channel 16-bit unsigned Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedParameters">Common parameters for nppiFilterGaussAdvanced functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvanced_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    const int nFilterTaps, const Npp32f * pKernel, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvanced_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                const int nFilterTaps, const Npp32f * pKernel);

/**
 * Three channel 16-bit unsigned Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedParameters">Common parameters for nppiFilterGaussAdvanced functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvanced_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    const int nFilterTaps, const Npp32f * pKernel, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvanced_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                const int nFilterTaps, const Npp32f * pKernel);

/**
 * Four channel 16-bit unsigned Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedParameters">Common parameters for nppiFilterGaussAdvanced functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvanced_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    const int nFilterTaps, const Npp32f * pKernel, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvanced_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                const int nFilterTaps, const Npp32f * pKernel);

/**
 * Four channel 16-bit unsigned Gauss filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedParameters">Common parameters for nppiFilterGaussAdvanced functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvanced_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     const int nFilterTaps, const Npp32f * pKernel, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvanced_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 const int nFilterTaps, const Npp32f * pKernel);

/**
 * Single channel 16-bit signed Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedParameters">Common parameters for nppiFilterGaussAdvanced functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvanced_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    const int nFilterTaps, const Npp32f * pKernel, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvanced_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                const int nFilterTaps, const Npp32f * pKernel);

/**
 * Three channel 16-bit signed Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedParameters">Common parameters for nppiFilterGaussAdvanced functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvanced_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    const int nFilterTaps, const Npp32f * pKernel, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvanced_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                const int nFilterTaps, const Npp32f * pKernel);

/**
 * Four channel 16-bit signed Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedParameters">Common parameters for nppiFilterGaussAdvanced functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvanced_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    const int nFilterTaps, const Npp32f * pKernel, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvanced_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                const int nFilterTaps, const Npp32f * pKernel);

/**
 * Four channel 16-bit signed Gauss filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedParameters">Common parameters for nppiFilterGaussAdvanced functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvanced_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     const int nFilterTaps, const Npp32f * pKernel, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvanced_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 const int nFilterTaps, const Npp32f * pKernel);

/**
 * Single channel 32-bit floating-point Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedParameters">Common parameters for nppiFilterGaussAdvanced functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvanced_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    const int nFilterTaps, const Npp32f * pKernel, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvanced_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                const int nFilterTaps, const Npp32f * pKernel);

/**
 * Three channel 32-bit floating-point Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedParameters">Common parameters for nppiFilterGaussAdvanced functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvanced_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    const int nFilterTaps, const Npp32f * pKernel, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvanced_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                const int nFilterTaps, const Npp32f * pKernel);

/**
 * Four channel 32-bit floating-point Gauss filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedParameters">Common parameters for nppiFilterGaussAdvanced functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvanced_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    const int nFilterTaps, const Npp32f * pKernel, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvanced_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                const int nFilterTaps, const Npp32f * pKernel);

/**
 * Four channel 32-bit floating-point Gauss filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedParameters">Common parameters for nppiFilterGaussAdvanced functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvanced_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     const int nFilterTaps, const Npp32f * pKernel, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvanced_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 const int nFilterTaps, const Npp32f * pKernel);

/** @} image_filter_gauss_advanced */

/** @defgroup image_filter_gauss_border FilterGaussBorder 
 * Filters the image using a Gaussian filter kernel with border control.  Use FilterGaussAdvancedBorder if you want to supply your own filter coefficients.
 *
 * If any portion of the mask overlaps the source
 * image boundary the requested border type operation is applied to all mask pixels
 * which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.
 *
 * Note that all FilterGaussBorder functions currently support mask sizes up to 15x15. Filter kernels for these functions are calculated
 * using a sigma value of 0.4F + (mask width / 2) * 0.6F.
 *
 * <h3><a name="CommonFilterGaussBorderParameters">Common parameters for nppiFilterGaussBorder functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param eMaskSize Enumeration value specifying the mask size.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussBorderParameters">Common parameters for nppiFilterGaussBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                             NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Three channel 8-bit unsigned Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussBorderParameters">Common parameters for nppiFilterGaussBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussBorder_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussBorder_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                             NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussBorderParameters">Common parameters for nppiFilterGaussBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussBorder_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussBorder_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                             NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned Gauss filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussBorderParameters">Common parameters for nppiFilterGaussBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussBorder_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                  NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussBorder_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Single channel 16-bit unsigned Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussBorderParameters">Common parameters for nppiFilterGaussBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussBorder_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                  NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussBorder_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Three channel 16-bit unsigned Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussBorderParameters">Common parameters for nppiFilterGaussBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussBorder_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                  NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussBorder_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussBorderParameters">Common parameters for nppiFilterGaussBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussBorder_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                  NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussBorder_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned Gauss filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussBorderParameters">Common parameters for nppiFilterGaussBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussBorder_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                   NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);
NppStatus 
nppiFilterGaussBorder_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Single channel 16-bit signed Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussBorderParameters">Common parameters for nppiFilterGaussBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussBorder_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                  NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussBorder_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Three channel 16-bit signed Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussBorderParameters">Common parameters for nppiFilterGaussBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussBorder_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                  NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussBorder_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussBorderParameters">Common parameters for nppiFilterGaussBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussBorder_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                  NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussBorder_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed Gauss filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussBorderParameters">Common parameters for nppiFilterGaussBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussBorder_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                   NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussBorder_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussBorderParameters">Common parameters for nppiFilterGaussBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                  NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Three channel 32-bit floating-point Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussBorderParameters">Common parameters for nppiFilterGaussBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussBorder_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                  NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussBorder_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussBorderParameters">Common parameters for nppiFilterGaussBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussBorder_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                  NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussBorder_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point Gauss filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussBorderParameters">Common parameters for nppiFilterGaussBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussBorder_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                   NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussBorder_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/** @} image_filter_gauss_border */

/** @defgroup image_filter_gauss_advanced_border FilterGaussAdvancedBorder 
 * Filters the image using a separable Gaussian filter kernel with user supplied floating point coefficients with border control.
 *
 * If any portion of the mask overlaps the source image boundary the requested border type operation is applied to all mask pixels
 * which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_REPLICATE and NPP_BORDER_MIRROR border type operations are supported.
 *
 * <h3><a name="CommonFilterGaussAdvancedBorderParameters">Common parameters for nppiFilterGaussAdvancedBorder functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nFilterTaps The number of filter taps where nFilterTaps =  2 * ((int)((float)ceil(radius) + 0.5F) ) + 1.
 * \param pKernel Pointer to an array of nFilterTaps kernel coefficients which sum to 1.0F. 
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedBorderParameters">Common parameters for nppiFilterGaussAdvancedBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvancedBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                         const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvancedBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Three channel 8-bit unsigned Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedBorderParameters">Common parameters for nppiFilterGaussAdvancedBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvancedBorder_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                         const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvancedBorder_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedBorderParameters">Common parameters for nppiFilterGaussAdvancedBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvancedBorder_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                         const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvancedBorder_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned Gauss filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedBorderParameters">Common parameters for nppiFilterGaussAdvancedBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvancedBorder_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                          const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvancedBorder_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                      const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Single channel 16-bit unsigned Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedBorderParameters">Common parameters for nppiFilterGaussAdvancedBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvancedBorder_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                          const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvancedBorder_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                      const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Three channel 16-bit unsigned Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedBorderParameters">Common parameters for nppiFilterGaussAdvancedBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvancedBorder_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                          const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvancedBorder_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                      const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedBorderParameters">Common parameters for nppiFilterGaussAdvancedBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvancedBorder_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                          const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvancedBorder_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                      const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned Gauss filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedBorderParameters">Common parameters for nppiFilterGaussAdvancedBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvancedBorder_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                           const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvancedBorder_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                       const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Single channel 16-bit signed Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedBorderParameters">Common parameters for nppiFilterGaussAdvancedBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvancedBorder_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                          const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvancedBorder_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                      const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Three channel 16-bit signed Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedBorderParameters">Common parameters for nppiFilterGaussAdvancedBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvancedBorder_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                          const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvancedBorder_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                      const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedBorderParameters">Common parameters for nppiFilterGaussAdvancedBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvancedBorder_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                          const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvancedBorder_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                      const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed Gauss filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedBorderParameters">Common parameters for nppiFilterGaussAdvancedBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvancedBorder_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                           const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvancedBorder_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                       const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedBorderParameters">Common parameters for nppiFilterGaussAdvancedBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvancedBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                          const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvancedBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                      const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Three channel 32-bit floating-point Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedBorderParameters">Common parameters for nppiFilterGaussAdvancedBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvancedBorder_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                          const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvancedBorder_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                      const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedBorderParameters">Common parameters for nppiFilterGaussAdvancedBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvancedBorder_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                          const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvancedBorder_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                      const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point Gauss filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussAdvancedBorderParameters">Common parameters for nppiFilterGaussAdvancedBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussAdvancedBorder_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                           const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussAdvancedBorder_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                       const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/** @} image_filter_gauss_advanced_border */

/** @defgroup image_filter_gauss_pyramid_layer_down_border FilterGaussPyramidLayerDownBorder 
 * Filters the image using a separable Gaussian filter kernel with user supplied floating point coefficients with downsampling and border control.
 *
 * If the downsampling rate is equivalent to an integer value then unnecessary source pixels are just skipped.
 * If any portion of the mask overlaps the source image boundary the requested border type operation is applied to all mask pixels
 * which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_MIRROR and NPP_BORDER_REPLICATE border type operations are supported.
 *
 * <h3><a name="CommonFilterGaussPyramidLayerDownBorderParameters">Common parameters for nppiFilterGaussPyramidLayerDownBorder functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nRate The downsampling rate to be used.  For integer equivalent rates unnecessary source pixels are just skipped.
 *              For non-integer rates the source image is bilinear interpolated. nRate must be > 1.0F and <= 10.0F. 
 * \param nFilterTaps The number of filter taps where nFilterTaps =  2 * ((int)((float)ceil(radius) + 0.5F) ) + 1.
 * \param pKernel Pointer to an array of nFilterTaps kernel coefficients which sum to 1.0F. 
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * Calculate destination image SizeROI width and height from source image ROI width and height and downsampling rate.
 * It is highly recommended that this function be use to determine the destination image ROI for consistent results. 
 *
 * \param nSrcROIWidth The desired source image ROI width, must be <= oSrcSize.width.
 * \param nSrcROIHeight The desired source image ROI height, must be <= oSrcSize.height.
 * \param pDstSizeROI Host memory pointer to the destination image roi_specification.
 * \param nRate The downsampling or upsampling rate to be used.  For integer equivalent rates unnecessary source pixels are just skipped.
 *              For non-integer rates the source image is bilinear interpolated. nRate must be > 1.0F and <= 10.0F. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 */

NppStatus 
nppiGetFilterGaussPyramidLayerDownBorderDstROI(int nSrcROIWidth, int nSrcROIHeight, NppiSize * pDstSizeROI, Npp32f nRate);

/**
 * Single channel 8-bit unsigned Gauss filter with downsampling and border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussPyramidLayerDownBorderParameters">Common parameters for nppiFilterGaussPyramidLayerDownBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussPyramidLayerDownBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                                 Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussPyramidLayerDownBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                             Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Three channel 8-bit unsigned Gauss filter with downsampling and border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussPyramidLayerDownBorderParameters">Common parameters for nppiFilterGaussPyramidLayerDownBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussPyramidLayerDownBorder_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                                 Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussPyramidLayerDownBorder_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                             Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Single channel 16-bit unsigned Gauss filter with downsampling and border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussPyramidLayerDownBorderParameters">Common parameters for nppiFilterGaussPyramidLayerDownBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussPyramidLayerDownBorder_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                                  Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussPyramidLayerDownBorder_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                              Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Three channel 16-bit unsigned Gauss filter with downsampling and border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussPyramidLayerDownBorderParameters">Common parameters for nppiFilterGaussPyramidLayerDownBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussPyramidLayerDownBorder_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                                  Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussPyramidLayerDownBorder_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                              Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point Gauss filter downsampling and with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussPyramidLayerDownBorderParameters">Common parameters for nppiFilterGaussPyramidLayerDownBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussPyramidLayerDownBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                                  Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussPyramidLayerDownBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                              Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Three channel 32-bit floating-point Gauss filter with downsampling and border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussPyramidLayerDownBorderParameters">Common parameters for nppiFilterGaussPyramidLayerDownBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussPyramidLayerDownBorder_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                                  Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussPyramidLayerDownBorder_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                              Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/** @} image_filter_gauss_pyramid_layer_down_border */

/** @defgroup image_filter_gauss_pyramid_layer_up_border FilterGaussPyramidLayerUpBorder 
 * Filters the image using a separable Gaussian filter kernel with user supplied floating point coefficients with upsampling and border control.
 *
 * If the upsampling rate is equivalent to an integer value then unnecessary source pixels are just skipped.
 * If any portion of the mask overlaps the source image boundary the requested border type operation is applied to all mask pixels
 * which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_MIRROR and NPP_BORDER_REPLICATE border type operations are supported.
 *
 * <h3><a name="CommonFilterGaussPyramidLayerUpBorderParameters">Common parameters for nppiFilterGaussPyramidLayerUpBorder functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nRate The upsampling rate to be used.  For integer equivalent rates unnecessary source pixels are just skipped.
 *              For non-integer rates the source image is bilinear interpolated. nRate must be > 1.0F and <= 10.0F. 
 * \param nFilterTaps The number of filter taps where nFilterTaps =  2 * ((int)((float)ceil(radius) + 0.5F) ) + 1.
 * \param pKernel Pointer to an array of nFilterTaps kernel coefficients which sum to 1.0F. 
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * Calculate destination image minimum and maximum SizeROI width and height from source image ROI width and height and upsampling rate.
 * It is highly recommended that this function be use to determine the best destination image ROI for consistent results. 
 *
 * \param nSrcROIWidth The desired source image ROI width, must be <= oSrcSize.width.
 * \param nSrcROIHeight The desired source image ROI height, must be <= oSrcSize.height.
 * \param pDstSizeROIMin Host memory pointer to the minimum recommended destination image roi_specification.
 * \param pDstSizeROIMax Host memory pointer to the maximum recommended destination image roi_specification.
 * \param nRate The upsampling rate to be used.  For integer equivalent rates unnecessary source pixels are just skipped.
 *              For non-integer rates the source image is bilinear interpolated. nRate must be > 1.0F and <= 10.0F. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 */

NppStatus 
nppiGetFilterGaussPyramidLayerUpBorderDstROI(int nSrcROIWidth, int nSrcROIHeight, NppiSize * pDstSizeROIMin, NppiSize * pDstSizeROIMax, Npp32f nRate);

/**
 * Single channel 8-bit unsigned Gauss filter with upsampling and border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussPyramidLayerUpBorderParameters">Common parameters for nppiFilterGaussPyramidLayerUpBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussPyramidLayerUpBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                               Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussPyramidLayerUpBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                           Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Three channel 8-bit unsigned Gauss filter with upsampling and border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussPyramidLayerUpBorderParameters">Common parameters for nppiFilterGaussPyramidLayerUpBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussPyramidLayerUpBorder_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                               Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussPyramidLayerUpBorder_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                           Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Single channel 16-bit unsigned Gauss filter with upsampling and border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussPyramidLayerUpBorderParameters">Common parameters for nppiFilterGaussPyramidLayerUpBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussPyramidLayerUpBorder_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                                Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussPyramidLayerUpBorder_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                            Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Three channel 16-bit unsigned Gauss filter with upsampling and border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussPyramidLayerUpBorderParameters">Common parameters for nppiFilterGaussPyramidLayerUpBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussPyramidLayerUpBorder_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                                Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussPyramidLayerUpBorder_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                            Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point Gauss filter upsampling and with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussPyramidLayerUpBorderParameters">Common parameters for nppiFilterGaussPyramidLayerUpBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussPyramidLayerUpBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                                Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussPyramidLayerUpBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                            Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/**
 * Three channel 32-bit floating-point Gauss filter with upsampling and border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGaussPyramidLayerUpBorderParameters">Common parameters for nppiFilterGaussPyramidLayerUpBorder functions</a>.
 *
 */
NppStatus 
nppiFilterGaussPyramidLayerUpBorder_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                                Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterGaussPyramidLayerUpBorder_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                            Npp32f nRate, const int nFilterTaps, const Npp32f * pKernel, NppiBorderType eBorderType);

/** @} image_filter_gauss_pyramid_layer_up_border */

/** @defgroup image_filter_bilateral_gauss_border FilterBilateralGaussBorder 
 * Filters the image using a bilateral Gaussian filter kernel with border control.
 *
 * If any portion of the mask overlaps the source image boundary the requested border type operation is applied to all mask pixels
 * which fall outside of the source image.
 *
 * For this filter the anchor point is always the central element of the kernel. 
 * Coefficients of the bilateral filter kernel depend on their position in the kernel and 
 * on the value of some source image pixels overlayed by the filter kernel. 
 * Only source image pixels with both coordinates divisible by nDistanceBetweenSrcPixels are used in calculations.
 *
 * The value of an output pixel \f$d\f$ is 
 * \f[d = \frac{\sum_{h=-nRadius}^{nRadius}\sum_{w=-nRadius}^{nRadius}W1(h,w)\cdot W2(h,w)\cdot S(h,w)}{\sum_{h=-nRadius}^{nRadius}\sum_{w=-nRadius}^{nRadius}W1(h,w)\cdot W2(h,w)}\f]
 * where h and w are the corresponding kernel width and height indexes, 
 * S(h,w) is the value of the source image pixel overlayed by filter kernel position (h,w),
 * W1(h,w) is func(nValSquareSigma, (S(h,w) - S(0,0))) where S(0,0) is the value of the source image pixel at the center of the kernel,
 * W2(h,w) is func(nPosSquareSigma, sqrt(h*h+w*w)), and func is the following formula
 * \f[func(S,I) = exp(-\frac{I^2}{2.0F\cdot S^2})\f]
 *
 * Currently only the NPP_BORDER_REPLICATE border type operations are supported.
 *
 * <h3><a name="CommonFilterBilateralGaussBorderParameters">Common parameters for nppiFilterBilateralGaussBorder functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nRadius The radius of the round filter kernel to be used.  A radius of 1 indicates a filter kernel size of 3 by 3, 2 indicates 5 by 5, etc.
 *        Radius values from 1 to 32 are supported.
 * \param nStepBetweenSrcPixels The step size between adjacent source image pixels processed by the filter kernel, most commonly 1. 
 * \param nValSquareSigma The square of the sigma for the relative intensity distance between a source image pixel in the filter kernel 
 *        and the source image pixel at the center of the filter kernel.
 * \param nPosSquareSigma The square of the sigma for the relative geometric distance between a source image pixel in the filter kernel 
 *        and the source image pixel at the center of the filter kernel.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned bilateral Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBilateralGaussBorderParameters">Common parameters for nppiFilterBilateralGaussBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBilateralGaussBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                          const int nRadius, const int nStepBetweenSrcPixels, const Npp32f nValSquareSigma, const Npp32f nPosSquareSigma, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBilateralGaussBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                      const int nRadius, const int nStepBetweenSrcPixels, const Npp32f nValSquareSigma, const Npp32f nPosSquareSigma, NppiBorderType eBorderType);

/**
 * Three channel 8-bit unsigned bilateral Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBilateralGaussBorderParameters">Common parameters for nppiFilterBilateralGaussBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBilateralGaussBorder_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                          const int nRadius, const int nStepBetweenSrcPixels, const Npp32f nValSquareSigma, const Npp32f nPosSquareSigma, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBilateralGaussBorder_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                      const int nRadius, const int nStepBetweenSrcPixels, const Npp32f nValSquareSigma, const Npp32f nPosSquareSigma, NppiBorderType eBorderType);

/**
 * Single channel 16-bit unsigned bilateral Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBilateralGaussBorderParameters">Common parameters for nppiFilterBilateralGaussBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBilateralGaussBorder_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                           const int nRadius, const int nStepBetweenSrcPixels, const Npp32f nValSquareSigma, const Npp32f nPosSquareSigma, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBilateralGaussBorder_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                       const int nRadius, const int nStepBetweenSrcPixels, const Npp32f nValSquareSigma, const Npp32f nPosSquareSigma, NppiBorderType eBorderType);

/**
 * Three channel 16-bit unsigned bilateral Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBilateralGaussBorderParameters">Common parameters for nppiFilterBilateralGaussBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBilateralGaussBorder_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                           const int nRadius, const int nStepBetweenSrcPixels, const Npp32f nValSquareSigma, const Npp32f nPosSquareSigma, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBilateralGaussBorder_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                       const int nRadius, const int nStepBetweenSrcPixels, const Npp32f nValSquareSigma, const Npp32f nPosSquareSigma, NppiBorderType eBorderType);

/**
 * One channel 32-bit floating-point bilateral Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBilateralGaussBorderParameters">Common parameters for nppiFilterBilateralGaussBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBilateralGaussBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                           const int nRadius, const int nStepBetweenSrcPixels, const Npp32f nValSquareSigma, const Npp32f nPosSquareSigma, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBilateralGaussBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                       const int nRadius, const int nStepBetweenSrcPixels, const Npp32f nValSquareSigma, const Npp32f nPosSquareSigma, NppiBorderType eBorderType);

/**
 * Three channel 32-bit floating-point bilateral Gauss filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterBilateralGaussBorderParameters">Common parameters for nppiFilterBilateralGaussBorder functions</a>.
 *
 */
NppStatus 
nppiFilterBilateralGaussBorder_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                           const int nRadius, const int nStepBetweenSrcPixels, const Npp32f nValSquareSigma, const Npp32f nPosSquareSigma, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterBilateralGaussBorder_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                       const int nRadius, const int nStepBetweenSrcPixels, const Npp32f nValSquareSigma, const Npp32f nPosSquareSigma, NppiBorderType eBorderType);

/** @} image_filter_bilateral_gauss_border */

/** @defgroup image_filter_high_pass FilterHighPass 
 * Filters the image using a high-pass filter kernel.
 *
 * <h3><a name="CommonFilterHighPassParameters">Common parameters for nppiFilterHighPass functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param eMaskSize Enumeration value specifying the mask size.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *      -1 & -1 & -1 \\
 *      -1 &  8 & -1 \\
 *      -1 & -1 & -1 \\
 *  \end{array} \right)
 *  \left( \begin{array}{rrrrr}
 *      -1 & -1 & -1 & -1 & -1 \\
 *      -1 & -1 & -1 & -1 & -1 \\
 *      -1 & -1 & 24 & -1 & -1 \\
 *      -1 & -1 & -1 & -1 & -1 \\
 *      -1 & -1 & -1 & -1 & -1 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassParameters">Common parameters for nppiFilterHighPass functions</a>.
 *
 */
NppStatus 
nppiFilterHighPass_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPass_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                          NppiMaskSize eMaskSize);

/**
 * Three channel 8-bit unsigned high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassParameters">Common parameters for nppiFilterHighPass functions</a>.
 *
 */
NppStatus 
nppiFilterHighPass_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPass_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                          NppiMaskSize eMaskSize);

/**
 * Four channel 8-bit unsigned high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassParameters">Common parameters for nppiFilterHighPass functions</a>.
 *
 */
NppStatus 
nppiFilterHighPass_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPass_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                          NppiMaskSize eMaskSize);

/**
 * Four channel 8-bit unsigned high-pass filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassParameters">Common parameters for nppiFilterHighPass functions</a>.
 *
 */
NppStatus 
nppiFilterHighPass_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPass_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                           NppiMaskSize eMaskSize);

/**
 * Single channel 16-bit unsigned high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassParameters">Common parameters for nppiFilterHighPass functions</a>.
 *
 */
NppStatus 
nppiFilterHighPass_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPass_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                           NppiMaskSize eMaskSize);

/**
 * Three channel 16-bit unsigned high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassParameters">Common parameters for nppiFilterHighPass functions</a>.
 *
 */
NppStatus 
nppiFilterHighPass_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPass_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                           NppiMaskSize eMaskSize);

/**
 * Four channel 16-bit unsigned high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassParameters">Common parameters for nppiFilterHighPass functions</a>.
 *
 */
NppStatus 
nppiFilterHighPass_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPass_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                           NppiMaskSize eMaskSize);

/**
 * Four channel 16-bit unsigned high-pass filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassParameters">Common parameters for nppiFilterHighPass functions</a>.
 *
 */
NppStatus 
nppiFilterHighPass_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPass_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                            NppiMaskSize eMaskSize);

/**
 * Single channel 16-bit signed high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassParameters">Common parameters for nppiFilterHighPass functions</a>.
 *
 */
NppStatus 
nppiFilterHighPass_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPass_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                           NppiMaskSize eMaskSize);

/**
 * Three channel 16-bit signed high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassParameters">Common parameters for nppiFilterHighPass functions</a>.
 *
 */
NppStatus 
nppiFilterHighPass_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPass_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                           NppiMaskSize eMaskSize);

/**
 * Four channel 16-bit signed high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassParameters">Common parameters for nppiFilterHighPass functions</a>.
 *
 */
NppStatus 
nppiFilterHighPass_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPass_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                           NppiMaskSize eMaskSize);

/**
 * Four channel 16-bit signed high-pass filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassParameters">Common parameters for nppiFilterHighPass functions</a>.
 *
 */
NppStatus 
nppiFilterHighPass_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPass_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                            NppiMaskSize eMaskSize);

/**
 * Single channel 32-bit floating-point high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassParameters">Common parameters for nppiFilterHighPass functions</a>.
 *
 */
NppStatus 
nppiFilterHighPass_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPass_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                           NppiMaskSize eMaskSize);

/**
 * Three channel 32-bit floating-point high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassParameters">Common parameters for nppiFilterHighPass functions</a>.
 *
 */
NppStatus 
nppiFilterHighPass_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPass_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                           NppiMaskSize eMaskSize);

/**
 * Four channel 32-bit floating-point high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassParameters">Common parameters for nppiFilterHighPass functions</a>.
 *
 */
NppStatus 
nppiFilterHighPass_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPass_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                           NppiMaskSize eMaskSize);

/**
 * Four channel 32-bit floating-point high-pass filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassParameters">Common parameters for nppiFilterHighPass functions</a>.
 *
 */
NppStatus 
nppiFilterHighPass_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPass_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                            NppiMaskSize eMaskSize);

/** @} image_filter_high_pass */

/** @defgroup image_filter_high_pass_border FilterHighPassBorder 
 * Filters the image using a high-pass filter kernel with border control.
 *
 * If any portion of the mask overlaps the source image boundary the requested 
 * border type operation is applied to all mask pixels which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported. 
 *
 * <h3><a name="CommonFilterHighPassBorderParameters">Common parameters for nppiFilterHighPassBorder functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param eMaskSize Enumeration value specifying the mask size.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *      -1 & -1 & -1 \\
 *      -1 &  8 & -1 \\
 *      -1 & -1 & -1 \\
 *  \end{array} \right)
 *  \left( \begin{array}{rrrrr}
 *      -1 & -1 & -1 & -1 & -1 \\
 *      -1 & -1 & -1 & -1 & -1 \\
 *      -1 & -1 & 24 & -1 & -1 \\
 *      -1 & -1 & -1 & -1 & -1 \\
 *      -1 & -1 & -1 & -1 & -1 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassBorderParameters">Common parameters for nppiFilterHighPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterHighPassBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPassBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Three channel 8-bit unsigned high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassBorderParameters">Common parameters for nppiFilterHighPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterHighPassBorder_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPassBorder_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassBorderParameters">Common parameters for nppiFilterHighPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterHighPassBorder_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPassBorder_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned high-pass filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassBorderParameters">Common parameters for nppiFilterHighPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterHighPassBorder_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPassBorder_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Single channel 16-bit unsigned high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassBorderParameters">Common parameters for nppiFilterHighPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterHighPassBorder_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPassBorder_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Three channel 16-bit unsigned high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassBorderParameters">Common parameters for nppiFilterHighPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterHighPassBorder_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPassBorder_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassBorderParameters">Common parameters for nppiFilterHighPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterHighPassBorder_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPassBorder_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned high-pass filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassBorderParameters">Common parameters for nppiFilterHighPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterHighPassBorder_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                      NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPassBorder_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                  NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Single channel 16-bit signed high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassBorderParameters">Common parameters for nppiFilterHighPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterHighPassBorder_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPassBorder_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Three channel 16-bit signed high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassBorderParameters">Common parameters for nppiFilterHighPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterHighPassBorder_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPassBorder_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassBorderParameters">Common parameters for nppiFilterHighPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterHighPassBorder_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPassBorder_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed high-pass filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassBorderParameters">Common parameters for nppiFilterHighPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterHighPassBorder_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                      NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPassBorder_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                  NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassBorderParameters">Common parameters for nppiFilterHighPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterHighPassBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPassBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Three channel 32-bit floating-point high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassBorderParameters">Common parameters for nppiFilterHighPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterHighPassBorder_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPassBorder_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassBorderParameters">Common parameters for nppiFilterHighPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterHighPassBorder_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPassBorder_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point high-pass filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterHighPassBorderParameters">Common parameters for nppiFilterHighPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterHighPassBorder_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                      NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHighPassBorder_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                  NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/** @} image_filter_high_pass_border */

/** @defgroup image_filter_low_pass FilterLowPass 
 * Filters the image using a low-pass filter kernel.
 *
 * <h3><a name="CommonFilterLowPassParameters">Common parameters for nppiFilterLowPass functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param eMaskSize Enumeration value specifying the mask size.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *      1/9 & 1/9 & 1/9 \\
 *      1/9 & 1/9 & 1/9 \\
 *      1/9 & 1/9 & 1/9 \\
 *  \end{array} \right)
 *  \left( \begin{array}{rrrrr}
 *      1/25 & 1/25 & 1/25 & 1/25 & 1/25 \\
 *      1/25 & 1/25 & 1/25 & 1/25 & 1/25 \\
 *      1/25 & 1/25 & 1/25 & 1/25 & 1/25 \\
 *      1/25 & 1/25 & 1/25 & 1/25 & 1/25 \\
 *      1/25 & 1/25 & 1/25 & 1/25 & 1/25 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned low-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassParameters">Common parameters for nppiFilterLowPass functions</a>.
 *
 */
NppStatus 
nppiFilterLowPass_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                             NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPass_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                         NppiMaskSize eMaskSize);

/**
 * Three channel 8-bit unsigned low-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassParameters">Common parameters for nppiFilterLowPass functions</a>.
 *
 */
NppStatus 
nppiFilterLowPass_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                             NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPass_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                         NppiMaskSize eMaskSize);

/**
 * Four channel 8-bit unsigned low-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassParameters">Common parameters for nppiFilterLowPass functions</a>.
 *
 */
NppStatus 
nppiFilterLowPass_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                             NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPass_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                         NppiMaskSize eMaskSize);

/**
 * Four channel 8-bit unsigned low-pass filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassParameters">Common parameters for nppiFilterLowPass functions</a>.
 *
 */
NppStatus 
nppiFilterLowPass_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPass_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                          NppiMaskSize eMaskSize);

/**
 * Single channel 16-bit unsigned low-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassParameters">Common parameters for nppiFilterLowPass functions</a>.
 *
 */
NppStatus 
nppiFilterLowPass_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPass_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                          NppiMaskSize eMaskSize);

/**
 * Three channel 16-bit unsigned low-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassParameters">Common parameters for nppiFilterLowPass functions</a>.
 *
 */
NppStatus 
nppiFilterLowPass_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPass_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                          NppiMaskSize eMaskSize);

/**
 * Four channel 16-bit unsigned low-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassParameters">Common parameters for nppiFilterLowPass functions</a>.
 *
 */
NppStatus 
nppiFilterLowPass_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPass_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                          NppiMaskSize eMaskSize);

/**
 * Four channel 16-bit unsigned low-pass filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassParameters">Common parameters for nppiFilterLowPass functions</a>.
 *
 */
NppStatus 
nppiFilterLowPass_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPass_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                           NppiMaskSize eMaskSize);

/**
 * Single channel 16-bit signed low-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassParameters">Common parameters for nppiFilterLowPass functions</a>.
 *
 */
NppStatus 
nppiFilterLowPass_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPass_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                          NppiMaskSize eMaskSize);

/**
 * Three channel 16-bit signed low-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassParameters">Common parameters for nppiFilterLowPass functions</a>.
 *
 */
NppStatus 
nppiFilterLowPass_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPass_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                          NppiMaskSize eMaskSize);

/**
 * Four channel 16-bit signed low-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassParameters">Common parameters for nppiFilterLowPass functions</a>.
 *
 */
NppStatus 
nppiFilterLowPass_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPass_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                          NppiMaskSize eMaskSize);

/**
 * Four channel 16-bit signed low-pass filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassParameters">Common parameters for nppiFilterLowPass functions</a>.
 *
 */
NppStatus 
nppiFilterLowPass_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPass_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                           NppiMaskSize eMaskSize);

/**
 * Single channel 32-bit floating-point low-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassParameters">Common parameters for nppiFilterLowPass functions</a>.
 *
 */
NppStatus 
nppiFilterLowPass_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPass_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                          NppiMaskSize eMaskSize);

/**
 * Three channel 32-bit floating-point low-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassParameters">Common parameters for nppiFilterLowPass functions</a>.
 *
 */
NppStatus 
nppiFilterLowPass_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPass_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                          NppiMaskSize eMaskSize);

/**
 * Four channel 32-bit floating-point low-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassParameters">Common parameters for nppiFilterLowPass functions</a>.
 *
 */
NppStatus 
nppiFilterLowPass_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                              NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPass_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                          NppiMaskSize eMaskSize);

/**
 * Four channel 32-bit floating-point high-pass filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassParameters">Common parameters for nppiFilterLowPass functions</a>.
 *
 */
NppStatus 
nppiFilterLowPass_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPass_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                           NppiMaskSize eMaskSize);

/** @} image_filter_low_pass */

/** @defgroup image_filter_low_pass_border FilterLowPassBorder 
 * Filters the image using a low-pass filter kernel with border control.
 *
 * If any portion of the mask overlaps the source image boundary the requested 
 * border type operation is applied to all mask pixels which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported. 
 *
 * <h3><a name="CommonFilterLowPassBorderParameters">Common parameters for nppiFilterLowPassBorder functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param eMaskSize Enumeration value specifying the mask size.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context.
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *      1/9 & 1/9 & 1/9 \\
 *      1/9 & 1/9 & 1/9 \\
 *      1/9 & 1/9 & 1/9 \\
 *  \end{array} \right)
 *  \left( \begin{array}{rrrrr}
 *      1/25 & 1/25 & 1/25 & 1/25 & 1/25 \\
 *      1/25 & 1/25 & 1/25 & 1/25 & 1/25 \\
 *      1/25 & 1/25 & 1/25 & 1/25 & 1/25 \\
 *      1/25 & 1/25 & 1/25 & 1/25 & 1/25 \\
 *      1/25 & 1/25 & 1/25 & 1/25 & 1/25 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassBorderParameters">Common parameters for nppiFilterLowPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLowPassBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                   NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPassBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                               NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Three channel 8-bit unsigned high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassBorderParameters">Common parameters for nppiFilterLowPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLowPassBorder_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                   NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPassBorder_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassBorderParameters">Common parameters for nppiFilterLowPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLowPassBorder_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                   NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPassBorder_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned high-pass filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassBorderParameters">Common parameters for nppiFilterLowPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLowPassBorder_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPassBorder_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Single channel 16-bit unsigned high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassBorderParameters">Common parameters for nppiFilterLowPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLowPassBorder_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPassBorder_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Three channel 16-bit unsigned high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassBorderParameters">Common parameters for nppiFilterLowPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLowPassBorder_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPassBorder_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassBorderParameters">Common parameters for nppiFilterLowPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLowPassBorder_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPassBorder_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned high-pass filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassBorderParameters">Common parameters for nppiFilterLowPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLowPassBorder_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPassBorder_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Single channel 16-bit signed high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassBorderParameters">Common parameters for nppiFilterLowPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLowPassBorder_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPassBorder_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Three channel 16-bit signed high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassBorderParameters">Common parameters for nppiFilterLowPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLowPassBorder_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPassBorder_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassBorderParameters">Common parameters for nppiFilterLowPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLowPassBorder_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPassBorder_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed high-pass filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassBorderParameters">Common parameters for nppiFilterLowPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLowPassBorder_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPassBorder_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassBorderParameters">Common parameters for nppiFilterLowPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLowPassBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPassBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Three channel 32-bit floating-point high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassBorderParameters">Common parameters for nppiFilterLowPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLowPassBorder_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPassBorder_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point high-pass filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassBorderParameters">Common parameters for nppiFilterLowPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLowPassBorder_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                    NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPassBorder_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point high-pass filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterLowPassBorderParameters">Common parameters for nppiFilterLowPassBorder functions</a>.
 *
 */
NppStatus 
nppiFilterLowPassBorder_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                     NppiMaskSize eMaskSize, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterLowPassBorder_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI,
                                 NppiMaskSize eMaskSize, NppiBorderType eBorderType);

/** @} image_filter_low_pass_border */

/** @defgroup image_filter_sharpen FilterSharpen
 *
 * Filters the image using a sharpening filter kernel:
 *
 * <h3><a name="CommonFilterSharpenParameters">Common parameters for nppiFilterSharpen functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *      -1/8 & -1/8 & -1/8 \\
 *      -1/8 & 16/8 & -1/8 \\
 *      -1/8 & -1/8 & -1/8 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned sharpening filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenParameters">Common parameters for nppiFilterSharpen functions</a>.
 *
 */
NppStatus 
nppiFilterSharpen_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpen_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Three channel 8-bit unsigned sharpening filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenParameters">Common parameters for nppiFilterSharpen functions</a>.
 *
 */
NppStatus 
nppiFilterSharpen_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpen_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 8-bit unsigned sharpening filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenParameters">Common parameters for nppiFilterSharpen functions</a>.
 *
 */
NppStatus 
nppiFilterSharpen_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpen_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 8-bit unsigned sharpening filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenParameters">Common parameters for nppiFilterSharpen functions</a>.
 *
 */
NppStatus 
nppiFilterSharpen_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpen_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Single channel 16-bit unsigned sharpening filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenParameters">Common parameters for nppiFilterSharpen functions</a>.
 *
 */
NppStatus 
nppiFilterSharpen_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpen_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Three channel 16-bit unsigned sharpening filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenParameters">Common parameters for nppiFilterSharpen functions</a>.
 *
 */
NppStatus 
nppiFilterSharpen_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpen_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 16-bit unsigned sharpening filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenParameters">Common parameters for nppiFilterSharpen functions</a>.
 *
 */
NppStatus 
nppiFilterSharpen_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpen_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 16-bit unsigned sharpening filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenParameters">Common parameters for nppiFilterSharpen functions</a>.
 *
 */
NppStatus 
nppiFilterSharpen_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpen_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Single channel 16-bit signed sharpening filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenParameters">Common parameters for nppiFilterSharpen functions</a>.
 *
 */
NppStatus 
nppiFilterSharpen_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpen_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Three channel 16-bit signed sharpening filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenParameters">Common parameters for nppiFilterSharpen functions</a>.
 *
 */
NppStatus 
nppiFilterSharpen_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpen_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 16-bit signed sharpening filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenParameters">Common parameters for nppiFilterSharpen functions</a>.
 *
 */
NppStatus 
nppiFilterSharpen_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpen_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 16-bit signed sharpening filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenParameters">Common parameters for nppiFilterSharpen functions</a>.
 *
 */
NppStatus 
nppiFilterSharpen_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpen_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Single channel 32-bit floating-point sharpening filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenParameters">Common parameters for nppiFilterSharpen functions</a>.
 *
 */
NppStatus 
nppiFilterSharpen_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpen_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Three channel 32-bit floating-point sharpening filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenParameters">Common parameters for nppiFilterSharpen functions</a>.
 *
 */
NppStatus 
nppiFilterSharpen_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpen_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 32-bit floating-point sharpening filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenParameters">Common parameters for nppiFilterSharpen functions</a>.
 *
 */
NppStatus 
nppiFilterSharpen_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpen_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/**
 * Four channel 32-bit floating-point sharpening filter, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenParameters">Common parameters for nppiFilterSharpen functions</a>.
 *
 */
NppStatus 
nppiFilterSharpen_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpen_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI);

/** @} image_filter_sharpen */

/** @defgroup image_filter_sharpen_border FilterSharpenBorder
 * Filters the image using a sharpening filter kernel with border control.
 *
 * If any portion of the 3x3 mask overlaps the source
 * image boundary the requested border type operation is applied to all mask pixels
 * which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.
 *
 * <h3><a name="CommonFilterSharpenBorderParameters">Common parameters for nppiFilterSharpenBorder functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *      -1/8 & -1/8 & -1/8 \\
 *      -1/8 & 16/8 & -1/8 \\
 *      -1/8 & -1/8 & -1/8 \\
 *  \end{array} \right)
 * \f]
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned sharpening filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenBorderParameters">Common parameters for nppiFilterSharpenBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSharpenBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpenBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Three channel 8-bit unsigned sharpening filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenBorderParameters">Common parameters for nppiFilterSharpenBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSharpenBorder_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpenBorder_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned sharpening filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenBorderParameters">Common parameters for nppiFilterSharpenBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSharpenBorder_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpenBorder_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned sharpening filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenBorderParameters">Common parameters for nppiFilterSharpenBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSharpenBorder_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpenBorder_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Single channel 16-bit unsigned sharpening filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenBorderParameters">Common parameters for nppiFilterSharpenBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSharpenBorder_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpenBorder_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Three channel 16-bit unsigned sharpening filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenBorderParameters">Common parameters for nppiFilterSharpenBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSharpenBorder_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpenBorder_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned sharpening filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenBorderParameters">Common parameters for nppiFilterSharpenBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSharpenBorder_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpenBorder_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 16-bit unsigned sharpening filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenBorderParameters">Common parameters for nppiFilterSharpenBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSharpenBorder_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpenBorder_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Single channel 16-bit signed sharpening filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenBorderParameters">Common parameters for nppiFilterSharpenBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSharpenBorder_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpenBorder_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Three channel 16-bit signed sharpening filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenBorderParameters">Common parameters for nppiFilterSharpenBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSharpenBorder_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpenBorder_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed sharpening filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenBorderParameters">Common parameters for nppiFilterSharpenBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSharpenBorder_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpenBorder_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed sharpening filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenBorderParameters">Common parameters for nppiFilterSharpenBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSharpenBorder_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpenBorder_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Single channel 32-bit floating-point sharpening filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenBorderParameters">Common parameters for nppiFilterSharpenBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSharpenBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpenBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Three channel 32-bit floating-point sharpening filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenBorderParameters">Common parameters for nppiFilterSharpenBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSharpenBorder_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpenBorder_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point sharpening filter with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenBorderParameters">Common parameters for nppiFilterSharpenBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSharpenBorder_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpenBorder_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/**
 * Four channel 32-bit floating-point sharpening filter with border control, ignoring alpha channel.
 *
 * For common parameter descriptions, see <a href="#CommonFilterSharpenBorderParameters">Common parameters for nppiFilterSharpenBorder functions</a>.
 *
 */
NppStatus 
nppiFilterSharpenBorder_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterSharpenBorder_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, NppiBorderType eBorderType);

/** @} image_filter_sharpen_border */

/** @defgroup image_filter_unsharp_border FilterUnsharpBorder
 * Filters the image using a unsharp-mask sharpening filter kernel with border control.
 *
 * The algorithm involves the following steps:
 * Smooth the original image with a Gaussian filter, with the width controlled by the nRadius.
 * Subtract the smoothed image from the original to create a high-pass filtered image.
 * Apply any clipping needed on the high-pass image, as controlled by the nThreshold.
 * Add a certain percentage of the high-pass filtered image to the original image, 
 * with the percentage controlled by the nWeight.
 * In pseudocode this algorithm can be written as:
 * HighPass = Image - Gaussian(Image)
 * Result = Image + nWeight * HighPass * ( |HighPass| >= nThreshold ) 
 * where nWeight is the amount, nThreshold is the threshold, and >= indicates a Boolean operation, 1 if true, or 0 otherwise.
 *
 * If any portion of the mask overlaps the source image boundary, the requested border type 
 * operation is applied to all mask pixels which fall outside of the source image.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.
 *
 * <h3><a name="CommonFilterUnsharpBorderParameters">Common parameters for nppiFilterUnsharpBorder functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nRadius The radius of the Gaussian filter, in pixles, not counting the center pixel.
 * \param nSigma The standard deviation of the Gaussian filter, in pixel.
 * \param nWeight The percentage of the difference between the original and the high pass image that is added back into the original.
 * \param nThreshold The threshold neede to apply the difference amount.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param pDeviceBuffer Pointer to the user-allocated device scratch buffer required for the unsharp operation.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * <h3><a name="CommonFilterUnsharpGetBufferSizeParameters">Common parameters for nppiFilterUnsharpGetBufferSize functions include:</a></h3>
 *
 * \param nRadius The radius of the Gaussian filter, in pixles, not counting the center pixel.
 * \param nSigma The standard deviation of the Gaussian filter, in pixel.
 * \param hpBufferSize Pointer to the size of the scratch buffer required for the unsharp operation.
 * \return \ref image_data_error_codes
 *
 * @{
 *
 */

/**
 * Single channel 8-bit unsigned unsharp filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpBorderParameters">Common parameters for nppiFilterUnsharpBorder functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
							       NppiBorderType eBorderType, Npp8u * pDeviceBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterUnsharpBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
							   NppiBorderType eBorderType, Npp8u * pDeviceBuffer);

/**
 * Three channel 8-bit unsigned unsharp filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpBorderParameters">Common parameters for nppiFilterUnsharpBorder functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpBorder_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
							       NppiBorderType eBorderType, Npp8u * pDeviceBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterUnsharpBorder_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
							   NppiBorderType eBorderType, Npp8u * pDeviceBuffer);

/**
 * Four channel 8-bit unsigned unsharp filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpBorderParameters">Common parameters for nppiFilterUnsharpBorder functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpBorder_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
							       NppiBorderType eBorderType, Npp8u * pDeviceBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterUnsharpBorder_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
							   NppiBorderType eBorderType, Npp8u * pDeviceBuffer);

/**
 * Four channel 8-bit unsigned unsharp filter (alpha channel is not processed).
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpBorderParameters">Common parameters for nppiFilterUnsharpBorder functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpBorder_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								    NppiBorderType eBorderType, Npp8u * pDeviceBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterUnsharpBorder_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								NppiBorderType eBorderType, Npp8u * pDeviceBuffer);

/**
 * Single channel 16-bit unsigned unsharp filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpBorderParameters">Common parameters for nppiFilterUnsharpBorder functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpBorder_16u_C1R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								    NppiBorderType eBorderType, Npp8u * pDeviceBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterUnsharpBorder_16u_C1R(const Npp16u * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								NppiBorderType eBorderType, Npp8u * pDeviceBuffer);

/**
 * Three channel 16-bit unsigned unsharp filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpBorderParameters">Common parameters for nppiFilterUnsharpBorder functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpBorder_16u_C3R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								    NppiBorderType eBorderType, Npp8u * pDeviceBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterUnsharpBorder_16u_C3R(const Npp16u * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								NppiBorderType eBorderType, Npp8u * pDeviceBuffer);

/**
 * Four channel 16-bit unsigned unsharp filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpBorderParameters">Common parameters for nppiFilterUnsharpBorder functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpBorder_16u_C4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								    NppiBorderType eBorderType, Npp8u * pDeviceBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterUnsharpBorder_16u_C4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								NppiBorderType eBorderType, Npp8u * pDeviceBuffer);

/**
 * Four channel 16-bit unsigned unsharp filter (alpha channel is not processed).
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpBorderParameters">Common parameters for nppiFilterUnsharpBorder functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpBorder_16u_AC4R_Ctx(const Npp16u * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								     NppiBorderType eBorderType, Npp8u * pDeviceBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterUnsharpBorder_16u_AC4R(const Npp16u * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp16u * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								 NppiBorderType eBorderType, Npp8u * pDeviceBuffer);

/**
 * Single channel 16-bit signed unsharp filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpBorderParameters">Common parameters for nppiFilterUnsharpBorder functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpBorder_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								    NppiBorderType eBorderType, Npp8u * pDeviceBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterUnsharpBorder_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								NppiBorderType eBorderType, Npp8u * pDeviceBuffer);

/**
 * Three channel 16-bit signed unsharp filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpBorderParameters">Common parameters for nppiFilterUnsharpBorder functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpBorder_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								    NppiBorderType eBorderType, Npp8u * pDeviceBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterUnsharpBorder_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								NppiBorderType eBorderType, Npp8u * pDeviceBuffer);

/**
 * Four channel 16-bit signed unsharp filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpBorderParameters">Common parameters for nppiFilterUnsharpBorder functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpBorder_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								    NppiBorderType eBorderType, Npp8u * pDeviceBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterUnsharpBorder_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								NppiBorderType eBorderType, Npp8u * pDeviceBuffer);

/**
 * Four channel 16-bit signed unsharp filter (alpha channel is not processed).
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpBorderParameters">Common parameters for nppiFilterUnsharpBorder functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpBorder_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								     NppiBorderType eBorderType, Npp8u * pDeviceBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterUnsharpBorder_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								 NppiBorderType eBorderType, Npp8u * pDeviceBuffer);

/**
 * Single channel 32-bit floating point unsharp filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpBorderParameters">Common parameters for nppiFilterUnsharpBorder functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								    NppiBorderType eBorderType, Npp8u * pDeviceBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterUnsharpBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								NppiBorderType eBorderType, Npp8u * pDeviceBuffer);

/**
 * Three channel 32-bit floating point unsharp filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpBorderParameters">Common parameters for nppiFilterUnsharpBorder functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpBorder_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								    NppiBorderType eBorderType, Npp8u * pDeviceBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterUnsharpBorder_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								NppiBorderType eBorderType, Npp8u * pDeviceBuffer);

/**
 * Four channel 32-bit floating point unsharp filter.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpBorderParameters">Common parameters for nppiFilterUnsharpBorder functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpBorder_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								    NppiBorderType eBorderType, Npp8u * pDeviceBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterUnsharpBorder_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								NppiBorderType eBorderType, Npp8u * pDeviceBuffer);

/**
 * Four channel 32-bit floating point unsharp filter (alpha channel is not processed).
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpBorderParameters">Common parameters for nppiFilterUnsharpBorder functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpBorder_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								     NppiBorderType eBorderType, Npp8u * pDeviceBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterUnsharpBorder_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, Npp32f nRadius, Npp32f nSigma, Npp32f nWeight, Npp32f nThreshold,
								 NppiBorderType eBorderType, Npp8u * pDeviceBuffer);

/**
 * Single channel 8-bit unsigned unsharp filter scratch memory size.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpGetBufferSizeParameters">Common parameters for nppiFilterUnsharpGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpGetBufferSize_8u_C1R(const Npp32f nRadius, const Npp32f nSigma, int * hpBufferSize);

/**
 * Three channel 8-bit unsigned unsharp filter scratch memory size.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpGetBufferSizeParameters">Common parameters for nppiFilterUnsharpGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpGetBufferSize_8u_C3R(const Npp32f nRadius, const Npp32f nSigma, int * hpBufferSize);

/**
 * Four channel 8-bit unsigned unsharp filter scratch memory size.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpGetBufferSizeParameters">Common parameters for nppiFilterUnsharpGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpGetBufferSize_8u_C4R(const Npp32f nRadius, const Npp32f nSigma, int * hpBufferSize);

/**
 * Four channel 8-bit unsigned unsharp filter scratch memory size (alpha channel is not processed).
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpGetBufferSizeParameters">Common parameters for nppiFilterUnsharpGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpGetBufferSize_8u_AC4R(const Npp32f nRadius, const Npp32f nSigma, int * hpBufferSize);

/**
 * Single channel 16-bit unsigned unsharp filter scratch memory size.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpGetBufferSizeParameters">Common parameters for nppiFilterUnsharpGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpGetBufferSize_16u_C1R(const Npp32f nRadius, const Npp32f nSigma, int * hpBufferSize);

/**
 * Three channel 16-bit unsigned unsharp filter scratch memory size.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpGetBufferSizeParameters">Common parameters for nppiFilterUnsharpGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpGetBufferSize_16u_C3R(const Npp32f nRadius, const Npp32f nSigma, int * hpBufferSize);

/**
 * Four channel 16-bit unsigned unsharp filter scratch memory size.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpGetBufferSizeParameters">Common parameters for nppiFilterUnsharpGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpGetBufferSize_16u_C4R(const Npp32f nRadius, const Npp32f nSigma, int * hpBufferSize);

/**
 * Four channel 16-bit unsigned unsharp filter scratch memory size (alpha channel is not processed).
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpGetBufferSizeParameters">Common parameters for nppiFilterUnsharpGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpGetBufferSize_16u_AC4R(const Npp32f nRadius, const Npp32f nSigma, int * hpBufferSize);

/**
 * Single channel 16-bit signed unsharp filter scratch memory size.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpGetBufferSizeParameters">Common parameters for nppiFilterUnsharpGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpGetBufferSize_16s_C1R(const Npp32f nRadius, const Npp32f nSigma, int * hpBufferSize);

/**
 * Three channel 16-bit signed unsharp filter scratch memory size.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpGetBufferSizeParameters">Common parameters for nppiFilterUnsharpGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpGetBufferSize_16s_C3R(const Npp32f nRadius, const Npp32f nSigma, int * hpBufferSize);

/**
 * Four channel 16-bit signed unsharp filter scratch memory size.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpGetBufferSizeParameters">Common parameters for nppiFilterUnsharpGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpGetBufferSize_16s_C4R(const Npp32f nRadius, const Npp32f nSigma, int * hpBufferSize);

/**
 * Four channel 16-bit signed unsharp filter scratch memory size (alpha channel is not processed).
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpGetBufferSizeParameters">Common parameters for nppiFilterUnsharpGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpGetBufferSize_16s_AC4R(const Npp32f nRadius, const Npp32f nSigma, int * hpBufferSize);

/**
 * Single channel 32-bit floating point unsharp filter scratch memory size.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpGetBufferSizeParameters">Common parameters for nppiFilterUnsharpGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpGetBufferSize_32f_C1R(const Npp32f nRadius, const Npp32f nSigma, int * hpBufferSize);

/**
 * Three channel 32-bit floating point unsharp filter scratch memory size.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpGetBufferSizeParameters">Common parameters for nppiFilterUnsharpGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpGetBufferSize_32f_C3R(const Npp32f nRadius, const Npp32f nSigma, int * hpBufferSize);

/**
 * Four channel 32-bit floating point unsharp filter scratch memory size.
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpGetBufferSizeParameters">Common parameters for nppiFilterUnsharpGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpGetBufferSize_32f_C4R(const Npp32f nRadius, const Npp32f nSigma, int * hpBufferSize);

/**
 * Four channel 32-bit floating point unsharp filter scratch memory size (alpha channel is not processed).
 *
 * For common parameter descriptions, see <a href="#CommonFilterUnsharpGetBufferSizeParameters">Common parameters for nppiFilterUnsharpGetBufferSize functions</a>.
 *
 */
NppStatus 
nppiFilterUnsharpGetBufferSize_32f_AC4R(const Npp32f nRadius, const Npp32f nSigma, int * hpBufferSize);

/** @} image_filter_unsharp_border */

/** @defgroup image_filter_wiener_border FilterWienerBorder
 * Noise removal filtering of an image using an adaptive Wiener filter with border control.
 *
 * Pixels under the source mask are used to generate statistics about the local neighborhood 
 * which are then used to control the amount of adaptive noise filtering locally applied.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.
 *
 * <h3><a name="CommonFilterWienerBorderParameters">Common parameters for nppiFilterWienerBorder functions include:</a></h3>
 *
 * \param pSrc  \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param oMaskSize Pixel Width and Height of the rectangular region of interest surrounding the source pixel.
 * \param oAnchor Positive X and Y relative offsets of primary pixel in region of interest surrounding the source pixel relative to bottom right of oMaskSize.
 * \param aNoise Fixed size array of per-channel noise variance level value in range of 0.0F to 1.0F.
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 * For each pixel in the source image the function estimates the local mean and variance in
 * the neighborhood defined by oMaskSize relative to the primary source pixel located at oAnchor.x and oAnchor.y. 
 * Given an oMaskSize with width \f$W\f$ and height \f$H\f$, the mean, variance, and destination pixel value
 * will be computed per channel as
 * \f[Mean = \frac{1}{W\cdot H}\sum_{j=0}^{H-1}\sum_{i=0}^{W-1}pSrc(j,i)\f]
 * \f[Variance^2 = \frac{1}{W\cdot H}\sum_{j=0}^{H-1}\sum_{i=0}^{W-1}(pSrc(j,i)^2-Mean^2)\f]
 * \f[pDst(j,i) = Mean+\frac{(Variance^2-NoiseVariance)}{Variance^2}\cdot {(pSrc(j,i)-Mean)}\f]
 *
 */

/**
 * Single channel 8-bit unsigned Wiener filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterWienerBorderParameters">Common parameters for nppiFilterWienerBorder functions</a>.
 *
 */
NppStatus
nppiFilterWienerBorder_8u_C1R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                  NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[1], NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterWienerBorder_8u_C1R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                              NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[1], NppiBorderType eBorderType);

/**
 * Three channel 8-bit unsigned Wiener filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterWienerBorderParameters">Common parameters for nppiFilterWienerBorder functions</a>.
 *
 */
NppStatus
nppiFilterWienerBorder_8u_C3R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                  NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[3], NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterWienerBorder_8u_C3R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                              NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[3], NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned Wiener filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterWienerBorderParameters">Common parameters for nppiFilterWienerBorder functions</a>.
 *
 */
NppStatus
nppiFilterWienerBorder_8u_C4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                  NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[4], NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterWienerBorder_8u_C4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                              NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[4], NppiBorderType eBorderType);

/**
 * Four channel 8-bit unsigned Wiener filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterWienerBorderParameters">Common parameters for nppiFilterWienerBorder functions</a>.
 *
 */
NppStatus
nppiFilterWienerBorder_8u_AC4R_Ctx(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                   NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[3], NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterWienerBorder_8u_AC4R(const Npp8u * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp8u * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[3], NppiBorderType eBorderType);

/**
 * Single channel 16-bit signed Wiener filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterWienerBorderParameters">Common parameters for nppiFilterWienerBorder functions</a>.
 *
 */
NppStatus
nppiFilterWienerBorder_16s_C1R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                   NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[1], NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterWienerBorder_16s_C1R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[1], NppiBorderType eBorderType);

/**
 * Three channel 16-bit signed Wiener filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterWienerBorderParameters">Common parameters for nppiFilterWienerBorder functions</a>.
 *
 */
NppStatus
nppiFilterWienerBorder_16s_C3R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                   NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[3], NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterWienerBorder_16s_C3R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[3], NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed Wiener filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterWienerBorderParameters">Common parameters for nppiFilterWienerBorder functions</a>.
 *
 */
NppStatus
nppiFilterWienerBorder_16s_C4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                   NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[4], NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterWienerBorder_16s_C4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[4], NppiBorderType eBorderType);

/**
 * Four channel 16-bit signed Wiener filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterWienerBorderParameters">Common parameters for nppiFilterWienerBorder functions</a>.
 *
 */
NppStatus
nppiFilterWienerBorder_16s_AC4R_Ctx(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                    NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[3], NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterWienerBorder_16s_AC4R(const Npp16s * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp16s * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[3], NppiBorderType eBorderType);

/**
 * Single channel 32-bit float Wiener filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterWienerBorderParameters">Common parameters for nppiFilterWienerBorder functions</a>.
 *
 */
NppStatus
nppiFilterWienerBorder_32f_C1R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                   NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[1], NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterWienerBorder_32f_C1R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[1], NppiBorderType eBorderType);

/**
 * Three channel 32-bit float Wiener filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterWienerBorderParameters">Common parameters for nppiFilterWienerBorder functions</a>.
 *
 */
NppStatus
nppiFilterWienerBorder_32f_C3R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                   NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[3], NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterWienerBorder_32f_C3R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[3], NppiBorderType eBorderType);

/**
 * Four channel 32-bit float Wiener filter with border control.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterWienerBorderParameters">Common parameters for nppiFilterWienerBorder functions</a>.
 *
 */
NppStatus
nppiFilterWienerBorder_32f_C4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                   NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[4], NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterWienerBorder_32f_C4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                               NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[4], NppiBorderType eBorderType);

/**
 * Four channel 32-bit float Wiener filter with border control, ignoring alpha channel.
 * 
 * For common parameter descriptions, see <a href="#CommonFilterWienerBorderParameters">Common parameters for nppiFilterWienerBorder functions</a>.
 *
 */
NppStatus
nppiFilterWienerBorder_32f_AC4R_Ctx(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                    NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[3], NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus
nppiFilterWienerBorder_32f_AC4R(const Npp32f * pSrc, Npp32s nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, Npp32f * pDst, Npp32s nDstStep, NppiSize oSizeROI, 
                                NppiSize oMaskSize, NppiPoint oAnchor, Npp32f aNoise[3], NppiBorderType eBorderType);

/** @} image_filter_wiener_border */

/** @defgroup image_filter_gradient_vector_prewitt_border GradientVectorPrewittBorder
 * 
 *  RGB Color to Prewitt Gradient Vector conversion using user selected fixed mask size and gradient distance method.
 *  Functions support up to 4 optional single channel output gradient vectors, X (vertical), Y (horizontal), magnitude, and angle
 *  with user selectable distance methods.  Output for a particular vector is disabled by supplying a NULL pointer for that
 *  vector. X and Y gradient vectors are in cartesian form in the destination data type.  
 *  Magnitude vectors are polar gradient form in the destination data type, angle is always in floating point polar gradient format.
 *  Only fixed mask sizes of 3x3 are supported.
 *  Only nppiNormL1 (sum) and nppiNormL2 (sqrt of sum of squares) distance methods are currently supported.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.  Borderless output can be accomplished by using a
 * larger source image than the destination and adjusting oSrcSize and oSrcOffset parameters accordingly.
 *
 * <h3><a name="CommonFilterGradientVectorPrewittBorderParameters">Common parameters for nppiFilterGradientVectorPrewittBorder functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDstX X vector destination_image_pointer.
 * \param nDstXStep X vector destination_image_line_step.
 * \param pDstY Y vector destination_image_pointer.
 * \param nDstYStep Y vector destination_image_line_step.
 * \param pDstMag magnitude destination_image_pointer.
 * \param nDstMagStep magnitude destination_image_line_step.
 * \param pDstAngle angle destination_image_pointer.
 * \param nDstAngleStep angle destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param eMaskSize fixed filter mask size to use.
 * \param eNorm gradient distance method to use.
 * \param eBorderType source image border type to use use.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * The following fixed kernel mask is used for producing the pDstX (vertical) output image.
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *   -1 & 0 & 1 \\
 *   -1 & 0 & 1 \\
 *   -1 & 0 & 1 \\
 *  \end{array} \right)
 * \f]
 *  
 * The following fixed kernel mask is used for producing the pDstY (horizontal) output image.
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *    1 &  1 &  1 \\
 *    0 &  0 &  0 \\
 *   -1 & -1 & -1 \\
 *  \end{array} \right)
 * \f]
 *
 * For the C1R versions of the function the pDstMag output image value for L1 normalization consists of 
 * the absolute value of the pDstX value plus the absolute value of the pDstY value at that particular image pixel location.
 * For the C1R versions of the function the pDstMag output image value for L2 normalization consists of 
 * the square root of the pDstX value squared plus the pDstY value squared at that particular image pixel location.
 * For the C1R versions of the function the pDstAngle output image value consists of the arctangent (atan2) of 
 * the pDstY value and the pDstX value at that particular image pixel location.
 *
 * For the C3C1R versions of the function, regardless of the selected normalization method, 
 * the L2 normalization value is first determined for each or the pDstX and pDstY values for each source channel then the largest L2
 * normalization value (largest gradient) is used to select which of the 3 pDstX channel values are output to the pDstX image or 
 * pDstY channel values are output to the pDstY image.
 * For the C3C1R versions of the function the pDstMag output image value for L1 normalizaton consists of the same technique
 * used for the C1R version for each source image channel.  Then the largest L2 normalization value is again used to select which
 * of the 3 pDstMag channel values to output to the pDstMag image.
 * For the C3C1R versions of the function the pDstMag output image value for L2 normalizaton consists of just outputting
 * the largest per source channel L2 normalization value to the pDstMag image.
 * For the C3C1R versions of the function the pDstAngle output image value consists of the same technique used for the C1R version
 * calculated for each source image channel.  Then the largest L2 normalization value is again used to select which of the 3 angle
 * values to output to the pDstAngle image. 
 *
 *
 * @{
 *
 */

/**
 * 1 channel 8-bit unsigned packed RGB to optional 1 channel 16-bit signed X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGradientVectorPrewittBorderParameters">Common parameters for nppiFilterGradientVectorPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiGradientVectorPrewittBorder_8u16s_C1R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                    Npp16s * pDstX, int nDstXStep, Npp16s * pDstY, int nDstYStep, Npp16s * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                              NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiGradientVectorPrewittBorder_8u16s_C1R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                Npp16s * pDstX, int nDstXStep, Npp16s * pDstY, int nDstYStep, Npp16s * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                          NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);

/**
 * 3 channel 8-bit unsigned packed RGB to optional 1 channel 16-bit signed X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGradientVectorPrewittBorderParameters">Common parameters for nppiFilterGradientVectorPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiGradientVectorPrewittBorder_8u16s_C3C1R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                      Npp16s * pDstX, int nDstXStep, Npp16s * pDstY, int nDstYStep, Npp16s * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                                NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiGradientVectorPrewittBorder_8u16s_C3C1R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                  Npp16s * pDstX, int nDstXStep, Npp16s * pDstY, int nDstYStep, Npp16s * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                            NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);

/**
 * 1 channel 16-bit signed packed RGB to optional 1 channel 32-bit floating point X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGradientVectorPrewittBorderParameters">Common parameters for nppiFilterGradientVectorPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiGradientVectorPrewittBorder_16s32f_C1R_Ctx(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                     Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                               NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiGradientVectorPrewittBorder_16s32f_C1R(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                 Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                           NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);

/**
 * 3 channel 16-bit signed packed RGB to optional 1 channel 32-bit floating point X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGradientVectorPrewittBorderParameters">Common parameters for nppiFilterGradientVectorPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiGradientVectorPrewittBorder_16s32f_C3C1R_Ctx(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                       Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                                 NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiGradientVectorPrewittBorder_16s32f_C3C1R(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                   Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                             NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);

/**
 * 1 channel 16-bit unsigned packed RGB to optional 1 channel 32-bit floating point X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGradientVectorPrewittBorderParameters">Common parameters for nppiFilterGradientVectorPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiGradientVectorPrewittBorder_16u32f_C1R_Ctx(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                     Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                               NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiGradientVectorPrewittBorder_16u32f_C1R(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                 Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                           NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);

/**
 * 3 channel 16-bit unsigned packed RGB to optional 1 channel 32-bit floating point X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGradientVectorPrewittBorderParameters">Common parameters for nppiFilterGradientVectorPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiGradientVectorPrewittBorder_16u32f_C3C1R_Ctx(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                       Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                                 NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiGradientVectorPrewittBorder_16u32f_C3C1R(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                   Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                             NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);

/**
 * 1 channel 32-bit floating point packed RGB to optional 1 channel 32-bit floating point X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGradientVectorPrewittBorderParameters">Common parameters for nppiFilterGradientVectorPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiGradientVectorPrewittBorder_32f_C1R_Ctx(const Npp32f * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                  Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                            NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiGradientVectorPrewittBorder_32f_C1R(const Npp32f * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                              Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                        NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);

/**
 * 3 channel 32-bit floating point packed RGB to optional 1 channel 32-bit floating point X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGradientVectorPrewittBorderParameters">Common parameters for nppiFilterGradientVectorPrewittBorder functions</a>.
 *
 */
NppStatus 
nppiGradientVectorPrewittBorder_32f_C3C1R_Ctx(const Npp32f * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                    Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                              NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiGradientVectorPrewittBorder_32f_C3C1R(const Npp32f * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                          NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);


/** @} image_filter_gradient_vector_prewitt_border */

/** @defgroup image_filter_gradient_vector_scharr_border GradientVectorScharrBorder
 * 
 *  RGB Color to Scharr Gradient Vector conversion using user selected fixed mask size and gradient distance method.
 *  Functions support up to 4 optional single channel output gradient vectors, X (vertical), Y (horizontal), magnitude, and angle
 *  with user selectable distance methods.  Output for a particular vector is disabled by supplying a NULL pointer for that
 *  vector. X and Y gradient vectors are in cartesian form in the destination data type.  
 *  Magnitude vectors are polar gradient form in the destination data type, angle is always in floating point polar gradient format.
 *  Only fixed mask sizes of 3x3 are supported.
 *  Only nppiNormL1 (sum) and nppiNormL2 (sqrt of sum of squares) distance methods are currently supported.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.  Borderless output can be accomplished by using a
 * larger source image than the destination and adjusting oSrcSize and oSrcOffset parameters accordingly.
 *
 * <h3><a name="CommonFilterGradientVectorScharrBorderParameters">Common parameters for nppiFilterGradientVectorScharrBorder functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDstX X vector destination_image_pointer.
 * \param nDstXStep X vector destination_image_line_step.
 * \param pDstY Y vector destination_image_pointer.
 * \param nDstYStep Y vector destination_image_line_step.
 * \param pDstMag magnitude destination_image_pointer.
 * \param nDstMagStep magnitude destination_image_line_step.
 * \param pDstAngle angle destination_image_pointer.
 * \param nDstAngleStep angle destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param eMaskSize fixed filter mask size to use.
 * \param eNorm gradient distance method to use.
 * \param eBorderType source image border type to use use.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * The following fixed kernel mask is used for producing the pDstX (vertical) output image.
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *    3 & 0 &  -3 \\
 *   10 & 0 & -10 \\
 *    3 & 0 &  -3 \\
 *  \end{array} \right)
 * \f]
 *  
 * The following fixed kernel mask is used for producing the pDstY (horizontal) output image.
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *    3 &  10 &  3 \\
 *    0 &   0 &  0 \\
 *   -3 & -10 & -3 \\
 *  \end{array} \right)
 * \f]
 *
 * For the C1R versions of the function the pDstMag output image value for L1 normalization consists of 
 * the absolute value of the pDstX value plus the absolute value of the pDstY value at that particular image pixel location.
 * For the C1R versions of the function the pDstMag output image value for L2 normalization consists of 
 * the square root of the pDstX value squared plus the pDstY value squared at that particular image pixel location.
 * For the C1R versions of the function the pDstAngle output image value consists of the arctangent (atan2) of 
 * the pDstY value and the pDstX value at that particular image pixel location.
 *
 * For the C3C1R versions of the function, regardless of the selected normalization method, 
 * the L2 normalization value is first determined for each or the pDstX and pDstY values for each source channel then the largest L2
 * normalization value (largest gradient) is used to select which of the 3 pDstX channel values are output to the pDstX image or 
 * pDstY channel values are output to the pDstY image.
 * For the C3C1R versions of the function the pDstMag output image value for L1 normalizaton consists of the same technique
 * used for the C1R version for each source image channel.  Then the largest L2 normalization value is again used to select which
 * of the 3 pDstMag channel values to output to the pDstMag image.
 * For the C3C1R versions of the function the pDstMag output image value for L2 normalizaton consists of just outputting
 * the largest per source channel L2 normalization value to the pDstMag image.
 * For the C3C1R versions of the function the pDstAngle output image value consists of the same technique used for the C1R version
 * calculated for each source image channel.  Then the largest L2 normalization value is again used to select which of the 3 angle
 * values to output to the pDstAngle image. 
 *
 * @{
 *
 */

/**
 * 1 channel 8-bit unsigned packed RGB to optional 1 channel 16-bit signed X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGradientVectorScharrBorderParameters">Common parameters for nppiFilterGradientVectorScharrBorder functions</a>.
 *
 */
NppStatus 
nppiGradientVectorScharrBorder_8u16s_C1R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                   Npp16s * pDstX, int nDstXStep, Npp16s * pDstY, int nDstYStep, Npp16s * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                             NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiGradientVectorScharrBorder_8u16s_C1R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                               Npp16s * pDstX, int nDstXStep, Npp16s * pDstY, int nDstYStep, Npp16s * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                         NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);

/**
 * 3 channel 8-bit unsigned packed RGB to optional 1 channel 16-bit signed X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGradientVectorScharrBorderParameters">Common parameters for nppiFilterGradientVectorScharrBorder functions</a>.
 *
 */
NppStatus 
nppiGradientVectorScharrBorder_8u16s_C3C1R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                     Npp16s * pDstX, int nDstXStep, Npp16s * pDstY, int nDstYStep, Npp16s * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                               NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiGradientVectorScharrBorder_8u16s_C3C1R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                 Npp16s * pDstX, int nDstXStep, Npp16s * pDstY, int nDstYStep, Npp16s * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                           NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);

/**
 * 1 channel 16-bit signed packed RGB to optional 1 channel 32-bit floating point X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGradientVectorScharrBorderParameters">Common parameters for nppiFilterGradientVectorScharrBorder functions</a>.
 *
 */
NppStatus 
nppiGradientVectorScharrBorder_16s32f_C1R_Ctx(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                    Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                              NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiGradientVectorScharrBorder_16s32f_C1R(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                          NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);

/**
 * 3 channel 16-bit signed packed RGB to optional 1 channel 32-bit floating point X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGradientVectorScharrBorderParameters">Common parameters for nppiFilterGradientVectorScharrBorder functions</a>.
 *
 */
NppStatus 
nppiGradientVectorScharrBorder_16s32f_C3C1R_Ctx(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                      Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                                NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiGradientVectorScharrBorder_16s32f_C3C1R(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                  Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                            NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);

/**
 * 1 channel 16-bit unsigned packed RGB to optional 1 channel 32-bit floating point X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGradientVectorScharrBorderParameters">Common parameters for nppiFilterGradientVectorScharrBorder functions</a>.
 *
 */
NppStatus 
nppiGradientVectorScharrBorder_16u32f_C1R_Ctx(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                    Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                              NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiGradientVectorScharrBorder_16u32f_C1R(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                          NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);

/**
 * 3 channel 16-bit unsigned packed RGB to optional 1 channel 32-bit floating point X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGradientVectorScharrBorderParameters">Common parameters for nppiFilterGradientVectorScharrBorder functions</a>.
 *
 */
NppStatus 
nppiGradientVectorScharrBorder_16u32f_C3C1R_Ctx(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                      Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                                NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiGradientVectorScharrBorder_16u32f_C3C1R(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                  Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                            NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);

/**
 * 1 channel 32-bit floating point packed RGB to optional 1 channel 32-bit floating point X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGradientVectorScharrBorderParameters">Common parameters for nppiFilterGradientVectorScharrBorder functions</a>.
 *
 */
NppStatus 
nppiGradientVectorScharrBorder_32f_C1R_Ctx(const Npp32f * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                 Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                           NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiGradientVectorScharrBorder_32f_C1R(const Npp32f * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                             Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                       NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);

/**
 * 3 channel 32-bit floating point packed RGB to optional 1 channel 32-bit floating point X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGradientVectorScharrBorderParameters">Common parameters for nppiFilterGradientVectorScharrBorder functions</a>.
 *
 */
NppStatus 
nppiGradientVectorScharrBorder_32f_C3C1R_Ctx(const Npp32f * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                   Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                             NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);


NppStatus 
nppiGradientVectorScharrBorder_32f_C3C1R(const Npp32f * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                               Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                         NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);


/** @} image_filter_gradient_vector_scharr_border */

/** @defgroup image_filter_gradient_vector_sobel_border GradientVectorSobelBorder
 * 
 *  RGB Color to Sobel Gradient Vector conversion using user selected fixed mask size and gradient distance method.
 *  Functions support up to 4 optional single channel output gradient vectors, X (vertical), Y (horizontal), magnitude, and angle
 *  with user selectable distance methods.  Output for a particular vector is disabled by supplying a NULL pointer for that
 *  vector. X and Y gradient vectors are in cartesian form in the destination data type.  
 *  Magnitude vectors are polar gradient form in the destination data type, angle is always in floating point polar gradient format.
 *  Only fixed mask sizes of 3x3 and 5x5 are supported.
 *  Only nppiNormL1 (sum) and nppiNormL2 (sqrt of sum of squares) distance methods are currently supported.
 *
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.  Borderless output can be accomplished by using a
 * larger source image than the destination and adjusting oSrcSize and oSrcOffset parameters accordingly.
 *
 * <h3><a name="CommonFilterGradientVectorSobelBorderParameters">Common parameters for nppiFilterGradientVectorSobelBorder functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDstX X vector destination_image_pointer.
 * \param nDstXStep X vector destination_image_line_step.
 * \param pDstY Y vector destination_image_pointer.
 * \param nDstYStep Y vector destination_image_line_step.
 * \param pDstMag magnitude destination_image_pointer.
 * \param nDstMagStep magnitude destination_image_line_step.
 * \param pDstAngle angle destination_image_pointer.
 * \param nDstAngleStep angle destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param eMaskSize fixed filter mask size to use.
 * \param eNorm gradient distance method to use.
 * \param eBorderType source image border type to use use.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * One of the following fixed kernel masks are used for producing the 3x3 or 5x5 pDstX (vertical) output image depending on selected mask size.
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *   -1 & 0 & 1 \\
 *   -2 & 0 & 2 \\
 *   -1 & 0 & 1 \\
 *  \end{array} \right)
 * \f]
 *  
 *
 * \f[
 *  \left( \begin{array}{rrrrr}
 *   -1 &  -2 & 0 &  2 & 1 \\
 *   -4 &  -8 & 0 &  8 & 4 \\
 *   -6 & -12 & 0 & 12 & 6 \\
 *   -4 &  -8 & 0 &  8 & 4 \\
 *   -1 &  -2 & 0 &  2 & 1 \\
 *  \end{array} \right)
 * \f]
 *  
 * One of the following fixed kernel masks are used for producing the 3x3 or 5x5 pDstY (horizontal) output image depending on selected mask size.
 *
 * \f[
 *  \left( \begin{array}{rrr}
 *    1 &  2 &  1 \\
 *    0 &  0 &  0 \\
 *   -1 & -2 & -1 \\
 *  \end{array} \right)
 * \f]
 *
 *
 * \f[
 *  \left( \begin{array}{rrrrr}
 *    1 &  4 &   6 &  4 &  1 \\
 *    2 &  8 &  12 &  8 &  2 \\
 *    0 &  0 &   0 &  0 &  0 \\
 *   -2 & -8 & -12 & -8 & -2 \\
 *   -1 & -4 &  -6 & -4 & -1 \\
 *  \end{array} \right)
 * \f]
 *
 * For the C1R versions of the function the pDstMag output image value for L1 normalization consists of 
 * the absolute value of the pDstX value plus the absolute value of the pDstY value at that particular image pixel location.
 * For the C1R versions of the function the pDstMag output image value for L2 normalization consists of 
 * the square root of the pDstX value squared plus the pDstY value squared at that particular image pixel location.
 * For the C1R versions of the function the pDstAngle output image value consists of the arctangent (atan2) of 
 * the pDstY value and the pDstX value at that particular image pixel location.
 *
 * For the C3C1R versions of the function, regardless of the selected normalization method, 
 * the L2 normalization value is first determined for each or the pDstX and pDstY values for each source channel then the largest L2
 * normalization value (largest gradient) is used to select which of the 3 pDstX channel values are output to the pDstX image or 
 * pDstY channel values are output to the pDstY image.
 * For the C3C1R versions of the function the pDstMag output image value for L1 normalizaton consists of the same technique
 * used for the C1R version for each source image channel.  Then the largest L2 normalization value is again used to select which
 * of the 3 pDstMag channel values to output to the pDstMag image.
 * For the C3C1R versions of the function the pDstMag output image value for L2 normalizaton consists of just outputting
 * the largest per source channel L2 normalization value to the pDstMag image.
 * For the C3C1R versions of the function the pDstAngle output image value consists of the same technique used for the C1R version
 * calculated for each source image channel.  Then the largest L2 normalization value is again used to select which of the 3 angle
 * values to output to the pDstAngle image. 
 *
 * @{
 *
 */

/**
 * 1 channel 8-bit unsigned packed RGB to optional 1 channel 16-bit signed X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGradientVectorSobelBorderParameters">Common parameters for nppiFilterGradientVectorSobelBorder functions</a>.
 *
 */
NppStatus 
nppiGradientVectorSobelBorder_8u16s_C1R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                  Npp16s * pDstX, int nDstXStep, Npp16s * pDstY, int nDstYStep, Npp16s * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                            NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiGradientVectorSobelBorder_8u16s_C1R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                              Npp16s * pDstX, int nDstXStep, Npp16s * pDstY, int nDstYStep, Npp16s * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                        NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);

/**
 * 3 channel 8-bit unsigned packed RGB to optional 1 channel 16-bit signed X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGradientVectorSobelBorderParameters">Common parameters for nppiFilterGradientVectorSobelBorder functions</a>.
 *
 */
NppStatus 
nppiGradientVectorSobelBorder_8u16s_C3C1R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                    Npp16s * pDstX, int nDstXStep, Npp16s * pDstY, int nDstYStep, Npp16s * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                              NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiGradientVectorSobelBorder_8u16s_C3C1R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                Npp16s * pDstX, int nDstXStep, Npp16s * pDstY, int nDstYStep, Npp16s * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                          NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);

/**
 * 1 channel 16-bit signed packed RGB to optional 1 channel 32-bit floating point X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGradientVectorSobelBorderParameters">Common parameters for nppiFilterGradientVectorSobelBorder functions</a>.
 *
 */
NppStatus 
nppiGradientVectorSobelBorder_16s32f_C1R_Ctx(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                   Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                             NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiGradientVectorSobelBorder_16s32f_C1R(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                               Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                         NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);

/**
 * 3 channel 16-bit signed packed RGB to optional 1 channel 32-bit floating point X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 */
NppStatus 
nppiGradientVectorSobelBorder_16s32f_C3C1R_Ctx(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                     Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                               NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiGradientVectorSobelBorder_16s32f_C3C1R(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                 Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                           NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);

/**
 * 1 channel 16-bit unsigned packed RGB to optional 1 channel 32-bit floating point X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGradientVectorSobelBorderParameters">Common parameters for nppiFilterGradientVectorSobelBorder functions</a>.
 *
 */
NppStatus 
nppiGradientVectorSobelBorder_16u32f_C1R_Ctx(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                   Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                             NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiGradientVectorSobelBorder_16u32f_C1R(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                               Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                         NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);

/**
 * 3 channel 16-bit unsigned packed RGB to optional 1 channel 32-bit floating point X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGradientVectorSobelBorderParameters">Common parameters for nppiFilterGradientVectorSobelBorder functions</a>.
 *
 */
NppStatus 
nppiGradientVectorSobelBorder_16u32f_C3C1R_Ctx(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                     Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                               NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiGradientVectorSobelBorder_16u32f_C3C1R(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                 Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                           NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);

/**
 * 1 channel 32-bit floating point packed RGB to optional 1 channel 32-bit floating point X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGradientVectorSobelBorderParameters">Common parameters for nppiFilterGradientVectorSobelBorder functions</a>.
 *
 */
NppStatus 
nppiGradientVectorSobelBorder_32f_C1R_Ctx(const Npp32f * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                          NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiGradientVectorSobelBorder_32f_C1R(const Npp32f * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                            Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                      NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);

/**
 * 3 channel 32-bit floating point packed RGB to optional 1 channel 32-bit floating point X (vertical), Y (horizontal), magnitude, 
 * and/or 32-bit floating point angle gradient vectors with user selectable fixed mask size and distance method with border control.
 *
 * For common parameter descriptions, see <a href="#CommonFilterGradientVectorSobelBorderParameters">Common parameters for nppiFilterGradientVectorSobelBorder functions</a>.
 *
 */
NppStatus 
nppiGradientVectorSobelBorder_32f_C3C1R_Ctx(const Npp32f * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                  Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                            NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiGradientVectorSobelBorder_32f_C3C1R(const Npp32f * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                              Npp32f * pDstX, int nDstXStep, Npp32f * pDstY, int nDstYStep, Npp32f * pDstMag, int nDstMagStep, Npp32f * pDstAngle, int nDstAngleStep,
                                        NppiSize oSizeROI, NppiMaskSize eMaskSize, NppiNorm eNorm, NppiBorderType eBorderType);


/** @} image_filter_gradient_vector_sobel_border */

/** @} fixed_filters */

/** @defgroup image_computer_vision_filtering_functions Computer Vision
 * The set of computer vision functions available in the library.
 * @{
 *
 */

/** @defgroup image_filter_canny_border FilterCannyBorder
 *  Performs Canny edge detection on a single channel 8-bit grayscale image and outputs a single channel 8-bit image consisting of 0x00 and 0xFF
 *  values with 0xFF representing edge pixels.  
 *
 *  The algorithm consists of three phases.  The first phase generates two output images consisting
 *  of a single channel 16-bit signed image containing magnitude values and a single channel 32-bit floating point image containing the angular
 *  direction of those magnitude values.   This phase is accomplished by calling the appropriate GradientVectorBorder filter function based on
 *  the filter type, filter mask size, and norm type requested.  The next phase uses those magnitude and direction images to suppress non-maximum
 *  magnitude values which are lower than the values of either of its two nearest neighbors in the same direction as the test magnitude pixel in 
 *  the 3x3 surrounding magnitude pixel neighborhood.  This phase outputs a new magnitude image with non-maximum pixel values suppressed.  Finally, in the
 *  third phase, the new magnitude image is passed through a hysteresis threshold filter that filters out any magnitude values that are not connected
 *  to another edge magnitude value.   In this phase, any magnitude value above the high threshold value is automatically accepted, any magnitude
 *  value below the low threshold value is automatically rejected.  For magnitude values that lie between the low and high threshold, values are
 *  only accepted if one of their two neighbors in the same direction in the 3x3 neighborhood around them lies above the low threshold value.  In other words,
 *  if they are connected to an active edge.   J. Canny recommends that the ratio of high to low threshold limit be in the range two or three to one, 
 *  based on predicted signal-to-noise ratios. The final output of the third phase consists of a single channel 8-bit unsigned image of 0x00 and 0xFF 
 *  values based on whether they are accepted or rejected during threshold testing.
 *    
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.  Borderless output can be accomplished by using a
 * larger source image than the destination and adjusting oSrcSize and oSrcOffset parameters accordingly.
 *
 * @{
 *
 */

/**
 * Calculate scratch buffer size needed for the FilterCannyBorder function based on destination image SizeROI width and height.
 *
 * \param oSizeROI \ref roi_specification.
 * \param hpBufferSize Required buffer size. Important: hpBufferSize is a 
 *        <em>host pointer.</em> \ref general_scratch_buffer.
 * \return \ref image_data_error_codes, \ref roi_error_codes
 */

NppStatus 
nppiFilterCannyBorderGetBufferSize(NppiSize oSizeROI, int * hpBufferSize);

/**
 * 1 channel 8-bit unsigned grayscale to 1 channel 8-bit unsigned black (0x00) and white (0xFF) image with border control.
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst output edge destination_image_pointer.
 * \param nDstStep output edge destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param eFilterType selects between Sobel or Scharr filter type.
 * \param eMaskSize fixed filter mask size to use.
 * \param nLowThreshold low hysteresis threshold value.
 * \param nHighThreshold high hysteresis threshold value.
 * \param eNorm gradient distance method to use.
 * \param eBorderType source image border type to use use.
 * \param pDeviceBuffer pointer to scratch DEVICE memory buffer of size hpBufferSize (see nppiFilterCannyBorderGetBufferSize() above)
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 */
NppStatus 
nppiFilterCannyBorder_8u_C1R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                       Npp8u * pDst, int nDstStep, NppiSize oSizeROI, NppiDifferentialKernel eFilterType,
                                 NppiMaskSize eMaskSize, Npp16s nLowThreshold, Npp16s nHighThreshold, NppiNorm eNorm, 
                                 NppiBorderType eBorderType, Npp8u * pDeviceBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterCannyBorder_8u_C1R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                   Npp8u * pDst, int nDstStep, NppiSize oSizeROI, NppiDifferentialKernel eFilterType,
                             NppiMaskSize eMaskSize, Npp16s nLowThreshold, Npp16s nHighThreshold, NppiNorm eNorm, 
                             NppiBorderType eBorderType, Npp8u * pDeviceBuffer);

/** @} image_filter_canny_border */

/** @defgroup image_filter_harris_corners_border FilterHarrisCornersBorder
 *  Performs Harris Corner detection on a single channel 8-bit grayscale image and outputs a single channel 32-bit floating point image 
 *  consisting the corner response at each pixel of the image.
 *  
 *  The algorithm consists of two phases.  The first phase generates the floating
 *  point product of XX, YY, and XY gradients at each pixel in the image.  The type of gradient used is controlled by the eFilterType and eMaskSize parameters.
 *  The second phase averages those products over a window of either 3x3 or 5x5 pixels around the center pixel then generates the Harris corner
 *  response at that pixel which is output in the destination image. The Harris response value is determined as H = ((XX * YY - XY * XY) - 
 *  (nK * ((XX + YY) * (XX + YY)))) * nScale.
 *    
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.  Borderless output can be accomplished by using a
 * larger source image than the destination and adjusting oSrcSize and oSrcOffset parameters accordingly.
 *
 * @{
 *
 */

/**
 * Calculate scratch buffer size needed for the FilterHarrisCornersBorder function based on destination image SizeROI width and height.
 *
 * \param oSizeROI \ref roi_specification.
 * \param hpBufferSize Required buffer size. Important: hpBufferSize is a 
 *        <em>host pointer.</em> \ref general_scratch_buffer.
 * \return \ref image_data_error_codes, \ref roi_error_codes
 */

NppStatus 
nppiFilterHarrisCornersBorderGetBufferSize(NppiSize oSizeROI, int * hpBufferSize);

/**
 * 1 channel 8-bit unsigned grayscale to 1 channel 32-bit floating point Harris corners response image with border control.
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param pDst output edge destination_image_pointer.
 * \param nDstStep output edge destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param eFilterType selects between Sobel or Scharr filter type.
 * \param eMaskSize fixed filter mask size to use (3x3 or 5x5 for Sobel).
 * \param eAvgWindowSize fixed window mask size to use (3x3 or 5x5).
 * \param nK Harris Corners constant (commonly used value is 0.04F).
 * \param nScale output is scaled by this scale factor.
 * \param eBorderType source image border type to use use.
 * \param pDeviceBuffer pointer to scratch DEVICE memory buffer of size hpBufferSize (see nppiFilterHarrisCornersBorderGetBufferSize() above)
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 */
NppStatus 
nppiFilterHarrisCornersBorder_8u32f_C1R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                                  Npp32f * pDst, int nDstStep, NppiSize oSizeROI, NppiDifferentialKernel eFilterType,
                                            NppiMaskSize eMaskSize, NppiMaskSize eAvgWindowSize, Npp32f nK, Npp32f nScale, 
                                            NppiBorderType eBorderType, Npp8u * pDeviceBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHarrisCornersBorder_8u32f_C1R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                              Npp32f * pDst, int nDstStep, NppiSize oSizeROI, NppiDifferentialKernel eFilterType,
                                        NppiMaskSize eMaskSize, NppiMaskSize eAvgWindowSize, Npp32f nK, Npp32f nScale, 
                                        NppiBorderType eBorderType, Npp8u * pDeviceBuffer);

/** @} image_filter_harris_corners_border */

/** @defgroup image_filter_hough_line FilterHoughLine
 *  Extracts Hough lines from a single channel 8-bit binarized (0, 255) source feature (canny edges, etc.) image.
 *
 *  Outputs a list of lines in point polar format
 *  representing the length (rho) and angle (theta) of each line from the origin of the normal to the line using the formula rho = x cos(theta) + y sin(theta).
 *  The level of discretization, nDelta, is specified as an input parameter. The performance and effectiveness of this function highly depends on
 *  this parameter with higher performance for larger numbers and more detailed results for lower numbers.  Also, lines are not guaranteed to
 *  be added to the pDeviceLines list in the same order from one call to the next.  However, all of the same lines will still be generated as long as
 *  nMaxLineCount is set large enough so that they all can fit in the list. To convert lines in point polar format back to cartesian lines
 *  use the following formula:
 *  \code
 *
 *  Npp32f nHough = ((sqrt(2.0F) * static_cast<Npp32f>(oSizeROI.height > oSizeROI.width ? oSizeROI.height 
 *                                                                                      : oSizeROI.width)) / 2.0F); 
 *  int nAccumulatorsHeight = nDelta.rho > 1.0F ? static_cast<int>(ceil(nHough * 2.0F)) 
 *                                              : static_cast<int>(ceil((nHough * 2.0F) / nDelta.rho));
 *  int nCenterX = oSizeROI.width >> 1;
 *  int nCenterY = oSizeROI.height >> 1;
 *  Npp32f nThetaRad = static_cast<Npp32f>(deviceline.theta) * 0.0174532925199433F;
 *  Npp32f nSinTheta = sin(nThetaRad);
 *  Npp32f nCosTheta = cos(nThetaRad);
 *  int nX1, nY1, nX2, nY2;
 *
 *  if (deviceline.theta >= 45 && deviceline.theta <= 135) // degrees
 *  {
 *      // y = (rho - x cos(theta)) / sin(theta)
 *      nX1 = minimum cartesian X boundary value;
 *      nY1 = static_cast<int>((static_cast<Npp32f>(deviceline.rho - (nAccumulatorsHeight >> 1)) - 
 *                             ((nX1 - nCenterX) * nCosTheta)) / nSinTheta + nCenterY);
 *      nX2 = maximum cartesian X boundary value;
 *      nY2 = static_cast<int>((static_cast<Npp32f>(deviceline.rho - (nAccumulatorsHeight >> 1)) - 
 *                             ((nX2 - nCenterX) * nCosTheta)) / nSinTheta + nCenterY);
 *  }
 *  else
 *  {
 *      // x = (rho - y sin(theta)) / cos(theta)
 *      nY1 = minimum cartesian Y boundary value;
 *      nX1 = static_cast<int>((static_cast<Npp32f>(deviceline.rho - (nAccumulatorsHeight >> 1)) - 
 *                             ((nY1 - nCenterY) * nSinTheta)) / nCosTheta + nCenterX);
 *      nY2 = maximum cartesian Y boundary value;
 *      nX2 = static_cast<int>((static_cast<Npp32f>(deviceline.rho - (nAccumulatorsHeight >> 1)) - 
 *                             ((nY2 - nCenterY) * nSinTheta)) / nCosTheta + nCenterX);
 *  }
 *  \endcode
 *    
 * @{
 *
 */

/**
 * Calculate scratch buffer size needed for the FilterHoughLine or FilterHoughLineRegion functions based on destination image SizeROI width and height and nDelta parameters.
 *
 * \param oSizeROI \ref roi_specification.
 * \param nDelta rho radial increment and theta angular increment that will be used in the FilterHoughLine or FilterHoughLineRegion function call.
 * \param nMaxLineCount The maximum number of lines expected from the FilterHoughLine or FilterHoughLineRegion function call.
 * \param hpBufferSize Required buffer size in bytes. Important: hpBufferSize is a 
 *        <em>host pointer.</em> \ref general_scratch_buffer.
 * \return \ref image_data_error_codes, \ref roi_error_codes
 */

NppStatus 
nppiFilterHoughLineGetBufferSize(NppiSize oSizeROI, NppPointPolar nDelta, int nMaxLineCount, int * hpBufferSize);

/**
 * 1 channel 8-bit unsigned binarized (0, 255) source feature (canny edges, etc.) source image to list of lines in point polar format
 * representing the length (rho) and angle (theta) of each line from the origin of the normal to the line using the formula rho = x cos(theta) + y sin(theta).
 * The level of discretization, nDelta, is specified as an input parameter. The performance and effectiveness of this function highly depends on
 * this parameter with higher performance for larger numbers and more detailed results for lower numbers. nDelta must have the same values as
 * those used in the nppiFilterHoughLineGetBufferSize() function call.
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nDelta Discretization steps, range 0.0F < radial increment nDelta.rho < 3.0F, 1.0F recommended, range 0.25F < angular increment nDelta.theta < 3.0F, 1.0F recommended.
 * \param nThreshold Minimum number of points to accept a line.
 * \param pDeviceLines Device pointer to (nMaxLineCount * sizeof(NppPointPolar) line objects.
 * \param nMaxLineCount The maximum number of lines to output.
 * \param pDeviceLineCount The number of lines detected by this function up to nMaxLineCount.
 * \param pDeviceBuffer pointer to scratch DEVICE memory buffer of size hpBufferSize (see nppiFilterHoughLineGetBufferSize() above)
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 */
NppStatus 
nppiFilterHoughLine_8u32f_C1R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSizeROI, NppPointPolar nDelta, int nThreshold, 
                                        NppPointPolar * pDeviceLines, int nMaxLineCount, int * pDeviceLineCount, Npp8u * pDeviceBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHoughLine_8u32f_C1R(const Npp8u * pSrc, int nSrcStep, NppiSize oSizeROI, NppPointPolar nDelta, int nThreshold, 
                                    NppPointPolar * pDeviceLines, int nMaxLineCount, int * pDeviceLineCount, Npp8u * pDeviceBuffer);


/**
 * 1 channel 8-bit unsigned binarized (0, 255) source feature (canny edges, etc.) source image to list of lines in point polar format
 * representing the length (rho) and angle (theta) of each line from the origin of the normal to the line using the formula rho = x cos(theta) + y sin(theta).
 * The level of discretization, nDelta, is specified as an input parameter. The performance and effectiveness of this function highly depends on
 * this parameter with higher performance for larger numbers and more detailed results for lower numbers. nDelta must have the same values as
 * those used in the nppiFilterHoughLineGetBufferSize() function call. The oDstROI region limits are used to limit accepted lines to those that fall within
 * those limits.
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nDelta Discretization steps, range 0.0F < radial increment nDelta.rho < 3.0F, 1.0F recommended, range 0.25F < angular increment nDelta.theta < 3.0F, 1.0F recommended.
 * \param nThreshold Minimum number of points to accept a line.
 * \param pDeviceLines Device pointer to (nMaxLineCount * sizeof(NppPointPolar) line objects.
 * \param oDstROI Region limits with oDstROI[0].rho <= accepted rho <= oDstROI[1].rho and oDstROI[0].theta <= accepted theta <= oDstROI[1].theta.
 * \param nMaxLineCount The maximum number of lines to output.
 * \param pDeviceLineCount The number of lines detected by this function up to nMaxLineCount.
 * \param pDeviceBuffer pointer to scratch DEVICE memory buffer of size hpBufferSize (see nppiFilterHoughLineGetBufferSize() above)
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 */
NppStatus 
nppiFilterHoughLineRegion_8u32f_C1R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSizeROI, NppPointPolar nDelta, int nThreshold, 
                                              NppPointPolar * pDeviceLines, NppPointPolar oDstROI[2], int nMaxLineCount, int * pDeviceLineCount, Npp8u * pDeviceBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiFilterHoughLineRegion_8u32f_C1R(const Npp8u * pSrc, int nSrcStep, NppiSize oSizeROI, NppPointPolar nDelta, int nThreshold, 
                                          NppPointPolar * pDeviceLines, NppPointPolar oDstROI[2], int nMaxLineCount, int * pDeviceLineCount, Npp8u * pDeviceBuffer);

/** @} image_filter_hough_line */

/** @defgroup image_filter_histogram_of_oriented_gradients_border HistogramOfOrientedGradientsBorder
 * Performs Histogram Of Oriented Gradients operation on source image generating separate windows of Histogram Descriptors for each requested location.
 *
 * This function implements the simplest form of functionality described by N. Dalal and B. Triggs. Histograms of Oriented Gradients for Human Detection. INRIA, 2005.
 * It supports overlapped contrast normalized block histogram output with L2 normalization only, no threshold clipping, and no pre or post gaussian smoothing of input images or 
 * histogram output values. It supports both single channel grayscale source images and three channel color images.  For color images, the color channel with the
 * highest magnitude value is used as that pixel's magnitude. Output is row order only. 
 * Descriptors are output consecutively with no separation padding if multiple descriptor output is requested (one desriptor per source image location).
 * For example, common HOG parameters are 9 histogram bins per 8 by 8 pixel cell, 2 by 2 cells per block, 
 * with a descriptor window size of 64 horizontal by 128 vertical pixels yielding 7 by 15 overlapping blocks 
 * (1 cell overlap in both horizontal and vertical directions).  This results in 9 bins * 4 cells * 7 horizontal overlapping blocks * 15 vertical overlapping blocks or 3780 
 * 32-bit floating point output values (bins) per descriptor window. 
 * 
 * The number of horizontal overlapping block histogram bins per descriptor window width is determined by
 * (((oHOGConfig.detectionWindowSize.width / oHOGConfig.histogramBlockSize) * 2) - 1) * oHOGConfig.nHistogramBins. 
 * The number of vertical overlapping block histograms per descriptor window height is determined by 
 * (((oHOGConfig.detectionWindowSize.height / oHOGConfig.histogramBlockSize) * 2) - 1)
 * The offset of each descriptor window in the descriptors output buffer is therefore 
 * horizontal histogram bins per descriptor window width * vertical histograms per descriptor window height 32-bit floating point values 
 * relative to the previous descriptor window output.
 *
 * The algorithm uses a 1D centered derivative mask of [-1, 0, +1] when generating input magnitude and angle gradients. 
 * Magnitudes are added to the two nearest histogram bins of oriented gradients between 0 and 180 degrees using a weighted linear interpolation of each
 * magnitude value across the 2 nearest angular bin orientations. 2D overlapping blocks of histogram bins consisting of the bins from 2D arrangements of cells are
 * then contrast normalized using L2 normalization and output to the corresponding histogram descriptor window for that particular window
 * location in the window locations list.
 *
 * Some restrictions include:
 *
 * \code
 * #define NPP_HOG_MAX_CELL_SIZE                          (16)
 * #define NPP_HOG_MAX_BLOCK_SIZE                         (64)
 * #define NPP_HOG_MAX_BINS_PER_CELL                      (16)
 * #define NPP_HOG_MAX_CELLS_PER_DESCRIPTOR              (256)
 * #define NPP_HOG_MAX_OVERLAPPING_BLOCKS_PER_DESCRIPTOR (256)
 * #define NPP_HOG_MAX_DESCRIPTOR_LOCATIONS_PER_CALL     (128)
 * \endcode 
 * 
 * Currently only the NPP_BORDER_REPLICATE border type operation is supported.
 *    
 * <h3><a name="CommonFilterHistogramOfGradientsBorderParameters">Common parameters for nppiFilterHistogramOfGradientsBorder functions include:</a></h3>
 *
 * \param pSrc \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param oSrcSize Source image width and height in pixels relative to pSrc.
 * \param oSrcOffset The pixel offset that pSrc points to relative to the origin of the source image. 
 * \param hpLocations Host pointer to array of NppiPoint source pixel starting locations of requested descriptor windows. Important: hpLocations is a 
 *        <em>host pointer.</em>
 * \param nLocations Number of NppiPoint in pLocations array.
 * \param pDstWindowDescriptorBuffer Output device memory buffer pointer of size hpDescriptorsSize bytes to first of nLoc descriptor windows (see nppiHistogramOfGradientsBorderGetDescriptorsSize() above).
 * \param oSizeROI \ref roi_specification of source image.
 * \param oHOGConfig Requested HOG configuration parameters structure.
 * \param pScratchBuffer Device memory buffer pointer of size hpBufferSize bytes to scratch memory buffer (see nppiHistogramOfGradientsBorderGetBufferSize() above).
 * \param eBorderType The border type operation to be applied at source image border boundaries.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *
 * @{
 *
 */

/**
 * Validates requested HOG configuration and calculates scratch buffer size needed for the HistogramOfGradientsBorder function 
 * based on requested HOG configuration, source image ROI, and number and locations of descriptor window locations.
 *
 * \param oHOGConfig Requested HOG configuration parameters structure.
 * \param hpLocations Host pointer to array of NppiPoint source pixel starting locations of requested descriptor windows. Important: hpLocations is a 
 *        <em>host pointer.</em>
 * \param nLocations Number of NppiPoint in pLocations array. 
 * \param oSizeROI \ref roi_specification of source image.
 * \param hpBufferSize Required buffer size in bytes. Important: hpBufferSize is a 
 *        <em>host pointer.</em> \ref general_scratch_buffer.
 * \return \ref image_data_error_codes, \ref roi_error_codes
 */

NppStatus 
nppiHistogramOfGradientsBorderGetBufferSize(const NppiHOGConfig oHOGConfig, const NppiPoint * hpLocations, int nLocations, NppiSize oSizeROI, int * hpBufferSize);

/**
 * Validates requested HOG configuration and calculates output window descriptors buffer size needed for the HistogramOfGradientsBorder function 
 * based on requested HOG configuration, and number of descriptor window locations, one descriptor window is output for each location.
 * Descriptor windows are located sequentially and contiguously in the descriptors buffer.
 *
 * The number of horizontal overlapping block histogram bins per descriptor window width is determined by
 * (((oHOGConfig.detectionWindowSize.width / oHOGConfig.histogramBlockSize) * 2) - 1) * oHOGConfig.nHistogramBins. 
 * The number of vertical overlapping block histograms per descriptor window height is determined by 
 * (((oHOGConfig.detectionWindowSize.height / oHOGConfig.histogramBlockSize) * 2) - 1)
 * The offset of each descriptor window in the descriptors output buffer is therefore 
 * horizontal histogram bins per descriptor window width * vertical histograms per descriptor window height floating point values 
 * relative to the previous descriptor window output.  
 *
 * \param oHOGConfig Requested HOG configuration parameters structure.
 * \param nLocations Number of NppiPoint in pLocations array. 
 * \param hpDescriptorsSize Required buffer size in bytes of output windows descriptors for nLocations descriptor windows. Important: hpDescriptorsSize is a 
 *        <em>host pointer.</em>
 * \return \ref image_data_error_codes, \ref roi_error_codes
 */

NppStatus 
nppiHistogramOfGradientsBorderGetDescriptorsSize(const NppiHOGConfig oHOGConfig, int nLocations, int * hpDescriptorsSize);

/**
 * 1 channel 8-bit unsigned grayscale per source image descriptor window location with source image border control 
 * to per descriptor window destination floating point histogram of gradients. Requires first calling nppiHistogramOfGradientsBorderGetBufferSize function
 * call to get required scratch (host) working buffer size and nppiHistogramOfGradientsBorderGetDescriptorsSize() function call to get
 * total size for nLocations of output histogram block descriptor windows.  
 *
 * For common parameter descriptions, see <a href="#CommonFilterHistogramOfGradientsBorderParameters">Common parameters for nppiFilterHistogramOfGradientsBorder functions</a>.
 *
 */
NppStatus 
nppiHistogramOfGradientsBorder_8u32f_C1R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                             const NppiPoint * hpLocations, int nLocations, Npp32f * pDstWindowDescriptorBuffer, 
                                             NppiSize oSizeROI, const NppiHOGConfig oHOGConfig, Npp8u * pScratchBuffer, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiHistogramOfGradientsBorder_8u32f_C1R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                         const NppiPoint * hpLocations, int nLocations, Npp32f * pDstWindowDescriptorBuffer, 
                                         NppiSize oSizeROI, const NppiHOGConfig oHOGConfig, Npp8u * pScratchBuffer, NppiBorderType eBorderType);


/**
 * 3 channel 8-bit unsigned color per source image descriptor window location with source image border control 
 * to per descriptor window destination floating point histogram of gradients. Requires first calling nppiHistogramOfGradientsBorderGetBufferSize function
 * call to get required scratch (host) working buffer size and nppiHistogramOfGradientsBorderGetDescriptorsSize() function call to get
 * total size for nLocations of output histogram block descriptor windows.  
 *
 * For common parameter descriptions, see <a href="#CommonFilterHistogramOfGradientsBorderParameters">Common parameters for nppiFilterHistogramOfGradientsBorder functions</a>.
 *
 */
NppStatus 
nppiHistogramOfGradientsBorder_8u32f_C3R_Ctx(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                             const NppiPoint * hpLocations, int nLocations, Npp32f * pDstWindowDescriptorBuffer, 
                                             NppiSize oSizeROI, const NppiHOGConfig oHOGConfig, Npp8u * pScratchBuffer, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiHistogramOfGradientsBorder_8u32f_C3R(const Npp8u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                         const NppiPoint * hpLocations, int nLocations, Npp32f * pDstWindowDescriptorBuffer, 
                                         NppiSize oSizeROI, const NppiHOGConfig oHOGConfig, Npp8u * pScratchBuffer, NppiBorderType eBorderType);

/**
 * 1 channel 16-bit unsigned grayscale per source image descriptor window location with source image border control 
 * to per descriptor window destination floating point histogram of gradients. Requires first calling nppiHistogramOfGradientsBorderGetBufferSize function
 * call to get required scratch (host) working buffer size and nppiHistogramOfGradientsBorderGetDescriptorsSize() function call to get
 * total size for nLocations of output histogram block descriptor windows.  
 *
 * For common parameter descriptions, see <a href="#CommonFilterHistogramOfGradientsBorderParameters">Common parameters for nppiFilterHistogramOfGradientsBorder functions</a>.
 *
 */
NppStatus 
nppiHistogramOfGradientsBorder_16u32f_C1R_Ctx(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                              const NppiPoint * hpLocations, int nLocations, Npp32f * pDstWindowDescriptorBuffer, 
                                              NppiSize oSizeROI, const NppiHOGConfig oHOGConfig, Npp8u * pScratchBuffer, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiHistogramOfGradientsBorder_16u32f_C1R(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                          const NppiPoint * hpLocations, int nLocations, Npp32f * pDstWindowDescriptorBuffer, 
                                          NppiSize oSizeROI, const NppiHOGConfig oHOGConfig, Npp8u * pScratchBuffer, NppiBorderType eBorderType);


/**
 * 3 channel 16-bit unsigned color per source image descriptor window location with source image border control 
 * to per descriptor window destination floating point histogram of gradients. Requires first calling nppiHistogramOfGradientsBorderGetBufferSize function
 * call to get required scratch (host) working buffer size and nppiHistogramOfGradientsBorderGetDescriptorsSize() function call to get
 * total size for nLocations of output histogram block descriptor windows.  
 *
 * For common parameter descriptions, see <a href="#CommonFilterHistogramOfGradientsBorderParameters">Common parameters for nppiFilterHistogramOfGradientsBorder functions</a>.
 *
 */
NppStatus 
nppiHistogramOfGradientsBorder_16u32f_C3R_Ctx(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                              const NppiPoint * hpLocations, int nLocations, Npp32f * pDstWindowDescriptorBuffer, 
                                              NppiSize oSizeROI, const NppiHOGConfig oHOGConfig, Npp8u * pScratchBuffer, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiHistogramOfGradientsBorder_16u32f_C3R(const Npp16u * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                          const NppiPoint * hpLocations, int nLocations, Npp32f * pDstWindowDescriptorBuffer, 
                                          NppiSize oSizeROI, const NppiHOGConfig oHOGConfig, Npp8u * pScratchBuffer, NppiBorderType eBorderType);

/**
 * 1 channel 16-bit signed grayscale per source image descriptor window location with source image border control 
 * to per descriptor window destination floating point histogram of gradients. Requires first calling nppiHistogramOfGradientsBorderGetBufferSize function
 * call to get required scratch (host) working buffer size and nppiHistogramOfGradientsBorderGetDescriptorsSize() function call to get
 * total size for nLocations of output histogram block descriptor windows.  
 *
 * For common parameter descriptions, see <a href="#CommonFilterHistogramOfGradientsBorderParameters">Common parameters for nppiFilterHistogramOfGradientsBorder functions</a>.
 *
 */
NppStatus 
nppiHistogramOfGradientsBorder_16s32f_C1R_Ctx(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                              const NppiPoint * hpLocations, int nLocations, Npp32f * pDstWindowDescriptorBuffer, 
                                              NppiSize oSizeROI, const NppiHOGConfig oHOGConfig, Npp8u * pScratchBuffer, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiHistogramOfGradientsBorder_16s32f_C1R(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                          const NppiPoint * hpLocations, int nLocations, Npp32f * pDstWindowDescriptorBuffer, 
                                          NppiSize oSizeROI, const NppiHOGConfig oHOGConfig, Npp8u * pScratchBuffer, NppiBorderType eBorderType);


/**
 * 3 channel 16-bit signed color per source image descriptor window location with source image border control 
 * to per descriptor window destination floating point histogram of gradients. Requires first calling nppiHistogramOfGradientsBorderGetBufferSize function
 * call to get required scratch (host) working buffer size and nppiHistogramOfGradientsBorderGetDescriptorsSize() function call to get
 * total size for nLocations of output histogram block descriptor windows.  
 *
 * For common parameter descriptions, see <a href="#CommonFilterHistogramOfGradientsBorderParameters">Common parameters for nppiFilterHistogramOfGradientsBorder functions</a>.
 *
 */
NppStatus 
nppiHistogramOfGradientsBorder_16s32f_C3R_Ctx(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                              const NppiPoint * hpLocations, int nLocations, Npp32f * pDstWindowDescriptorBuffer, 
                                              NppiSize oSizeROI, const NppiHOGConfig oHOGConfig, Npp8u * pScratchBuffer, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiHistogramOfGradientsBorder_16s32f_C3R(const Npp16s * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                          const NppiPoint * hpLocations, int nLocations, Npp32f * pDstWindowDescriptorBuffer, 
                                          NppiSize oSizeROI, const NppiHOGConfig oHOGConfig, Npp8u * pScratchBuffer, NppiBorderType eBorderType);

/**
 * 1 channel 32-bit floating point grayscale per source image descriptor window location with source image border control 
 * to per descriptor window destination floating point histogram of gradients. Requires first calling nppiHistogramOfGradientsBorderGetBufferSize function
 * call to get required scratch (host) working buffer size and nppiHistogramOfGradientsBorderGetDescriptorsSize() function call to get
 * total size for nLocations of output histogram block descriptor windows.  
 *
 * For common parameter descriptions, see <a href="#CommonFilterHistogramOfGradientsBorderParameters">Common parameters for nppiFilterHistogramOfGradientsBorder functions</a>.
 *
 */
NppStatus 
nppiHistogramOfGradientsBorder_32f_C1R_Ctx(const Npp32f * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                           const NppiPoint * hpLocations, int nLocations, Npp32f * pDstWindowDescriptorBuffer, 
                                           NppiSize oSizeROI, const NppiHOGConfig oHOGConfig, Npp8u * pScratchBuffer, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiHistogramOfGradientsBorder_32f_C1R(const Npp32f * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                       const NppiPoint * hpLocations, int nLocations, Npp32f * pDstWindowDescriptorBuffer, 
                                       NppiSize oSizeROI, const NppiHOGConfig oHOGConfig, Npp8u * pScratchBuffer, NppiBorderType eBorderType);


/**
 * 3 channel 32-bit floating point color per source image descriptor window location with source image border control 
 * to per descriptor window destination floating point histogram of gradients. Requires first calling nppiHistogramOfGradientsBorderGetBufferSize function
 * call to get required scratch (host) working buffer size and nppiHistogramOfGradientsBorderGetDescriptorsSize() function call to get
 * total size for nLocations of output histogram block descriptor windows.  
 *
 * For common parameter descriptions, see <a href="#CommonFilterHistogramOfGradientsBorderParameters">Common parameters for nppiFilterHistogramOfGradientsBorder functions</a>.
 *
 */
NppStatus 
nppiHistogramOfGradientsBorder_32f_C3R_Ctx(const Npp32f * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                           const NppiPoint * hpLocations, int nLocations, Npp32f * pDstWindowDescriptorBuffer, 
                                           NppiSize oSizeROI, const NppiHOGConfig oHOGConfig, Npp8u * pScratchBuffer, NppiBorderType eBorderType, NppStreamContext nppStreamCtx);

NppStatus 
nppiHistogramOfGradientsBorder_32f_C3R(const Npp32f * pSrc, int nSrcStep, NppiSize oSrcSize, NppiPoint oSrcOffset, 
                                       const NppiPoint * hpLocations, int nLocations, Npp32f * pDstWindowDescriptorBuffer, 
                                       NppiSize oSizeROI, const NppiHOGConfig oHOGConfig, Npp8u * pScratchBuffer, NppiBorderType eBorderType);

/** @} image_filter_histogram_of_oriented_gradients_border */

/** @defgroup image_filter_label_markers LabelMarkers
 * Generate image connected region label markers to be used for later image segmentation.
 *
 * @{
 *
 */

/** @name LabelMarkersGetBufferSize
 *
 * Before calling any of the LabelMarkers functions the application first needs to call the corresponding
 * LabelMarkersGetBufferSize function to determine the amount of device memory to allocate as a working buffer.  The application allocated device memory
 * is then passed as the pBuffer parameter to the corresponding LabelMarkers function. 
 *  
 * @{
 *
 */

/**
 * Calculate scratch buffer size needed for 1 channel 8-bit unsigned integer LabelMarkers function based on destination image oSizeROI width and height.
 *
 * \param oSizeROI \ref roi_specification.
 * \param hpBufferSize Required buffer size in bytes.
 *  
 */
NppStatus 
nppiLabelMarkersGetBufferSize_8u_C1R(NppiSize oSizeROI, int * hpBufferSize);

/**
 * Calculate scratch buffer size needed for 1 channel 8-bit to 1 channel 32-bit unsigned integer LabelMarkers function based on destination image oSizeROI width and height.
 *
 * \param oSizeROI \ref roi_specification.
 * \param hpBufferSize Required buffer size in bytes.
 *  
 */
NppStatus 
nppiLabelMarkersGetBufferSize_8u32u_C1R(NppiSize oSizeROI, int * hpBufferSize);

/**
 * Calculate scratch buffer size needed for 1 channel 16-bit unsigned integer LabelMarkers function based on destination image oSizeROI width and height.
 *
 * \param oSizeROI \ref roi_specification.
 * \param hpBufferSize Required buffer size in bytes.
 *  
 */
NppStatus 
nppiLabelMarkersGetBufferSize_16u_C1R(NppiSize oSizeROI, int * hpBufferSize);

/** @} label_markers_get_buffer_size */

/** @name LabelMarkers
 *
 * Generate image connected region label markers to be used for later image segmentation. A connected region is any 
 * non-zero pixel region bounded by pixel values less than or equal to nMinVal. 
 * Pixels outside the ROI are always treated as having values of 0 thus resulting in a connected region segmentation boundary in that direction. 
 * Note that while marker label IDs start at ID number 1, they are not generated in any particular order and there may 
 * be numeric gaps between sequential marker IDs. The pNumber parameter returns the value of the maximum label ID 
 * generated in the first pass.  However during convergence passes higher label ID numbers in connected regions containing
 * multiple label ID numbers will be replaced with the lowest label ID number in the region. However, connected region label IDs 
 * will be unique unless the value returned by pNumber exceeds the data type range of a destination pixel in which
 * case ID values will roll over causing some to be used on multiple connected regions.  If this occurs for 8 bit data 
 * it is recommended that you use the 8u32u version of the function for 8 bit data. Pixels not connected
 * to any connected region like those with values less than or equal to nMinVal will be assigned a marker label ID value of 0. 
 *
 * Before calling any of the LabelMarkers functions the application first needs to call the corresponding
 * LabelMarkersGetBufferSize to determine the amount of device memory to allocate as a working buffer.  The allocated device memory
 * is then passed as the pBuffer parameter to the corresponding LabelMarkers function. 
 *    
 * @{
 *
 */

/**
 * 1 channel 8-bit unsigned integer in place label markers image generation.
 * 
 * \param pSrcDst  \ref in_place_image_pointer.
 * \param nSrcDstStep \ref source_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nMinVal Pixel values less than or equal to nMinVal will be excluded as members of any connected region and given a label ID of 0.
 * \param eNorm Type of pixel connectivity test to use, nppiNormInf will use 8 way connectivity and nppiNormL1 will use 4 way connectivity. 
 * \param pNumber Pointer to host memory integer value where the maximum generated marker label ID will be returned.
 * \param pBuffer Pointer to device memory scratch buffer at least as large as value returned by the corresponding LabelMarkersGetBufferSize call.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *    
 */
NppStatus 
nppiLabelMarkers_8u_C1IR_Ctx(Npp8u * pSrcDst, int nSrcDstStep, NppiSize oSizeROI, 
                             Npp8u nMinVal, NppiNorm eNorm, int * pNumber, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiLabelMarkers_8u_C1IR(Npp8u * pSrcDst, int nSrcDstStep, NppiSize oSizeROI, 
                         Npp8u nMinVal, NppiNorm eNorm, int * pNumber, Npp8u * pBuffer);

/**
 * 1 channel 8-bit to 32-bit unsigned integer label markers image generation.
 * 
 * \param pSrc  \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst  \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nMinVal Pixel values less than or equal to nMinVal will be excluded as members of any connected region and given a label ID of 0.
 * \param eNorm Type of pixel connectivity test to use, nppiNormInf will use 8 way connectivity and nppiNormL1 will use 4 way connectivity. 
 * \param pNumber Pointer to host memory integer value where the maximum generated marker label ID will be returned.
 * \param pBuffer Pointer to device memory scratch buffer at least as large as value returned by the corresponding LabelMarkersGetBufferSize call.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *    
 */
NppStatus 
nppiLabelMarkers_8u32u_C1R_Ctx(Npp8u * pSrc, int nSrcStep, Npp32u * pDst, int nDstStep, NppiSize oSizeROI, 
                               Npp8u nMinVal, NppiNorm eNorm, int * pNumber, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiLabelMarkers_8u32u_C1R(Npp8u * pSrc, int nSrcStep, Npp32u * pDst, int nDstStep, NppiSize oSizeROI, 
                           Npp8u nMinVal, NppiNorm eNorm, int * pNumber, Npp8u * pBuffer);

/**
 * 1 channel 16-bit unsigned integer in place label markers image generation.
 * 
 * \param pSrcDst  \ref in_place_image_pointer.
 * \param nSrcDstStep \ref source_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nMinVal Pixel values less than or equal to nMinVal will be excluded as members of any connected region and given a label ID of 0.
 * \param eNorm Type of pixel connectivity test to use, nppiNormInf will use 8 way connectivity and nppiNormL1 will use 4 way connectivity. 
 * \param pNumber Pointer to host memory integer value where the maximum generated marker label ID will be returned.
 * \param pBuffer Pointer to device memory scratch buffer at least as large as value returned by the corresponding LabelMarkersGetBufferSize call.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *    
 */
NppStatus 
nppiLabelMarkers_16u_C1IR_Ctx(Npp16u * pSrcDst, int nSrcDstStep, NppiSize oSizeROI, 
                              Npp16u nMinVal, NppiNorm eNorm, int * pNumber, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiLabelMarkers_16u_C1IR(Npp16u * pSrcDst, int nSrcDstStep, NppiSize oSizeROI, 
                          Npp16u nMinVal, NppiNorm eNorm, int * pNumber, Npp8u * pBuffer);

/** @} label_markers */

/** @name LabelMarkersUFGetBufferSize
 *
 * Before calling any of the LabelMarkersUF functions the application first needs to call the
 * LabelMarkersGetBufferSize function to determine the amount of device memory to allocate as a working buffer.  The application allocated device memory
 * is then passed as the pBuffer parameter to the corresponding LabelMarkersUF function.
 *
 * @{
 *
 */

/**
 * Calculate scratch buffer size needed 1 channel 32-bit unsigned integer LabelMarkersUF function based on destination image oSizeROI width and height.
 *
 * \param oSizeROI \ref roi_specification.
 * \param hpBufferSize Required buffer size in bytes.
 */
NppStatus 
nppiLabelMarkersUFGetBufferSize_32u_C1R(NppiSize oSizeROI, int * hpBufferSize);

/** @} label_markers_uf_get_buffer_size */

/** @name LabelMarkersUF
 *
 * Generate image connected region label markers to be used for later image segmentation. 
 *  
 * A connected region is any pixel region where all pixels in the region have the same pixel value. 
 * Note that marker label IDs generally increase in value from image left to right and top to bottom they are not generated in any particular order and there may 
 * be numeric gaps between sequential marker IDs.  To limit the number of marker IDs generated the application should pass the image 
 * through a threshold filter before calling this funcion.  Doing so however does not necessarily limit the maximum marker ID value generated by this function. 
 * Note that this function currently only supports image ROI sizes up to 4 gigapixels. 
 *  
 * Before calling any of the LabelMarkersUF functions the application first needs to call the
 * LabelMarkersUFGetBufferSize to determine the amount of device memory to allocate as a working buffer.  The allocated device memory
 * is then passed as the pBuffer parameter to the corresponding LabelMarkersUF function. 
 *  
 * The algorithm used in this implementation is based on the one described in "An Optimized Union-Find Algorithm for Connected Components Labeling Using GPUs" by Jun Chen and others. 
 *  
 *
 * @{
 *
 */

/**
 * 1 channel 8-bit to 32-bit unsigned integer label markers image generation.
 * 
 * \param pSrc  \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst  \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param eNorm Type of pixel connectivity test to use, nppiNormInf will use 8 way connectivity and nppiNormL1 will use 4 way connectivity. 
 * \param pBuffer Pointer to device memory scratch buffer at least as large as value returned by the corresponding LabelMarkersUFGetBufferSize call.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 */
NppStatus 
nppiLabelMarkersUF_8u32u_C1R_Ctx(Npp8u * pSrc, int nSrcStep, Npp32u * pDst, int nDstStep, NppiSize oSizeROI, NppiNorm eNorm, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiLabelMarkersUF_8u32u_C1R(Npp8u * pSrc, int nSrcStep, Npp32u * pDst, int nDstStep, NppiSize oSizeROI, NppiNorm eNorm, Npp8u * pBuffer);

/**
 * 1 channel 16-bit to 32-bit unsigned integer label markers image generation.
 * 
 * \param pSrc  \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst  \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param eNorm Type of pixel connectivity test to use, nppiNormInf will use 8 way connectivity and nppiNormL1 will use 4 way connectivity. 
 * \param pBuffer Pointer to device memory scratch buffer at least as large as value returned by the corresponding LabelMarkersUFGetBufferSize call.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 */
NppStatus 
nppiLabelMarkersUF_16u32u_C1R_Ctx(Npp16u * pSrc, int nSrcStep, Npp32u * pDst, int nDstStep, NppiSize oSizeROI, NppiNorm eNorm, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiLabelMarkersUF_16u32u_C1R(Npp16u * pSrc, int nSrcStep, Npp32u * pDst, int nDstStep, NppiSize oSizeROI, NppiNorm eNorm, Npp8u * pBuffer);

/**
 * 1 channel 32-bit to 32-bit unsigned integer label markers image generation.
 * 
 * \param pSrc  \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst  \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param eNorm Type of pixel connectivity test to use, nppiNormInf will use 8 way connectivity and nppiNormL1 will use 4 way connectivity. 
 * \param pBuffer Pointer to device memory scratch buffer at least as large as value returned by the corresponding LabelMarkersUFGetBufferSize call.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 */
NppStatus 
nppiLabelMarkersUF_32u_C1R_Ctx(Npp32u * pSrc, int nSrcStep, Npp32u * pDst, int nDstStep, NppiSize oSizeROI, NppiNorm eNorm, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiLabelMarkersUF_32u_C1R(Npp32u * pSrc, int nSrcStep, Npp32u * pDst, int nDstStep, NppiSize oSizeROI, NppiNorm eNorm, Npp8u * pBuffer);

/** @} label_markers_uf */

/** @} image_filter_label_markers */

/** @defgroup image_filter_compress_marker_labels CompressMarkerLabels
 * Removes sparseness between marker label IDs output from LabelMarkers call.
 *    
 * @{
 *
 */

/** @name CompressMarkerLabelsGetBufferSize
 *
 * Before calling any of the CompressMarkerLabels functions the application first needs to call the corresponding
 * CompressMarkerLabelsGetBufferSize function to determine the amount of device memory to allocate as a working buffer.  
 * The application allocated device memory is then passed as the pBuffer parameter to the corresponding CompressMarkerLabels function. 
 *  
 * NOTE: When compressing labels generated by the nppiLabelMarkersUF() functions the value of the nStartingNumber parameter below MUST 
 *       be set to ROI width * ROI height. 
 *    
 * @{
 *
 */

/**
 * Calculate scratch buffer size needed for 1 channel 8-bit unsigned integer CompressMarkerLabels function based on the number returned in pNumber from a previous nppiLabelMarkers call.
 *
 * \param nStartingNumber The value returned from a previous call to the nppiLabelMarkers_8u function.
 * \param hpBufferSize Required buffer size in bytes.
 *    
 */
NppStatus 
nppiCompressMarkerLabelsGetBufferSize_8u_C1R(int nStartingNumber, int * hpBufferSize);

/**
 * Calculate scratch buffer size needed for 1 channel 32-bit unsigned integer to 8-bit unsigned integer CompressMarkerLabels function based on the number returned in pNumber from a previous nppiLabelMarkers call.
 *
 * \param nStartingNumber The value returned from a previous call to the nppiLabelMarkers_8u32u function or ROI width * ROI height for images generated by the nppiLabelMarkersUF funcions.
 * \param hpBufferSize Required buffer size in bytes.
 *    
 */
NppStatus 
nppiCompressMarkerLabelsGetBufferSize_32u8u_C1R(int nStartingNumber, int * hpBufferSize);

/**
 * Calculate scratch buffer size needed for 1 channel 16-bit unsigned integer CompressMarkerLabels function based on the number returned in pNumber from a previous nppiLabelMarkers call.
 *
 * \param nStartingNumber The value returned from a previous call to the nppiLabelMarkers_16u function.
 * \param hpBufferSize Required buffer size in bytes.
 *    
 */
NppStatus 
nppiCompressMarkerLabelsGetBufferSize_16u_C1R(int nStartingNumber, int * hpBufferSize);

/**
 * Calculate scratch buffer size needed for 1 channel 32-bit unsigned integer to 16-bit unsigned integer CompressMarkerLabels function based on the number returned in pNumber from a previous nppiLabelMarkers call.
 *
 * \param nStartingNumber The value returned from a previous call to the nppiLabelMarkers_8u32u function or ROI width * ROI height for images generated by the nppiLabelMarkersUF funcions.
 * \param hpBufferSize Required buffer size in bytes.
 *    
 */
NppStatus 
nppiCompressMarkerLabelsGetBufferSize_32u16u_C1R(int nStartingNumber, int * hpBufferSize);

/**
 * Calculate scratch buffer size needed for 1 channel 32-bit unsigned integer CompressMarkerLabels function based on the number returned in pNumber from a previous nppiLabelMarkers call.
 *
 * \param nStartingNumber The value returned from a previous call to the nppiLabelMarkers_32u function or ROI width * ROI height for images generated by the nppiLabelMarkersUF funcions.
 * \param hpBufferSize Required buffer size in bytes.
 *    
 */
NppStatus 
nppiCompressMarkerLabelsGetBufferSize_32u_C1R(int nStartingNumber, int * hpBufferSize);

/** @} compress_marker_labels_get_buffer_size */

/** @name CompressMarkerLabels
 *
 * Renumber connected region marker label IDs from a previous call to nppiLabelMarkers to eliminate label numbering sparseness.
 * Note that while marker label IDs still start at ID number 1, the value of pNewNumber returned by this function will represent
 * the minimum number of label IDs to give each connected region in the image a unique label ID. However, if you initially used
 * nppiLabelMarkers_8u and the pNumber value returned by that function was over 255 then you will still have one or more disjoint
 * connected regions with the same label ID in the final output from this function. Also, the output of nppiCompressMarkerLabels_32u8u
 * will only produce correct results if the pNewNumber value returned by this function is less than 256.
 *
 * Before calling any of the CompressMarkerLabels functions the application first needs to call the corresponding
 * CompressMarkerLabelsGetBufferSize to determine the amount of device memory to allocate as a working buffer.  The allocated device memory
 * is then passed as the pBuffer parameter to the corresponding CompressMarkerLabels function.
 *    
 * NOTE: When compressing labels generated by the nppiLabelMarkersUF() functions the value of the nStartingNumber parameter below MUST 
 *       be set to ROI width * ROI height. 
 *  
 * @{
 *
 */

/**
 * 1 channel 8-bit unsigned integer in place connected region marker label renumbering with numbering sparseness elimination.
 * 
 * \param pSrcDst  \ref in_place_image_pointer.
 * \param nSrcDstStep \ref source_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nStartingNumber The value returned from a previous call to the nppiLabelMarkers_8u function.
 * \param pNewNumber Pointer to host memory integer value where the maximum renumbered marker label ID will be returned.
 * \param pBuffer Pointer to device memory scratch buffer at least as large as value returned by the corresponding CompressMarkerLabelsGetBufferSize call.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *    
 */
NppStatus 
nppiCompressMarkerLabels_8u_C1IR_Ctx(Npp8u * pSrcDst, int nSrcDstStep, NppiSize oSizeROI, int nStartingNumber, int * pNewNumber, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiCompressMarkerLabels_8u_C1IR(Npp8u * pSrcDst, int nSrcDstStep, NppiSize oSizeROI, int nStartingNumber, int * pNewNumber, Npp8u * pBuffer);

/**
 * 1 channel 32-bit unsigned integer to 8-bit unsigned integer connected region marker label renumbering with numbering sparseness elimination.
 * 
 * \param pSrc  \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst  \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nStartingNumber The value returned from a previous call to the nppiLabelMarkers_8u32u function or ROI width * ROI height for images generated by the nppiLabelMarkersUF funcions.
 * \param pNewNumber Pointer to host memory integer value where the maximum renumbered marker label ID will be returned.
 * \param pBuffer Pointer to device memory scratch buffer at least as large as value returned by the corresponding CompressMarkerLabelsGetBufferSize call.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *    
 */
NppStatus 
nppiCompressMarkerLabels_32u8u_C1R_Ctx(Npp32u * pSrc, int nSrcStep, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, int nStartingNumber, int * pNewNumber, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiCompressMarkerLabels_32u8u_C1R(Npp32u * pSrc, int nSrcStep, Npp8u * pDst, int nDstStep, NppiSize oSizeROI, int nStartingNumber, int * pNewNumber, Npp8u * pBuffer);

/**
 * 1 channel 16-bit unsigned integer in place connected region marker label renumbering with numbering sparseness elimination.
 * 
 * \param pSrcDst  \ref in_place_image_pointer.
 * \param nSrcDstStep \ref source_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nStartingNumber The value returned from a previous call to the nppiLabelMarkers_16u function.
 * \param pNewNumber Pointer to host memory integer value where the maximum renumbered marker label ID will be returned.
 * \param pBuffer Pointer to device memory scratch buffer at least as large as value returned by the corresponding CompressMarkerLabelsGetBufferSize call.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *    
 */
NppStatus 
nppiCompressMarkerLabels_16u_C1IR_Ctx(Npp16u * pSrcDst, int nSrcDstStep, NppiSize oSizeROI, int nStartingNumber, int * pNewNumber, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiCompressMarkerLabels_16u_C1IR(Npp16u * pSrcDst, int nSrcDstStep, NppiSize oSizeROI, int nStartingNumber, int * pNewNumber, Npp8u * pBuffer);

/**
 * 1 channel 32-bit unsigned integer to 16-bit unsigned integer connected region marker label renumbering with numbering sparseness elimination.
 * 
 * \param pSrc  \ref source_image_pointer.
 * \param nSrcStep \ref source_image_line_step.
 * \param pDst  \ref destination_image_pointer.
 * \param nDstStep \ref destination_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nStartingNumber The value returned from a previous call to the nppiLabelMarkers_32u function or ROI width * ROI height for images generated by the nppiLabelMarkersUF funcions.
 * \param pNewNumber Pointer to host memory integer value where the maximum renumbered marker label ID will be returned.
 * \param pBuffer Pointer to device memory scratch buffer at least as large as value returned by the corresponding CompressMarkerLabelsGetBufferSize call.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *    
 */
NppStatus 
nppiCompressMarkerLabels_32u16u_C1R_Ctx(Npp32u * pSrc, int nSrcStep, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, int nStartingNumber, int * pNewNumber, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiCompressMarkerLabels_32u16u_C1R(Npp32u * pSrc, int nSrcStep, Npp16u * pDst, int nDstStep, NppiSize oSizeROI, int nStartingNumber, int * pNewNumber, Npp8u * pBuffer);

/**
 * 1 channel 32-bit unsigned integer in place connected region marker label renumbering with numbering sparseness elimination.
 * 
 * \param pSrcDst  \ref in_place_image_pointer.
 * \param nSrcDstStep \ref source_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nStartingNumber The value returned from a previous call to the nppiLabelMarkers_8u32u function or ROI width * ROI height for images generated by the nppiLabelMarkersUF funcions.
 * \param pNewNumber Pointer to host memory integer value where the maximum renumbered marker label ID will be returned.
 * \param pBuffer Pointer to device memory scratch buffer at least as large as value returned by the corresponding CompressMarkerLabelsGetBufferSize call.
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 *    
 */
NppStatus 
nppiCompressMarkerLabels_32u_C1IR_Ctx(Npp32u * pSrcDst, int nSrcDstStep, NppiSize oSizeROI, int nStartingNumber, int * pNewNumber, Npp8u * pBuffer, NppStreamContext nppStreamCtx);

NppStatus 
nppiCompressMarkerLabels_32u_C1IR(Npp32u * pSrcDst, int nSrcDstStep, NppiSize oSizeROI, int nStartingNumber, int * pNewNumber, Npp8u * pBuffer);

/** @} compress_marker_labels */

/** @} image_filter_compress_marker_labels */

/** @defgroup image_filter_bound_segments BoundSegments
 * Adds boundary borders around connected regions using a border value of nBorderVal.
 *
 * While this function is intended
 * to be used on images output from nppiLabelMarkers function calls it will work on any image which contains regions
 * surrounded by pixel values of 0.  This function always uses an 8-way connectivity search.
 *
 * @{
 *
 */

/**
 * 1 channel 8-bit unsigned integer in place region boundary border image generation.
 * 
 * \param pSrcDst  \ref in_place_image_pointer.
 * \param nSrcDstStep \ref source_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nBorderVal Pixel value to be used at connected region boundary borders
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 */
NppStatus 
nppiBoundSegments_8u_C1IR_Ctx(Npp8u * pSrcDst, int nSrcDstStep, NppiSize oSizeROI, Npp8u nBorderVal, NppStreamContext nppStreamCtx);

NppStatus 
nppiBoundSegments_8u_C1IR(Npp8u * pSrcDst, int nSrcDstStep, NppiSize oSizeROI, Npp8u nBorderVal);

/**
 * 1 channel 16-bit unsigned integer in place region boundary border image generation.
 * 
 * \param pSrcDst  \ref in_place_image_pointer.
 * \param nSrcDstStep \ref source_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nBorderVal Pixel value to be used at connected region boundary borders
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 */
NppStatus 
nppiBoundSegments_16u_C1IR_Ctx(Npp16u * pSrcDst, int nSrcDstStep, NppiSize oSizeROI, Npp16u nBorderVal, NppStreamContext nppStreamCtx);

NppStatus 
nppiBoundSegments_16u_C1IR(Npp16u * pSrcDst, int nSrcDstStep, NppiSize oSizeROI, Npp16u nBorderVal);

/**
 * 1 channel 32-bit unsigned integer in place region boundary border image generation.
 * 
 * \param pSrcDst  \ref in_place_image_pointer.
 * \param nSrcDstStep \ref source_image_line_step.
 * \param oSizeROI \ref roi_specification.
 * \param nBorderVal Pixel value to be used at connected region boundary borders
 * \param nppStreamCtx \ref application_managed_stream_context. 
 * \return \ref image_data_error_codes, \ref roi_error_codes
 */
NppStatus 
nppiBoundSegments_32u_C1IR_Ctx(Npp32u * pSrcDst, int nSrcDstStep, NppiSize oSizeROI, Npp32u nBorderVal, NppStreamContext nppStreamCtx);

NppStatus 
nppiBoundSegments_32u_C1IR(Npp32u * pSrcDst, int nSrcDstStep, NppiSize oSizeROI, Npp32u nBorderVal);

/** @} image_filter_bound_segments */

/** @} image_computer_vision_filtering_functions */

/** @} image_filtering_functions */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* NV_NPPI_FILTERING_FUNCTIONS_H */
