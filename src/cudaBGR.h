#ifndef __CUDA_BGR_CONVERT_H
#define __CUDA_BGR_CONVERT_H


#include "cudaUtility.h"
#include <stdint.h>


/**
 * Convert 8-bit fixed-point BGR image to 32-bit floating-point RGBA image
 * @ingroup util
 */
cudaError_t cudaBGRA8ToRGBA32( uchar4* input, float4* output, size_t width, size_t height );

#endif
