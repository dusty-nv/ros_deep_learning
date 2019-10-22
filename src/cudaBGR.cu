#include "cudaBGR.h"

//-------------------------------------------------------------------------------------------------------------------------

template<bool isBGR>
__global__ void RGBAToRGBAf(uchar4* srcImage,
                           float4* dstImage,
                           int width, int height)
{
	const int x = (blockIdx.x * blockDim.x) + threadIdx.x;
	const int y = (blockIdx.y * blockDim.y) + threadIdx.y;
	
	const int pixel = y * width + x;

	if( x >= width )
		return; 

	if( y >= height )
		return;

//	printf("cuda thread %i %i  %i %i pixel %i \n", x, y, width, height, pixel);
		
	const float  s  = 1.0f;
	const uchar4 px = srcImage[pixel];
	
	if( isBGR )
		dstImage[pixel] = make_float4(px.z * s, px.y * s, px.x * s, px.w * s);
	else
		dstImage[pixel] = make_float4(px.x * s, px.y * s, px.z * s, px.w * s);
}

/**
 * Convert 8-bit fixed-point BGRA image to 32-bit floating-point RGBA image
 * @ingroup util
 */
cudaError_t cudaBGRA8ToRGBA32( uchar4* srcDev, float4* destDev, size_t width, size_t height )
{
	if( !srcDev || !destDev )
		return cudaErrorInvalidDevicePointer;

	const dim3 blockDim(8,8,1);
	const dim3 gridDim(iDivUp(width,blockDim.x), iDivUp(height,blockDim.y), 1);

	RGBAToRGBAf<true><<<gridDim, blockDim>>>( srcDev, destDev, width, height );
	
	return CUDA(cudaGetLastError());
}

