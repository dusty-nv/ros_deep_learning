/*
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "image_converter.h"

#include <jetson-utils/cudaRGB.h>
#include <jetson-utils/cudaMappedMemory.h>

#include <ros/ros.h>


// constructor
imageConverter::imageConverter()
{
	mWidth  = 0;
	mHeight = 0;
	mSize   = 0;

	mInputCPU = NULL;
	mInputGPU = NULL;

	mOutputCPU = NULL;
	mOutputGPU = NULL;
}


// destructor
imageConverter::~imageConverter()
{

}


// Convert
bool imageConverter::Convert( const sensor_msgs::ImageConstPtr& input )
{
	ROS_INFO("converting %ux%u %s image", input->width, input->height, input->encoding.c_str());

	// confirm bgr8 encoding
	if( input->encoding.compare("bgr8") != 0 )
	{
		ROS_ERROR("%ux%u image is in %s format, expected bgr8", input->width, input->height, input->encoding.c_str());
		return false;
	}

	// confirm step size
	const uint32_t input_stride = input->width * sizeof(uint8_t) * 3;

	if( input->step != input_stride )
	{
		ROS_ERROR("%ux%u image has step size of %u bytes, expected %u bytes", input->width, input->height, input->step, input_stride);
		return false;
	}

	// assure memory allocation
	const size_t input_size  = input->width * input->height * sizeof(uint8_t) * 3;
	const size_t output_size = input->width * input->height * sizeof(float) * 4;

	if( mWidth != input->width || mHeight != input->height )
	{
		if( mInputCPU != NULL )
			CUDA(cudaFreeHost(mInputCPU));

		if( mOutputCPU != NULL )
			CUDA(cudaFreeHost(mOutputCPU));

		if( !cudaAllocMapped((void**)&mInputCPU, (void**)&mInputGPU, input_size) ||
		    !cudaAllocMapped((void**)&mOutputCPU, (void**)&mOutputGPU, output_size) )
		{
			ROS_ERROR("failed to allocate memory for %ux%u image conversion", input->width, input->height);
			return false;
		}

		ROS_INFO("allocated CUDA memory for %ux%u image conversion", input->width, input->height);

		mWidth  = input->width;
		mHeight = input->height;
		mSize   = output_size;
	}
	
	// copy input to shared memory
	memcpy(mInputCPU, input->data.data(), input_size);			
	
	// convert to RGBA32f format
	if( CUDA_FAILED(cudaBGR8ToRGBA32((uchar3*)mInputGPU, (float4*)mOutputGPU, mWidth, mHeight)) )
	{
		ROS_ERROR("failed to convert %ux%u image with CUDA", mWidth, mHeight);
		return false;
	}

	return true;
}



