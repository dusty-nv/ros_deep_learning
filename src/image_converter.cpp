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
#include <sensor_msgs/image_encodings.h>


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
	if( input->encoding != sensor_msgs::image_encodings::BGR8 )
	{
		ROS_ERROR("%ux%u image is in %s format, expected %s", input->width, input->height, input->encoding.c_str(), sensor_msgs::image_encodings::BGR8.c_str());
		return false;
	}

	// confirm step size
	const uint32_t input_stride = input->width * sizeof(uchar3);

	if( input->step != input_stride )
	{
		ROS_ERROR("%ux%u image has step size of %u bytes, expected %u bytes", input->width, input->height, input->step, input_stride);
		return false;
	}

	// assure memory allocation
	if( !Resize(input->width, input->height) )
		return false;
	
	// copy input to shared memory
	memcpy(mInputCPU, input->data.data(), input->width * input->height * sizeof(uchar3));	// note: 3 channels assumes bgr/rgb			
	
	// convert to RGBA32f format
	if( CUDA_FAILED(cudaBGR8ToRGBA32((uchar3*)mInputGPU, (float4*)mOutputGPU, mWidth, mHeight)) )
	{
		ROS_ERROR("failed to convert %ux%u image with CUDA", mWidth, mHeight);
		return false;
	}

	return true;
}


// Convert
bool imageConverter::Convert( sensor_msgs::Image& msg, const std::string& encoding )
{
	if( !mInputCPU || !mOutputCPU || mWidth == 0 || mHeight == 0 || mSize == 0 )
		return false;
	
	size_t px_depth = 0;	 // pixel depth (in bytes) determined below
	bool   in_place = false;	 // if no conversion required

	// perform colorspace conversion into the desired encoding
	// in this direction, we reverse use of input/output pointers
	if( encoding == sensor_msgs::image_encodings::BGR8 )
	{
		if( CUDA_FAILED(cudaRGBA32ToBGR8((float4*)mOutputGPU, (uchar3*)mInputGPU, mWidth, mHeight)) )
		{
			ROS_ERROR("failed to convert %ux%u RGBA32 image to BGR8 with CUDA", mWidth, mHeight);
			return false;
		}

		px_depth = sizeof(uchar3);
	}
	else if( encoding == sensor_msgs::image_encodings::RGB8 )
	{
		if( CUDA_FAILED(cudaRGBA32ToRGB8((float4*)mOutputGPU, (uchar3*)mInputGPU, mWidth, mHeight)) )
		{
			ROS_ERROR("failed to convert %ux%u RGBA32 image to RGB8 with CUDA", mWidth, mHeight);
			return false;
		}

		px_depth = sizeof(uchar3);
	}
	else if( encoding == sensor_msgs::image_encodings::BGRA8 )
	{
		if( CUDA_FAILED(cudaRGBA32ToBGRA8((float4*)mOutputGPU, (uchar4*)mInputGPU, mWidth, mHeight)) )
		{
			ROS_ERROR("failed to convert %ux%u RGBA32 image to BGRA8 with CUDA", mWidth, mHeight);
			return false;
		}

		px_depth = sizeof(uchar4);
	}
	else if( encoding == sensor_msgs::image_encodings::RGBA8 )
	{
		if( CUDA_FAILED(cudaRGBA32ToRGBA8((float4*)mOutputGPU, (uchar4*)mInputGPU, mWidth, mHeight)) )
		{
			ROS_ERROR("failed to convert %ux%u RGBA32 image to RGBA8 with CUDA", mWidth, mHeight);
			return false;
		}

		px_depth = sizeof(uchar4);
	}
	else if( encoding == sensor_msgs::image_encodings::MONO8 )
	{
		px_depth = sizeof(uint8_t);	
		in_place = true;	
	}
	else
	{
		ROS_ERROR("image_converter -- unsupported format requested '%s'", encoding.c_str());
	}

	// calculate size of the msg
	const size_t msg_size = mWidth * mHeight * px_depth;

	// allocate msg storage
	msg.data.resize(msg_size);

	// copy the converted image into the msg
	memcpy(msg.data.data(), in_place ? (void*)mOutputCPU : (void*)mInputCPU, msg_size);

	// populate metadata
	msg.width  = mWidth;
	msg.height = mHeight;
	msg.step   = mWidth * px_depth;

	msg.encoding     = encoding;
	msg.is_bigendian = false;
	
	return true;
}


// Resize
bool imageConverter::Resize( uint32_t width, uint32_t height )
{
	const size_t input_size  = width * height * sizeof(uchar4);	 // 4 channels in case of bgra/rgba message conversion
	const size_t output_size = width * height * sizeof(float4);	 // assumes float4 output (user might use less)

	if( mWidth != width || mHeight != height )
	{
		if( mInputCPU != NULL )
			CUDA(cudaFreeHost(mInputCPU));

		if( mOutputCPU != NULL )
			CUDA(cudaFreeHost(mOutputCPU));

		if( !cudaAllocMapped((void**)&mInputCPU, (void**)&mInputGPU, input_size) ||
		    !cudaAllocMapped((void**)&mOutputCPU, (void**)&mOutputGPU, output_size) )
		{
			ROS_ERROR("failed to allocate memory for %ux%u image conversion", width, height);
			return false;
		}

		ROS_INFO("allocated CUDA memory for %ux%u image conversion", width, height);

		mWidth  = width;
		mHeight = height;
		mSize   = output_size;
	}

	return true;
}


