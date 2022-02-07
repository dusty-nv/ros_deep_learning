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

#include <jetson-utils/cudaColorspace.h>
#include <jetson-utils/cudaMappedMemory.h>



static imageFormat imageFormatFromEncoding( const std::string& encoding )
{
	if( encoding == sensor_msgs::image_encodings::BGR8 )
		return IMAGE_BGR8;
	else if( encoding == sensor_msgs::image_encodings::BGRA8 )
		return IMAGE_BGRA8;
	else if( encoding == sensor_msgs::image_encodings::RGB8 )
		return IMAGE_RGB8;
	else if( encoding == sensor_msgs::image_encodings::RGBA8 )
		return IMAGE_RGBA8;
	else if( encoding == sensor_msgs::image_encodings::MONO8 )
		return IMAGE_GRAY8;
	else if( encoding == sensor_msgs::image_encodings::YUV422 )
		return IMAGE_UYVY;
	else if( encoding == sensor_msgs::image_encodings::BAYER_RGGB8 )
		return IMAGE_BAYER_RGGB;
	else if( encoding == sensor_msgs::image_encodings::BAYER_BGGR8 )
		return IMAGE_BAYER_BGGR;
	else if( encoding == sensor_msgs::image_encodings::BAYER_GBRG8 )
		return IMAGE_BAYER_GBRG;
	else if( encoding == sensor_msgs::image_encodings::BAYER_GRBG8 )
		return IMAGE_BAYER_GRBG;

	return IMAGE_UNKNOWN;
}

static std::string imageFormatToEncoding( imageFormat fmt )
{
	switch(fmt)
	{
		case IMAGE_BGR8:		return sensor_msgs::image_encodings::BGR8;
		case IMAGE_BGRA8:		return sensor_msgs::image_encodings::BGRA8;
		case IMAGE_RGB8:		return sensor_msgs::image_encodings::RGB8;
		case IMAGE_RGBA8:		return sensor_msgs::image_encodings::RGBA8;
		case IMAGE_GRAY8:		return sensor_msgs::image_encodings::MONO8;
		case IMAGE_UYVY:		return sensor_msgs::image_encodings::YUV422;
		case IMAGE_BAYER_RGGB:	return sensor_msgs::image_encodings::BAYER_RGGB8;
		case IMAGE_BAYER_BGGR:	return sensor_msgs::image_encodings::BAYER_BGGR8;
		case IMAGE_BAYER_GBRG:	return sensor_msgs::image_encodings::BAYER_GBRG8;
		case IMAGE_BAYER_GRBG:	return sensor_msgs::image_encodings::BAYER_GRBG8;
	}

	return "invalid";
}


// constructor
imageConverter::imageConverter()
{
	mWidth  	  = 0;
	mHeight 	  = 0;
	mSizeInput  = 0;
	mSizeOutput = 0;

	mInputCPU = NULL;
	mInputGPU = NULL;

	mOutputCPU = NULL;
	mOutputGPU = NULL;
}


// destructor
imageConverter::~imageConverter()
{
	Free();	
}


// Free
void imageConverter::Free()
{
	if( mInputCPU != NULL )
	{
		CUDA(cudaFreeHost(mInputCPU));

		mInputCPU = NULL;
		mInputGPU = NULL;
	}

	if( mOutputCPU != NULL )
	{
		CUDA(cudaFreeHost(mOutputCPU));

		mOutputCPU = NULL;
		mOutputGPU = NULL;
	}
}

// Convert
bool imageConverter::Convert( const sensor_msgs::ImageConstPtr& input )
{
	ROS_DEBUG("converting %ux%u %s image", input->width, input->height, input->encoding.c_str());

	// parse the input format
	const imageFormat input_format = imageFormatFromEncoding(input->encoding);

	if( input_format == IMAGE_UNKNOWN )
	{
		ROS_ERROR("image encoding %s is not a compatible format to use with ros_deep_learning", input->encoding.c_str());
		return false;
	}

	// assure memory allocation
	if( !Resize(input->width, input->height, input_format) )
		return false;
	
	// copy input to shared memory
	memcpy(mInputCPU, input->data.data(), imageFormatSize(input_format, input->width, input->height));			
	
	// convert image format
	if( CUDA_FAILED(cudaConvertColor(mInputGPU, input_format, mOutputGPU, InternalFormat, input->width, input->height)) )
	{
		ROS_ERROR("failed to convert %ux%u image (from %s to %s) with CUDA", mWidth, mHeight, imageFormatToStr(input_format), imageFormatToStr(InternalFormat));
		return false;
	}

	return true;
}


// Convert
bool imageConverter::Convert( sensor_msgs::Image& msg, imageFormat format )
{
	return Convert(msg, format, ImageGPU());
}


// Convert
bool imageConverter::Convert( sensor_msgs::Image& msg, imageFormat format, PixelType* imageGPU )
{
	if( !mInputCPU || !imageGPU || mWidth == 0 || mHeight == 0 || mSizeInput == 0 || mSizeOutput == 0 )
		return false;
	
	// perform colorspace conversion into the desired encoding
	// in this direction, we reverse use of input/output pointers
	if( CUDA_FAILED(cudaConvertColor(imageGPU, InternalFormat, mInputGPU, format, mWidth, mHeight)) )
	{
		ROS_ERROR("failed to convert %ux%u image (from %s to %s) with CUDA", mWidth, mHeight, imageFormatToStr(InternalFormat), imageFormatToStr(format));
		return false;
	}

	// calculate size of the msg
	const size_t msg_size = imageFormatSize(format, mWidth, mHeight);

	// allocate msg storage
	msg.data.resize(msg_size);

	// copy the converted image into the msg
	if (format != IMAGE_GRAY8)
	{
		memcpy(msg.data.data(), mInputCPU, msg_size);
	}
	else 
	{
		memcpy(msg.data.data(), mOutputCPU, msg_size);
	}
		
	// populate metadata
	msg.width  = mWidth;
	msg.height = mHeight;
	msg.step   = (mWidth * imageFormatDepth(format)) / 8;

	msg.encoding     = imageFormatToEncoding(format);
	msg.is_bigendian = false;
	
	return true;
}


// Resize
bool imageConverter::Resize( uint32_t width, uint32_t height, imageFormat inputFormat )
{
	const size_t input_size  = imageFormatSize(inputFormat, width, height);
	const size_t output_size = imageFormatSize(InternalFormat, width, height);

	if( input_size != mSizeInput || output_size != mSizeOutput || mWidth != width || mHeight != height )
	{
		Free();

		if( !cudaAllocMapped((void**)&mInputCPU, (void**)&mInputGPU, input_size) ||
		    !cudaAllocMapped((void**)&mOutputCPU, (void**)&mOutputGPU, output_size) )
		{
			ROS_ERROR("failed to allocate memory for %ux%u image conversion", width, height);
			return false;
		}

		ROS_INFO("allocated CUDA memory for %ux%u image conversion", width, height);

		mWidth      = width;
		mHeight     = height;
		mSizeInput  = input_size;
		mSizeOutput = output_size;		
	}

	return true;
}


