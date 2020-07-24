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

#ifndef __ROS_DEEP_LEARNING_IMAGE_CONVERTER_H_
#define __ROS_DEEP_LEARNING_IMAGE_CONVERTER_H_

#include <jetson-utils/cudaUtility.h>
#include <jetson-utils/imageFormat.h>

#include "ros_compat.h"


/**
 * GPU image conversion
 */
class imageConverter
{
public:
	/**
	 * Output image pixel type
	 */
	typedef uchar3 PixelType;

	/**
	 * Image format used for internal CUDA processing
	 */
	static const imageFormat InternalFormat = IMAGE_RGB8;

	/**
	 * Image format used for outputting ROS image messages
	 */
	static const imageFormat ROSOutputFormat = IMAGE_BGR8;

	/**
	 * Constructor
	 */
	imageConverter();

	/**
	 * Destructor
	 */
	~imageConverter();

	/**
	 * Free the memory
	 */
	void Free();

	/**
	 * Convert to 32-bit RGBA floating point
	 */
	bool Convert( const sensor_msgs::ImageConstPtr& input );

	/**
	 * Convert to ROS sensor_msgs::Image message
	 */
	bool Convert( sensor_msgs::Image& msg_out, imageFormat outputFormat );

	/**
	 * Convert to ROS sensor_msgs::Image message
	 */
	bool Convert( sensor_msgs::Image& msg_out, imageFormat outputFormat, PixelType* imageGPU );

	/**
	 * Resize the memory (if necessary)
	 */
	bool Resize( uint32_t width, uint32_t height, imageFormat inputFormat );

	/**
	 * Retrieve the converted image width
	 */
	inline uint32_t GetWidth() const		{ return mWidth; }

	/**
	 * Retrieve the converted image height
	 */
	inline uint32_t GetHeight() const		{ return mHeight; }

	/**
	 * Retrieve the GPU pointer of the converted image
	 */
	inline PixelType* ImageGPU() const		{ return mOutputGPU; }

private:

	uint32_t mWidth;
	uint32_t mHeight;	
	size_t   mSizeInput;
	size_t   mSizeOutput;

	void* mInputCPU;
	void* mInputGPU;

	PixelType* mOutputCPU;
	PixelType* mOutputGPU;
};

#endif

