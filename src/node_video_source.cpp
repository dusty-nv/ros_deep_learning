/*
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
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

#include "ros_compat.h"
#include "image_converter.h"

#include <jetson-utils/videoSource.h>



// globals	
videoSource* stream = NULL;
imageConverter* image_cvt = NULL;
Publisher<sensor_msgs::Image> image_pub = NULL;


// aquire and publish camera frame
bool aquireFrame()
{
	imageConverter::PixelType* nextFrame = NULL;

	// get the latest frame
	if( !stream->Capture(&nextFrame, 1000) )
	{
		ROS_ERROR("failed to capture next frame");
		return false;
	}

	// assure correct image size
	if( !image_cvt->Resize(stream->GetWidth(), stream->GetHeight(), imageConverter::ROSOutputFormat) )
	{
		ROS_ERROR("failed to resize camera image converter");
		return false;
	}

	// populate the message
	sensor_msgs::Image msg;

	if( !image_cvt->Convert(msg, imageConverter::ROSOutputFormat, nextFrame) )
	{
		ROS_ERROR("failed to convert video stream frame to sensor_msgs::Image");
		return false;
	}

	// populate timestamp in header field
	msg.header.stamp = ROS_TIME_NOW();

	// publish the message
	image_pub->publish(msg);
	ROS_INFO("published %ux%u video frame", stream->GetWidth(), stream->GetHeight());
	return true;
}


// node main loop
int main(int argc, char **argv)
{
	/*
	 * create node instance
	 */
	CREATE_NODE("video_source");

	/*
	 * retrieve parameters
	 */
#if 0
	std::string resource_str;

	if( !GET_PARAMETER("resource", resource_str) )
	{
		ROS_ERROR("resource param wasn't set - please set the resource parameter to the input device/filename/URL");
		return 0;
	}

	ROS_INFO("opening video source: %s", resource_str.c_str());

	/*
	 * open video source
	 */
	stream = videoSource::Create(resource_str.c_str());
#else
	stream = videoSource::Create(argc, argv, ARG_POSITION(0)); 
#endif
	
	if( !stream )
	{
		ROS_ERROR("failed to open video source");
		return 0;
	}


	/*
	 * create image converter
	 */
	image_cvt = new imageConverter();

	if( !image_cvt )
	{
		ROS_ERROR("failed to create imageConverter");
		return 0;
	}


	/*
	 * advertise publisher topics
	 */
	CREATE_PUBLISHER(sensor_msgs::Image, "raw", 2, image_pub);


	/*
	 * start the camera streaming
	 */
	if( !stream->Open() )
	{
		ROS_ERROR("failed to start streaming video source");
		return 0;
	}


	/*
	 * start publishing video frames
	 */
	while( ros::ok() )
	{
		if( !aquireFrame() )
		{
			if( !stream->IsStreaming() )
			{
				ROS_INFO("stream is closed or reached EOS, exiting node...");
				break;
			}
		}

		ROS_SPIN_ONCE();
	}


	/*
	 * free resources
	 */
	delete stream;
	delete image_cvt;

	return 0;
}

