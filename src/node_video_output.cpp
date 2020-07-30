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

#include <jetson-utils/videoOutput.h>



// globals	
videoOutput* stream = NULL;
imageConverter* image_cvt = NULL;

std::string topic_name;


// input image subscriber callback
void img_callback( const sensor_msgs::ImageConstPtr input )
{
	// convert the image to reside on GPU
	if( !image_cvt || !image_cvt->Convert(input) )
	{
		ROS_INFO("failed to convert %ux%u %s image", input->width, input->height, input->encoding.c_str());
		return;	
	}

	// render the image
	stream->Render(image_cvt->ImageGPU(), image_cvt->GetWidth(), image_cvt->GetHeight());

	// update status bar
	char str[256];
	sprintf(str, "%s (%ux%u) | %.1f FPS", topic_name.c_str(), image_cvt->GetWidth(), image_cvt->GetHeight(), stream->GetFrameRate());
	stream->SetStatus(str);	

	// check for EOS
	if( !stream->IsStreaming() )
		ROS_SHUTDOWN();
}


// node main loop
int main(int argc, char **argv)
{
	/*
	 * create node instance
	 */
	ROS_CREATE_NODE("video_output");


	/*
	 * declare parameters
	 */
	videoOptions video_options;

	std::string resource_str;
	std::string codec_str;

	int video_bitrate = video_options.bitRate;

	ROS_DECLARE_PARAMETER("resource", resource_str);
	ROS_DECLARE_PARAMETER("codec", codec_str);
	ROS_DECLARE_PARAMETER("bitrate", video_bitrate);
	
	/*
	 * retrieve parameters
	 */
	ROS_GET_PARAMETER("resource", resource_str);
	ROS_GET_PARAMETER("codec", codec_str);
	ROS_GET_PARAMETER("bitrate", video_bitrate);

	if( resource_str.size() == 0 )
	{
		ROS_ERROR("resource param wasn't set - please set the node's resource parameter to the input device/filename/URL");
		return 0;
	}

	if( codec_str.size() != 0 )
		video_options.codec = videoOptions::CodecFromStr(codec_str.c_str());

	video_options.bitRate = video_bitrate;

	ROS_INFO("opening video output: %s", resource_str.c_str());


	/*
	 * create stream
	 */
	stream = videoOutput::Create(resource_str.c_str(), video_options); 
	
	if( !stream )
	{
		ROS_ERROR("failed to open video output");
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
	 * subscribe to image topic
	 */
	auto img_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "image_in", 5, img_callback);
	
	topic_name = ROS_SUBSCRIBER_TOPIC(img_sub);


	/*
	 * start streaming
	 */
	if( !stream->Open() )
	{
		ROS_ERROR("failed to start streaming video source");
		return 0;
	}


	/*
	 * start publishing video frames
	 */
	ROS_INFO("video_output node initialized, waiting for messages");
	ROS_SPIN();


	/*
	 * free resources
	 */
	delete stream;
	delete image_cvt;

	return 0;
}

