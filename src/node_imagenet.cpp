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

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
//#include <image_transport/image_transport.h>
#include <jetson-inference/imageNet.h>

#include "image_converter.h"


// globals
imageNet* 	 net = NULL;
imageConverter* cvt = NULL;


// input image subscriber callback
void img_callback( const sensor_msgs::ImageConstPtr& input )
{
	// convert the image to reside on GPU
	if( !cvt || !cvt->Convert(input) )
	{
		ROS_INFO("failed to convert %ux%u %s image", input->width, input->height, input->encoding.c_str());
		return;	
	}

	// classify the image
	float confidence = 0.0f;
	const int img_class = net->Classify(cvt->ImageGPU(), cvt->GetWidth(), cvt->GetHeight(), &confidence);
	

	// overlay the classification on the image
	if( img_class >= 0 )
		ROS_INFO("classified image, %f %s (class=%i)", confidence, net->GetClassDesc(img_class), img_class);
	else
		ROS_ERROR("failed to classify image");
}


// node main loop
int main(int argc, char **argv)
{
	ros::init(argc, argv, "imagenet");
 
	ros::NodeHandle nh;


	/*
	 * load image recognition network
	 */
	net = imageNet::Create();

	if( !net )
	{
		ROS_ERROR("failed to load imageNet model");
		return 0;
	}

	/*
	 * create an image converter object
	 */
	cvt = new imageConverter();
	
	if( !cvt )
	{
		ROS_ERROR("failed to create imageConverter object");
		return 0;
	}

	ROS_INFO("done new imageConverter()");

	/*
	 * subscribe to image topic
	 */
	//image_transport::ImageTransport it(nh);	// BUG - stack smashing on TX2?
	//image_transport::Subscriber img_sub = it.subscribe("image", 1, img_callback);

	ros::Subscriber img_sub = nh.subscribe("image", 1, img_callback);
	
	/*
	 * wait for messages
	 */
	ROS_INFO("imagenet node initialized, waiting for messages");

	ros::spin();

	return 0;
}

