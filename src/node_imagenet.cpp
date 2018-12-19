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
#include <vision_msgs/Classification2D.h>

//#include <image_transport/image_transport.h>
#include <jetson-inference/imageNet.h>

#include "image_converter.h"


// globals
imageNet* 	 net = NULL;
imageConverter* cvt = NULL;

ros::Publisher* classify_pub = NULL;


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
	

	// verify the output	
	if( img_class >= 0 )
	{
		ROS_INFO("classified image, %f %s (class=%i)", confidence, net->GetClassDesc(img_class), img_class);
		
		// create the classification message
		vision_msgs::Classification2D msg;
		vision_msgs::ObjectHypothesis obj;

		obj.id    = img_class;
		obj.score = confidence;

		msg.results.push_back(obj);	// TODO optionally add source image to msg
	
		// publish the classification message
		classify_pub->publish(msg);
	}
	else
	{
		// an error occurred if the output class is < 0
		ROS_ERROR("failed to classify %ux%u image", input->width, input->height);
	}
}


// node main loop
int main(int argc, char **argv)
{
	ros::init(argc, argv, "imagenet");
 
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	/*
	 * retrieve parameters
	 */
	std::string prototxt_path;
	std::string model_path;
	std::string class_labels_path;
	std::string model_name;

	bool use_model_name = false;

	// determine if custom model paths were specified
	if( !private_nh.getParam("prototxt_path", prototxt_path) ||
	    !private_nh.getParam("model_path", model_path) ||
	    !private_nh.getParam("class_labels_path", class_labels_path) )
	{
		// without custom model, use one of the built-in pretrained models
		private_nh.param<std::string>("model_name", model_name, "googlenet");
		use_model_name = true;
	}

	
	/*
	 * load image recognition network
	 */
	if( use_model_name )
	{
		// determine which built-in model was requested
		imageNet::NetworkType model = imageNet::NetworkTypeFromStr(model_name.c_str());

		if( model == imageNet::CUSTOM )
		{
			ROS_ERROR("invalid built-in pretrained model name '%s', defaulting to googlenet", model_name.c_str());
			model = imageNet::GOOGLENET;
		}

		// create network using the built-in model
		net = imageNet::Create(model);
	}
	else
	{
		// create network using custom model paths
		net = imageNet::Create(prototxt_path.c_str(), model_path.c_str(), NULL, class_labels_path.c_str());
	}

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

	/*
	 * subscribe to image topic
	 */
	//image_transport::ImageTransport it(nh);	// BUG - stack smashing on TX2?
	//image_transport::Subscriber img_sub = it.subscribe("image", 1, img_callback);
	ros::Subscriber img_sub = private_nh.subscribe("image_in", 5, img_callback);
	
	/*
	 * advertise classification publisher
	 */
	ros::Publisher pub = private_nh.advertise<vision_msgs::Classification2D>("classification", 5);
	classify_pub = &pub; // we need to publish from the subscriber callback

	/*
	 * wait for messages
	 */
	ROS_INFO("imagenet node initialized, waiting for messages");

	ros::spin();

	return 0;
}

