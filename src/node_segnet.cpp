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
#include <sensor_msgs/image_encodings.h>
#include <vision_msgs/VisionInfo.h>

#include <jetson-inference/segNet.h>
#include <jetson-utils/cudaMappedMemory.h>

#include "image_converter.h"

#include <unordered_map>


// globals
segNet* net = NULL;

segNet::FilterMode overlay_filter = segNet::FILTER_LINEAR;
segNet::FilterMode mask_filter    = segNet::FILTER_LINEAR;

imageConverter* input_cvt      = NULL;
imageConverter* overlay_cvt    = NULL;
imageConverter* mask_color_cvt = NULL;
imageConverter* mask_class_cvt = NULL;

ros::Publisher* overlay_pub    = NULL;
ros::Publisher* mask_color_pub = NULL;
ros::Publisher* mask_class_pub = NULL;

vision_msgs::VisionInfo info_msg;


// callback triggered when a new subscriber connected to vision_info topic
void info_connect( const ros::SingleSubscriberPublisher& pub )
{
	ROS_INFO("new subscriber '%s' connected to vision_info topic '%s', sending VisionInfo msg", pub.getSubscriberName().c_str(), pub.getTopic().c_str());
	pub.publish(info_msg);
}


// publish overlay image
bool publish_overlay( uint32_t width, uint32_t height )
{
	// assure correct image size
	if( !overlay_cvt->Resize(width, height) )
		return false;

	// generate the overlay
	if( !net->Overlay(overlay_cvt->ImageGPU(), width, height, overlay_filter) )
		return false;

	// populate the message
	sensor_msgs::Image msg;

	if( !overlay_cvt->Convert(msg, sensor_msgs::image_encodings::BGR8) )
		return false;

	// publish the message
	overlay_pub->publish(msg);
}


// publish color mask
bool publish_mask_color( uint32_t width, uint32_t height )
{
	// assure correct image size
	if( !mask_color_cvt->Resize(width, height) )
		return false;

	// generate the overlay
	if( !net->Mask(mask_color_cvt->ImageGPU(), width, height, mask_filter) )
		return false;

	// populate the message
	sensor_msgs::Image msg;

	if( !mask_color_cvt->Convert(msg, sensor_msgs::image_encodings::BGR8) )
		return false;

	// publish the message
	mask_color_pub->publish(msg);
}


// publish class mask
bool publish_mask_class( uint32_t width, uint32_t height )
{
	// assure correct image size
	if( !mask_class_cvt->Resize(width, height) )
		return false;

	// generate the overlay
	if( !net->Mask((uint8_t*)mask_class_cvt->ImageGPU(), width, height) )
		return false;

	// populate the message
	sensor_msgs::Image msg;

	if( !mask_class_cvt->Convert(msg, sensor_msgs::image_encodings::MONO8) )
		return false;

	// publish the message
	mask_class_pub->publish(msg);
}


// input image subscriber callback
void img_callback( const sensor_msgs::ImageConstPtr& input )
{
	// convert the image to reside on GPU
	if( !input_cvt || !input_cvt->Convert(input) )
	{
		ROS_INFO("failed to convert %ux%u %s image", input->width, input->height, input->encoding.c_str());
		return;	
	}

	// process the segmentation network
	if( !net->Process(input_cvt->ImageGPU(), input_cvt->GetWidth(), input_cvt->GetHeight()) )
	{
		ROS_ERROR("failed to process segmentation on %ux%u image", input->width, input->height);
		return;
	}

	// color overlay
	if( overlay_pub->getNumSubscribers() > 0 )
		publish_overlay(input->width, input->height);

	// color mask
	if( mask_color_pub->getNumSubscribers() > 0 )
		publish_mask_color(input->width, input->height);

	// class mask
	if( mask_class_pub->getNumSubscribers() > 0 )
		publish_mask_class(net->GetGridWidth(), net->GetGridHeight());
}


// node main loop
int main(int argc, char **argv)
{
	ros::init(argc, argv, "segnet");
 
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	/*
	 * retrieve parameters
	 */
	std::string class_labels_path;
	std::string class_colors_path;
	std::string prototxt_path;
	std::string model_path;
	std::string model_name;

	bool use_model_name = false;

	// determine if custom model paths were specified
	if( !private_nh.getParam("prototxt_path", prototxt_path) ||
	    !private_nh.getParam("model_path", model_path) ||
	    !private_nh.getParam("class_labels_path", class_labels_path) )
	{
		// without custom model, use one of the pretrained built-in models
		private_nh.param<std::string>("model_name", model_name, "cityscapes");
		use_model_name = true;
	}
	else
	{
		// optional parameters for custom models
		private_nh.getParam("class_colors_path", class_colors_path);
	}
	
	// retrieve filter mode settings
	std::string overlay_filter_str = "linear";
	std::string mask_filter_str    = "linear";

	private_nh.param<std::string>("overlay_filter", overlay_filter_str, overlay_filter_str);
	private_nh.param<std::string>("mask_filter", mask_filter_str, mask_filter_str);

	overlay_filter = segNet::FilterModeFromStr(overlay_filter_str.c_str(), segNet::FILTER_LINEAR);
	mask_filter    = segNet::FilterModeFromStr(mask_filter_str.c_str(), segNet::FILTER_LINEAR);


	/*
	 * load segmentation network
	 */
	if( use_model_name )
	{
		// determine which built-in model was requested
		segNet::NetworkType model = segNet::NetworkTypeFromStr(model_name.c_str());

		if( model == segNet::SEGNET_CUSTOM )
		{
			ROS_ERROR("invalid built-in pretrained model name '%s', defaulting to cityscapes", model_name.c_str());
			model = segNet::FCN_ALEXNET_CITYSCAPES_HD;
		}

		// create network using the built-in model
		net = segNet::Create(model);
	}
	else
	{
		// get the class labels path (optional)
		private_nh.getParam("class_labels_path", class_labels_path);

		// create network using custom model paths
		net = segNet::Create(prototxt_path.c_str(), model_path.c_str(), class_labels_path.c_str(), class_colors_path.c_str());
	}

	if( !net )
	{
		ROS_ERROR("failed to load segNet model");
		return 0;
	}


	/*
	 * create the class labels parameter vector
	 */
	std::hash<std::string> model_hasher;  // hash the model path to avoid collisions on the param server
	std::string model_hash_str = std::string(net->GetModelPath()) + std::string(net->GetClassPath());
	const size_t model_hash = model_hasher(model_hash_str);
	
	ROS_INFO("model hash => %zu", model_hash);
	ROS_INFO("hash string => %s", model_hash_str.c_str());

	// obtain the list of class descriptions
	std::vector<std::string> class_descriptions;
	const uint32_t num_classes = net->GetNumClasses();

	for( uint32_t n=0; n < num_classes; n++ )
	{
		const char* label = net->GetClassDesc(n);

		if( label != NULL )
			class_descriptions.push_back(label);
	}

	// create the key on the param server
	std::string class_key = std::string("class_labels_") + std::to_string(model_hash);
	private_nh.setParam(class_key, class_descriptions);
		
	// populate the vision info msg
	std::string node_namespace = private_nh.getNamespace();
	ROS_INFO("node namespace => %s", node_namespace.c_str());

	info_msg.database_location = node_namespace + std::string("/") + class_key;
	info_msg.database_version  = 0;
	info_msg.method 		  = net->GetModelPath();
	
	ROS_INFO("class labels => %s", info_msg.database_location.c_str());


	/*
	 * create image converters
	 */
	input_cvt      = new imageConverter();
	overlay_cvt    = new imageConverter();
	mask_color_cvt = new imageConverter();
	mask_class_cvt = new imageConverter();

	if( !input_cvt || !overlay_cvt || !mask_color_cvt || !mask_class_cvt )
	{
		ROS_ERROR("failed to create imageConverter objects");
		return 0;
	}


	/*
	 * advertise publisher topics
	 */
	ros::Publisher overlay_publsh = private_nh.advertise<sensor_msgs::Image>("overlay", 2);
	overlay_pub = &overlay_publsh;

	ros::Publisher color_mask_pub = private_nh.advertise<sensor_msgs::Image>("color_mask", 2);
	mask_color_pub = &color_mask_pub;

	ros::Publisher class_mask_pub = private_nh.advertise<sensor_msgs::Image>("class_mask", 2);
	mask_class_pub = &class_mask_pub;

	// the vision info topic only publishes upon a new connection
	ros::Publisher info_pub = private_nh.advertise<vision_msgs::VisionInfo>("vision_info", 1, (ros::SubscriberStatusCallback)info_connect);


	/*
	 * subscribe to image topic
	 */
	//image_transport::ImageTransport it(nh);	// BUG - stack smashing on TX2?
	//image_transport::Subscriber img_sub = it.subscribe("image", 1, img_callback);
	ros::Subscriber img_sub = private_nh.subscribe("image_in", 5, img_callback);
	

	/*
	 * wait for messages
	 */
	ROS_INFO("segnet node initialized, waiting for messages");

	ros::spin();

	return 0;
}

