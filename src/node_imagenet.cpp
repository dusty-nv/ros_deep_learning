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

#include "ros_compat.h"
#include "image_converter.h"

#include <jetson-inference/imageNet.h>
#include <jetson-utils/cudaFont.h>

#include <unordered_map>


// globals
imageNet* net = NULL;
cudaFont* font = NULL;

imageConverter* input_cvt = NULL;
imageConverter* overlay_cvt = NULL;

Publisher<vision_msgs::Classification2D> classify_pub = NULL;
Publisher<sensor_msgs::Image> overlay_pub = NULL;
Publisher<vision_msgs::VisionInfo> info_pub = NULL;

vision_msgs::VisionInfo info_msg;


// triggered when a new subscriber connected
void info_callback()
{
	ROS_INFO("new subscriber connected to vision_info topic, sending VisionInfo msg");
	info_pub->publish(info_msg);
}	


// publish overlay image
bool publish_overlay( int img_class, float confidence )
{
	// create font for image overlay
	font = cudaFont::Create();
	
	if( !font )
	{
		ROS_ERROR("failed to load font for overlay");
		return false;
	}

	// get the image dimensions
	const uint32_t width  = input_cvt->GetWidth();
	const uint32_t height = input_cvt->GetHeight();

	// assure correct image size
	if( !overlay_cvt->Resize(width, height, imageConverter::ROSOutputFormat) )
		return false;

	// generate the overlay
	char str[256];
	sprintf(str, "%05.2f%% %s", confidence * 100.0f, net->GetClassDesc(img_class));

	font->OverlayText(input_cvt->ImageGPU(), width, height,
			        str, 5, 5, make_float4(255, 255, 255, 255), make_float4(0, 0, 0, 100));

	// populate the message
	sensor_msgs::Image msg;

	if( !overlay_cvt->Convert(msg, imageConverter::ROSOutputFormat, input_cvt->ImageGPU()) )
		return false;

	// populate timestamp in header field
	msg.header.stamp = ROS_TIME_NOW();

	// publish the message	
	overlay_pub->publish(msg);
	ROS_DEBUG("publishing %ux%u overlay image", width, height);
}


// triggered when recieved a new image on input topic
void img_callback( const sensor_msgs::ImageConstPtr input )
{
	// convert the image to reside on GPU
	if( !input_cvt || !input_cvt->Convert(input) )
	{
		ROS_INFO("failed to convert %ux%u %s image", input->width, input->height, input->encoding.c_str());
		return;	
	}

	// classify the image
	float confidence = 0.0f;
	const int img_class = net->Classify(input_cvt->ImageGPU(), input_cvt->GetWidth(), input_cvt->GetHeight(), &confidence);
	
	// verify the output	
	if( img_class >= 0 )
	{
		ROS_INFO("classified image, %f %s (class=%i)", confidence, net->GetClassDesc(img_class), img_class);
		
		// create the classification message
		vision_msgs::Classification2D msg;
		vision_msgs::ObjectHypothesis obj;

	#if ROS_DISTRO >= ROS_GALACTIC
		obj.class_id = img_class;
	#else
		obj.id = img_class;
	#endif
		obj.score = confidence;

		msg.results.push_back(obj);	// TODO optionally add source image to msg
	
		// populate timestamp in header field
		msg.header.stamp = ROS_TIME_NOW();

		// publish the classification message
		classify_pub->publish(msg);

		// generate the overlay (if there are subscribers)
		if( ROS_NUM_SUBSCRIBERS(overlay_pub) > 0 )
			publish_overlay(img_class, confidence);
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
	/*
	 * create node instance
	 */
	ROS_CREATE_NODE("imagenet");


	/*
	 * declare parameters
	 */
	std::string model_name = "googlenet";
	std::string model_path;	
	std::string prototxt_path;
	std::string class_labels_path;
	
	std::string input_blob = IMAGENET_DEFAULT_INPUT;
	std::string output_blob = IMAGENET_DEFAULT_OUTPUT;

	ROS_DECLARE_PARAMETER("model_name", model_name);
	ROS_DECLARE_PARAMETER("model_path", model_path);
	ROS_DECLARE_PARAMETER("prototxt_path", prototxt_path);
	ROS_DECLARE_PARAMETER("class_labels_path", class_labels_path);
	ROS_DECLARE_PARAMETER("input_blob", input_blob);
	ROS_DECLARE_PARAMETER("output_blob", output_blob);


	/*
	 * retrieve parameters
	 */
	ROS_GET_PARAMETER("model_name", model_name);
	ROS_GET_PARAMETER("model_path", model_path);
	ROS_GET_PARAMETER("prototxt_path", prototxt_path);
	ROS_GET_PARAMETER("class_labels_path", class_labels_path);
	ROS_GET_PARAMETER("input_blob", input_blob);
	ROS_GET_PARAMETER("output_blob", output_blob);

	
	/*
	 * load image recognition network
	 */
	if( model_path.size() > 0 )
	{
		// create network using custom model paths
		net = imageNet::Create(prototxt_path.c_str(), model_path.c_str(),
						   NULL, class_labels_path.c_str(), 
						   input_blob.c_str(), output_blob.c_str());

	}
	else
	{
		// create network using the built-in model
		net = imageNet::Create(model_name.c_str());
	}

	if( !net )
	{
		ROS_ERROR("failed to load imageNet model");
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
		class_descriptions.push_back(net->GetClassDesc(n));


	// create the key on the param server
	std::string class_key = std::string("class_labels_") + std::to_string(model_hash);

	ROS_DECLARE_PARAMETER(class_key, class_descriptions);
	ROS_SET_PARAMETER(class_key, class_descriptions);
	
	// populate the vision info msg
	const std::string node_namespace = ROS_GET_NAMESPACE();
	ROS_INFO("node namespace => %s", node_namespace.c_str());

	info_msg.database_location = node_namespace + std::string("/") + class_key;
	info_msg.database_version  = 0;
	info_msg.method 		  = net->GetModelPath();
	
	ROS_INFO("class labels => %s", info_msg.database_location.c_str());


	/*
	 * create image converter objects
	 */
	input_cvt = new imageConverter();
	overlay_cvt = new imageConverter();

	if( !input_cvt || !overlay_cvt )
	{
		ROS_ERROR("failed to create imageConverter objects");
		return 0;
	}


	/*
	 * advertise publisher topics
	 */
	ROS_CREATE_PUBLISHER(vision_msgs::Classification2D, "classification", 5, classify_pub);
	ROS_CREATE_PUBLISHER(sensor_msgs::Image, "overlay", 2, overlay_pub);
		
	ROS_CREATE_PUBLISHER_STATUS(vision_msgs::VisionInfo, "vision_info", 1, info_callback, info_pub);


	/*
	 * subscribe to image topic
	 */
	auto img_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "image_in", 5, img_callback);


	/*
	 * wait for messages
	 */
	ROS_INFO("imagenet node initialized, waiting for messages");
	ROS_SPIN();


	/*
	 * free resources
	 */
	delete net;
	delete input_cvt;
	delete overlay_cvt;

	return 0;
}

