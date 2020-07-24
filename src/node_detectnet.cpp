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

#include <jetson-inference/detectNet.h>
#include <jetson-utils/cudaMappedMemory.h>

#include <unordered_map>


// globals
detectNet* net = NULL;
uint32_t overlay_flags = detectNet::OVERLAY_NONE;

imageConverter* input_cvt   = NULL;
imageConverter* overlay_cvt = NULL;

Publisher<vision_msgs::Detection2DArray> detection_pub = NULL;
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
bool publish_overlay( detectNet::Detection* detections, int numDetections )
{
	// get the image dimensions
	const uint32_t width  = input_cvt->GetWidth();
	const uint32_t height = input_cvt->GetHeight();

	// assure correct image size
	if( !overlay_cvt->Resize(width, height) )
		return false;

	// generate the overlay
	if( !net->Overlay((void*)input_cvt->ImageGPU(), (void*)overlay_cvt->ImageGPU(), width, height, IMAGE_RGBA32F, detections, numDetections, overlay_flags) )
		return false;

	// populate the message
	sensor_msgs::Image msg;

	if( !overlay_cvt->Convert(msg, sensor_msgs::image_encodings::BGR8) )
		return false;

	// populate timestamp in header field
	msg.header.stamp = ROS_TIME_NOW();

	// publish the message	
	overlay_pub->publish(msg);
	ROS_INFO("publishing %ux%u overlay image", width, height);
}


// input image subscriber callback
void img_callback( const sensor_msgs::ImageConstPtr input )
{
	// convert the image to reside on GPU
	if( !input_cvt || !input_cvt->Convert(input) )
	{
		ROS_INFO("failed to convert %ux%u %s image", input->width, input->height, input->encoding.c_str());
		return;	
	}

	// classify the image
	detectNet::Detection* detections = NULL;

	const int numDetections = net->Detect(input_cvt->ImageGPU(), input_cvt->GetWidth(), input_cvt->GetHeight(), &detections, detectNet::OVERLAY_NONE);

	// verify success	
	if( numDetections < 0 )
	{
		ROS_ERROR("failed to run object detection on %ux%u image", input->width, input->height);
		return;
	}

	// if objects were detected, send out message
	if( numDetections > 0 )
	{
		ROS_INFO("detected %i objects in %ux%u image", numDetections, input->width, input->height);
		
		// create a detection for each bounding box
		vision_msgs::Detection2DArray msg;

		for( int n=0; n < numDetections; n++ )
		{
			detectNet::Detection* det = detections + n;

			ROS_INFO("object %i class #%u (%s)  confidence=%f", n, det->ClassID, net->GetClassDesc(det->ClassID), det->Confidence);
			ROS_INFO("object %i bounding box (%f, %f)  (%f, %f)  w=%f  h=%f", n, det->Left, det->Top, det->Right, det->Bottom, det->Width(), det->Height()); 
			
			// create a detection sub-message
			vision_msgs::Detection2D detMsg;

			detMsg.bbox.size_x = det->Width();
			detMsg.bbox.size_y = det->Height();
			
			float cx, cy;
			det->Center(&cx, &cy);

			detMsg.bbox.center.x = cx;
			detMsg.bbox.center.y = cy;

			detMsg.bbox.center.theta = 0.0f;		// TODO optionally output object image

			// create classification hypothesis
			vision_msgs::ObjectHypothesisWithPose hyp;
			
			hyp.id = det->ClassID;
			hyp.score = det->Confidence;

			detMsg.results.push_back(hyp);
			msg.detections.push_back(detMsg);
		}

		// populate timestamp in header field
		msg.header.stamp = ROS_TIME_NOW();

		// publish the detection message
		detection_pub->publish(msg);
	}

	// generate the overlay (if there are subscribers)
	if( NUM_SUBSCRIBERS(overlay_pub) > 0 )
		publish_overlay(detections, numDetections);
}


// node main loop
int main(int argc, char **argv)
{
	/*
	 * create node instance
	 */
	CREATE_NODE("detectnet");

	/*
	 * retrieve parameters
	 */
	std::string class_labels_path;
	std::string prototxt_path;
	std::string model_path;
	std::string model_name;

	bool use_model_name = false;

	// determine if custom model paths were specified
	if( !GET_PARAMETER("prototxt_path", prototxt_path) && !GET_PARAMETER("model_path", model_path) )
	{
		// without custom model, use one of the built-in pretrained models
		GET_PARAMETER_OR("model_name", model_name, std::string("ssd-mobilenet-v2"));
		use_model_name = true;
	}

	// set mean pixel and threshold defaults
	float mean_pixel = 0.0f;
	float threshold  = DETECTNET_DEFAULT_THRESHOLD;
	
	GET_PARAMETER_OR("mean_pixel_value", mean_pixel, mean_pixel); //private_nh.param<float>("mean_pixel_value", mean_pixel, mean_pixel);
	GET_PARAMETER_OR("threshold", threshold, threshold);

	// parse overlay flags
	std::string overlay_str = "box,labels,conf";
	GET_PARAMETER_OR("overlay_flags", overlay_str, overlay_str);
	overlay_flags = detectNet::OverlayFlagsFromStr(overlay_str.c_str());


	/*
	 * load object detection network
	 */
	if( use_model_name )
	{
		// determine which built-in model was requested
		detectNet::NetworkType model = detectNet::NetworkTypeFromStr(model_name.c_str());

		if( model == detectNet::CUSTOM )
		{
			ROS_ERROR("invalid built-in pretrained model name '%s', defaulting to pednet", model_name.c_str());
			model = detectNet::SSD_MOBILENET_V2;
		}

		// create network using the built-in model
		net = detectNet::Create(model);
	}
	else
	{
		// get input/output layer names
		std::string input_blob = DETECTNET_DEFAULT_INPUT;
		std::string output_cvg = DETECTNET_DEFAULT_COVERAGE;
		std::string output_box = DETECTNET_DEFAULT_BBOX;

		GET_PARAMETER_OR("input_blob", input_blob, input_blob);
		GET_PARAMETER_OR("output_cvg", output_cvg, output_cvg);
		GET_PARAMETER_OR("output_bbox", output_box, output_box);

		// get the class labels path (optional)
		GET_PARAMETER("class_labels_path", class_labels_path);

		// create network using custom model paths
		net = detectNet::Create(prototxt_path.c_str(), model_path.c_str(), mean_pixel, class_labels_path.c_str(), threshold, input_blob.c_str(), output_cvg.c_str(), output_box.c_str());
	}

	if( !net )
	{
		ROS_ERROR("failed to load detectNet model");
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
	SET_PARAMETER(class_key, class_descriptions);
		
	// populate the vision info msg
	std::string node_namespace = GET_NAMESPACE();
	ROS_INFO("node namespace => %s", node_namespace.c_str());

	info_msg.database_location = node_namespace + std::string("/") + class_key;
	info_msg.database_version  = 0;
	info_msg.method 		  = net->GetModelPath();
	
	ROS_INFO("class labels => %s", info_msg.database_location.c_str());


	/*
	 * create an image converter object
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
	CREATE_PUBLISHER(vision_msgs::Detection2DArray, "detections", 25, detection_pub);
	CREATE_PUBLISHER(sensor_msgs::Image, "overlay", 2, overlay_pub);
	
	CREATE_PUBLISHER_STATUS(vision_msgs::VisionInfo, "vision_info", 1, info_callback, info_pub);


	/*
	 * subscribe to image topic
	 */
	auto img_sub = CREATE_SUBSCRIBER(sensor_msgs::Image, "image_in", 5, img_callback);

	
	/*
	 * wait for messages
	 */
	ROS_INFO("detectnet node initialized, waiting for messages");
	ROS_SPIN();

	return 0;
}

