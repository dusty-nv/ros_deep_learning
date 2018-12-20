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
#include <vision_msgs/Detection2DArray.h>

#include <jetson-inference/detectNet.h>
#include <jetson-utils/cudaMappedMemory.h>

#include "image_converter.h"


// globals
detectNet* 	 net = NULL;
imageConverter* cvt = NULL;

uint32_t objClasses = 0;
uint32_t maxBoxes   = 0;

float* bbCPU   = NULL;
float* bbGPU   = NULL;
float* confCPU = NULL;
float* confGPU = NULL;

ros::Publisher* detection_pub = NULL;


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
	int numBoundingBoxes = maxBoxes;
	const bool result = net->Detect(cvt->ImageGPU(), cvt->GetWidth(), cvt->GetHeight(), bbCPU, &numBoundingBoxes, confCPU);

	// verify success	
	if( !result )
	{
		ROS_ERROR("failed to run detection on %ux%u image", input->width, input->height);
		return;
	}

	// if objects were detected, send out message
	if( numBoundingBoxes > 0 )
	{
		ROS_INFO("detected %i objects in %ux%u image", numBoundingBoxes, input->width, input->height);
		
		// create a detection for each bounding box
		vision_msgs::Detection2DArray msg;

		for( int n=0; n < numBoundingBoxes; n++ )
		{
			// extract class/confidence pairs
			const float obj_conf = confCPU[n*2];
			const int  obj_class = confCPU[n*2+1];

			// extract the bounding box
			float* bb = bbCPU + (n * 4);

			const float bbWidth  = bb[2] - bb[0];
			const float bbHeight = bb[3] - bb[1];
			
			printf("object %i bounding box (%f, %f)  (%f, %f)  w=%f  h=%f\n", n, bb[0], bb[1], bb[2], bb[3], bbWidth, bbHeight); 
			printf("object %i class=%i confidence=%f\n", n, obj_class, obj_conf);

			// create a detection sub-message
			vision_msgs::Detection2D det;

			det.bbox.size_x = bbWidth;
			det.bbox.size_y = bbHeight;
			
			det.bbox.center.x = bb[0] + bbWidth * 0.5f;
			det.bbox.center.y = bb[1] + bbHeight * 0.5f;

			det.bbox.center.theta = 0.0f;		// TODO optionally output object image

			// create classification hypothesis
			vision_msgs::ObjectHypothesisWithPose hyp;
			
			hyp.id = obj_class;
			hyp.score = obj_conf;

			det.results.push_back(hyp);
			msg.detections.push_back(det);
		}

		// publish the detection message
		detection_pub->publish(msg);
	}
}


// node main loop
int main(int argc, char **argv)
{
	ros::init(argc, argv, "detectnet");
 
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	/*
	 * retrieve parameters
	 */
	std::string prototxt_path;
	std::string model_path;
	std::string model_name;

	bool use_model_name = false;

	// determine if custom model paths were specified
	if( !private_nh.getParam("prototxt_path", prototxt_path) ||
	    !private_nh.getParam("model_path", model_path) )
	{
		// without custom model, use one of the built-in pretrained models
		private_nh.param<std::string>("model_name", model_name, "pednet");
		use_model_name = true;
	}

	// set mean pixel and threshold defaults
	float mean_pixel = 0.0f;
	float threshold  = 0.5f;
	
	private_nh.param<float>("mean_pixel_value", mean_pixel, mean_pixel);
	private_nh.param<float>("threshold", threshold, threshold);

	/*
	 * load image recognition network
	 */
	if( use_model_name )
	{
		// determine which built-in model was requested
		detectNet::NetworkType model = detectNet::NetworkTypeFromStr(model_name.c_str());

		if( model == detectNet::CUSTOM )
		{
			ROS_ERROR("invalid built-in pretrained model name '%s', defaulting to pednet", model_name.c_str());
			model = detectNet::PEDNET;
		}

		// create network using the built-in model
		net = detectNet::Create(model);
	}
	else
	{
		// create network using custom model paths
		net = detectNet::Create(prototxt_path.c_str(), model_path.c_str(), mean_pixel, threshold);
	}

	if( !net )
	{
		ROS_ERROR("failed to load detectNet model");
		return 0;
	}

	/*
	 * alloc memory for bounding box & confidence value outputs
	 */
	objClasses = net->GetNumClasses();
	maxBoxes   = net->GetMaxBoundingBoxes();		
	
	ROS_INFO("object classes:  %u", objClasses);
	ROS_INFO("maximum bounding boxes:  %u\n", maxBoxes);
	
	if( !cudaAllocMapped((void**)&bbCPU, (void**)&bbGPU, maxBoxes * sizeof(float) * 4) ||
	    !cudaAllocMapped((void**)&confCPU, (void**)&confGPU, maxBoxes * objClasses * sizeof(float)) )
	{
		ROS_ERROR("detectnet:  failed to alloc bounding box output memory");
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
	 * advertise detection publisher
	 */
	ros::Publisher pub = private_nh.advertise<vision_msgs::Detection2DArray>("detections", 5);
	detection_pub = &pub; // we need to publish from the subscriber callback

	/*
	 * wait for messages
	 */
	ROS_INFO("detectnet node initialized, waiting for messages");

	ros::spin();

	return 0;
}

