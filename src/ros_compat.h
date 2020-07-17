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

#ifndef __ROS_DEEP_LEARNING_COMPATABILITY_H_
#define __ROS_DEEP_LEARNING_COMPATABILITY_H_

#ifdef ROS1

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <vision_msgs/Classification2D.h>
#include <vision_msgs/VisionInfo.h>

#define CREATE_NODE(name)				\
		ros::init(argc, argv, name); 		\
 		ros::NodeHandle nh; 			\
		ros::NodeHandle private_nh("~");

template<class MessageType>
using Publisher = std::shared_ptr<ros::Publisher>;

#define CREATE_PUBLISHER(msg, topic, queue)			private_nh.advertise<msg>(topic, queue)
#define CREATE_SUBSCRIBER(msg, topic, queue, callback)	private_nh.subscribe(topic, queue, callback)

#define GET_NAMESPACE()							private_nh.getNamespace()
#define GET_PARAMETER(name, val)					private_nh.getParam(name, val)
#define GET_PARAMETER_OR(name, val, alt)			private_nh.param<std::string>(name, val, alt)
#define SET_PARAMETER(name, val)					private_nh.setParam(name, val)

#define ROS_SPIN()								ros::spin()

#elif ROS2

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/classification2_d.hpp>
#include <vision_msgs/msg/vision_info.hpp>

namespace vision_msgs
{
	typedef msg::Classification2D Classification2D;
	typedef msg::ObjectHypothesis ObjectHypothesis;
	typedef msg::VisionInfo VisionInfo;
}

namespace sensor_msgs
{
	typedef msg::Image Image;
	typedef msg::Image::SharedPtr ImagePtr;
	typedef msg::Image::ConstSharedPtr ImageConstPtr;
}

namespace ros = rclcpp;

#define ROS_INFO(...)	RCUTILS_LOG_INFO_ONCE(__VA_ARGS__)
#define ROS_ERROR(...)   RCUTILS_LOG_ERROR_ONCE(__VA_ARGS__)

#define CREATE_NODE(name)							\
		rclcpp::init(argc, argv);					\
		rclcpp::NodeOptions node_options;				\
		node_options.allow_undeclared_parameters(true); 	\
		auto node = rclcpp::Node::make_shared(name, "/" name,  node_options);

template<class MessageType>
using Publisher = std::shared_ptr<ros::Publisher<MessageType>>;

#define CREATE_PUBLISHER(msg, topic, queue)			node->create_publisher<msg>(topic, queue)
#define CREATE_SUBSCRIBER(msg, topic, queue, callback)	node->create_subscription<msg>(topic, queue, callback)

#define GET_NAMESPACE()							node->get_namespace()
#define GET_PARAMETER(name, val)					node->get_parameter(name, val)
#define GET_PARAMETER_OR(name, val, alt)			node->get_parameter_or(name, val, std::string(alt))	// TODO set undefined params in param server
#define SET_PARAMETER(name, val)					node->set_parameter(rclcpp::Parameter(name, val))

#define ROS_SPIN()								rclcpp::spin(node)

#endif
#endif

