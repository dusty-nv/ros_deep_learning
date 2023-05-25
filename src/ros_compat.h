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

#define ROS_KINETIC  0
#define ROS_MELODIC  1
#define ROS_NOETIC   2
#define ROS_DASHING  3
#define ROS_ELOQUENT 4
#define ROS_FOXY     5
#define ROS_GALACTIC 6
#define ROS_HUMBLE   7
#define ROS_IRON     8

#ifdef ROS1

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <vision_msgs/Classification2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/VisionInfo.h>

#define ROS_CREATE_NODE(name)				\
		ros::init(argc, argv, name); 		\
 		ros::NodeHandle nh; 			\
		ros::NodeHandle private_nh("~");

template<class MessageType>
using Publisher = ros::Publisher*;

#define ROS_CREATE_PUBLISHER(msg, topic, queue, ptr)				ros::Publisher __publisher_##ptr = private_nh.advertise<msg>(topic, queue); ptr = &__publisher_##ptr
#define ROS_CREATE_PUBLISHER_STATUS(msg, topic, queue, callback, ptr)	ros::Publisher __publisher_##ptr = private_nh.advertise<msg>(topic, queue, [&](const ros::SingleSubscriberPublisher& connect_msg){callback();}); ptr = &__publisher_##ptr

#define ROS_CREATE_SUBSCRIBER(msg, topic, queue, callback)			private_nh.subscribe(topic, queue, callback)
#define ROS_SUBSCRIBER_TOPIC(subscriber)						subscriber.getTopic()

#define ROS_NUM_SUBSCRIBERS(publisher)							publisher->getNumSubscribers()
#define ROS_GET_NAMESPACE()									private_nh.getNamespace()
#define ROS_GET_PARAMETER(name, val)							private_nh.getParam(name, val)
#define ROS_GET_PARAMETER_OR(name, val, alt)						private_nh.param(name, val, alt)
#define ROS_SET_PARAMETER(name, val)							private_nh.setParam(name, val)

template<typename T> static void __ros_declare_parameter( ros::NodeHandle& nh, const std::string& name, const T& default_value )
{
	if( !nh.hasParam(name) )
		nh.setParam(name, default_value);
}

#define ROS_DECLARE_PARAMETER(name, default_value)				__ros_declare_parameter(private_nh, name, default_value)

#define ROS_TIME_NOW()										ros::Time::now()
#define ROS_SPIN()											ros::spin()
#define ROS_SPIN_ONCE()										ros::spinOnce()
#define ROS_OK()											ros::ok()
#define ROS_SHUTDOWN() 										ros::shutdown()

#elif ROS2

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#if ROS_DISTRO >= ROS_GALACTIC
	#include <vision_msgs/msg/classification.hpp>
#else
	#include <vision_msgs/msg/classification2_d.hpp>
#endif

#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/vision_info.hpp>

namespace vision_msgs
{
#if ROS_DISTRO >= ROS_GALACTIC
	typedef msg::Classification   Classification2D;
#else
	typedef msg::Classification2D Classification2D;
#endif
	typedef msg::Detection2D		Detection2D;
	typedef msg::Detection2DArray Detection2DArray;
	typedef msg::ObjectHypothesis ObjectHypothesis;
	typedef msg::ObjectHypothesisWithPose ObjectHypothesisWithPose;
	typedef msg::VisionInfo 		VisionInfo;
}

namespace sensor_msgs
{
	typedef msg::Image Image;
	typedef msg::Image::SharedPtr ImagePtr;
	typedef msg::Image::ConstSharedPtr ImageConstPtr;
}

namespace ros = rclcpp;

extern std::string __node_name_;

#define ROS_INFO(...)	RCUTILS_LOG_INFO_NAMED(__node_name_.c_str(), __VA_ARGS__)
#define ROS_DEBUG(...)	RCUTILS_LOG_DEBUG_NAMED(__node_name_.c_str(), __VA_ARGS__)
#define ROS_ERROR(...)   RCUTILS_LOG_ERROR_NAMED(__node_name_.c_str(), __VA_ARGS__)

#define ROS_CREATE_NODE(name)							\
		rclcpp::init(argc, argv);					\
		auto node = rclcpp::Node::make_shared(name, "/" name); \
		__node_name_ = name; \
		__global_clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

template<class MessageType>
using Publisher = std::shared_ptr<ros::Publisher<MessageType>>;

#define ROS_CREATE_PUBLISHER(msg, topic, queue, ptr)				ptr = node->create_publisher<msg>(topic, queue)
#define ROS_CREATE_PUBLISHER_STATUS(msg, topic, queue, callback, ptr)	ptr = node->create_publisher<msg>(topic, queue); rclcpp::TimerBase::SharedPtr __timer_publisher_##ptr = node->create_wall_timer(std::chrono::milliseconds(500), \
														[&](){ static int __subscribers_##ptr=0; const size_t __subscription_count=ptr->get_subscription_count(); if(__subscribers_##ptr != __subscription_count) { if(__subscription_count > __subscribers_##ptr) callback(); __subscribers_##ptr=__subscription_count; }}) 

#define ROS_CREATE_SUBSCRIBER(msg, topic, queue, callback)			node->create_subscription<msg>(topic, queue, callback)
#define ROS_SUBSCRIBER_TOPIC(subscriber)						subscriber->get_topic_name();

#define ROS_NUM_SUBSCRIBERS(publisher)							publisher->get_subscription_count()
#define ROS_GET_NAMESPACE()									node->get_namespace()
#define ROS_GET_PARAMETER(name, val)							node->get_parameter(name, val)
#define ROS_GET_PARAMETER_OR(name, val, alt)						node->get_parameter_or(name, val, alt)	// TODO set undefined params in param server
#define ROS_SET_PARAMETER(name, val)							node->set_parameter(rclcpp::Parameter(name, val))

#define ROS_DECLARE_PARAMETER(name, default_value)				node->declare_parameter(name, default_value)

extern rclcpp::Clock::SharedPtr __global_clock_;

#define ROS_TIME_NOW()										__global_clock_->now()
#define ROS_SPIN()											rclcpp::spin(node)
#define ROS_SPIN_ONCE()										rclcpp::spin_some(node)
#define ROS_OK()											rclcpp::ok()
#define ROS_SHUTDOWN() 										rclcpp::shutdown()

#endif
#endif

