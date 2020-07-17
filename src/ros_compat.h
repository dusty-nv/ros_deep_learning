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

#endif
#endif

