# ros_deep_learning
This repo contains deep learning inference nodes for ROS with support for Jetson Nano/TX1/TX2/Xavier and TensorRT.

The nodes use the image recognition, object detection, and semantic segmentation DNN's from the [`jetson-inference`](https://github.com/dusty-nv/jetson-inference) library and NVIDIA [Hello AI World](https://developer.nvidia.com/embedded/twodaystoademo) tutorial, which come with several built-in pretrained networks for classification, detection, and segmentation and the ability to load customized user-trained models.

ROS Melodic (for JetPack 4.2 and Ubuntu 18.04) is recommended, but ROS Kinetic (JetPack 3.3 and Ubuntu 16.04 on TX1/TX2) should work as well.  ROS Melodic is supported on Nano/TX1/TX2/Xavier, while Kinetic runs on TX1/TX2 only.

### Table of Contents

* [Installation](#installation)
	* [jetson-inference](#jetson-inference)
	* [ROS Core](#ros-core)
	* [ros_deep_learning](#ros_deep_learning-1)
* [Testing](#testing)
	* [imageNet Node](#imagenet-node)
	* [detectNet Node](#detectnet-node)

## Installation

First, install the latest [JetPack](https://developer.nvidia.com/embedded/jetpack) on your Jetson (JetPack 4.2.2 for ROS Melodic or JetPack 3.3 for ROS Kinetic on TX1/TX2).

Then, follow the installation steps below to install the needed components on your Jetson:

### jetson-inference

These ROS nodes use the DNN objects from the [`jetson-inference`](https://github.com/dusty-nv/jetson-inference) project (aka Hello AI World).  To build and install it, see [this page](https://github.com/dusty-nv/jetson-inference/blob/master/docs/building-repo-2.md) or run the commands below:

```bash
$ cd ~
$ sudo apt-get install git cmake
$ git clone --recursive https://github.com/dusty-nv/jetson-inference
$ cd jetson-inference
$ mkdir build
$ cd build
$ cmake ../
$ make
$ sudo make install
```
Before proceeding, it's worthwhile to test that `jetson-inference` is working properly on your system by following this step of the Hello AI World tutorial:
* [Classifying Images with ImageNet](https://github.com/dusty-nv/jetson-inference/blob/master/docs/imagenet-console-2.md)

### ROS Core

Install the `ros-melodic-ros-base` or `ros-kinetic-ros-base` package on your Jetson following these directions:

* ROS Melodic (JetPack 4.2) - [ROS Install Instructions](http://wiki.ros.org/melodic/Installation/Ubuntu)
* ROS Kinetic (JetPack 3.3) - [JetsonHacks Post](https://www.jetsonhacks.com/2018/04/27/robot-operating-system-ros-on-nvidia-jetson-tx-development-kits/)

Depending on which version of ROS you're using, install some additional dependencies:

#### ROS Melodic
```bash
$ sudo apt-get install ros-melodic-image-transport
$ sudo apt-get install ros-melodic-image-publisher
$ sudo apt-get install ros-melodic-vision-msgs
```

#### ROS Kinetic
```bash
$ sudo apt-get install ros-kinetic-image-transport
$ sudo apt-get install ros-kinetic-image-publisher
$ sudo apt-get install ros-kinetic-vision-msgs
```

#### Catkin Workspace

Then, create a Catkin workspace (`~/catkin_ws`) using these steps:  
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace


### ros_deep_learning

Next, navigate into your Catkin workspace and clone and build `ros_deep_learning`:

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/dusty-nv/ros_deep_learning
$ cd ../
$ catkin_make
```

The inferencing nodes should now be built and ready to use.

## Testing

Before proceeding, make sure that `roscore` is running first:

```bash
$ roscore
```

### imageNet Node

First, to stream some image data for the inferencing node to process, open another terminal and start an [`image_publisher`](http://wiki.ros.org/image_publisher), which loads a specified image from disk.  We tell it to load one of the test images that come with jetson-inference, but you can substitute your own images here as well:

```bash
$ rosrun image_publisher image_publisher __name:=image_publisher ~/jetson-inference/data/images/orange_0.jpg
```

Next, open a new terminal, overlay your Catkin workspace, and start the [`imagenet`](src/node_imagenet.cpp) node:

```bash
$ source ~/catkin_ws/devel/setup.bash
$ rosrun ros_deep_learning imagenet /imagenet/image_in:=/image_publisher/image_raw _model_name:=googlenet
```
Here, we remap imagenet's `image_in` input topic to the output of the `image_publisher`, and tell it to load the GoogleNet model using the node's `model_name` parameter.  See [this table](https://github.com/dusty-nv/jetson-inference/blob/master/docs/imagenet-console-2.md#downloading-other-classification-models) for other classification models that you can download and substitute for `model_name`.

In another terminal, you should be able to verify the [`vision_msgs/Classification2D`](http://docs.ros.org/melodic/api/vision_msgs/html/msg/Classification2D.html) message output of the node, which is published to the `imagenet/classification` topic:

```bash
$ rostopic echo /imagenet/classification
```

### detectNet Node

Kill the other nodes you launched above, and start publishing a new image with people in it for the [`detectnet`](src/node_detectnet.cpp) node to process:

```bash
$ rosrun image_publisher image_publisher __name:=image_publisher ~/jetson-inference/data/images/peds-004.jpg 
$ rosrun ros_deep_learning detectnet /detectnet/image_in:=/image_publisher/image_raw _model_name:=pednet
```

See [this table](https://github.com/dusty-nv/jetson-inference/blob/master/docs/detectnet-console-2.md#pre-trained-detection-models-available) for the built-in detection models available.  Here's an example of launching with the model that detects dogs:

```bash
$ rosrun image_publisher image_publisher __name:=image_publisher ~/jetson-inference/data/images/dog_0.jpg
$ rosrun ros_deep_learning detectnet /detectnet/image_in:=/image_publisher/image_raw _model_name:=coco-dog
```

To inspect the [`vision_msgs/Detection2DArray`](http://docs.ros.org/melodic/api/vision_msgs/html/msg/Detection2DArray.html) message output of the node, subscribe to the `detectnet/detections` topic:

```bash
$ rostopic echo /detectnet/detections
```


