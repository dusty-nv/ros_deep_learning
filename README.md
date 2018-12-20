# ros_deep_learning
This repo contains deep learning inference nodes for ROS with support for NVIDIA Jetson TX1/TX2/Xavier and TensorRT.

The nodes use the image recognition and object detection vision objects from the [`jetson-inference`](https://github.com/dusty-nv/jetson-inference) library and [NVIDIA Two Days to a Demo](https://developer.nvidia.com/embedded/twodaystoademo) tutorial, which come with several built-in pretrained network models and the ability to load customized user-trained models.

ROS Kinetic (for TX1/TX2) and ROS Melodic (for Xavier) are supported.

# Installation

## jetson-inference

First, install the latest [JetPack](https://developer.nvidia.com/embedded/jetpack) on your Jetson (JetPack 3.3 for TX1/TX2 and JetPack 4.1.1 for Xavier).

Then, build and install [`jetson-inference`](https://github.com/dusty-nv/jetson-inference)

```bash
$ cd ~
$ sudo apt-get install git cmake
$ git clone https://github.com/dusty-nv/jetson-inference
$ cd jetson-inference
$ git submodule update --init
$ mkdir build
$ cd build
$ cmake ../
$ make
$ sudo make install
```
Before proceeding, it's worthwhile to test that `jetson-inference` is working properly on your system by following this step of the Two Days to a Demo tutorial:
* [Running the Image Recognition Program on Jetson](https://github.com/dusty-nv/jetson-inference#using-the-console-program-on-jetson)

## ROS

Install the `ros-base` package on your Jetson following these directions:

* TX1/TX2 (ROS Kinetic) - [JetsonHacks Post](https://www.jetsonhacks.com/2018/04/27/robot-operating-system-ros-on-nvidia-jetson-tx-development-kits/)
* Xavier (ROS Melodic) - [ROS Install Instructions](http://wiki.ros.org/melodic/Installation/Ubuntu)

Then, create a Catkin workspace (`~/catkin_ws`) using these steps:

http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace

Depending on which Jetson you're using, install some additional dependencies:

#### TX1/TX2 (ROS Kinetic)
```bash
$ sudo apt-get install ros-kinetic-image-transport
$ sudo apt-get install ros-kinetic-image-publisher
$ sudo apt-get install ros-kinetic-vision-msgs
```
#### Xavier (ROS Melodic)
```bash
$ sudo apt-get install ros-melodic-image-transport
$ sudo apt-get install ros-melodic-image-publisher
$ sudo apt-get install ros-melodic-vision-msgs
```

## ros_deep_learning

Next, navigate into your Catkin workspace and clone and build `ros_deep_learning`:

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/dusty-nv/ros_deep_learning
$ cd ../
$ catkin_make
```

The inferencing nodes should now be built and ready to use.

# Testing

Before proceeding, make sure that `roscore` is running first:

```bash
$ roscore
```

## ImageNet

First, to stream some image data for the inferencing node to process, open another terminal and start an [`image_publisher`](http://wiki.ros.org/image_publisher), which loads a specified image from disk.  We tell it to load one of the test images that come with jetson-inference, but you can substitute your own images here as well:

```bash
$ rosrun image_publisher image_publisher __name:=image_publisher ~/jetson-inference/data/images/orange_0.jpg
```

Next, open a new terminal, overlay your Catkin workspace, and start the [`imagenet`](src/node_imagenet.cpp) node:

```bash
$ source ~/catkin_ws/devel/setup.bash
$ rosrun ros_deep_learning imagenet /imagenet/image_in:=/image_publisher/image_raw _model_name:=googlenet
```
Here, we remap imagenet's `image_in` input topic to the output of the `image_publisher`, and tell it to load the GoogleNet model using the node's `model_name` parameter.  You can substitute `alexnet` and `googlenet-12` here, with the `googlenet` model being loaded by default.

In another terminal, you should be able to verify the [`vision_msgs/Classification2D`](http://docs.ros.org/melodic/api/vision_msgs/html/msg/Classification2D.html) message output of the node, which is published to the `imagenet/classification` topic:

```bash
$ rostopic echo /imagenet/classification
```

## DetectNet

Kill the other nodes you launched above, and start publishing a new image with people in it for the [`detectnet`](src/node_detectnet.cpp) node to process:

```bash
$ rosrun image_publisher image_publisher __name:=image_publisher ~/jetson-inference/data/images/peds-004.jpg 
$ rosrun ros_deep_learning detectnet /detectnet/image_in:=/image_publisher/image_raw _model_name:=pednet
```

See [here](https://github.com/dusty-nv/jetson-inference#pretrained-detectnet-models-available) for the built-in detection models available.  Here's an example of launching with the model that detects dogs:

```bash
$ rosrun image_publisher image_publisher __name:=image_publisher ~/jetson-inference/data/images/dog_0.jpg
$ rosrun ros_deep_learning detectnet /detectnet/image_in:=/image_publisher/image_raw _model_name:=coco-dog
```

To inspect the [`vision_msgs/Detection2DArray`](http://docs.ros.org/melodic/api/vision_msgs/html/msg/Detection2DArray.html) message output of the node, subscribe to the `detectnet/detections` topic:

```bash
$ rostopic echo /detectnet/detections
```







