
cmake_minimum_required(VERSION 3.5)
project(ros_deep_learning)

# locate dependencies
find_package(jetson-utils REQUIRED)
find_package(jetson-inference REQUIRED)

find_package(CUDA REQUIRED)

# detect ROS1 vs ROS2
find_package(catkin QUIET)

if( catkin_FOUND )
	message("detected ROS1 (catkin_make)")

	find_package(catkin REQUIRED COMPONENTS
		image_transport
		roscpp
		sensor_msgs
		vision_msgs
		std_msgs
	)
	 
	catkin_package(
	    LIBRARIES ros_deep_learning_nodelets
	    CATKIN_DEPENDS nodelet roscpp image_transport sensor_msgs
	)

	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")	# enable c++11 (TensorRT requirement)
	include_directories(${catkin_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
	add_definitions(-DROS1)

else()
	find_package(ament_cmake QUIET)

	if( ament_cmake_FOUND )
		message("detected ROS2 (ament_cmake)")

		find_package(rclcpp REQUIRED)
		find_package(std_msgs REQUIRED)
		find_package(sensor_msgs REQUIRED)
		find_package(vision_msgs REQUIRED)

		set(CMAKE_CXX_STANDARD 14)
		include_directories(${CUDA_INCLUDE_DIRS})
		add_definitions(-DROS2)

	else()
		message(FATAL_ERROR "could not find either ROS1 (catkin_make) or ROS2 (ament_cmake) packages")
	endif()
endif()


# inference nodes
add_executable(imagenet src/node_imagenet.cpp src/image_converter.cpp)
target_link_libraries(imagenet ${catkin_LIBRARIES} jetson-inference)

if( ament_cmake_FOUND )
	ament_target_dependencies(imagenet rclcpp std_msgs sensor_msgs vision_msgs)
endif()


#add_executable(detectnet src/node_detectnet.cpp src/image_converter.cpp)
#target_link_libraries(detectnet ${catkin_LIBRARIES} jetson-inference)

#add_executable(segnet src/node_segnet.cpp src/image_converter.cpp)
#target_link_libraries(segnet ${catkin_LIBRARIES} jetson-inference)


# the library we are going to compile (has all of the nodelets in it)
#add_library(ros_deep_learning_nodelets src/nodelet_imagenet.cpp src/image_converter.cpp)
#target_link_libraries(ros_deep_learning_nodelets ${catkin_LIBRARIES} jetson-inference)


#if(catkin_EXPORTED_LIBRARIES)
#    add_dependencies(ros_deep_learning_nodelets ${catkin_EXPORTED_LIBRARIES})
#endif()

if( ament_cmake_FOUND )
	ament_package()
endif()


