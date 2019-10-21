
cmake_minimum_required(VERSION 2.8.3)
project(ros_deep_learning)

# locate dependencies
find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  sensor_msgs
  vision_msgs
  std_msgs
)

find_package(jetson-utils REQUIRED)
find_package(jetson-inference REQUIRED)

find_package(CUDA REQUIRED)
#find_package(Qt4 REQUIRED)

#include(${QT_USE_FILE})
#add_definitions(${QT_DEFINITIONS})


# define catkin package
catkin_package(
    LIBRARIES ros_deep_learning_nodelets
    CATKIN_DEPENDS nodelet roscpp image_transport sensor_msgs
)

# include paths
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

# enable c++11 (TensorRT requirement)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


# inference nodes
add_executable(imagenet src/node_imagenet.cpp src/image_converter.cpp)
target_link_libraries(imagenet ${catkin_LIBRARIES} jetson-inference)

add_executable(detectnet src/node_detectnet.cpp src/image_converter.cpp)
target_link_libraries(detectnet ${catkin_LIBRARIES} jetson-inference)

add_executable(segnet src/node_segnet.cpp src/image_converter.cpp)
target_link_libraries(segnet ${catkin_LIBRARIES} jetson-inference)


# the library we are going to compile (has all of the nodelets in it)
add_library(ros_deep_learning_nodelets src/nodelet_imagenet.cpp src/image_converter.cpp)
target_link_libraries(ros_deep_learning_nodelets ${catkin_LIBRARIES} jetson-inference)


#if(catkin_EXPORTED_LIBRARIES)
#    add_dependencies(ros_deep_learning_nodelets ${catkin_EXPORTED_LIBRARIES})
#endif()

