cmake_minimum_required(VERSION 2.8.3)
project(ros_deep_learning)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
find_package(jetson-inference REQUIRED)
find_package(CUDA REQUIRED)
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)
add_executable(hello_world src/ji_hw.cpp)
set_target_properties(hello_world PROPERTIES COMPILE_FLAGS "-std=c++11")
add_executable(imagenet_node src/imagenet_node.cpp)
set_target_properties(imagenet_node PROPERTIES COMPILE_FLAGS "-std=c++11")

target_link_libraries(hello_world
  ${catkin_LIBRARIES}
)
target_link_libraries(imagenet_node
  ${catkin_LIBRARIES}
  jetson-inference
)
