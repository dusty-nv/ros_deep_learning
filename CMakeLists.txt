cmake_minimum_required(VERSION 2.8.3)
project(ros_deep_learning)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
find_package(jetson-utils REQUIRED)
find_package(jetson-inference REQUIRED)
find_package(CUDA REQUIRED)
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

catkin_package(
    LIBRARIES ros_deep_learning_nodelets
    CATKIN_DEPENDS nodelet roscpp cv_bridge image_transport sensor_msgs
)

# the library we are going to compile (has all of the nodelets in it)
add_library(ros_deep_learning_nodelets src/nodelet_imagenet.cpp)
# includes
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)
# compile option
set_target_properties(ros_deep_learning_nodelets PROPERTIES COMPILE_FLAGS "-std=c++11")
# link agains these objects
target_link_libraries(ros_deep_learning_nodelets
  ${catkin_LIBRARIES}
  jetson-inference
)

#if(catkin_EXPORTED_LIBRARIES)
#    add_dependencies(ros_deep_learning_nodelets ${catkin_EXPORTED_LIBRARIES})
#endif()
