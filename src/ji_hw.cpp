#include <ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "hello_world");
    ROS_INFO("hello world\n");
    return 0;
}
