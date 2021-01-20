
#include <ros/ros.h>

#include "interface.hpp"

std::string node_name = "interface_node";

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ROS_INFO("Launched %s.", node_name.c_str());
    ReflexInterface ri = ReflexInterface();
    ros::spin();
}