
#include <ros/ros.h>

#include "reflex_interface/interface.hpp"

std::string node_name = "reflex_interface_node";

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ROS_INFO("Launched %s.", node_name.c_str());
    ReflexInterface ri = ReflexInterface();
    ros::spin();
}