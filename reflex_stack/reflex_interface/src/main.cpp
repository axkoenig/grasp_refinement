
#include <ros/ros.h>

#include "reflex_interface/reflex_interface.hpp"

std::string node_name = "reflex_interface_node";

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ROS_INFO("Launched %s.", node_name.c_str());
    ReflexInterface ri = ReflexInterface(&nh);
    ros::spin();
}