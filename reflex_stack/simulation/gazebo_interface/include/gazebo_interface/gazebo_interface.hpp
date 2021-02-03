#ifndef GAZEBO_INTERFACE_H
#define GAZEBO_INTERFACE_H

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>

using namespace std;

bool setModelPoseSim(ros::NodeHandle *nh, string model_name, tf2::Transform pose, bool verbose = true);

bool objectTouchesGround();

tf2::Transform getModelPoseSim(ros::NodeHandle *nh, string model_name, string relative_entity_name = "world", bool verbose = true);

tf2::Transform getLinkPoseSim(ros::NodeHandle *nh, string link_name, string reference_frame = "world", bool verbose = true);

template <typename T>
void getParam(ros::NodeHandle *nh, T *param, const string param_name)
{
    // wait for param_name on parameter server
    while (ros::ok())
    {
        if (nh->getParam(param_name, *param))
        {
            ROS_INFO_STREAM("Obtained " << param_name << ": " << *param << " from parameter server.");
            return;
        }
        else
        {
            ROS_WARN("Could not find parameter '%s' on parameter server.", param_name.c_str());
            ros::Duration(1.0).sleep();
        }
    }
}

#endif