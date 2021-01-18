#ifndef HELPERS_H
#define HELPERS_H

#include <ros/ros.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>

using namespace std;

bool setModelPoseSim(ros::NodeHandle *nh, string model_name, tf2::Transform pose, bool verbose = true);

void printPose(tf2::Transform pose, string name);

void logExperiment(ros::NodeHandle *nh,
                   int final_state,
                   float duration,
                   float pos_error[3],
                   float polar,
                   float azimuthal,
                   float offset,
                   std::string status_msg);

bool objectTouchesGround();

void moveObjectOutOfWay(ros::NodeHandle *nh, std::string &object_name, tf2::Transform &old_pose);

tf2::Transform getModelPoseSim(ros::NodeHandle *nh, string model_name, string relative_entity_name = "world", bool verbose = true);

tf2::Transform getLinkPoseSim(ros::NodeHandle *nh, string link_name, string reference_frame = "world", bool verbose = true);

tf2::Transform getTcpToWristFrame();

tf2::Transform calcInitWristPose(ros::NodeHandle *nh,
                                 float pos_error[3] = {},
                                 float polar = 0,
                                 float azimuthal = 0,
                                 float offset = 0.3,
                                 float z_rot = 0);


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