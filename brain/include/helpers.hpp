#ifndef HELPERS_H
#define HELPERS_H

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>

using namespace std;

void printPose(tf2::Transform pose, string name);

void logExperiment(ros::NodeHandle *nh,
                   int final_state,
                   float duration,
                   float obj_error[3],
                   float reflex_error[3],
                   float polar,
                   float azimuthal,
                   float offset,
                   std::string status_msg);

void moveObjectOutOfWay(ros::NodeHandle *nh, std::string &object_name, tf2::Transform &old_pose);

tf2::Transform getTcpToWristFrame();

tf2::Transform calcInitWristPose(ros::NodeHandle *nh,
                                 float obj_error[3] = {},
                                 float reflex_error[3] = {},
                                 float polar = 0,
                                 float azimuthal = 0,
                                 float offset = 0.3,
                                 float z_rot = 0);

#endif