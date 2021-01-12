#ifndef HELPERS_H
#define HELPERS_H

#include <ros/ros.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>

using namespace std;

bool setModelPoseSim(ros::NodeHandle *nh, string model_name, tf2::Transform pose, bool verbose = true);

void printPose(tf2::Transform pose, string name);

template <typename T>
void getParam(ros::NodeHandle *nh, T *param, const string param_name);

void logExperiment(ros::NodeHandle *nh,
                   int final_state,
                   float duration,
                   float pos_error[3],
                   float polar,
                   float azimuthal,
                   float offset);


bool objectTouchesGround();

void moveObjectOutOfWay(ros::NodeHandle* nh, std::string &object_name, tf2::Transform &old_pose);

tf2::Transform getModelPoseSim(ros::NodeHandle *nh, string model_name, bool verbose = true);

tf2::Transform getLinkPoseSim(ros::NodeHandle *nh, string link_name, bool verbose = true);

tf2::Transform getTcpToWristFrame();

tf2::Transform calcInitWristPose(ros::NodeHandle *nh,
                                 float pos_error[3] = {},
                                 float polar = 0,
                                 float azimuthal = 0,
                                 float offset = 0.3);

#endif