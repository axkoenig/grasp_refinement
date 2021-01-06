#ifndef BASELINE_CONTROLLER_H
#define BASELINE_CONTROLLER_H

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <reflex_msgs/Hand.h>

#include "hand_state.hpp"

using namespace std;

string source_frame = "world";
string target_frame = "reflex";

string open_srv_name = "reflex/open";
string sph_open_srv_name = "reflex/spherical_open";
string sph_close_srv_name = "reflex/spherical_close";
string state_topic_name = "reflex/hand_state";

class BaselineController
{
private:
    ros::NodeHandle *nh;
    std_srvs::Trigger trigger;
    ros::ServiceClient open_client;
    ros::ServiceClient sph_open_client;
    ros::ServiceClient sph_close_client;
    ros::Subscriber reflex_state_sub;

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped ts;
    tf2::Transform desired_pose, init_wrist_pose, waypoint, goal_wrist_pose;

    float backoff_factor = 1.0;
    float step_size = 0.001;
    bool grasped = false;
    bool finished = false;
    HandState hand_state = HandState();

public:
    BaselineController(ros::NodeHandle *nh, tf2::Transform init_wrist_pose, tf2::Transform goal_wrist_pose, bool simulation_only);

    bool isFinished() { return finished; }

    void moveToInitPoseSim();

    void moveToInitPoseReal();

    void waitUntilReachedPoseSim(tf2::Transform desired_pose, string name);

    bool reachedPoseSim(tf2::Transform desired_pose, float position_thresh = 0.01);

    void callbackHandState(const reflex_msgs::Hand &msg);
    
    void moveAlongVector(tf2::Vector3 vector);

    void sendTransform(tf2::Transform transform);

    void timeStep();
};

#endif