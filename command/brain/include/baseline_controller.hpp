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
public:
    BaselineController(ros::NodeHandle *nh, tf2::Transform init_wrist_pose, tf2::Transform goal_wrist_pose, bool simulation_only, float time_out);

    bool isFinished() const { return finished; }

    void moveToInitPoseSim();

    void moveToInitPoseReal();

    void waitUntilWristReachedPoseSim(tf2::Transform desired_pose, string name);

    bool reachedPoseSim(tf2::Transform desired_pose, float position_thresh = 0.02, float rotation_thresh = 0.03);

    void callbackHandState(const reflex_msgs::Hand &msg);

    void moveAlongVector(tf2::Vector3 vec);

    tf2::Vector3 getApproachDirectionSingleContact();

    void sendWristTransform(tf2::Transform transform);

    void timeStep();

    enum State
    {
        Failure = -1,            // something went wrong
        NotGrasped = 0,          // did not make grasping attempt yet
        GraspedButNotLifted = 1, // made grasping attempt
        GraspedAndLifted = 2,    // made grasping attempt and lifted object from floor
        GraspedAndInGoalPose = 3 // made grasping attempt, moved wrist to goal pose and object in hand
    };

    State getState() const { return state; }
    ros::Time getStartTime() const { return start_time; }
    std::string getStatusMsg() const { return status_msg; }

private:
    float backoff_factor = 1.0;
    float step_size = 0.001;
    float time_out;
    bool grasped = false;
    bool finished = false;
    std::string status_msg = "All good.";
    State state = NotGrasped;
    HandState hand_state = HandState();

    ros::NodeHandle *nh;
    std_srvs::Trigger trigger;
    ros::ServiceClient open_client;
    ros::ServiceClient sph_open_client;
    ros::ServiceClient sph_close_client;
    ros::Subscriber reflex_state_sub;
    ros::Time start_time;

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped ts;
    tf2::Transform desired_pose, init_wrist_pose, goal_wrist_pose, init_object_pose;
    tf2::Vector3 step_reflex_z = tf2::Vector3{0, 0, step_size};

    void checkTimeOut();
    void resetWorldSim();
    void stopExperiment();
};

#endif