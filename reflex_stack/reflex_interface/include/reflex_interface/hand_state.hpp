#ifndef HAND_STATE_H
#define HAND_STATE_H

#include <vector>

#include <ros/ros.h>
#include <reflex_msgs/Hand.h>
#include <tf2_ros/transform_broadcaster.h>
#include <reflex_interface/HandStateStamped.h>

#include "reflex_interface/finger_state.hpp"
#include "reflex_interface/grasp_quality.hpp"

class HandStateVariables
{
public:
    int num_contacts = 0;
    float epsilon = 0;
    float epsilon_force = 0;
    float epsilon_torque = 0;
    std::vector<int> num_sensors_in_contact_per_finger = {0, 0, 0}; // example: two sensors in contact on finger 1 and one on finger 2: {2, 1, 0}
    std::vector<bool> fingers_in_contact = {0, 0, 0};               // example: fingers 1 and 3 in contact: {1, 0, 1}

    // all variables in world frame!
    std::vector<tf2::Transform> contact_frames;
    std::vector<tf2::Vector3> contact_forces;
    std::vector<tf2::Vector3> contact_torques;
    std::vector<tf2::Vector3> contact_positions;
    std::vector<tf2::Vector3> contact_normals;
};

class HandState
{
public:
    HandState(ros::NodeHandle *nh, bool use_sim_data_hand, bool use_sim_data_obj);
    FingerState *finger_states[3];

    enum ContactState
    {
        NoContact,
        SingleFingerContact,
        MultipleFingerContact
    };

    ContactState getContactState();
    int getNumFingersInContact();
    int getFingerIdSingleContact();
    float getEpsilon(tf2::Vector3 object_com_world);
    void fillEpsilonFTSeparate(tf2::Vector3 object_com_world, float &epsilon_force, float &epsilon_torque);
    HandStateVariables getVars() { return vars; };
    bool allFingersInContact();
    const int num_fingers = 3;
    const int num_motors = 4;
    const int num_sensors_per_finger = 9;

private:
    ros::NodeHandle *nh;
    ros::Subscriber state_sub;
    ros::Publisher hand_state_pub;
    GraspQuality grasp_quality = GraspQuality();
    HandStateVariables vars = HandStateVariables();
    std::string object_name;
    ContactState cur_state;
    bool use_sim_data_hand;
    bool use_sim_data_obj;

    void reflex_callback(const reflex_msgs::Hand &msg);
    void updateHandStateWorldSim();
    void updateHandStateWorldReal();
    void broadcastModelState(tf2::Transform tf, std::string source_frame, std::string target_frame, tf2_ros::TransformBroadcaster *br);

    reflex_interface::HandStateStamped getHandStateMsg();
    tf2_ros::TransformBroadcaster br_reflex_measured;
    tf2_ros::TransformBroadcaster br_obj_measured;
};

#endif