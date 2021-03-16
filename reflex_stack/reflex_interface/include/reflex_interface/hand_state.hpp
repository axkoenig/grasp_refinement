#ifndef HAND_STATE_H
#define HAND_STATE_H

#include <vector>

#include <ros/ros.h>
#include <reflex_msgs/Hand.h>
#include <tf2_ros/transform_broadcaster.h>
#include <reflex_interface/TactileInfoStamped.h>

#include "reflex_interface/finger_state.hpp"
#include "reflex_interface/grasp_quality.hpp"

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
    std::vector<tf2::Vector3> getContactPositionsWorld() { return contact_positions_world; };
    std::vector<tf2::Vector3> getContactNormalsWorld() { return contact_normals_world; };
    std::vector<int> getNumSensorsInContactPerFinger() { return num_sensors_in_contact_per_finger; };
    std::vector<bool> getFingersInContact() { return fingers_in_contact; };
    bool allFingersInContact();
    const int num_fingers = 3;
    const int num_motors = 4;
    const int num_sensors_per_finger = 9;

private:
    ros::NodeHandle *nh;
    ros::Subscriber state_sub;
    ros::Publisher epsilon_pub;
    ros::Publisher epsilon_f_pub;
    ros::Publisher epsilon_t_pub;
    ros::Publisher num_contacts_pub;
    ros::Publisher tactile_poses_pub;
    GraspQuality grasp_quality = GraspQuality();

    std::string object_name;
    ContactState cur_state;
    bool use_sim_data_hand;
    bool use_sim_data_obj;
    std::vector<tf2::Vector3> contact_positions_world;
    std::vector<tf2::Vector3> contact_normals_world;
    std::vector<int> num_sensors_in_contact_per_finger = {0, 0, 0}; // example: two sensors in contact on finger 1 and one on finger 2: {2, 1, 0}
    std::vector<bool> fingers_in_contact = {0, 0, 0};               // example: fingers 1 and 3 in contact: {1, 0, 1}
    void callback(const reflex_msgs::Hand &msg);
    void updateHandStateWorldSim();
    void updateHandStateWorldReal();
    void publishTactilePoses();
    void broadcastModelState(tf2::Transform tf, std::string source_frame, std::string target_frame, tf2_ros::TransformBroadcaster *br);

    reflex_interface::TactileInfoStamped getTactilePosesMsg();
    tf2_ros::TransformBroadcaster br_reflex_measured;
    tf2_ros::TransformBroadcaster br_obj_measured;
};

#endif