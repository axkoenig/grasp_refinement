#ifndef HAND_STATE_H
#define HAND_STATE_H

#include <vector>

#include <ros/ros.h>
#include <reflex_msgs/Hand.h>

#include "reflex_interface/finger_state.hpp"
#include "reflex_interface/grasp_quality.hpp"

class HandState
{
public:
    HandState(ros::NodeHandle *nh, bool use_sim_data_hand, bool use_sim_data_obj);
    ros::NodeHandle *nh;
    ros::Subscriber state_sub;
    ros::Publisher grasp_quality_pub;
    FingerState *finger_states[3];
    GraspQuality grasp_quality = GraspQuality();

    enum ContactState
    {
        NoContact,
        SingleFingerContact,
        MultipleFingerContact
    };

    ContactState getContactState();
    int getNumFingersInContact();
    int getFingerIdSingleContact();
    float getGraspQuality();                                // use this to get object c.o.m. from simulation 
    float getGraspQuality(tf2::Vector3 object_com_world);   // use this if you can estimate the object c.o.m. from e.g. computer vision  
    std::vector<tf2::Transform> getContactFramesWorld() { return contactFramesWorld; };
    std::vector<int> getContactFingerIds() { return contactFingerIds; };

private:
    ContactState cur_state;
    bool use_sim_data_hand;
    bool use_sim_data_obj;
    const int num_fingers = 3;
    const int num_motors = 4;
    std::vector<tf2::Transform> contactFramesWorld;
    std::vector<int> contactFingerIds; // example: two sensors in contact on finger 1 and one on finger 2: {1,1,2}
    void callback(const reflex_msgs::Hand &msg);
    void updateContactFramesWorldSim();
    void updateContactFramesWorldReal();
};

#endif