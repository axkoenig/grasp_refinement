#ifndef HAND_STATE_H
#define HAND_STATE_H

#include <ros/ros.h>
#include <reflex_msgs/Hand.h>

#include "finger_state.hpp"

class HandState
{
public:
    HandState(ros::NodeHandle *nh);
    ros::Subscriber state_sub;

    enum ContactState
    {
        NoContact,
        SingleFingerContact,
        MultipleFingerContact
    };

    ContactState getContactState() { return cur_state; };
    int countFingersInContact();
    int getFingerIdSingleContact();
    void callback(const reflex_msgs::Hand &msg);
    FingerState finger_states[3] = {FingerState(1), FingerState(2), FingerState(3)};

private:
    ContactState cur_state;
    const int num_fingers = 3;
    const int num_motors = 4;
};

#endif