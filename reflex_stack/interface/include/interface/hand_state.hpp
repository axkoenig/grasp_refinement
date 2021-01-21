#ifndef HAND_STATE_H
#define HAND_STATE_H

#include <reflex_msgs/Hand.h>

#include "finger_state.hpp"

class HandState
{
public:
    enum ContactState
    {
        NoContact,
        SingleFingerContact,
        MultipleFingerContact
    };
    
    ContactState getContactState();
    int countFingersInContact();
    int getFingerIdSingleContact();
    void setFingerStateFromMsg(const reflex_msgs::Hand &msg);
    FingerState finger_states[3] = {FingerState(1), FingerState(2), FingerState(3)};

private:
    ContactState cur_state;
    void updateState();
    const int num_fingers = 3;
    const int num_motors = 4;
};

#endif