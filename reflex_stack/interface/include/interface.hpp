#ifndef INTERFACE_H
#define INTERFACE_H

#include "hand_command.hpp"
#include "hand_state.hpp"

class ReflexInterface
{
private:
    ros::NodeHandle nh;

public:
    HandState hand_state = HandState();
    HandCommand hand_command = HandCommand(&nh);
};
#endif