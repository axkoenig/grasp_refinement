#ifndef INTERFACE_H
#define INTERFACE_H

#include "hand_command.hpp"

class ReflexInterface
{
private:
    ros::NodeHandle nh;

public:
    HandCommand hand_command = HandCommand(&nh);
};
#endif