#ifndef INTERFACE_H
#define INTERFACE_H

#include "hand_command.hpp"
#include "hand_state.hpp"

class ReflexInterface
{
public:
    ReflexInterface(ros::NodeHandle *nh)
        : state(new HandState(nh, true)),
          command(new HandCommand(nh))
    {
    }
    HandState *state;
    HandCommand *command;
};
#endif