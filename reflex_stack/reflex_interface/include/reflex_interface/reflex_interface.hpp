#ifndef INTERFACE_H
#define INTERFACE_H

#include "hand_command.hpp"
#include "hand_state.hpp"

class ReflexInterface
{
public:
    // if use_sim_data true, we use exact data from simulation
    // else we compute hand state on basis of info that we could
    // also obtain from real Reflex hand. inaccuracies may be higher.

    ReflexInterface(ros::NodeHandle *nh, bool use_sim_data)
        : state(new HandState(nh, use_sim_data)),
          command(new HandCommand(nh))
    {
    }
    HandState *state;
    HandCommand *command;
};
#endif