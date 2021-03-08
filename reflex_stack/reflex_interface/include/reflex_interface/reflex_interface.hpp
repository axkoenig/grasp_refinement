#ifndef REFLEX_INTERFACE_H
#define REFLEX_INTERFACE_H

#include "reflex_interface/hand_command.hpp"
#include "reflex_interface/hand_state.hpp"

class ReflexInterface
{
public:

    // if use_sim_data_hand true, we measure hand state directly from simulation
    // else we compute hand state on basis of info that we would also be available 
    // from real Reflex hand (i.e. flexure angles). inaccuracies may be higher.

    // if use_sim_data_obj true, we obtain the object center of mass from sim
    // else you will need to provide the center of mass e.g. from computer vision
    // so far this only matters when computing grasp quality
    
    ReflexInterface(ros::NodeHandle *nh, bool use_sim_data_hand, bool use_sim_data_obj)
        : state(new HandState(nh, use_sim_data_hand, use_sim_data_obj)),
          command(new HandCommand(nh, state))
    {
    }
    HandState *state;
    HandCommand *command;
};
#endif