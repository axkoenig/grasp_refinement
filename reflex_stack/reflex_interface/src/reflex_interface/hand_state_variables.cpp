#include "reflex_interface/hand_state_variables.hpp"

void HandStateVariables::clear_all()
{
    num_contacts = epsilon = epsilon_force = epsilon_torque = 0;
    num_sensors_in_contact_per_finger = {0, 0, 0};
    fingers_in_contact = {0, 0, 0};
    contact_frames.clear();
    contact_forces.clear();
    contact_torques.clear();
    contact_positions.clear();
    contact_normals.clear();
}