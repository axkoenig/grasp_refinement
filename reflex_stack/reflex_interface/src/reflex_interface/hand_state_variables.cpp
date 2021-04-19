#include "reflex_interface/hand_state_variables.hpp"

void HandStateVariables::clear_all()
{
    num_contacts = epsilon = epsilon_force = epsilon_torque = delta_cur = delta_task = sum_contact_forces = 0;
    num_sensors_in_contact_per_finger = {0, 0, 0};
    fingers_in_contact = {0, 0, 0};
    contact_frames.clear();
    contact_forces.clear();
    contact_torques.clear();
    contact_positions.clear();
    contact_normals.clear();
    finger_ids.clear();
    sensor_ids.clear(); // to which sensor does each contact belong (each int ranges from 1 to 9)
    contact_force_magnitudes.clear();
    contact_torque_magnitudes.clear();
}