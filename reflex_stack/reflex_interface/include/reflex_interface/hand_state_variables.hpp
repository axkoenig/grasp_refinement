#ifndef HAND_STATE_VARIABLES
#define HAND_STATE_VARIABLES

#include <vector>

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

class HandStateVariables
{
public:
    int num_contacts = 0;
    float epsilon = 0;
    float epsilon_force = 0;
    float epsilon_torque = 0;
    float delta_cur = 0;
    float delta_task = 0;
    float sum_contact_forces = 0;
    std::vector<int> num_sensors_in_contact_per_finger = {0, 0, 0}; // example: two sensors in contact on finger 1 and one on finger 2: {2, 1, 0}
    std::vector<bool> fingers_in_contact = {0, 0, 0};               // example: fingers 1 and 3 in contact: {1, 0, 1}

    // all variables in world frame!
    std::vector<tf2::Transform> contact_frames;
    std::vector<tf2::Vector3> contact_forces;
    std::vector<tf2::Vector3> contact_torques;
    std::vector<tf2::Vector3> contact_positions;
    std::vector<tf2::Vector3> contact_normals;
    std::vector<int> link_ids; // to which link does each contact belong (ranges from 0 to 3, 0 means palm contact, 1-3 are fingerIds)
    std::vector<int> sensor_ids; // to which sensor does each contact belong (each int ranges from 1 to 9)
    std::vector<float> contact_force_magnitudes;
    std::vector<float> contact_torque_magnitudes;

    void clear_all();
};

#endif