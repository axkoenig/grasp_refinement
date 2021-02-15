#include <vector>

#include "gazebo_interface/gazebo_interface.hpp"
#include "reflex_interface/hand_state.hpp"

HandState::HandState(ros::NodeHandle *nh, bool use_sim_data)
    : finger_states{new FingerState(nh, 1), new FingerState(nh, 2), new FingerState(nh, 3)}
{
    this->nh = nh;
    this->use_sim_data = use_sim_data;
    state_sub = nh->subscribe("reflex/hand_state", 1, &HandState::callback, this);
}

int HandState::getNumFingersInContact()
{
    int count = 0;
    for (int i = 0; i < num_fingers; i++)
    {
        if (finger_states[i]->hasContact() == true)
        {
            count++;
        }
    }
    return count;
}

int HandState::getFingerIdSingleContact()
{
    for (int i = 0; i < num_fingers; i++)
    {
        if (finger_states[i]->hasContact() == true)
        {
            return i;
        }
    }
    return -1;
}

void HandState::callback(const reflex_msgs::Hand &msg)
{
    for (int i = 0; i < num_fingers; i++)
    {
        finger_states[i]->setProximalAngleFromMsg(msg.finger[i].proximal);
        finger_states[i]->setDistalAngleFromMsg(msg.finger[i].distal_approx);
        finger_states[i]->setSensorContactsFromMsg(msg.finger[i].contact);
        finger_states[i]->setSensorPressuresFromMsg(msg.finger[i].pressure);

        if (i != 2)
        {
            // set preshape angle for fingers 1 and 2 (finger 3 doesn't have a preshape angle)
            finger_states[i]->setPreshapeAngleFromMsg(msg.motor[3].joint_angle / 2);
        }
    }
    use_sim_data ? updateContactFramesWorldSim() : updateContactFramesWorldReal();
}

HandState::ContactState HandState::getContactState()
{
    switch (getNumFingersInContact())
    {
    case 0:
        return NoContact;
    case 1:
        return SingleFingerContact;
    default:
        return MultipleFingerContact;
    }
}

void HandState::updateContactFramesWorldSim()
{
    // reset variables
    contactFramesWorld = {};
    contactFingerIds = {};

    // iterate over fingers
    for (int i = 0; i < num_fingers; i++)
    {
        std::vector<tf2::Transform> frames = finger_states[i]->getContactFramesWorldSim();

        if (!frames.empty())
        {
            contactFramesWorld.insert(contactFramesWorld.end(), frames.begin(), frames.end());
            contactFingerIds.push_back(i + 1);
        }
    }
}

void HandState::updateContactFramesWorldReal()
{
    // TODO: solve with forward kinematics and compare with results we obtain from simulation.
    // because we want this in world coosy we also need the gripper pose.
    throw "Not implemented.";
}

float HandState::getGraspQuality(tf2::Vector3 object_com_world, bool com_from_sim)
{
    // get object center of mass from simulation
    if (com_from_sim)
    {
        std::string object_name;
        getParam(nh, &object_name, "object_name");

        // assume com is object origin
        object_com_world = getModelPoseSim(nh, object_name, "world", false).getOrigin();
    }
    return grasp_quality.getEpsilon(contactFramesWorld, object_com_world);
}