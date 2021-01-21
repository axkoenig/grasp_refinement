
#include "reflex_interface/hand_state.hpp"

void HandState::updateState()
{
    int contact_count = countFingersInContact();
    switch (contact_count)
    {
    case 0:
        cur_state = NoContact;
        break;
    case 1:
        cur_state = SingleFingerContact;
        break;
    default:
        cur_state = MultipleFingerContact;
        break;
    }
}

HandState::ContactState HandState::getContactState()
{
    updateState();
    return cur_state;
}

int HandState::countFingersInContact()
{
    int count = 0;
    for (int i = 0; i < num_fingers; i++)
    {
        if (finger_states[i].hasContact() == true)
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
        if (finger_states[i].hasContact() == true)
        {
            return i;
        }
    }
    return -1;
}

void HandState::setFingerStateFromMsg(const reflex_msgs::Hand &msg)
{
    for (int i = 0; i < num_fingers; i++)
    {
        finger_states[i].setProximalAngleFromMsg(msg.finger[i].proximal);
        finger_states[i].setDistalAngleFromMsg(msg.finger[i].distal_approx);
        finger_states[i].setSensorContactsFromMsg(msg.finger[i].contact);
        finger_states[i].setSensorPressureFromMsg(msg.finger[i].pressure);

        if (i != 2)
        {
            // set preshape angle for fingers 1 and 2 (finger 3 doesn't have a preshape angle)
            finger_states[i].setPreshapeAngleFromMsg(msg.motor[3].joint_angle / 2);
        }
    }
}