#include "hand_state.hpp"

FingerState::FingerState(int finger_id)
{
    proximal_angle = 0.0;
    distal_angle = 0.0;
    this->finger_id = finger_id;
}

void FingerState::setProximalAngleFromMsg(float proximal_angle)
{
    this->proximal_angle = proximal_angle;
}

void FingerState::setDistalAngleFromMsg(float distal_angle)
{
    this->distal_angle = distal_angle;
}

void FingerState::setSensorContactsFromMsg(boost::array<unsigned char, 9> sensor_contacts)
{
    for (int i = 0; i < num_sensors; i++)
    {
        this->sensor_contacts[i] = sensor_contacts[i];
    }
}

void FingerState::setSensorPressureFromMsg(boost::array<float, 9> pressure)
{
    for (int i = 0; i < num_sensors; i++)
    {
        this->pressure[i] = pressure[i];
    }
}

bool FingerState::hasContact()
{
    for (int i = 0; i < num_sensors; i++)
    {
        if (sensor_contacts[i] == true)
        {
            return true;
        }
    }
    return false;
}

bool FingerState::hasProximalContact()
{
    for (int i = 0; i < 5; i++)
    {
        if (sensor_contacts[i] == true)
        {
            return true;
        }
    }
    return false;
}

bool FingerState::hasDistalContact()
{
    for (int i = 5; i < num_sensors; i++)
    {
        if (sensor_contacts[i] == true)
        {
            return true;
        }
    }
    return false;
}

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

HandState::State HandState::getCurrentState()
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