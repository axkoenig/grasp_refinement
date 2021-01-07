#ifndef HAND_STATE_H
#define HAND_STATE_H

#include <array>
#include <boost/array.hpp>

class FingerState
{
private:
    int finger_id;
    const int num_sensors = 9;
    std::array<bool, 9> sensor_contacts;
    std::array<float, 9> pressure;
    float proximal_angle;
    float distal_angle;

public:
    FingerState(int finger_id);
    void setProximalAngleFromMsg(float proximal_angle);
    void setDistalAngleFromMsg(float distal_angle);
    void setSensorContactsFromMsg(boost::array<unsigned char, 9> sensor_contacts);
    void setSensorPressureFromMsg(boost::array<float, 9> pressure);
    bool hasContact();
    bool hasProximalContact();
    bool hasDistalContact();
};

class HandState
{
public:
    enum State
    {
        NoContact,
        SingleFingerContact,
        MultipleFingerContact
    };
    State cur_state;
    const int num_fingers = 3;
    const int num_motors = 4;
    FingerState finger_states[3] = {FingerState(1), FingerState(2), FingerState(3)};

    void updateState();
    State getCurrentState();
    int countFingersInContact();
};

#endif