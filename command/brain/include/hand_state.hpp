#ifndef HAND_STATE_H
#define HAND_STATE_H

#include <array>
#include <boost/array.hpp>

#include <tf2/LinearMath/Vector3.h>
#include <reflex_msgs/Hand.h>

class FingerState
{
private:
    int finger_id;
    const int num_sensors = 9;
    std::array<bool, 9> sensor_contacts;
    std::array<float, 9> sensor_pressure;
    float proximal_angle = 0.0;
    float distal_angle = 0.0;
    float preshape_angle = 0.0;
    tf2::Vector3 proximal_normal;
    tf2::Vector3 distal_normal;
    void updateNormals();

public:
    FingerState(int finger_id);
    void setProximalAngleFromMsg(float angle);
    void setDistalAngleFromMsg(float angle);
    void setPreshapeAngleFromMsg(float angle);
    void setSensorContactsFromMsg(boost::array<unsigned char, 9> sensor_contacts);
    void setSensorPressureFromMsg(boost::array<float, 9> sensor_pressure);
    bool hasContact();
    bool hasProximalContact();
    bool hasDistalContact();
    tf2::Vector3 getProximalNormal();
    tf2::Vector3 getDistalNormal();
};

class HandState
{
public:
    enum ContactState
    {
        NoContact,
        SingleFingerContact,
        MultipleFingerContact
    };
    
    ContactState getContactState();
    int countFingersInContact();
    void setFingerStateFromMsg(const reflex_msgs::Hand &msg);

private:
    ContactState cur_state;
    void updateState();
    const int num_fingers = 3;
    const int num_motors = 4;
    FingerState finger_states[3] = {FingerState(1), FingerState(2), FingerState(3)};

};

#endif