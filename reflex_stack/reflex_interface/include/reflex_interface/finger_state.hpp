#ifndef FINGER_STATE_H
#define FINGER_STATE_H

#include <array>
#include <vector>
#include <boost/array.hpp>

#include <ros/ros.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>

class FingerState
{
private:
    int finger_id;
    ros::NodeHandle nh;
    std::array<bool, 9> sensor_contacts;
    std::array<float, 9> sensor_pressures;
    float proximal_angle = 0.0;
    float distal_angle = 0.0;
    float preshape_angle = 0.0; // is only modified for fingers 1 and 2
    tf2::Vector3 proximal_normal = tf2::Vector3{0, 0, 0};
    tf2::Vector3 distal_normal = tf2::Vector3{0, 0, 0};
    tf2::Transform proximal_joint_frame;
    std::vector<tf2::Transform> contact_frames_world = {};
    void setProximalJointFrame();
    void updateNormalsFromMeasuredJointAngles();
    void updateNormalsExactSim();

public:
    const int num_sensors = 9;
    FingerState(ros::NodeHandle *nh, int finger_id);
    void setProximalAngleFromMsg(float angle);
    void setDistalAngleFromMsg(float angle);
    void setPreshapeAngleFromMsg(float angle);
    void setSensorContactsFromMsg(boost::array<unsigned char, 9> sensor_contacts);
    void setSensorPressuresFromMsg(boost::array<float, 9> sensor_pressures);
    std::array<bool, 9> getSensorContacts() { return sensor_contacts; };
    std::array<float, 9> getSensorPressures() { return sensor_pressures; };
    float getProximalAngle() { return proximal_angle; };
    float getDistalAngle() { return distal_angle; };
    float getPreshapeAngle() { return preshape_angle; };
    tf2::Vector3 getProximalNormal();
    tf2::Vector3 getDistalNormal();
    std::vector<tf2::Transform> getContactFramesWorldSim();
    bool hasContact();
    bool hasProximalContact();
    bool hasDistalContact();
};

#endif