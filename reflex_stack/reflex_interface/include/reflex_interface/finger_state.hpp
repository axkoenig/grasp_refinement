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
public:
    FingerState(ros::NodeHandle *nh, int finger_id);
    const int num_sensors = 9;
    void setProximalAngleFromMsg(float angle);
    void setDistalAngleFromMsg(float angle);
    void setPreshapeAngleFromMsg(float angle);
    void setSensorContactsFromMsg(boost::array<unsigned char, 9> sensor_contacts);
    void setSensorPressuresFromMsg(boost::array<float, 9> sensor_pressures);
    float getProximalAngle() { return proximal_angle; };
    float getDistalAngle() { return distal_angle; };
    float getPreshapeAngle() { return preshape_angle; };
    tf2::Vector3 getProximalNormal();
    tf2::Vector3 getDistalNormal();
    void fillContactInfoWorldSim(std::vector<tf2::Vector3> &contact_positions_world, std::vector<tf2::Vector3> &contact_normals_world);
    std::array<bool, 9> getSensorContacts() { return sensor_contacts; };
    std::array<float, 9> getSensorPressures() { return sensor_pressures; };
    bool hasContact();
    bool hasProximalContact();
    bool hasDistalContact();

private:
    int finger_id;
    ros::NodeHandle nh;
    std::array<bool, 9> sensor_contacts;
    std::array<float, 9> sensor_pressures;
    float proximal_angle = 0.0;
    float distal_angle = 0.0;
    float preshape_angle = 0.0; // is only modified for fingers 1 and 2
    tf2::Vector3 prox_normal_in_shell_frame = tf2::Vector3{0, 0, 0};
    tf2::Vector3 dist_normal_in_shell_frame = tf2::Vector3{0, 0, 0};
    tf2::Transform proximal_joint_frame;
    tf2::Transform getProximalJointFrame();
    void updateFingerNormalsInShellFrame();
};

#endif