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
    std::array<bool, 9> getSensorContacts() { return sensor_contacts; };
    std::array<float, 9> getSensorPressures() { return sensor_pressures; };
    tf2::Vector3 getProximalNormalInShellFrame();
    tf2::Vector3 getDistalNormalInShellFrame();
    bool hasContact();
    bool hasProximalContact();
    bool hasDistalContact();
    void fillContactInfo(std::vector<tf2::Vector3> &contact_positions,
                         std::vector<tf2::Vector3> &contact_normals,
                         int &num_contacts_on_finger,
                         bool use_sim_data_hand,
                         std::string frame);
    std::vector<tf2::Vector3> getTactilePositionsInShellFrame();
    void updateCurLinkFramesInShellFrameReal();
    void updateCurLinkFramesInShellFrameSim();

private:
    int finger_id;
    ros::NodeHandle nh;
    std::array<bool, 9> sensor_contacts;
    std::array<float, 9> sensor_pressures;
    float proximal_angle = 0.0;
    float distal_angle = 0.0;
    float preshape_angle = 0.0; // is only modified for fingers 1 and 2

    tf2::Transform static_prox_link_in_shell_frame;
    tf2::Transform cur_prox_link_in_shell_frame = tf2::Transform::getIdentity();
    tf2::Transform cur_dist_link_in_shell_frame = tf2::Transform::getIdentity();

    tf2::Transform getStaticProximalJointFrame();
};

#endif