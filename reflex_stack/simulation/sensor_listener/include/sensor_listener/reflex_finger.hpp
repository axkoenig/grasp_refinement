#ifndef REFLEX_FINGER_H
#define REFLEX_FINGER_H

#include <ros/ros.h>
#include <control_msgs/JointControllerState.h>

#include "sensor_listener/reflex_sensor.hpp"

class ReflexFinger
{
private:
    int finger_id;
    ros::NodeHandle nh;

    ros::Subscriber proximal_sub;
    ros::Subscriber proximal_to_flex_sub;
    ros::Subscriber flex_to_distal_sub;
    ros::Subscriber proximal_sensor_link_sub;
    ros::Subscriber distal_sensor_link_sub;

    std::string proximal_sensor_link_name;
    std::string distal_sensor_link_name;

    float proximal_angle = 0.0;
    float proximal_to_flex_angle = 0.0;
    float flex_to_distal_angle = 0.0;

public:
    ReflexSensor sensors[9];

    ReflexFinger(int finger_id);
    void proximal_callback(const control_msgs::JointControllerState &msg);
    void proximal_to_flex_callback(const control_msgs::JointControllerState &msg);
    void flex_to_distal_callback(const control_msgs::JointControllerState &msg);
    float getProximalAngle();
    float getDistalAngle();
    void proximal_contacts_callback(const gazebo_msgs::ContactsState &msg);
    void distal_contacts_callback(const gazebo_msgs::ContactsState &msg);
};

#endif