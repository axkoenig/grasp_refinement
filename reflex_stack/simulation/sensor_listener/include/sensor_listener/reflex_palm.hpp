#ifndef REFLEX_PALM_H
#define REFLEX_PALM_H

#include <ros/ros.h>
#include <tf2/LinearMath/Vector3.h>

#include "sensor_listener/ContactFrame.h"
#include "sensor_listener/reflex_sensor.hpp"

class ReflexPalm
{
private:
    ros::NodeHandle nh;
    std::string sensor_link_name = "pad";
    ros::Subscriber sensor_link_sub;

    float ignore_force_thresh = 1e-4;
    void contacts_callback(const gazebo_msgs::ContactsState &msg);

public:
    tf2::Vector3 create_vec_from_msg(const geometry_msgs::Vector3 &msg);
    std::vector<sensor_listener::ContactFrame> contact_frames;
    ReflexPalm();
};
#endif