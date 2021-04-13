#ifndef REFLEX_FINGER_H
#define REFLEX_FINGER_H

#include <ros/ros.h>
#include <control_msgs/JointControllerState.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>

#include "sensor_listener/ContactFrame.h"
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
    int num_prox_sensors = 5;
    int num_dist_sensors = 4;

    float ignore_force_thresh = 1e-4;
    float prox_sensor_boundaries[4] = {};
    float dist_sensor_boundaries[3] = {};
    int which_sensor(const float &contact_x, const float sensor_boundaries[], const int &num_boundaries);

    // values taken from URDF
    float len_prox_pad = 0.052;
    float len_dist_pad = 0.0385;
    tf2::Vector3 prox_link_to_prox_pad_origin = tf2::Vector3{0.035, 0, 0.004};
    tf2::Vector3 dist_link_to_dist_pad_origin = tf2::Vector3{0.01875, 0, 0.007};
    void proximal_contacts_callback(const gazebo_msgs::ContactsState &msg);
    void distal_contacts_callback(const gazebo_msgs::ContactsState &msg);
    void eval_contacts_callback(const gazebo_msgs::ContactsState &msg,
                                std::vector<sensor_listener::ContactFrame> &contact_frames,
                                const int &first_sensor_idx,
                                const int &num_sensors_on_link,
                                const float sensor_boundaries[],
                                const tf2::Transform &world_to_link,
                                const tf2::Vector3 &link_to_pad_origin);
    void proximal_callback(const control_msgs::JointControllerState &msg);
    void proximal_to_flex_callback(const control_msgs::JointControllerState &msg);
    void flex_to_distal_callback(const control_msgs::JointControllerState &msg);

public:
    ReflexSensor sensors[9];
    std::vector<sensor_listener::ContactFrame> prox_contact_frames;
    std::vector<sensor_listener::ContactFrame> dist_contact_frames;
    ReflexFinger(const int &finger_id);
    float getProximalAngle();
    float getDistalAngle();
    tf2::Vector3 create_vec_from_msg(const geometry_msgs::Vector3 &msg);
};

#endif