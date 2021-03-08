#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include "gazebo_interface/gazebo_interface.hpp"
#include "sensor_listener/reflex_finger.hpp"

ReflexFinger::ReflexFinger(int finger_id)
{
    this->finger_id = finger_id;
    std::string finger_id_str = std::to_string(this->finger_id);

    std::string proximal_topic = "gazebo/finger_" + finger_id_str + "_proximal_position_controller/state";
    std::string proximal_to_flex_topic = "gazebo/finger_" + finger_id_str + "_proximal_to_flex_position_controller/state";
    std::string flex_to_distal_topic = "gazebo/finger_" + finger_id_str + "_flex_to_distal_position_controller/state";
    proximal_sensor_link_name = "gazebo/proximal_" + finger_id_str;
    distal_sensor_link_name = "gazebo/distal_" + finger_id_str;
    std::string proximal_sensor_link_topic = proximal_sensor_link_name + "_sensor_bumper";
    std::string distal_sensor_link_topic = distal_sensor_link_name + "_sensor_bumper";

    proximal_sub = nh.subscribe(proximal_topic, 1, &ReflexFinger::proximal_callback, this);
    proximal_to_flex_sub = nh.subscribe(proximal_to_flex_topic, 1, &ReflexFinger::proximal_to_flex_callback, this);
    flex_to_distal_sub = nh.subscribe(flex_to_distal_topic, 1, &ReflexFinger::flex_to_distal_callback, this);
    proximal_sensor_link_sub = nh.subscribe(proximal_sensor_link_topic, 1, &ReflexFinger::proximal_contacts_callback, this);
    distal_sensor_link_sub = nh.subscribe(distal_sensor_link_topic, 1, &ReflexFinger::distal_contacts_callback, this);
}

void ReflexFinger::proximal_callback(const control_msgs::JointControllerState &msg)
{
    proximal_angle = msg.process_value;
}

void ReflexFinger::proximal_to_flex_callback(const control_msgs::JointControllerState &msg)
{
    proximal_to_flex_angle = msg.process_value;
}

void ReflexFinger::flex_to_distal_callback(const control_msgs::JointControllerState &msg)
{
    flex_to_distal_angle = msg.process_value;
}

float ReflexFinger::getProximalAngle()
{
    return proximal_angle;
}

float ReflexFinger::getDistalAngle()
{
    // note that (as with the real reflex hand) this is a rough approximation
    // we take the average of both flexure joints
    // TODO: compare with real reflex sensor readings
    return (proximal_to_flex_angle + flex_to_distal_angle) / 2;
}

void ReflexFinger::proximal_contacts_callback(const gazebo_msgs::ContactsState &msg)
{
    // get pose of proximal link
    tf2::Transform prox_link_pose = getLinkPoseSim(&nh, proximal_sensor_link_name, "world", false);

    // iterate over contacts

    // multiply all contacts positions with inverse transform

    // assign sensor by contact position
}

void ReflexFinger::distal_contacts_callback(const gazebo_msgs::ContactsState &msg)
{
    tf2::Transform dist_link_pose = getLinkPoseSim(&nh, distal_sensor_link_name, "world", false);
}