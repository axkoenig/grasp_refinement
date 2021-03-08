#include "sensor_listener/reflex_motor.hpp"

ReflexFingerMotor::ReflexFingerMotor(int finger_id)
{
    this->finger_id = finger_id;
    std::string finger_id_str = std::to_string(this->finger_id);

    std::string proximal_topic = "gazebo/finger_" + finger_id_str + "_proximal_position_controller/state";
    std::string proximal_to_flex_topic = "gazebo/finger_" + finger_id_str + "_proximal_to_flex_position_controller/state";
    std::string flex_to_distal_topic = "gazebo/finger_" + finger_id_str + "_flex_to_distal_position_controller/state";

    proximal_sub = nh.subscribe(proximal_topic, 1, &ReflexFingerMotor::proximal_callback, this);
    proximal_to_flex_sub = nh.subscribe(proximal_to_flex_topic, 1, &ReflexFingerMotor::proximal_to_flex_callback, this);
    flex_to_distal_sub = nh.subscribe(flex_to_distal_topic, 1, &ReflexFingerMotor::flex_to_distal_callback, this);
}

void ReflexFingerMotor::proximal_callback(const control_msgs::JointControllerState &msg)
{
    proximal_angle = msg.process_value;
    proximal_vel = msg.process_value_dot;
}

void ReflexFingerMotor::proximal_to_flex_callback(const control_msgs::JointControllerState &msg)
{
    proximal_to_flex_angle = msg.process_value;
    proximal_to_flex_vel = msg.process_value_dot;
}

void ReflexFingerMotor::flex_to_distal_callback(const control_msgs::JointControllerState &msg)
{
    flex_to_distal_angle = msg.process_value;
    flex_to_distal_vel = msg.process_value_dot;
}

float ReflexFingerMotor::getAngle()
{
    // assume motor angle is sum of all finger angles
    // TODO: compare with real reflex sensor readings
    return proximal_angle + proximal_to_flex_angle + flex_to_distal_angle;
}
float ReflexFingerMotor::getVelocity()
{
    // assume motor velocity is sum of all finger velocities
    // TODO: compare with real reflex sensor readings
    return proximal_vel + proximal_to_flex_vel + flex_to_distal_vel;
}

ReflexPreshapeMotor::ReflexPreshapeMotor()
{
    std::string preshape_f1_topic = "gazebo/finger_1_preshape_position_controller/state";
    std::string preshape_f2_topic = "gazebo/finger_2_preshape_position_controller/state";

    preshape_f1_sub = nh.subscribe(preshape_f1_topic, 1, &ReflexPreshapeMotor::preshape_1_callback, this);
    preshape_f2_sub = nh.subscribe(preshape_f2_topic, 1, &ReflexPreshapeMotor::preshape_2_callback, this);
}

void ReflexPreshapeMotor::preshape_1_callback(const control_msgs::JointControllerState &msg)
{
    preshape_1_angle = msg.process_value;
    preshape_1_vel = msg.process_value_dot;
}

void ReflexPreshapeMotor::preshape_2_callback(const control_msgs::JointControllerState &msg)
{
    preshape_2_angle = msg.process_value;
    preshape_2_vel = msg.process_value_dot;
}

float ReflexPreshapeMotor::getAngle()
{
    // assume motor angle is sum of both preshape motor angles
    // TODO: compare with real reflex sensor readings
    return preshape_1_angle + preshape_2_angle;
}

float ReflexPreshapeMotor::getVelocity()
{
    // assume motor angle is sum of both preshape motor velocities
    // TODO: compare with real reflex sensor readings
    return preshape_1_vel + preshape_2_vel;
}