#ifndef REFLEX_MOTOR_H
#define REFLEX_MOTOR_H

#include <ros/ros.h>
#include <control_msgs/JointControllerState.h>

class ReflexMotor
{
public:
    virtual float getAngle() { return 0.0; }
    virtual float getVelocity() { return 0.0; }
};

class ReflexFingerMotor : public ReflexMotor
{
private:
    int finger_id;
    ros::NodeHandle nh;

    ros::Subscriber proximal_sub;
    ros::Subscriber proximal_to_flex_sub;
    ros::Subscriber flex_to_distal_sub;

    float proximal_angle = 0.0;
    float proximal_to_flex_angle = 0.0;
    float flex_to_distal_angle = 0.0;

    float proximal_vel = 0.0;
    float proximal_to_flex_vel = 0.0;
    float flex_to_distal_vel = 0.0;

public:
    ReflexFingerMotor(int finger_id);
    void proximal_callback(const control_msgs::JointControllerState &msg);
    void proximal_to_flex_callback(const control_msgs::JointControllerState &msg);
    void flex_to_distal_callback(const control_msgs::JointControllerState &msg);
    virtual float getAngle() override;
    virtual float getVelocity() override;
};

class ReflexPreshapeMotor : public ReflexMotor
{
private:
    ros::NodeHandle nh;

    // in simulation there are two motors (on fingers 1 and 2)
    ros::Subscriber preshape_f1_sub;
    ros::Subscriber preshape_f2_sub;

    float preshape_1_angle = 0.0;
    float preshape_2_angle = 0.0;
    float preshape_1_vel = 0.0;
    float preshape_2_vel = 0.0;

public:
    ReflexPreshapeMotor();
    void preshape_1_callback(const control_msgs::JointControllerState &msg);
    void preshape_2_callback(const control_msgs::JointControllerState &msg);
    virtual float getAngle() override;
    virtual float getVelocity() override;
};

#endif