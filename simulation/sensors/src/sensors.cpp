#include <string>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/ContactsState.h>
#include <reflex_msgs/Hand.h>

class ReflexSensor
{
private:
    ros::Subscriber sensor_sub;
    double total_force = 0.0;

public:
    ReflexSensor(ros::NodeHandle *nh, std::string topic)
    {
        sensor_sub = nh->subscribe(topic, 1, &ReflexSensor::callback, this);
    }

    void callback(const gazebo_msgs::ContactsState &msg)
    {
        std::string sensor_name = msg.header.frame_id;
        total_force = 0.0;

        if (!msg.states.empty())
        {
            // retrieve force components of first contact
            double fx = msg.states[0].total_wrench.force.x;
            double fy = msg.states[0].total_wrench.force.y;
            double fz = msg.states[0].total_wrench.force.z;

            // compute euclidean norm
            total_force = sqrt(pow(fx, 2) + pow(fy, 2) + pow(fz, 2));
        }
        ROS_INFO("[%s]: [%lf]", sensor_name.c_str(), total_force);
    }
};

// class ReflexFinger
// {
// private:
//     int finger_idx;

// public:
//     ReflexFinger(ros::NodeHandle *nh, int finger_idx)
//     {
//         finger_idx = finger_idx;
//         ReflexSensor proximal_sensor_1(nh, "gazebo/proximal_" + std::to_string(finger_idx) + "_sensor_1_bumper");
//         ReflexSensor proximal_sensor_2(nh, "gazebo/proximal_" + std::to_string(finger_idx) + "_sensor_2_bumper");
//         ReflexSensor proximal_sensor_3(nh, "gazebo/proximal_" + std::to_string(finger_idx) + "_sensor_3_bumper");
//         ReflexSensor proximal_sensor_4(nh, "gazebo/proximal_" + std::to_string(finger_idx) + "_sensor_4_bumper");
//         ReflexSensor proximal_sensor_5(nh, "gazebo/proximal_" + std::to_string(finger_idx) + "_sensor_5_bumper");

//         ReflexSensor distal_sensor_1(nh, "gazebo/distal_" + std::to_string(finger_idx) + "_sensor_1_bumper");
//         ReflexSensor distal_sensor_2(nh, "gazebo/distal_" + std::to_string(finger_idx) + "_sensor_2_bumper");
//         ReflexSensor distal_sensor_3(nh, "gazebo/distal_" + std::to_string(finger_idx) + "_sensor_3_bumper");
//         ReflexSensor distal_sensor_4(nh, "gazebo/distal_" + std::to_string(finger_idx) + "_sensor_4_bumper");
//     }
// };

// class ReflexHand
// {
// public:
//     ReflexHand(ros::NodeHandle *nh)
//     {   
//         ReflexFinger(nh, 1);
//         ReflexFinger(nh, 2);
//         ReflexFinger(nh, 3);
//     }
// };

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor");
    ros::NodeHandle nh;

    // TODO get classes ReflexHand and ReflexFinger to work

    // finger 1 
    ReflexSensor proximal_1_sensor_1 = ReflexSensor(&nh, "gazebo/proximal_1_sensor_1_bumper");
    ReflexSensor proximal_1_sensor_2 = ReflexSensor(&nh, "gazebo/proximal_1_sensor_2_bumper");
    ReflexSensor proximal_1_sensor_3 = ReflexSensor(&nh, "gazebo/proximal_1_sensor_3_bumper");
    ReflexSensor proximal_1_sensor_4 = ReflexSensor(&nh, "gazebo/proximal_1_sensor_4_bumper");
    ReflexSensor proximal_1_sensor_5 = ReflexSensor(&nh, "gazebo/proximal_1_sensor_5_bumper");
    ReflexSensor distal_1_sensor_1 = ReflexSensor(&nh, "gazebo/distal_1_sensor_1_bumper");
    ReflexSensor distal_1_sensor_2 = ReflexSensor(&nh, "gazebo/distal_1_sensor_2_bumper");
    ReflexSensor distal_1_sensor_3 = ReflexSensor(&nh, "gazebo/distal_1_sensor_3_bumper");
    ReflexSensor distal_1_sensor_4 = ReflexSensor(&nh, "gazebo/distal_1_sensor_4_bumper");

    // finger 2
    ReflexSensor proximal_2_sensor_1 = ReflexSensor(&nh, "gazebo/proximal_2_sensor_1_bumper");
    ReflexSensor proximal_2_sensor_2 = ReflexSensor(&nh, "gazebo/proximal_2_sensor_2_bumper");
    ReflexSensor proximal_2_sensor_3 = ReflexSensor(&nh, "gazebo/proximal_2_sensor_3_bumper");
    ReflexSensor proximal_2_sensor_4 = ReflexSensor(&nh, "gazebo/proximal_2_sensor_4_bumper");
    ReflexSensor proximal_2_sensor_5 = ReflexSensor(&nh, "gazebo/proximal_2_sensor_5_bumper");
    ReflexSensor distal_2_sensor_1 = ReflexSensor(&nh, "gazebo/distal_2_sensor_1_bumper");
    ReflexSensor distal_2_sensor_2 = ReflexSensor(&nh, "gazebo/distal_2_sensor_2_bumper");
    ReflexSensor distal_2_sensor_3 = ReflexSensor(&nh, "gazebo/distal_2_sensor_3_bumper");
    ReflexSensor distal_2_sensor_4 = ReflexSensor(&nh, "gazebo/distal_2_sensor_4_bumper");
    
    // finger 3
    ReflexSensor proximal_3_sensor_1 = ReflexSensor(&nh, "gazebo/proximal_3_sensor_1_bumper");
    ReflexSensor proximal_3_sensor_2 = ReflexSensor(&nh, "gazebo/proximal_3_sensor_2_bumper");
    ReflexSensor proximal_3_sensor_3 = ReflexSensor(&nh, "gazebo/proximal_3_sensor_3_bumper");
    ReflexSensor proximal_3_sensor_4 = ReflexSensor(&nh, "gazebo/proximal_3_sensor_4_bumper");
    ReflexSensor proximal_3_sensor_5 = ReflexSensor(&nh, "gazebo/proximal_3_sensor_5_bumper");
    ReflexSensor distal_3_sensor_1 = ReflexSensor(&nh, "gazebo/distal_3_sensor_1_bumper");
    ReflexSensor distal_3_sensor_2 = ReflexSensor(&nh, "gazebo/distal_3_sensor_2_bumper");
    ReflexSensor distal_3_sensor_3 = ReflexSensor(&nh, "gazebo/distal_3_sensor_3_bumper");
    ReflexSensor distal_3_sensor_4 = ReflexSensor(&nh, "gazebo/distal_3_sensor_4_bumper");

    ros::spin();
}