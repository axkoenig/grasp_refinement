#include <ros/ros.h>
#include <reflex_msgs/Hand.h>

#include "sensor_listener/reflex_hand.hpp"

std::string node_name = "sensor_listener";
std::string topic_name = "reflex/hand_state";


int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<reflex_msgs::Hand>(topic_name, 1);
    ROS_INFO("Launched %s node.", node_name.c_str());

    ReflexHand hand;
    reflex_msgs::Hand msg;
    ros::Rate rate(50);

    ROS_INFO("Starting to listen to Gazebo sensor values.");
    ROS_INFO("Publishing to %s ...", topic_name.c_str());

    while (ros::ok())
    {
        // iterate over fingers
        for (int i = 0; i < hand.num_fingers; i++)
        {
            msg.finger[i].proximal = hand.fingers[i].getProximalAngle();
            msg.finger[i].distal_approx = hand.fingers[i].getDistalAngle();

            // iterate over sensors
            for (int j = 0; j < hand.num_sensors; j++)
            {
                msg.finger[i].pressure[j] = hand.fingers[i].sensors[j].getPressure();
                msg.finger[i].contact[j] = hand.fingers[i].sensors[j].getContact();
            }
        }

        // iterate over motors
        for (int i = 0; i < hand.num_motors; i++)
        {
            msg.motor[i].joint_angle = hand.motors[i]->getAngle();
            msg.motor[i].velocity = hand.motors[i]->getVelocity();
        }

        pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }
}