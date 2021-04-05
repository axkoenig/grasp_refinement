#include <ros/ros.h>
#include <reflex_msgs/Hand.h>

#include "sensor_listener/ContactFrames.h"
#include "sensor_listener/reflex_hand.hpp"
#include "gazebo_interface/gazebo_interface.hpp"

std::string node_name = "sensor_listener";
std::string hand_state_topic = "reflex/hand_state";
std::string contact_frames_topic = "reflex/sim_contact_frames";

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::Publisher hand_pub = nh.advertise<reflex_msgs::Hand>(hand_state_topic, 1);
    ros::Publisher cf_pub = nh.advertise<sensor_listener::ContactFrames>(contact_frames_topic, 1);
    ROS_INFO("Launched %s node.", node_name.c_str());

    ReflexHand hand;
    reflex_msgs::Hand hand_msg;

    float reflex_tactile_sensor_rate;
    getParam(&nh, &reflex_tactile_sensor_rate, "reflex_tactile_sensor_rate");
    ros::Rate rate(reflex_tactile_sensor_rate);

    ROS_INFO("Starting to listen to Gazebo sensor values.");
    ROS_INFO_STREAM("Publishing to " << hand_state_topic << " and " << contact_frames_topic);

    while (ros::ok())
    {
        sensor_listener::ContactFrames cfs_msg;

        // iterate over fingers
        for (int i = 0; i < hand.num_fingers; i++)
        {
            hand_msg.finger[i].proximal = hand.fingers[i].getProximalAngle();
            hand_msg.finger[i].distal_approx = hand.fingers[i].getDistalAngle();

            // iterate over sensors
            for (int j = 0; j < hand.num_sensors; j++)
            {
                hand_msg.finger[i].pressure[j] = hand.fingers[i].sensors[j].getPressure();
                hand_msg.finger[i].contact[j] = hand.fingers[i].sensors[j].getContact();
            }

            int prox_size = hand.fingers[i].prox_contact_frames.size();
            int dist_size = hand.fingers[i].dist_contact_frames.size();
            cfs_msg.num_contact_frames += (prox_size + dist_size);

            // fill contact frames message with prox and distal contacts
            for (int j = 0; j < prox_size; j++)
            {
                cfs_msg.contact_frames.push_back(hand.fingers[i].prox_contact_frames[j]);
            }
            for (int j = 0; j < dist_size; j++)
            {
                cfs_msg.contact_frames.push_back(hand.fingers[i].dist_contact_frames[j]);
            }
        }

        // iterate over motors
        for (int i = 0; i < hand.num_motors; i++)
        {
            hand_msg.motor[i].joint_angle = hand.motors[i]->getAngle();
            hand_msg.motor[i].velocity = hand.motors[i]->getVelocity();
        }

        cfs_msg.header.frame_id = "world";
        cfs_msg.header.stamp = ros::Time::now();

        hand_pub.publish(hand_msg);
        cf_pub.publish(cfs_msg);
        ros::spinOnce();
        rate.sleep();
    }
}