#include <string>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/ContactsState.h>
#include <reflex_msgs/Hand.h>

std::string node_name = "debug_listener";
std::string ns = "gazebo";

class ReflexLink
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sensor_sub;
    ros::Publisher force_pub;
    ros::Publisher depth_pub;
    double force = 0.0;
    double depth = 0.0;
    int num_states = 0;

public:
    void setTopic(std::string topic)
    {
        sensor_sub = nh.subscribe(topic, 1, &ReflexLink::callback, this);
        force_pub = nh.advertise<std_msgs::Float64>(topic + "_force", 1);
        depth_pub = nh.advertise<std_msgs::Float64>(topic + "_depth", 1);
    }

    void callback(const gazebo_msgs::ContactsState &msg)
    {
        std::string sensor_name = msg.header.frame_id;
        num_states = msg.states.size();

        // reset variables
        force = 0.0;
        depth = 0.0;

        // always get first contact
        int contact_idx = 0;

        // get first state in batch
        int state_idx = 0;

        if (num_states > 0)
        {
            // wrench force of first contact
            double f[] = {msg.states[state_idx].wrenches[contact_idx].force.x,
                          msg.states[state_idx].wrenches[contact_idx].force.y,
                          msg.states[state_idx].wrenches[contact_idx].force.z};

            // take norm
            force = sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);

            // penetration depth of first contact
            depth = msg.states[state_idx].depths[contact_idx];
        }
        std_msgs::Float64 force_msg;
        std_msgs::Float64 depth_msg;
        force_msg.data = force;
        depth_msg.data = depth; 
        force_pub.publish(force_msg);
        depth_pub.publish(depth_msg);
    }
};

class ReflexFinger
{
private:
    int finger_id;

public:
    ReflexLink links[2];

    ReflexFinger(int finger_id)
    {
        finger_id = finger_id;
        links[0].setTopic(ns + "/proximal_" + std::to_string(finger_id) + "_sensor_bumper");
        links[1].setTopic(ns + "/distal_" + std::to_string(finger_id) + "_sensor_bumper");
    }
};

class ReflexHand
{
public:
    ReflexFinger fingers[3] = {ReflexFinger(1), ReflexFinger(2), ReflexFinger(3)};
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ROS_INFO("Launched %s node.", node_name.c_str());

    ReflexHand hand;
    reflex_msgs::Hand msg;

    ROS_INFO("Starting to listen to Gazebo sensor values.");

    ros::spin();
}