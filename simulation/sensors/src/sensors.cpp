#include <string>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/ContactsState.h>
#include <reflex_msgs/Hand.h>

std::string node_name = "sensor";
std::string ns = "gazebo";
std::string topic_name = "reflex/hand_state";

class ReflexSensor
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sensor_sub;
    double pressure = 0.0;
    bool contact = false;
    int num_contacts = 0;

public:
    double getPressure()
    {
        return pressure;
    }

    bool getContact()
    {
        return contact;
    }

    void setTopic(std::string topic)
    {
        sensor_sub = nh.subscribe(topic, 1, &ReflexSensor::callback, this);
    }

    void callback(const gazebo_msgs::ContactsState &msg)
    {
        std::string sensor_name = msg.header.frame_id;
        pressure = 0.0;
        contact = false;
        num_contacts = msg.states.size();

        if (num_contacts > 0)
        {
            for (int i = 0; i < num_contacts; i++)
            {
                // retrieve force components of first contact
                double fx = msg.states[i].total_wrench.force.x;
                double fy = msg.states[i].total_wrench.force.y;
                double fz = msg.states[i].total_wrench.force.z;

                // compute euclidean norm (note this is actually a force)
                pressure += sqrt(pow(fx, 2) + pow(fy, 2) + pow(fz, 2));
            }
            contact = true;
        }
    }
};

class ReflexFinger
{
private:
    int finger_idx;

public:
    ReflexSensor sensors[9];

    ReflexFinger(int finger_idx)
    {
        finger_idx = finger_idx;

        sensors[0].setTopic(ns + "/proximal_" + std::to_string(finger_idx) + "_sensor_1_bumper");
        sensors[1].setTopic(ns + "/proximal_" + std::to_string(finger_idx) + "_sensor_2_bumper");
        sensors[2].setTopic(ns + "/proximal_" + std::to_string(finger_idx) + "_sensor_3_bumper");
        sensors[3].setTopic(ns + "/proximal_" + std::to_string(finger_idx) + "_sensor_4_bumper");
        sensors[4].setTopic(ns + "/proximal_" + std::to_string(finger_idx) + "_sensor_5_bumper");
        sensors[5].setTopic(ns + "/distal_" + std::to_string(finger_idx) + "_sensor_1_bumper");
        sensors[6].setTopic(ns + "/distal_" + std::to_string(finger_idx) + "_sensor_2_bumper");
        sensors[7].setTopic(ns + "/distal_" + std::to_string(finger_idx) + "_sensor_3_bumper");
        sensors[8].setTopic(ns + "/distal_" + std::to_string(finger_idx) + "_sensor_4_bumper");
    }
};

class ReflexHand
{
public:
    ReflexFinger fingers[3] = {ReflexFinger(0), ReflexFinger(1), ReflexFinger(2)};
};

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
        for (int i = 0; i < 3; i++)
        {
            // iterate over sensors
            for (int j = 0; j < 9; j++)
            {
                msg.finger[i].pressure[j] = hand.fingers[i].sensors[j].getPressure();
                msg.finger[i].contact[j] = hand.fingers[i].sensors[j].getContact();
            }
        }
        pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }
}