#include <string>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/ContactsState.h>
#include <reflex_msgs/Hand.h>

std::string node_name = "sensor";
std::string ns = "gazebo";
std::string topic_name = "reflex/hand_state";

double calcDotProduct(double vec_1[], double vec_2[], int dim)
{
    double result = 0.0;
    for (int i = 0; i < dim; i++)
    {
        result += vec_1[i] * vec_2[i];
    }
    return result;
}

class ReflexSensor
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sensor_sub;
    double pressure = 0.0;
    bool contact = false;
    int num_contacts = 0;

    // TODO: find real scaling factor (for now pressure magnitudes don't matter)
    double scaling_factor = 1.0;

public:
    double getPressure()
    {
        return pressure * scaling_factor;
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
            contact = true;
            
            for (int i = 0; i < num_contacts; i++)
            {
                // TODO: find out why we get multiple wrenches for the same contact
                int num_wrenches = msg.states[i].wrenches.size();

                for (int j = 0; j < num_wrenches; j++)
                {
                    // wrench force
                    double f[] = {msg.states[i].wrenches[j].force.x,
                                  msg.states[i].wrenches[j].force.y,
                                  msg.states[i].wrenches[j].force.z};

                    // contact normal
                    double n[] = {msg.states[i].contact_normals[j].x,
                                  msg.states[i].contact_normals[j].y,
                                  msg.states[i].contact_normals[j].z};

                    // add only wrench force in normal direction
                    pressure += calcDotProduct(f, n, 3);
                }
            }
        }
    }
};

class ReflexFinger
{
private:
    int finger_id;

public:
    ReflexSensor sensors[9];

    ReflexFinger(int finger_id)
    {
        finger_id = finger_id;
        
        sensors[0].setTopic(ns + "/proximal_" + std::to_string(finger_id) + "_sensor_1_bumper");
        sensors[1].setTopic(ns + "/proximal_" + std::to_string(finger_id) + "_sensor_2_bumper");
        sensors[2].setTopic(ns + "/proximal_" + std::to_string(finger_id) + "_sensor_3_bumper");
        sensors[3].setTopic(ns + "/proximal_" + std::to_string(finger_id) + "_sensor_4_bumper");
        sensors[4].setTopic(ns + "/proximal_" + std::to_string(finger_id) + "_sensor_5_bumper");
        sensors[5].setTopic(ns + "/distal_" + std::to_string(finger_id) + "_sensor_1_bumper");
        sensors[6].setTopic(ns + "/distal_" + std::to_string(finger_id) + "_sensor_2_bumper");
        sensors[7].setTopic(ns + "/distal_" + std::to_string(finger_id) + "_sensor_3_bumper");
        sensors[8].setTopic(ns + "/distal_" + std::to_string(finger_id) + "_sensor_4_bumper");
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