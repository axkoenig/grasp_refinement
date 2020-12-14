#include <string>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/ContactsState.h>
#include <control_msgs/JointControllerState.h>
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
    int num_states = 0;

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
        num_states = msg.states.size();

        // reset variables
        pressure = 0.0;
        contact = false;

        if (num_states > 0)
        {
            contact = true;

            for (int i = 0; i < num_states; i++)
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
    ros::NodeHandle nh;

    ros::Subscriber proximal_sub;
    ros::Subscriber proximal_to_flex_sub;
    ros::Subscriber flex_to_distal_sub;

    float proximal_angle = 0.0;
    float proximal_to_flex_angle = 0.0;
    float flex_to_distal_angle = 0.0;

public:
    ReflexSensor sensors[9];

    ReflexFinger(int finger_id)
    {
        this->finger_id = finger_id;
        std::string finger_id_str = std::to_string(this->finger_id);

        sensors[0].setTopic(ns + "/proximal_" + finger_id_str + "_sensor_1_bumper");
        sensors[1].setTopic(ns + "/proximal_" + finger_id_str + "_sensor_2_bumper");
        sensors[2].setTopic(ns + "/proximal_" + finger_id_str + "_sensor_3_bumper");
        sensors[3].setTopic(ns + "/proximal_" + finger_id_str + "_sensor_4_bumper");
        sensors[4].setTopic(ns + "/proximal_" + finger_id_str + "_sensor_5_bumper");
        sensors[5].setTopic(ns + "/distal_" + finger_id_str + "_sensor_1_bumper");
        sensors[6].setTopic(ns + "/distal_" + finger_id_str + "_sensor_2_bumper");
        sensors[7].setTopic(ns + "/distal_" + finger_id_str + "_sensor_3_bumper");
        sensors[8].setTopic(ns + "/distal_" + finger_id_str + "_sensor_4_bumper");

        std::string proximal_topic = ns + "/finger_" + finger_id_str + "_proximal_position_controller/state";
        std::string proximal_to_flex_topic = ns + "/finger_" + finger_id_str + "_proximal_to_flex_position_controller/state";
        std::string flex_to_distal_topic = ns + "/finger_" + finger_id_str + "_flex_to_distal_position_controller/state";

        proximal_sub = nh.subscribe(proximal_topic, 1, &ReflexFinger::proximal_callback, this);
        proximal_to_flex_sub = nh.subscribe(proximal_to_flex_topic, 1, &ReflexFinger::proximal_to_flex_callback, this);
        flex_to_distal_sub = nh.subscribe(flex_to_distal_topic, 1, &ReflexFinger::flex_to_distal_callback, this);
    }

    void proximal_callback(const control_msgs::JointControllerState &msg)
    {
        proximal_angle = msg.process_value;
    }

    void proximal_to_flex_callback(const control_msgs::JointControllerState &msg)
    {
        proximal_to_flex_angle = msg.process_value;
    }

    void flex_to_distal_callback(const control_msgs::JointControllerState &msg)
    {
        flex_to_distal_angle = msg.process_value;
    }

    float getProximalAngle()
    {
        return proximal_angle;
    }

    float getDistalAngle()
    {
        // note that (as with the real reflex hand) this is a rough approximation
        // we take the average of both flexure joints
        // TODO: compare with real reflex sensor readings
        return (proximal_to_flex_angle + flex_to_distal_angle) / 2;
    }
};

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
    ReflexFingerMotor(int finger_id)
    {
        this->finger_id = finger_id;
        std::string finger_id_str = std::to_string(this->finger_id);

        std::string proximal_topic = ns + "/finger_" + finger_id_str + "_proximal_position_controller/state";
        std::string proximal_to_flex_topic = ns + "/finger_" + finger_id_str + "_proximal_to_flex_position_controller/state";
        std::string flex_to_distal_topic = ns + "/finger_" + finger_id_str + "_flex_to_distal_position_controller/state";

        proximal_sub = nh.subscribe(proximal_topic, 1, &ReflexFingerMotor::proximal_callback, this);
        proximal_to_flex_sub = nh.subscribe(proximal_to_flex_topic, 1, &ReflexFingerMotor::proximal_to_flex_callback, this);
        flex_to_distal_sub = nh.subscribe(flex_to_distal_topic, 1, &ReflexFingerMotor::flex_to_distal_callback, this);
    }

    void proximal_callback(const control_msgs::JointControllerState &msg)
    {
        proximal_angle = msg.process_value;
        proximal_vel = msg.process_value_dot;
    }

    void proximal_to_flex_callback(const control_msgs::JointControllerState &msg)
    {
        proximal_to_flex_angle = msg.process_value;
        proximal_to_flex_vel = msg.process_value_dot;
    }

    void flex_to_distal_callback(const control_msgs::JointControllerState &msg)
    {
        flex_to_distal_angle = msg.process_value;
        flex_to_distal_vel = msg.process_value_dot;
    }

    virtual float getAngle() override
    {
        // assume motor angle is sum of all finger angles
        // TODO: compare with real reflex sensor readings
        return proximal_angle + proximal_to_flex_angle + flex_to_distal_angle;
    }
    virtual float getVelocity() override
    {
        // assume motor velocity is sum of all finger velocities
        // TODO: compare with real reflex sensor readings
        return proximal_vel + proximal_to_flex_vel + flex_to_distal_vel;
    }
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
    ReflexPreshapeMotor()
    {
        std::string preshape_f1_topic = ns + "/finger_1_preshape_position_controller/state";
        std::string preshape_f2_topic = ns + "/finger_2_proximal_to_flex_position_controller/state";

        preshape_f1_sub = nh.subscribe(preshape_f1_topic, 1, &ReflexPreshapeMotor::preshape_1_callback, this);
        preshape_f2_sub = nh.subscribe(preshape_f2_topic, 1, &ReflexPreshapeMotor::preshape_2_callback, this);
    }

    void preshape_1_callback(const control_msgs::JointControllerState &msg)
    {
        preshape_1_angle = msg.process_value;
        preshape_1_vel = msg.process_value_dot;
    }

    void preshape_2_callback(const control_msgs::JointControllerState &msg)
    {
        preshape_2_angle = msg.process_value;
        preshape_2_vel = msg.process_value_dot;
    }

    virtual float getAngle() override
    {
        // assume motor angle is sum of both preshape motor angles
        // TODO: compare with real reflex sensor readings
        return preshape_1_angle + preshape_2_angle;
    }

    virtual float getVelocity() override
    {
        // assume motor angle is sum of both preshape motor velocities
        // TODO: compare with real reflex sensor readings
        return preshape_1_vel + preshape_2_vel;
    }
};

class ReflexHand
{
public:
    ReflexFinger fingers[3] = {ReflexFinger(1), ReflexFinger(2), ReflexFinger(3)};
    ReflexMotor *motors[4];

    ReflexHand()
    {
        motors[0] = new ReflexFingerMotor(1);
        motors[1] = new ReflexFingerMotor(2);
        motors[2] = new ReflexFingerMotor(3);
        motors[3] = new ReflexPreshapeMotor();
    }
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

    int num_fingers = 3;
    int num_sensors = 9;
    int num_motors = 4;

    while (ros::ok())
    {
        // iterate over fingers
        for (int i = 0; i < num_fingers; i++)
        {
            msg.finger[i].proximal = hand.fingers[i].getProximalAngle();
            msg.finger[i].distal_approx = hand.fingers[i].getDistalAngle();

            // iterate over sensors
            for (int j = 0; j < num_sensors; j++)
            {
                msg.finger[i].pressure[j] = hand.fingers[i].sensors[j].getPressure();
                msg.finger[i].contact[j] = hand.fingers[i].sensors[j].getContact();
            }
        }

        // iterate over motors
        for (int i = 0; i < num_motors; i++)
        {
            msg.motor[i].joint_angle = hand.motors[i]->getAngle();
            msg.motor[i].velocity = hand.motors[i]->getVelocity();
        }

        pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }
}