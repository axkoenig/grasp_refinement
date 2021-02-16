#include <string>
#include <cmath>
#include <vector>
#include <numeric>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/ContactsState.h>
#include <control_msgs/JointControllerState.h>
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

    // use for filtering of noisy sensing from simulation
    int buf_size = 5;
    std::vector<bool> contact_buffer = {0, 0, 0, 0, 0};
    std::vector<float> pressure_buffer = {0, 0, 0, 0, 0};

    // TODO: find real scaling factor (for now pressure magnitudes don't matter)
    double scaling_factor = 1.0;

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
        num_contacts = msg.states.size();

        // reset variables
        pressure = 0.0;
        contact = false;

        // rotate buffers right
        std::rotate(contact_buffer.rbegin(), contact_buffer.rbegin() + 1, contact_buffer.rend());
        std::rotate(pressure_buffer.rbegin(), pressure_buffer.rbegin() + 1, pressure_buffer.rend());

        if (num_contacts > 0)
        {
            double f[3] = {0, 0, 0};

            // get sum of all contact forces on sensor (NOTE: since the real reflex hand
            // has no torque sensors, we disregard the torque information from the
            // Gazebo sensors. UPDATE: checked the torque values and they are tiny
            // (order of e-05), which we expect since DART uses a hard contact model.
            // Only soft contact models can transmit torques)
            for (int i = 0; i < num_contacts; i++)
            {
                f[0] += msg.states[i].total_wrench.force.x;
                f[1] += msg.states[i].total_wrench.force.y;
                f[2] += msg.states[i].total_wrench.force.z;
            }

            // average over all contacts
            f[0] /= num_contacts;
            f[1] /= num_contacts;
            f[2] /= num_contacts;

            // pressure is magnitude of total force vector
            pressure_buffer[0] = sqrt(pow(f[0], 2) + pow(f[1], 2) + pow(f[2], 2)) * scaling_factor;
            contact_buffer[0] = true;
        }
        else
        {
            pressure_buffer[0] = 0.0;
            contact_buffer[0] = false;
        }

        // if any element in contact buffer is true, we return true
        contact = true ? std::any_of(contact_buffer.begin(), contact_buffer.end(), [](bool v) { return v; }) : false;

        // we average pressure over buffer
        pressure = std::accumulate(pressure_buffer.begin(), pressure_buffer.end(), 0.0) / buf_size;
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
        std::string preshape_f2_topic = ns + "/finger_2_preshape_position_controller/state";

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