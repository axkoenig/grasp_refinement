// TODO check if I need all these imports
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include <std_srvs/Trigger.h>
#include <reflex_msgs/Hand.h>

std::string node_name = "baseline_commander_node";
std::string source_frame = "world";
std::string target_frame = "reflex";

std::string sph_open_srv_name = "reflex/spherical_open";
std::string sph_close_srv_name = "reflex/spherical_close";
std::string state_topic_name = "reflex/hand_state";

class FingerState
{
private:
    int finger_id;
    int num_sensors = 9;
    std::array<bool, 9> sensor_contacts;
    std::array<float, 9> pressure;
    float proximal_angle = 0.0;
    float distal_angle = 0.0;

public:
    FingerState(int finger_id)
    {
        this->finger_id = finger_id;
    }

    void setProximalFromMsg(float proximal_angle)
    {
        this->proximal_angle = proximal_angle;
    }

    void setDistalFromMsg(float distal_angle)
    {
        this->distal_angle = distal_angle;
    }

    void setSensorContactsFromMsg(boost::array<unsigned char, 9> sensor_contacts)
    {
        for (int i = 0; i < num_sensors; i++)
        {
            this->sensor_contacts[i] = sensor_contacts[i];
        }
    }

    void setPressureFromMsg(boost::array<float, 9> pressure)
    {
        for (int i = 0; i < num_sensors; i++)
        {
            this->pressure[i] = pressure[i];
        }
    }

    bool hasContact()
    {
        for (int i = 0; i < num_sensors; i++)
        {
            if (sensor_contacts[i] == true)
            {
                return true;
            }
        }
        return false;
    }

    bool hasProximalContact()
    {
        for (int i = 0; i < 5; i++)
        {
            if (sensor_contacts[i] == true)
            {
                return true;
            }
        }
        return false;
    }

    bool hasDistalContact()
    {
        for (int i = 5; i < num_sensors; i++)
        {
            if (sensor_contacts[i] == true)
            {
                return true;
            }
        }
        return false;
    }
};

class HandState
{
public:
    enum State
    {
        NoContact,
        SingleFingerContact,
        MultipleFingerContact
    };

    State cur_state;
    int num_fingers = 3;
    int num_motors = 4;

    FingerState finger_states[3] = {FingerState(1), FingerState(2), FingerState(3)};

    void updateState()
    {
        int contact_count = countFingersInContact();
        switch (contact_count)
        {
        case 0:
            cur_state = NoContact;
            break;
        case 1:
            cur_state = SingleFingerContact;
            break;
        default:
            cur_state = MultipleFingerContact;
            break;
        }
    }

    State getCurrentState()
    {
        updateState();
        return cur_state;
    }

    int countFingersInContact()
    {
        int count = 0;
        for (int i = 0; i < num_fingers; i++)
        {
            if (finger_states[i].hasContact() == true)
            {
                count++;
            }
        }
        return count;
    }
};

class BaselineCommander
{
private:
    std_srvs::Trigger trigger;
    ros::ServiceClient sph_open_client;
    ros::ServiceClient sph_close_client;
    ros::Subscriber reflex_state_sub;

    // transform broadcaster
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped ts;
    tf2::Transform cur_transform;

    float backoff_factor = 1.0;
    HandState hand_state = HandState();
    bool grasped = false;

    // TODO calculate this
    std::array<float, 6> init_euler = {0, 0, 0.07, -M_PI / 2, 0, 0};
    std::array<float, 6> cur_euler = init_euler;

    tf2::Transform calcReflexInWorld(std::array<float, 6> pose)
    {
        tf2::Vector3 t = {pose[0], pose[1], pose[2]};
        tf2::Quaternion q;
        q.setRPY(pose[3], pose[4], pose[5]);
        tf2::Transform transform(q, t);

        return transform.inverse();
    }

    tf2::Transform calcInitPose()
    {
        // TODO take from normal distribution
        return calcReflexInWorld(init_euler);
    }

public:
    BaselineCommander(ros::NodeHandle *nh)
    {
        sph_open_client = nh->serviceClient<std_srvs::Trigger>(sph_open_srv_name);
        sph_close_client = nh->serviceClient<std_srvs::Trigger>(sph_close_srv_name);
        reflex_state_sub = nh->subscribe(state_topic_name, 1, &BaselineCommander::callbackHandState, this);

        // populate initial wrist transform
        cur_transform = this->calcInitPose();
        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = source_frame;
        ts.child_frame_id = target_frame;
        ts.transform = tf2::toMsg(cur_transform);
        br.sendTransform(ts);

        // put fingers in spherical open position
        sph_open_client.call(trigger);
        ROS_INFO("%s", trigger.response.message.c_str());
    }

    void callbackHandState(const reflex_msgs::Hand &msg)
    {
        for (int i = 0; i < hand_state.num_fingers; i++)
        {
            hand_state.finger_states[i].setProximalFromMsg(msg.finger[i].proximal);
            hand_state.finger_states[i].setDistalFromMsg(msg.finger[i].distal_approx);
            hand_state.finger_states[i].setSensorContactsFromMsg(msg.finger[i].contact);
            hand_state.finger_states[i].setPressureFromMsg(msg.finger[i].pressure);
        }
    }

    std::array<float, 6> addArrays(std::array<float, 6> array_1, std::array<float, 6> array_2)
    {
        for (int i = 0; i < 6; i++)
        {
            array_1[i] += array_2[i];
        }
        return array_1;
    }

    void approachAlongAxis(std::array<float, 6> axis)
    {
        // approach along reflex z axis
        cur_euler = addArrays(cur_euler, axis);
        cur_transform = calcReflexInWorld(cur_euler);

        // send TransformStamped
        ts.header.stamp = ros::Time::now();
        ts.transform = tf2::toMsg(cur_transform);
        br.sendTransform(ts);
    }

    void timeStep()
    {
        if (grasped == false)
        {
            // check if we can make grasping attempt
            switch (hand_state.getCurrentState())
            {
            case HandState::State::NoContact:
            {
                ROS_INFO("No contact --> Approach");
                std::array<float, 6> reflex_z_axis = {0, 0.001, 0, 0, 0, 0};
                approachAlongAxis(reflex_z_axis);
                break;
            }
            case HandState::State::SingleFingerContact:
            {
                // TODO update approach direction
                ROS_INFO("Single contact --> Approach");
                std::array<float, 6> updated_axis = {0, 0.001, 0, 0, 0, 0};
                approachAlongAxis(updated_axis);
                break;
            }
            case HandState::State::MultipleFingerContact:
            {
                // do spherical grasp
                ROS_INFO("Multi contact --> Grasping");
                sph_close_client.call(trigger);
                grasped = true;
                break;
            }
            }
        }
        else
        {
            // we already grasped, now move up
            ROS_INFO("Grasped object --> Manipulating");
            std::array<float, 6> updated_axis = {0, 0, 0.001, 0, 0, 0};
            approachAlongAxis(updated_axis);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::Rate rate(40);
    ROS_INFO("Launched %s node.", node_name.c_str());

    BaselineCommander bc = BaselineCommander(&nh);

    ros::Duration(1.0).sleep();
    ROS_INFO("Starting autonomous control.");

    while (ros::ok())
    {
        bc.timeStep();
        ros::spinOnce();
        rate.sleep();
    }
}