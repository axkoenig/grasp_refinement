#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "hand_state.hpp"

FingerState::FingerState(int finger_id)
{
    proximal_angle = 0.0;
    distal_angle = 0.0;
    this->finger_id = finger_id;
}

void FingerState::setProximalAngleFromMsg(float angle)
{
    proximal_angle = angle;
}

void FingerState::setDistalAngleFromMsg(float angle)
{
    distal_angle = angle;
}

void FingerState::setPreshapeAngleFromMsg(float angle)
{
    preshape_angle = angle;
}

void FingerState::setSensorContactsFromMsg(boost::array<unsigned char, 9> sensor_contacts)
{
    for (int i = 0; i < num_sensors; i++)
    {
        this->sensor_contacts[i] = sensor_contacts[i];
    }
}

void FingerState::setSensorPressureFromMsg(boost::array<float, 9> sensor_pressure)
{
    for (int i = 0; i < num_sensors; i++)
    {
        this->sensor_pressure[i] = sensor_pressure[i];
    }
}

bool FingerState::hasContact()
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

bool FingerState::hasProximalContact()
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

bool FingerState::hasDistalContact()
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

void FingerState::updateNormals()
{
    geometry_msgs::TransformStamped ts_msg;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    try
    {
        // NOTE: we get transform from reflex base to center of proximal joint from the ROS tf tree.
        // It will also be available when we run on the real robot. The transforms on the ROS tf tree
        // are not modified by Gazebo when robot is moved in simulation.
        std::string proximal_name = "proximal_" + std::to_string(finger_id);
        ts_msg = tfBuffer.lookupTransform("shell", proximal_name, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }

    // convert msg to transform stamped
    tf2::Stamped<tf2::Transform> prox_joint_frame;
    tf2::fromMsg(ts_msg, prox_joint_frame);

    // rotate proximal joint angle around negative y axis
    tf2::Quaternion q;
    q.setRPY(0, -proximal_angle, 0);
    tf2::Transform rotate_prox_angle = tf2::Transform(q, tf2::Vector3{0, 0, 0});
    prox_joint_frame *= rotate_prox_angle;

    // take into account preshape rotation for fingers 1 and 2
    if (finger_id == 1 || finger_id == 2)
    {
        // rotate around z axis (negative for finger 1, positive for finger 2)
        (finger_id == 1) ? q.setRPY(0, 0, -preshape_angle) : q.setRPY(0, 0, preshape_angle);
        tf2::Transform rotate_preshape_angle = tf2::Transform(q, tf2::Vector3{0, 0, 0});
        prox_joint_frame *= rotate_preshape_angle;
    }

    // proximal normal is updated z axis of prox joint frame as it is normal to proximal pad surface by default (see RViz)
    proximal_normal = prox_joint_frame * tf2::Vector3{0, 0, 1};

    // rotate distal joint angle around negative y axis
    q.setRPY(0, -distal_angle, 0);
    tf2::Transform rotate_distal_angle = tf2::Transform(q, tf2::Vector3{0, 0, 0});
    prox_joint_frame *= rotate_prox_angle;

    // distal normal is updated z axis of prox joint frame
    distal_normal = prox_joint_frame * tf2::Vector3{0, 0, 1};

    ROS_INFO_STREAM("Finger " << finger_id << " proximal_n: " << proximal_normal);
    ROS_INFO_STREAM("Finger " << finger_id << " distal_n: " << distal_normal);
}

tf2::Vector3 FingerState::getProximalNormal()
{
    updateNormals();
    return proximal_normal;
}

tf2::Vector3 FingerState::getDistalNormal()
{
    updateNormals();
    return distal_normal;
}

void HandState::updateState()
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

HandState::ContactState HandState::getContactState()
{
    updateState();
    return cur_state;
}

int HandState::countFingersInContact()
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

void HandState::setFingerStateFromMsg(const reflex_msgs::Hand &msg)
{
    for (int i = 0; i < num_fingers; i++)
    {
        finger_states[i].setProximalAngleFromMsg(msg.finger[i].proximal);
        finger_states[i].setDistalAngleFromMsg(msg.finger[i].distal_approx);
        finger_states[i].setSensorContactsFromMsg(msg.finger[i].contact);
        finger_states[i].setSensorPressureFromMsg(msg.finger[i].pressure);

        if (i != 2)
        {
            // set preshape angle for fingers 1 and 2 (finger 3 doesn't have a preshape angle)
            finger_states[i].setPreshapeAngleFromMsg(msg.motor[3].joint_angle / 2);
        }
    }
}