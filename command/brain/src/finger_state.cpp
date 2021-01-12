#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gazebo_msgs/GetLinkState.h>

#include "finger_state.hpp"

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

// TODO REMOVE
using namespace std;

tf2::Transform getLinkPoseSim(string link_name, bool verbose = false)
{
    // NOTE we can't simply use the getModelPositionSim function because in Gazebo the Reflex model is
    // rooted in the origin (hence position is always in origin). Rather we have to obtain it via the
    // position of the underlying links of the Reflex model.
    ros::NodeHandle nh;
    // setup service client
    string service_name = "/gazebo/get_link_state";
    ros::service::waitForService(service_name);
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::GetLinkState>(service_name);

    // setup service message
    gazebo_msgs::GetLinkState srv;
    srv.request.link_name = link_name;
    srv.request.reference_frame = "world";

    // obtain position of reflex
    client.call(srv);
    tf2::Vector3 t = {srv.response.link_state.pose.position.x,
                      srv.response.link_state.pose.position.y,
                      srv.response.link_state.pose.position.z};
    tf2::Quaternion r = {srv.response.link_state.pose.orientation.x,
                         srv.response.link_state.pose.orientation.y,
                         srv.response.link_state.pose.orientation.z,
                         srv.response.link_state.pose.orientation.w};

    if (verbose == true)
    {
        ROS_INFO_STREAM("Link pose of " << link_name << " is: x=" << t[0] << ", y=" << t[1] << ", z=" << t[2]);
        ROS_INFO_STREAM("Link orientation of " << link_name << " is: x=" << r[0] << ", y=" << r[1] << ", z=" << r[2] << ", w=" << r[3]);
    }

    return tf2::Transform(r, t);
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

    // TODO REMOVE
    //     ROS_INFO_STREAM("Finger " << finger_id << " proximal_n: " << proximal_normal[0] << ", " << proximal_normal[1] << ", " << proximal_normal[2]);
    //     ROS_INFO_STREAM("Finger " << finger_id << " distal_n: " << distal_normal[0] << ", " << distal_normal[1] << ", " << distal_normal[2]);

    //     // get ground truth values
    //     std::string name = "proximal_" + std::to_string(finger_id) + "_sensor_1";
    //     tf2::Transform ground_truth = getLinkPoseSim(name);

    //     tf2::Vector3 vec = tf2::Vector3{0, 0, 1};
    //     vec = ground_truth * vec;
    //     ROS_INFO_STREAM("Finger " << finger_id << " proximal truth: " << vec[0] << ", " << vec[1] << ", " << vec[2]);

    //     name = "distal_" + std::to_string(finger_id) + "_sensor_1";
    //     tf2::Transform gt = getLinkPoseSim(name);
    //     vec = tf2::Vector3{0, 0, 1};
    //     vec = gt * vec;
    //     ROS_INFO_STREAM("Finger " << finger_id << " distal truth: " << vec[0] << ", " << vec[1] << ", " << vec[2]);
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