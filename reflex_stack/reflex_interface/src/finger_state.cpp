#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gazebo_msgs/GetLinkState.h>

#include "reflex_interface/finger_state.hpp"
// #include "helpers.hpp"

FingerState::FingerState(int finger_id)
{
    this->finger_id = finger_id;
    setProximalJointFrame();
}

void FingerState::setProximalJointFrame()
{
    // NOTE: we get transform from reflex base (i.e. from the "shell" frame) to center of
    // proximal joint from the ROS tf tree. This transform will always be constant and is not
    // modified as Gazebo is moving the joints.

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    std::string source_frame = "shell";
    std::string target_frame = "proximal_" + std::to_string(finger_id);

    while (!tfBuffer.canTransform(source_frame, target_frame, ros::Time(0), ros::Duration(1)))
    {
        ROS_INFO_STREAM("Could not find transform from '" << source_frame << "' to '"
                                                          << target_frame << "'. Waiting to fill up buffer.");
    }
    geometry_msgs::TransformStamped ts_msg = tfBuffer.lookupTransform(source_frame, target_frame, ros::Time(0));

    // convert msg to transform stamped
    tf2::Stamped<tf2::Transform> ts;
    tf2::fromMsg(ts_msg, ts);

    proximal_joint_frame = tf2::Transform(ts.getRotation(), ts.getOrigin());
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

std::vector<tf2::Transform> FingerState::getContactFramesWorldSim()
{
    // reset variables
    contact_frames_world = {};
    std::string sensor_name = "";

    for (int i = 0; i < num_sensors; i++)
    {
        if (sensor_contacts[i])
        {
            if (i < 5) // proximal contact
            {
                sensor_name = "proximal_" + std::to_string(finger_id) + "_sensor_" + std::to_string(i + 1);
            }
            else // distal contact
            {
                sensor_name = "distal_" + std::to_string(finger_id) + "_sensor_" + std::to_string(i - 4);
            }
            contact_frames_world.push_back(getLinkPoseSim(&nh, sensor_name, "world", false));
        }
    }
    return contact_frames_world;
}

void FingerState::setSensorPressuresFromMsg(boost::array<float, 9> sensor_pressures)
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

void FingerState::updateNormalsExactSim()
{
    // NOTE: this method gets the exact surface normals of the links in "shell" frame (not "world"!) via the simulation.
    // This method serves as a sanity check for the method updateNormalsFromMeasuredJointAngles(). It directly queries
    // the simulation for the current link joint frames and returns their z axis as they are normal to the link surface.

    // TODO when I create ReflexInterface class, pass its NodeHandle in here. Currently this creates a NodeHandle every
    // time the method is called.
    // ROS_WARN("THIS METHOD IS VERY INEFFICIENT.");
    // ros::NodeHandle nh;

    // std::string link_name = "proximal_" + std::to_string(finger_id);
    // tf2::Transform transform = getLinkPoseSim(&nh, link_name, "shell");
    // proximal_normal = transform * tf2::Vector3{0, 0, 1};

    // link_name = "distal_" + std::to_string(finger_id);
    // transform = getLinkPoseSim(&nh, link_name, "shell");
    // distal_normal = transform * tf2::Vector3{0, 0, 1};
}

void FingerState::updateNormalsFromMeasuredJointAngles()
{
    // NOTE: this method calculates approximate surface normals in "shell" frame (not "world"!) from measured joint angles.
    // Idea: z axis of proximal_joint_frame points in normal direction of proximal pad (see RViz). We factor in the rotation
    // around preshape, proximal and distal joints and return this z axis.

    tf2::Transform transform = proximal_joint_frame;

    // 1) rotate around negative y axis for current measured proximal_angle
    tf2::Quaternion q;
    q.setRPY(0, -proximal_angle, 0);
    tf2::Transform rotate_prox_angle = tf2::Transform(q, tf2::Vector3{0, 0, 0});
    transform *= rotate_prox_angle;

    // 2) take into account preshape rotation for fingers 1 and 2
    if (finger_id == 1 || finger_id == 2)
    {
        // rotate around z axis (negative for finger 1, positive for finger 2)
        (finger_id == 1) ? q.setRPY(0, 0, -preshape_angle) : q.setRPY(0, 0, preshape_angle);
        tf2::Transform rotate_preshape_angle = tf2::Transform(q, tf2::Vector3{0, 0, 0});
        transform *= rotate_preshape_angle;
    }

    proximal_normal = transform * tf2::Vector3{0, 0, 1};

    // 3) rotate negative y axis for current measured distal_angle
    q.setRPY(0, -distal_angle, 0);
    tf2::Transform rotate_distal_angle = tf2::Transform(q, tf2::Vector3{0, 0, 0});
    transform *= rotate_distal_angle;

    distal_normal = transform * tf2::Vector3{0, 0, 1};
}

tf2::Vector3 FingerState::getProximalNormal()
{
    updateNormalsFromMeasuredJointAngles();
    return proximal_normal;
}

tf2::Vector3 FingerState::getDistalNormal()
{
    updateNormalsFromMeasuredJointAngles();
    return distal_normal;
}