#include <vector>
#include <algorithm>
#include <numeric>

#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#include "gazebo_interface/gazebo_interface.hpp"
#include "reflex_interface/hand_state.hpp"

HandState::HandState(ros::NodeHandle *nh, bool use_sim_data_hand, bool use_sim_data_obj)
    : finger_states{new FingerState(nh, 1), new FingerState(nh, 2), new FingerState(nh, 3)}
{
    this->nh = nh;
    this->use_sim_data_hand = use_sim_data_hand;
    this->use_sim_data_obj = use_sim_data_obj;
    state_sub = nh->subscribe("reflex/hand_state", 1, &HandState::callback, this);
    num_contacts_pub = nh->advertise<std_msgs::Int32>("reflex/num_contacts", 1);
    epsilon_pub = nh->advertise<std_msgs::Float64>("/reflex/epsilon", 1);
    epsilon_f_pub = nh->advertise<std_msgs::Float64>("/reflex/epsilon_force", 1);
    epsilon_t_pub = nh->advertise<std_msgs::Float64>("/reflex/epsilon_torque", 1);
    tactile_poses_pub = nh->advertise<reflex_interface::TactileInfoStamped>("/reflex/tactile_poses", 1);
    getParam(nh, &object_name, "object_name", false);
}

bool HandState::allFingersInContact()
{
    return std::all_of(fingers_in_contact.begin(), fingers_in_contact.end(), [](bool v) { return v; });
}

int HandState::getNumFingersInContact()
{
    int count = 0;
    for (int i = 0; i < num_fingers; i++)
    {
        if (finger_states[i]->hasContact() == true)
        {
            count++;
        }
    }
    return count;
}

int HandState::getFingerIdSingleContact()
{
    for (int i = 0; i < num_fingers; i++)
    {
        if (finger_states[i]->hasContact() == true)
        {
            return i;
        }
    }
    return -1;
}

void HandState::callback(const reflex_msgs::Hand &msg)
{
    for (int i = 0; i < num_fingers; i++)
    {
        finger_states[i]->setProximalAngleFromMsg(msg.finger[i].proximal);
        finger_states[i]->setDistalAngleFromMsg(msg.finger[i].distal_approx);
        finger_states[i]->setSensorContactsFromMsg(msg.finger[i].contact);
        finger_states[i]->setSensorPressuresFromMsg(msg.finger[i].pressure);

        if (i != 2)
        {
            // set preshape angle for fingers 1 and 2 (finger 3 doesn't have a preshape angle)
            finger_states[i]->setPreshapeAngleFromMsg(msg.motor[3].joint_angle / 2);
        }
    }

    if (use_sim_data_hand)
    {
        updateHandStateWorldSim();

        // TODO shift this to wrist_controller broadcast Gazebo wrist pose to ROS tf tree
        tf2::Transform wrist_measured = getLinkPoseSim(nh, "shell", "world", false);
        broadcastModelState(wrist_measured, "world", "reflex_interface/wrist_measured", &br_reflex_measured);
    }
    else
    {
        updateHandStateWorldReal();
    }

    std_msgs::Float64 epsilon_msg, epsilon_f_msg, epsilon_t_msg;
    float epsilon_force = 0, epsilon_torque = 0;
    std_msgs::Int32 num_contacts;
    num_contacts.data = std::accumulate(num_sensors_in_contact_per_finger.begin(), num_sensors_in_contact_per_finger.end(), 0);

    if (use_sim_data_obj)
    {
        // broadcast Gazebo object pose to ROS tf tree
        tf2::Transform obj_measured = getModelPoseSim(nh, object_name, "world", false);
        broadcastModelState(obj_measured, "world", "reflex_interface/obj_measured", &br_obj_measured);
        fillEpsilonFTSeparate(obj_measured.getOrigin(), epsilon_force, epsilon_torque);
        epsilon_f_msg.data = epsilon_force;
        epsilon_t_msg.data = epsilon_torque;
        // TODO commented out for now because not using this and heavy computation
        // epsilon_msg.data = getEpsilon(obj_measured.getOrigin());
    }
    else
    {
        // we could obtain center of mass from computer vision or manual estimates.
        // could listen to a ROS topic here to obtain object_com_world and then:
        // epsilon_msg.data = getEpsilon(object_com_world)
        throw "Not implemented.";
    }
    num_contacts_pub.publish(num_contacts);
    epsilon_pub.publish(epsilon_msg);
    epsilon_f_pub.publish(epsilon_f_msg);
    epsilon_t_pub.publish(epsilon_t_msg);
    tactile_poses_pub.publish(getTactilePosesMsg());
}

void HandState::broadcastModelState(tf2::Transform tf, std::string source_frame, std::string target_frame, tf2_ros::TransformBroadcaster *br)
{
    geometry_msgs::TransformStamped ts;
    ts.header.frame_id = source_frame;
    ts.child_frame_id = target_frame;
    ts.header.stamp = ros::Time::now();
    ts.transform = tf2::toMsg(tf);
    br->sendTransform(ts);
}

HandState::ContactState HandState::getContactState()
{
    switch (getNumFingersInContact())
    {
    case 0:
        return NoContact;
    case 1:
        return SingleFingerContact;
    default:
        return MultipleFingerContact;
    }
}

void HandState::updateHandStateWorldSim()
{
    // reset variables
    contact_positions_world.clear();
    contact_normals_world.clear();
    num_sensors_in_contact_per_finger = {0, 0, 0};
    fingers_in_contact = {0, 0, 0};

    for (int i = 0; i < num_fingers; i++)
    {
        finger_states[i]->updateCurLinkFramesInShellFrameSim();
        int num_contacts_on_finger = 0;
        finger_states[i]->fillContactInfoInWorldFrameSim(contact_positions_world, contact_normals_world, num_contacts_on_finger);

        if (num_contacts_on_finger > 0)
        {
            num_sensors_in_contact_per_finger[i] = num_contacts_on_finger;
            fingers_in_contact[i] = true;
        }
    }
}

reflex_interface::TactileInfoStamped HandState::getTactilePosesMsg()
{
    // create msg for positions and normals of finger surface above each tactile sensor

    reflex_interface::TactileInfoStamped tps;
    tps.header.stamp = ros::Time::now();
    tps.header.frame_id = "shell";

    for (int i = 0; i < num_fingers; i++)
    {
        std::vector<tf2::Vector3> tactile_pos = finger_states[i]->getTactilePositionsInShellFrame();
        
        tps.prox_normal[i] = tf2::toMsg(finger_states[i]->getProximalNormalInShellFrame());
        tps.dist_normal[i] = tf2::toMsg(finger_states[i]->getDistalNormalInShellFrame());
        tps.prox_in_contact[i] = finger_states[i]->hasProximalContact();
        tps.dist_in_contact[i] = finger_states[i]->hasDistalContact();
        tps.finger_in_contact[i] = finger_states[i]->hasContact();

        for (int j = 0; j < num_sensors_per_finger; j++)
        {
            tf2::toMsg(tactile_pos[j], tps.tactile_position[j + i * num_sensors_per_finger]);
        } 
    }
    return tps;
}

void HandState::updateHandStateWorldReal()
{
    for (int i = 0; i < num_fingers; i++)
    {
        finger_states[i]->updateCurLinkFramesInShellFrameReal();
    }
    // TODO: solve with forward kinematics and compare with results we obtain from simulation.
    // because we want this in world coosy we also need the gripper pose.
    throw "Not implemented.";
}

float HandState::getEpsilon(tf2::Vector3 object_com_world)
{
    return grasp_quality.getEpsilon(contact_positions_world, contact_normals_world, object_com_world);
}

void HandState::fillEpsilonFTSeparate(tf2::Vector3 object_com_world, float &epsilon_force, float &epsilon_torque)
{
    grasp_quality.fillEpsilonFTSeparate(contact_positions_world, contact_normals_world, object_com_world, epsilon_force, epsilon_torque);
}