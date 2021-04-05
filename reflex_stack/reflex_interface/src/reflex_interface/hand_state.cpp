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
    state_sub = nh->subscribe("reflex/hand_state", 1, &HandState::reflex_callback, this);
    hand_state_pub = nh->advertise<reflex_interface::HandStateStamped>("/reflex_interface/hand_state", 1);
    getParam(nh, &object_name, "object_name", false);
}

bool HandState::allFingersInContact()
{
    return std::all_of(vars.fingers_in_contact.begin(), vars.fingers_in_contact.end(), [](bool v) { return v; });
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

void HandState::reflex_callback(const reflex_msgs::Hand &msg)
{
    // reset variables
    vars.num_contacts = 0;
    vars.epsilon = 0;
    vars.epsilon_force = 0;
    vars.epsilon_torque = 0;

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

    vars.num_contacts = std::accumulate(vars.num_sensors_in_contact_per_finger.begin(), vars.num_sensors_in_contact_per_finger.end(), 0);

    if (use_sim_data_obj)
    {
        // broadcast Gazebo object pose to ROS tf tree
        tf2::Transform obj_measured = getModelPoseSim(nh, object_name, "world", false);
        broadcastModelState(obj_measured, "world", "reflex_interface/obj_measured", &br_obj_measured);
        fillEpsilonFTSeparate(obj_measured.getOrigin(), vars.epsilon_force, vars.epsilon_torque);
        // TODO commented out for now because not using this and heavy computation
        // epsilon = getEpsilon(obj_measured.getOrigin());
    }
    else
    {
        // we could obtain center of mass from computer vision or manual estimates.
        // could listen to a ROS topic here to obtain object_com_world and then:
        // epsilon_msg.data = getEpsilon(object_com_world)
        throw "Not implemented.";
    }
    hand_state_pub.publish(getHandStateMsg());
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
    vars.contact_positions_world.clear();
    vars.contact_normals_world.clear();
    vars.num_sensors_in_contact_per_finger = {0, 0, 0};
    vars.fingers_in_contact = {0, 0, 0};

    for (int i = 0; i < num_fingers; i++)
    {
        finger_states[i]->updateCurLinkFramesInShellFrameSim();
        int num_contacts_on_finger = 0;
        finger_states[i]->fillContactInfoInWorldFrameSim(vars.contact_positions_world, vars.contact_normals_world, num_contacts_on_finger);

        if (num_contacts_on_finger > 0)
        {
            vars.num_sensors_in_contact_per_finger[i] = num_contacts_on_finger;
            vars.fingers_in_contact[i] = true;
        }
    }
}

reflex_interface::HandStateStamped HandState::getHandStateMsg()
{
    // create msg for positions and normals of finger surface above each tactile sensor

    reflex_interface::HandStateStamped hss;
    hss.header.stamp = ros::Time::now();
    hss.header.frame_id = "shell";
    hss.preshape_angle = 2 * finger_states[0]->getPreshapeAngle();
    hss.num_contacts = vars.num_contacts;
    hss.epsilon = vars.epsilon;
    hss.epsilon_force = vars.epsilon_force;
    hss.epsilon_torque = vars.epsilon_torque;

    for (int i = 0; i < num_fingers; i++)
    {
        hss.finger_state[i].prox_normal = tf2::toMsg(finger_states[i]->getProximalNormalInShellFrame());
        hss.finger_state[i].dist_normal = tf2::toMsg(finger_states[i]->getDistalNormalInShellFrame());
        hss.finger_state[i].prox_in_contact = finger_states[i]->hasProximalContact();
        hss.finger_state[i].dist_in_contact = finger_states[i]->hasDistalContact();
        hss.finger_state[i].finger_in_contact = finger_states[i]->hasContact();
        hss.finger_state[i].proximal_angle = finger_states[i]->getProximalAngle();
        hss.finger_state[i].distal_angle = finger_states[i]->getDistalAngle();

        std::vector<tf2::Vector3> tactile_pos = finger_states[i]->getTactilePositionsInShellFrame();

        for (int j = 0; j < num_sensors_per_finger; j++)
        {
            tf2::toMsg(tactile_pos[j], hss.finger_state[i].tactile_position[j]);
            hss.finger_state[i].sensor_contact[j] = finger_states[i]->getSensorContacts()[j];
            hss.finger_state[i].sensor_pressure[j] = finger_states[i]->getSensorPressures()[j];
        }
    }
    return hss;
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
    return grasp_quality.getEpsilon(vars.contact_positions_world, vars.contact_normals_world, object_com_world);
}

void HandState::fillEpsilonFTSeparate(tf2::Vector3 object_com_world, float &epsilon_force, float &epsilon_torque)
{
    grasp_quality.fillEpsilonFTSeparate(vars.contact_positions_world, vars.contact_normals_world, object_com_world, vars.epsilon_force, vars.epsilon_torque);
}