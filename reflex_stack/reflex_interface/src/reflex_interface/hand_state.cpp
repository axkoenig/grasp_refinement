#include <vector>
#include <algorithm>
#include <numeric>
#include <boost/thread/lock_guard.hpp>

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
    reflex_state_sub = nh->subscribe("reflex_takktile/hand_state", 1, &HandState::reflex_state_callback, this);
    hand_state_pub = nh->advertise<reflex_interface::HandStateStamped>("/reflex_interface/hand_state", 1);

    if (use_sim_data_hand)
    {
        sim_state_sub = nh->subscribe("reflex_takktile/sim_contact_frames", 1, &HandState::sim_state_callback, this);
    }
}

bool HandState::allFingersInContact()
{
    boost::lock_guard<boost::mutex> guard(mtx);
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

tf2::Vector3 HandState::create_vec_from_msg(const geometry_msgs::Vector3 &msg)
{
    return tf2::Vector3{msg.x, msg.y, msg.z};
}

void HandState::sim_state_callback(const sensor_listener::ContactFrames &msg)
{
    // reset variables
    mtx.lock();
    vars.clear_all();
    vars.num_contacts = msg.contact_frames.size();

    for (int i = 0; i < vars.num_contacts; i++)
    {
        tf2::Transform transform;
        tf2::fromMsg(msg.contact_frames[i].contact_frame, transform);
        vars.contact_frames.push_back(transform);
        vars.contact_forces.push_back(create_vec_from_msg(msg.contact_frames[i].contact_wrench.force));
        vars.contact_torques.push_back(create_vec_from_msg(msg.contact_frames[i].contact_wrench.torque));
        vars.contact_positions.push_back(create_vec_from_msg(msg.contact_frames[i].contact_position));
        vars.contact_normals.push_back(create_vec_from_msg(msg.contact_frames[i].contact_normal));
        vars.sensor_ids.push_back(msg.contact_frames[i].sensor_id);
        vars.contact_force_magnitudes.push_back(msg.contact_frames[i].contact_force_magnitude);
        vars.contact_torque_magnitudes.push_back(msg.contact_frames[i].contact_torque_magnitude);
        vars.sum_contact_forces += msg.contact_frames[i].contact_force_magnitude;
        int finger_id = msg.contact_frames[i].finger_id;
        vars.link_ids.push_back(finger_id);
        if (!msg.contact_frames[i].palm_contact)
        {
            // this should never happen but checking anyway due to vector accessing below
            if (finger_id < 1 || finger_id > 3)
            {
                ROS_ERROR("Your finger id is out of bounds. Ignoring this.");
                continue;
            }
            vars.fingers_in_contact[finger_id - 1] = true;
            vars.num_sensors_in_contact_per_finger[finger_id - 1] += 1; // TODO actually this variable now represents num_sim_contacts_per_finger
        }
    }
    mtx.unlock();
    updateHandStateSim();
    // TODO shift this to wrist_controller
    // broadcast Gazebo wrist pose to ROS tf tree
    // TODO maybe add back in (once ROS fixes this bug https://github.com/ros/geometry2/issues/467 and I dont get bombarded with warning messages anymore)
    // tf2::Transform wrist_measured = getLinkPoseSim(nh, "shell", "world", false);
    // broadcastModelState(wrist_measured, "world", "reflex_interface/wrist_measured", &br_reflex_measured);

    updateQualityMetrics();
    hand_state_pub.publish(getHandStateMsg());
}

HandStateVariables HandState::getVars()
{
    boost::lock_guard<boost::mutex> guard(mtx);
    return vars;
}

void HandState::reflex_state_callback(const reflex_msgs::Hand &msg)
{
    for (int i = 0; i < num_fingers; i++)
    {
        finger_states[i]->setProximalAngle(msg.finger[i].proximal);
        finger_states[i]->setDistalAngle(msg.finger[i].distal_approx);
        finger_states[i]->setSensorContacts(msg.finger[i].contact);
        finger_states[i]->setSensorPressures(msg.finger[i].pressure);

        if (i != 2)
        {
            // set preshape angle for fingers 1 and 2 (finger 3 doesn't have a preshape angle)
            finger_states[i]->setPreshapeAngle(msg.motor[3].joint_angle);
        }
    }
    for (int i = 0; i < num_motors; i++)
    {
        motor_states[i]->setJointAngle(msg.motor[i].joint_angle);
        motor_states[i]->setRawAngle(msg.motor[i].raw_angle);
        motor_states[i]->setVelocity(msg.motor[i].velocity);
        motor_states[i]->setLoad(msg.motor[i].load);
        motor_states[i]->setVoltage(msg.motor[i].voltage);
        motor_states[i]->setTemperature(msg.motor[i].temperature);
        motor_states[i]->setErrorState(msg.motor[i].error_state);
    }

    if (!use_sim_data_hand)
    {
        updateHandStateReal();
        updateQualityMetrics();
        hand_state_pub.publish(getHandStateMsg());
    }
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

void HandState::updateQualityMetrics()
{
    if (use_sim_data_obj)
    {
        // broadcast Gazebo object pose to ROS tf tree
        getParam(nh, &object_name, "object_name", false);
        obj_measured = getModelPoseSim(nh, object_name, "world", false);
        // TODO maybe add back in (once ROS fixes this bug https://github.com/ros/geometry2/issues/467 and I dont get bombarded with warning messages anymore)
        // broadcastModelState(obj_measured, "world", "reflex_interface/obj_measured", &br_obj_measured);
        boost::lock_guard<boost::mutex> guard(mtx);
        grasp_quality.fillEpsilonFTSeparate(vars.contact_positions, vars.contact_normals, obj_measured.getOrigin(), vars.epsilon_force, vars.epsilon_torque);
        vars.delta_cur = grasp_quality.getSlipMargin(vars.contact_normals, vars.contact_forces, vars.contact_force_magnitudes, vars.num_contacts);
        vars.delta_task = grasp_quality.getSlipMarginWithTaskWrenches(vars.contact_forces, vars.contact_normals, vars.contact_frames, obj_measured.getOrigin(), vars.num_contacts);
    }
    else
    {
        // on real hand (without knowing the object pose) we can only calculate the epsilon force
    }
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

void HandState::updateHandStateSim()
{
    // updates poses of finger links from simulation
    for (int i = 0; i < num_fingers; i++)
    {
        finger_states[i]->updateCurLinkFramesInShellFrameSim();
    }
}

void HandState::updateHandStateReal()
{
    // reset variables
    boost::lock_guard<boost::mutex> guard(mtx);
    vars.clear_all();

    for (int i = 0; i < num_fingers; i++)
    {
        int num_contacts_on_finger = 0;
        finger_states[i]->updateCurLinkFramesInShellFrameReal();
        finger_states[i]->fillContactInfo(vars.contact_positions, vars.contact_normals, num_contacts_on_finger, false, "shell");

        // TODO: solve with forward kinematics and compare with results we obtain from simulation.
        // TODO because we want this in world coosy we also need the gripper pose.
        if (num_contacts_on_finger > 0)
        {
            vars.num_sensors_in_contact_per_finger[i] = num_contacts_on_finger;
            vars.fingers_in_contact[i] = true;
        }
    }
    vars.num_contacts = std::accumulate(vars.num_sensors_in_contact_per_finger.begin(), vars.num_sensors_in_contact_per_finger.end(), 0);
}

reflex_interface::HandStateStamped HandState::getHandStateMsg()
{
    // create msg for positions and normals of finger surface above each tactile sensor

    reflex_interface::HandStateStamped hss;
    hss.header.stamp = ros::Time::now();
    hss.header.frame_id = "shell";
    hss.preshape_angle = finger_states[0]->getPreshapeAngle();
    mtx.lock();
    hss.num_contacts = vars.num_contacts;
    hss.epsilon = vars.epsilon;
    hss.epsilon_force = vars.epsilon_force;
    hss.delta_cur = vars.delta_cur;
    hss.delta_task = vars.delta_task;
    hss.epsilon_torque = vars.epsilon_torque;
    hss.sum_contact_forces = vars.sum_contact_forces;
    mtx.unlock();

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