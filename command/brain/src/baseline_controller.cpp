#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "hand_state.hpp"
#include "baseline_controller.hpp"
#include "helpers.hpp"

using namespace std;

BaselineController::BaselineController(ros::NodeHandle *nh, tf2::Transform init_wrist_pose, tf2::Transform goal_wrist_pose, bool simulation_only)
{
    this->nh = nh;
    this->init_wrist_pose = init_wrist_pose;
    this->goal_wrist_pose = goal_wrist_pose;

    open_client = nh->serviceClient<std_srvs::Trigger>(open_srv_name);
    sph_open_client = nh->serviceClient<std_srvs::Trigger>(sph_open_srv_name);
    sph_close_client = nh->serviceClient<std_srvs::Trigger>(sph_close_srv_name);
    reflex_state_sub = nh->subscribe(state_topic_name, 1, &BaselineController::callbackHandState, this);

    // TODO find another, more elegant solution for this
    // wait before publishing first transform to fix warning from wrist_controller_node
    // ""reflex" passed to lookupTransform argument target_frame does not exist."
    ros::Duration(0.2).sleep();

    // move to init wrist pose
    simulation_only ? moveToInitPoseSim() : moveToInitPoseReal();

    // put fingers in spherical open position
    sph_open_client.call(trigger);
    ROS_INFO("%s", trigger.response.message.c_str());
}

void BaselineController::moveToInitPoseSim()
{
    ROS_INFO("Running baseline controller in simulation only.");

    // remember object pose
    string object_name;
    getParam(nh, &object_name, "object_name");
    tf2::Transform old_object_pose = getModelPoseSim(nh, object_name);
    
    // move object somewhere else s.t. we don't collide with it while moving to init pose
    ROS_INFO_STREAM("Moving object out of the way.");
    float x_offset = 10;
    tf2::Transform new_object_pose = old_object_pose;
    tf2::Vector3 t = new_object_pose.getOrigin();
    t[0] += x_offset;
    new_object_pose.setOrigin(t);
    setModelPoseSim(nh, object_name, new_object_pose);

    // move robot hand to init pose 
    desired_pose = init_wrist_pose;
    sendTransform(desired_pose);
    waitUntilReachedPoseSim(desired_pose, "initial wrist");

    // move object back to old pose
    ROS_INFO_STREAM("Moving object back to old pose.");
    setModelPoseSim(nh, object_name, old_object_pose);
}

void BaselineController::moveToInitPoseReal()
{
    ROS_INFO("Running baseline controller on real robot, sending init_wrist_pose directly.");
    desired_pose = init_wrist_pose;
    sendTransform(desired_pose);
    // TODO: wait until real robot actually reached the init pose
}

void BaselineController::waitUntilReachedPoseSim(tf2::Transform desired_pose, string name)
{
    while (!reachedPoseSim(desired_pose))
    {
        ROS_INFO_STREAM("Waiting to reach " << name << " pose...");
        ros::Duration(0.1).sleep();
    }
    ROS_INFO_STREAM("Reached " << name << " pose!");
}

bool BaselineController::reachedPoseSim(tf2::Transform desired_pose, float position_thresh)
{
    // checks whether current wrist pose is within positional threshold of a desired wrist pose
    tf2::Vector3 des_pos = desired_pose.getOrigin();

    // get current position of reflex origin (i.e. wrist)
    tf2::Vector3 cur_pos = getLinkPoseSim(nh, "shell", false).getOrigin();

    for (int i = 0; i < 3; i++)
    {
        if (abs(cur_pos[i] - des_pos[i]) > position_thresh)
        {
            return false;
        }
    }
    return true;
}

void BaselineController::callbackHandState(const reflex_msgs::Hand &msg)
{
    for (int i = 0; i < hand_state.num_fingers; i++)
    {
        hand_state.finger_states[i].setProximalAngleFromMsg(msg.finger[i].proximal);
        hand_state.finger_states[i].setDistalAngleFromMsg(msg.finger[i].distal_approx);
        hand_state.finger_states[i].setSensorContactsFromMsg(msg.finger[i].contact);
        hand_state.finger_states[i].setSensorPressureFromMsg(msg.finger[i].pressure);
    }
}

void BaselineController::moveAlongVector(tf2::Vector3 vector)
{
    tf2::Transform increment = tf2::Transform();
    increment.setIdentity();
    increment.setOrigin(vector);
    desired_pose *= increment;

    sendTransform(desired_pose);
}

void BaselineController::sendTransform(tf2::Transform transform)
{
    ts.header.frame_id = source_frame;
    ts.child_frame_id = target_frame;
    ts.header.stamp = ros::Time::now();
    ts.transform = tf2::toMsg(transform);
    br.sendTransform(ts);
}

void BaselineController::timeStep()
{
    if (grasped == false)
    {
        // check if we can make grasping attempt
        switch (hand_state.getCurrentState())
        {
        case HandState::State::NoContact:
        {
            ROS_INFO("No contact --> Approach");
            tf2::Vector3 reflex_z_increment = {0, 0, step_size};
            moveAlongVector(reflex_z_increment);
            break;
        }
        case HandState::State::SingleFingerContact:
        {
            // TODO update approach direction
            ROS_INFO("Single contact --> Approach.");
            tf2::Vector3 reflex_z_increment = {0, 0, step_size};
            moveAlongVector(reflex_z_increment);
            break;
        }
        case HandState::State::MultipleFingerContact:
        {
            // do spherical grasp
            ROS_INFO("Multi contact --> Grasping, then lifting.");
            sph_close_client.call(trigger);
            grasped = true;
            ros::Duration(0.5).sleep();

            // lift object by 0.2m to get some clearance from ground before moving to goal
            tf2::Vector3 origin = desired_pose.getOrigin();
            origin[2] += 0.2;
            desired_pose.setOrigin(origin);
            sendTransform(desired_pose);
            waitUntilReachedPoseSim(desired_pose, "goal waypoint");
            break;
        }
        }
    }
    else
    {
        // we grasped, move to goal pose
        ROS_INFO("Grasped object --> Moving to goal pose.");
        sendTransform(goal_wrist_pose);
        waitUntilReachedPoseSim(goal_wrist_pose, "goal");

        ROS_INFO("Dropping object.");
        open_client.call(trigger);
        ROS_INFO("%s", trigger.response.message.c_str());
        
        finished = true;
        ROS_INFO("Finished. Have a nice day.");
    }
}