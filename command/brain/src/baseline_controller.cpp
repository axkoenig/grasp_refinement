#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include "hand_state.hpp"
#include "baseline_controller.hpp"
#include "helpers.hpp"

using namespace std;

BaselineController::BaselineController(ros::NodeHandle *nh, tf2::Transform init_wrist_pose, tf2::Transform goal_wrist_pose, bool simulation_only, float time_out)
{
    this->nh = nh;
    this->init_wrist_pose = init_wrist_pose;
    this->goal_wrist_pose = goal_wrist_pose;
    this->time_out = time_out;

    open_client = nh->serviceClient<std_srvs::Trigger>(open_srv_name);
    sph_open_client = nh->serviceClient<std_srvs::Trigger>(sph_open_srv_name);
    sph_close_client = nh->serviceClient<std_srvs::Trigger>(sph_close_srv_name);
    reflex_state_sub = nh->subscribe(state_topic_name, 1, &BaselineController::callbackHandState, this);

    // TODO find another, more elegant solution for this
    // wait before publishing first transform to fix warning from wrist_controller_node
    // ""reflex" passed to lookupTransform argument target_frame does not exist."
    ros::Duration(0.2).sleep();
    start_time = ros::Time::now();

    // move to init wrist pose
    simulation_only ? moveToInitPoseSim() : moveToInitPoseReal();

    // put fingers in spherical open position
    sph_open_client.call(trigger);
    ROS_INFO("%s", trigger.response.message.c_str());
}

void BaselineController::checkTimeOut()
{
    ros::Duration allowed_duration(time_out);
    ros::Duration duration = ros::Time::now() - start_time;

    if (duration > allowed_duration && state == NotGrasped)
    {
        ROS_INFO("Time-out occured. Stopping experiment as we did not make grasp attempt yet.");
        resetWorldSim();
        finished = true;
    }
}

void BaselineController::moveToInitPoseSim()
{
    ROS_INFO("Running baseline controller in simulation only.");

    // remember object pose
    string object_name;
    getParam(nh, &object_name, "object_name");
    init_object_pose = getModelPoseSim(nh, object_name);

    // move object s.t. it doesn't accidentally collide with robotic hand
    moveObjectOutOfWay(nh, object_name, init_object_pose);

    // move robot hand to init pose
    desired_pose = init_wrist_pose;
    sendWristTransform(desired_pose);
    waitUntilWristReachedPoseSim(desired_pose, "initial");

    // move object back to old pose
    ROS_INFO_STREAM("Moving object back to old pose.");
    setModelPoseSim(nh, object_name, init_object_pose);
}

void BaselineController::moveToInitPoseReal()
{
    ROS_INFO("Running baseline controller on real robot, sending init_wrist_pose directly.");
    desired_pose = init_wrist_pose;
    sendWristTransform(desired_pose);
    // TODO: wait until real robot actually reached the init pose
}

void BaselineController::waitUntilWristReachedPoseSim(tf2::Transform desired_pose, string name)
{
    while (!reachedPoseSim(desired_pose))
    {
        ROS_INFO_STREAM("Waiting to reach " << name << " wrist pose...");
        ros::Duration(0.1).sleep();
    }
    ROS_INFO_STREAM("Reached " << name << " wrist pose!");
}

bool BaselineController::reachedPoseSim(tf2::Transform desired_pose, float position_thresh, float rotation_thresh)
{
    // checks whether current wrist pose is within positional threshold of a desired wrist pose
    tf2::Vector3 des_pos = desired_pose.getOrigin();
    tf2::Quaternion des_rot = desired_pose.getRotation();

    // get current position of reflex origin (i.e. wrist)
    tf2::Vector3 cur_pos = getLinkPoseSim(nh, "shell", false).getOrigin();
    tf2::Quaternion cur_rot = getLinkPoseSim(nh, "shell", false).getRotation();

    if ((des_rot - cur_rot).length() > rotation_thresh)
    {
        return false;
    }

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
    hand_state.setFingerStateFromMsg(msg);
}

void BaselineController::moveAlongApproachDir()
{
    tf2::Transform increment = tf2::Transform();
    increment.setIdentity();
    increment.setOrigin(approach_direction);
    desired_pose *= increment;

    sendWristTransform(desired_pose);
}

void BaselineController::sendWristTransform(tf2::Transform transform)
{
    ts.header.frame_id = source_frame;
    ts.child_frame_id = target_frame;
    ts.header.stamp = ros::Time::now();
    ts.transform = tf2::toMsg(transform);
    br.sendTransform(ts);
}

void BaselineController::updateApproachDirectionSingleContact()
{
    tf2::Vector3 vec = hand_state.finger_states[0].getProximalNormal();
    tf2::Vector3 vec2 = hand_state.finger_states[1].getProximalNormal();
    tf2::Vector3 vec3 = hand_state.finger_states[2].getProximalNormal();
}

void BaselineController::timeStep()
{
    if (state == NotGrasped)
    {
        // check if we can make grasping attempt
        switch (hand_state.getContactState())
        {
        case HandState::ContactState::NoContact:
        {
            ROS_INFO("No contact --> Approach");
            updateApproachDirectionSingleContact();
            moveAlongApproachDir();
            break;
        }
        case HandState::ContactState::SingleFingerContact:
        {
            ROS_INFO("Single contact --> Update Approach Vector.");
            updateApproachDirectionSingleContact();
            moveAlongApproachDir();
            break;
        }
        case HandState::ContactState::MultipleFingerContact:
        {
            // do spherical grasp
            ROS_INFO("Multi contact --> Grasping, then lifting.");
            sph_close_client.call(trigger);
            state = GraspedButNotLifted;
            ros::Duration(0.5).sleep();

            // lift object by 0.2m to get some clearance from ground before moving to goal
            tf2::Vector3 origin = desired_pose.getOrigin();
            origin[2] += 0.2;
            desired_pose.setOrigin(origin);
            sendWristTransform(desired_pose);
            waitUntilWristReachedPoseSim(desired_pose, "lifted");

            // update state
            objectTouchesGround() ? state = GraspedButNotLifted : state = GraspedAndLifted;
            break;
        }
        }
    }
    else
    {
        // we grasped, move to goal pose
        ROS_INFO("Grasped object --> Moving to goal pose.");
        sendWristTransform(goal_wrist_pose);
        waitUntilWristReachedPoseSim(goal_wrist_pose, "goal");

        // update state
        if (!objectTouchesGround())
        {
            state = GraspedAndInGoalPose;
        }

        ROS_INFO("Opening hand.");
        open_client.call(trigger);
        ROS_INFO("%s", trigger.response.message.c_str());

        resetWorldSim();
        finished = true;
        ROS_INFO("Finished. Have a nice day.");
    }
    checkTimeOut();
}

void BaselineController::resetWorldSim()
{
    ROS_INFO("Resetting simulated world to ensure repeatable experiments.");
    tf2::Transform world_frame;
    world_frame.setIdentity();

    string object_name;
    getParam(nh, &object_name, "object_name");

    moveObjectOutOfWay(nh, object_name, init_object_pose);
    sendWristTransform(world_frame);
    waitUntilWristReachedPoseSim(world_frame, "origin");

    // move object back to old pose
    ROS_INFO_STREAM("Moving object back to initial pose.");
    setModelPoseSim(nh, object_name, init_object_pose);
}