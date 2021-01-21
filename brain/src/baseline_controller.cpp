#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include "reflex_interface/hand_command.hpp"
#include "baseline_controller.hpp"
#include "helpers.hpp"

using namespace std;

BaselineController::BaselineController(ros::NodeHandle *nh, tf2::Transform init_wrist_pose, tf2::Transform goal_wrist_pose, bool simulation_only, float time_out)
{
    this->nh = nh;
    this->init_wrist_pose = init_wrist_pose;
    this->goal_wrist_pose = goal_wrist_pose;
    this->time_out = time_out;

    // TODO find another, more elegant solution for this
    // wait before publishing first transform to fix warning from wrist_controller_node
    // ""reflex" passed to lookupTransform argument target_frame does not exist."
    ros::Duration(2).sleep();
    start_time = ros::Time::now();

    // move to init wrist pose
    simulation_only ? moveToInitPoseSim() : moveToInitPoseReal();

    // put fingers in spherical open position
    ri.command.executePrimitive(HandCommand::Primitive::SphericalOpen);
}

void BaselineController::checkTimeOut()
{
    ros::Duration allowed_duration(time_out);
    ros::Duration duration = ros::Time::now() - start_time;

    if (duration > allowed_duration && state == NotGrasped)
    {
        ROS_INFO("Time-out occured. Stopping experiment as we did not make grasp attempt yet.");
        stopExperiment();
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

    // close fingers s.t. they do not collide with ground_plane upon moving to init_wrist_pose
    ri.command.executePrimitive(HandCommand::Primitive::SphericalClose);

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
    float time_increment = 0.1;
    float time_sum = 0;
    float stop_thresh = 30; // if we dont reach pose within 30s something is wrong, stop experiment
    while (!reachedPoseSim(desired_pose))
    {
        ROS_INFO_STREAM("Waiting to reach " << name << " wrist pose...");
        ros::Duration(time_increment).sleep();
        time_sum += time_increment;
        if (time_sum >= stop_thresh)
        {
            ROS_INFO_STREAM("Did not reach " << name << " wrist pose within " << stop_thresh << " seconds. Something is wrong.");
            ROS_WARN("Setting state to 'Failure'");
            state = Failure;
            status_msg = "Got stuck when moving to " + name + " wrist pose.";
            stopExperiment();
            return;
        }
    }
    ROS_INFO_STREAM("Reached " << name << " wrist pose!");
}

bool BaselineController::reachedPoseSim(tf2::Transform desired_pose, float position_thresh, float rotation_thresh)
{
    // checks whether current wrist pose is within positional threshold of a desired wrist pose
    tf2::Vector3 des_pos = desired_pose.getOrigin();
    tf2::Quaternion des_rot = desired_pose.getRotation();

    // get current position of reflex origin (i.e. wrist)
    tf2::Transform transform = getLinkPoseSim(nh, "shell", "world", false);
    tf2::Vector3 cur_pos = transform.getOrigin();
    tf2::Quaternion cur_rot = transform.getRotation();

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

void BaselineController::moveAlongVector(tf2::Vector3 vec)
{
    tf2::Transform increment = tf2::Transform();
    increment.setIdentity();
    increment.setOrigin(vec);
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

tf2::Vector3 BaselineController::getApproachDirectionSingleContact()
{
    int finger_id = hand_state.getFingerIdSingleContact();

    // check if it's a proximal contact (if false, we know it is a distal contact)
    bool prox;
    hand_state.finger_states[finger_id].hasProximalContact() ? prox = true : false;

    tf2::Vector3 normal;
    prox
        ? normal = hand_state.finger_states[finger_id].getProximalNormal()
        : normal = hand_state.finger_states[finger_id].getDistalNormal();

    // backoff should be 9/10 the size of a step_size
    float scaling_factor = 0.9 * step_size;

    // subtract normal vector from current approach direction to back-off a little
    return step_reflex_z - (normal * scaling_factor);
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
            moveAlongVector(step_reflex_z);
            break;
        }
        case HandState::ContactState::SingleFingerContact:
        {
            ROS_INFO("Single contact --> Update Approach Vector.");
            moveAlongVector(getApproachDirectionSingleContact());
            break;
        }
        case HandState::ContactState::MultipleFingerContact:
        {
            // do spherical grasp
            ROS_INFO("Multi contact --> Grasping, then lifting.");
            ri.command.executePrimitive(HandCommand::Primitive::SphericalClose);
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
        ri.command.executePrimitive(HandCommand::Primitive::Open);

        stopExperiment();
    }
    checkTimeOut();
}

void BaselineController::resetWorldSim()
{
    ROS_INFO("Resetting simulated world to ensure repeatable experiments.");
    tf2::Transform world_frame;
    world_frame.setIdentity();

    // we need waypoint above origin because otherwise hand will sometimes
    // not reach origin due to friction with ground plane
    tf2::Transform waypoint_frame = world_frame;
    waypoint_frame.setOrigin(tf2::Vector3{0, 0, 0.3});

    string object_name;
    getParam(nh, &object_name, "object_name");
    moveObjectOutOfWay(nh, object_name, init_object_pose);

    // close fingers s.t. they do not collide with ground_plane upon moving to waypoint_frame
    ri.command.executePrimitive(HandCommand::Primitive::SphericalClose);
    sendWristTransform(waypoint_frame);
    waitUntilWristReachedPoseSim(waypoint_frame, "waypoint origin");
    sendWristTransform(world_frame);
    waitUntilWristReachedPoseSim(world_frame, "origin");

    // open fingers to neural position
    ri.command.executePrimitive(HandCommand::Primitive::Open);

    // move object back to old pose
    ROS_INFO_STREAM("Moving object back to initial pose.");
    setModelPoseSim(nh, object_name, init_object_pose);
}

void BaselineController::stopExperiment()
{
    ROS_INFO("Stopping experiment.");
    resetWorldSim();
    finished = true;
    ROS_INFO("Done! Have a nice day.");
}