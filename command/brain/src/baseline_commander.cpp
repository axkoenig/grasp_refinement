#include <math.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Trigger.h>
#include <reflex_msgs/Hand.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetLinkState.h>

#include "hand_state.h"

using namespace std;

string node_name = "baseline_commander_node";
string source_frame = "world";
string target_frame = "reflex";

string sph_open_srv_name = "reflex/spherical_open";
string sph_close_srv_name = "reflex/spherical_close";
string state_topic_name = "reflex/hand_state";

// TODO move to seperate file.
tf2::Vector3 getModelPositionSim(ros::NodeHandle *nh, string frame_name, bool verbose = true)
{
    // setup service client
    string service_name = "/gazebo/get_model_state";
    ros::service::waitForService(service_name);
    ros::ServiceClient client = nh->serviceClient<gazebo_msgs::GetModelState>(service_name);

    // setup service message
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = frame_name;
    srv.request.relative_entity_name = "world";

    // obtain position of object (we don't care about its orientation for now)
    client.call(srv);
    tf2::Vector3 t = {srv.response.pose.position.x,
                      srv.response.pose.position.y,
                      srv.response.pose.position.z};

    if (verbose == true)
    {
        ROS_INFO("Obtained object pose in world coordinates.");
        ROS_INFO_STREAM("Object position: x=" << t[0] << ", y=" << t[1] << ", z=" << t[2]);
    }

    return t;
}

tf2::Vector3 getReflexPositionSim(ros::NodeHandle *nh, bool verbose = true)
{
    // NOTE we can't simply use the getModelPositionSim function because in Gazebo the Reflex model is 
    // rooted in the origin (hence position is always in origin). Rather we have to obtain it via the 
    // position of the underlying links of the Reflex model. 

    // setup service client
    string service_name = "/gazebo/get_link_state";
    ros::service::waitForService(service_name);
    ros::ServiceClient client = nh->serviceClient<gazebo_msgs::GetLinkState>(service_name);

    // setup service message
    gazebo_msgs::GetLinkState srv;
    srv.request.link_name = "shell";
    srv.request.reference_frame = "world";

    // obtain position of reflex
    client.call(srv);
    tf2::Vector3 t = {srv.response.link_state.pose.position.x,
                      srv.response.link_state.pose.position.y,
                      srv.response.link_state.pose.position.z};
                      
    if (verbose == true)
    {
        ROS_INFO("Obtained reflex pose in world coordinates.");
        ROS_INFO_STREAM("Reflex position: x=" << t[0] << ", y=" << t[1] << ", z=" << t[2]);
    }

    return t;
}

tf2::Transform getTcpToWristFrame()
{
    // translate from TCP to wrist frame (values taken from Reflex CAD drawing available on website)
    // z_offset: distance from Reflex origin to palm surface
    // x_offset: approx. distance along x axis from origin to palm center
    float z_offset = -0.09228;
    float x_offset = -0.015;

    tf2::Transform translate_to_wrist = tf2::Transform();
    translate_to_wrist.setIdentity();
    translate_to_wrist.setOrigin(tf2::Vector3{x_offset, 0, z_offset});

    return translate_to_wrist;
}

void printPose(tf2::Transform pose, string name)
{
    // debug information
    tf2::Vector3 t = pose.getOrigin();
    tf2::Quaternion r = pose.getRotation();
    ROS_INFO_STREAM("Done calculating " << name << " pose.");
    ROS_INFO_STREAM("--> " << name << " position: x=" << t[0] << ", y=" << t[1] << ", z=" << t[2]);
    ROS_INFO_STREAM("--> " << name << " orientation: x=" << r[0] << ", y=" << r[1] << ", z=" << r[2] << ", w=" << r[3]);
}

tf2::Transform calcInitWristPose(ros::NodeHandle *nh,
                                 tf2::Vector3 pos_error = {0, 0, 0},
                                 float polar = M_PI / 4,
                                 float azimuth = 0,
                                 float offset = 0.3)
{
    ROS_INFO("Calculating initial wrist pose.");

    ///////////////////////////////////////////////////////////
    // A) GET GROUND TRUTH OBJECT POSITION AND INTRODUCE ERRORS
    ///////////////////////////////////////////////////////////

    // NOTE: for now not introducing orientation errors because we are currently
    // basing the algorithm only on object position. This is also why we later only translate
    // but do not rotate from world to object frame.

    // get object name from parameter server
    // TODO: make a generic function out of this -> with template
    string object_name;
    nh->getParam("object_name", object_name);
    tf2::Vector3 t = getModelPositionSim(nh, object_name);

    // TODO: get error from param server
    t += pos_error;
    tf2::Transform translate_to_object(tf2::Quaternion{0, 0, 0, 1}, t);

    /////////////////////////////////////////////////////////////////////
    // B) ROTATE AND TRANSLATE IN SPHERICAL COORDINATES TO GET WRIST POSE
    /////////////////////////////////////////////////////////////////////

    // add M_PI to polar angle, s.t. Reflex z points in opposite direction of world z
    polar += M_PI;

    // rotation with spherical coordinates
    tf2::Quaternion q;
    q.setRPY(0, polar, azimuth);
    tf2::Transform rotate_spherical;
    rotate_spherical.setIdentity();
    rotate_spherical.setRotation(q);

    // translation to tool center point (TCP) along negative z axis
    tf2::Vector3 tcp_to_object_offset = tf2::Vector3{0, 0, -1.0 * offset};
    tf2::Transform translate_to_tcp = tf2::Transform();
    translate_to_tcp.setIdentity();
    translate_to_tcp.setOrigin(tcp_to_object_offset);

    tf2::Transform init_wrist_pose = translate_to_object * rotate_spherical * translate_to_tcp * getTcpToWristFrame();
    printPose(init_wrist_pose, "wrist");

    return init_wrist_pose;
}

tf2::Transform calcWristWaypointPose(ros::NodeHandle *nh, float clearance = 0.3)
{
    // NOTE: waypoint has to be passed before going into init_wrist_pose s.t. we don't run into the object while we move
    string object_name;
    nh->getParam("object_name", object_name);
    tf2::Vector3 t = getModelPositionSim(nh, object_name, true);
    tf2::Transform translate_to_object(tf2::Quaternion{0, 0, 0, 1}, t);

    // TODO: check how I can write this as one Transform (same problem above)
    // rotate s.t. Reflex faces downwards
    tf2::Quaternion q_down;
    q_down.setRPY(0, M_PI, 0);
    tf2::Transform rotate_down;
    rotate_down.setIdentity();
    rotate_down.setRotation(q_down);

    // translate s.t. there is enough clearance to move to init pose
    tf2::Transform translate_to_clearance = tf2::Transform();
    translate_to_clearance.setIdentity();
    translate_to_clearance.setOrigin(tf2::Vector3{0, 0, -1.0 * clearance});

    tf2::Transform wrist_waypoint_pose = translate_to_object * rotate_down * translate_to_clearance * getTcpToWristFrame();
    printPose(wrist_waypoint_pose, "waypoint");

    return wrist_waypoint_pose;
}

class BaselineCommander
{
private:
    ros::NodeHandle *nh;
    std_srvs::Trigger trigger;
    ros::ServiceClient sph_open_client;
    ros::ServiceClient sph_close_client;
    ros::Subscriber reflex_state_sub;

    // transform broadcaster
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped ts;
    tf2::Transform desired_pose, init_wrist_pose, waypoint, goal_wrist_pose;

    float backoff_factor = 1.0;
    float step_size = 0.001;
    bool grasped = false;
    HandState hand_state = HandState();

public:
    BaselineCommander(ros::NodeHandle *nh, tf2::Transform init_wrist_pose, tf2::Transform goal_wrist_pose, bool simulation_only)
    {
        this->nh = nh;
        this->init_wrist_pose = init_wrist_pose;
        this->goal_wrist_pose = goal_wrist_pose;

        sph_open_client = nh->serviceClient<std_srvs::Trigger>(sph_open_srv_name);
        sph_close_client = nh->serviceClient<std_srvs::Trigger>(sph_close_srv_name);
        reflex_state_sub = nh->subscribe(state_topic_name, 1, &BaselineCommander::callbackHandState, this);

        // TODO find another, more elegant solution for this
        // wait before publishing first transform to fix warning from wrist_controller_node
        // ""reflex" passed to lookupTransform argument target_frame does not exist."
        ros::Duration(1).sleep();

        // move to init wrist pose
        simulation_only ? moveToInitPoseSim() : moveToInitPoseReal();

        // put fingers in spherical open position
        sph_open_client.call(trigger);
        ROS_INFO("%s", trigger.response.message.c_str());

        // wait before first time_step call
        ros::Duration(0.5).sleep();
    }

    void moveToInitPoseSim()
    {
        ROS_INFO("Running baseline controller in simulation only.");
        ROS_INFO("Moving to waypoint to avoid collision with object.");

        tf2::Transform waypoint_pose = calcWristWaypointPose(nh);
        desired_pose = waypoint_pose;
        sendTransform(desired_pose);
        waitUntilReachedPoseSim(desired_pose, "waypoint");

        ROS_INFO("Reached waypoint. Now moving to init_wrist_pose.");

        desired_pose = init_wrist_pose;
        sendTransform(desired_pose);
        waitUntilReachedPoseSim(desired_pose, "initial wrist");

        ROS_INFO("Reached init_wrist_pose. Now starting to approach object.");
    }

    void moveToInitPoseReal()
    {
        ROS_INFO("Running baseline controller on real robot, sending init_wrist_pose directly.");
        desired_pose = init_wrist_pose;
        sendTransform(desired_pose);
        // TODO: wait until real robot actually reached the init pose
    }

    void waitUntilReachedPoseSim(tf2::Transform desired_pose, string name)
    {
        while (!reachedPoseSim(desired_pose))
        {
            ROS_INFO_STREAM("Waiting to reach " << name << " pose...");
            ros::Duration(0.1).sleep();
        }
    }

    bool reachedPoseSim(tf2::Transform desired_pose, float position_thresh = 0.02)
    {
        // checks whether current wrist pose is within positional threshold of a desired wrist pose
        tf2::Vector3 des_pos = desired_pose.getOrigin();

        // get current position of reflex origin (i.e. wrist)
        tf2::Vector3 cur_pos = getReflexPositionSim(nh, false);

        for (int i = 0; i < 3; i++)
        {
            if (abs(cur_pos[i] - des_pos[i]) > position_thresh)
            {
                ROS_INFO_STREAM("i: " << i << "d = " << abs(cur_pos[i] - des_pos[i]));
                return false;
            }
        }
        return true;
    }

    void callbackHandState(const reflex_msgs::Hand &msg)
    {
        for (int i = 0; i < hand_state.num_fingers; i++)
        {
            hand_state.finger_states[i].setProximalAngle(msg.finger[i].proximal);
            hand_state.finger_states[i].setDistalAngle(msg.finger[i].distal_approx);
            hand_state.finger_states[i].setSensorContacts(msg.finger[i].contact);
            hand_state.finger_states[i].setSensorPressure(msg.finger[i].pressure);
        }
    }

    // moves along vector in reflex coordinates
    // length of vector is step size
    void moveAlongVector(tf2::Vector3 vector)
    {
        tf2::Transform increment = tf2::Transform();
        increment.setIdentity();
        increment.setOrigin(vector);
        desired_pose *= increment;

        sendTransform(desired_pose);
    }

    void sendTransform(tf2::Transform transform)
    {
        ts.header.frame_id = source_frame;
        ts.child_frame_id = target_frame;
        ts.header.stamp = ros::Time::now();
        ts.transform = tf2::toMsg(transform);
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
                tf2::Vector3 reflex_z_increment = {0, 0, step_size};
                moveAlongVector(reflex_z_increment);
                break;
            }
            case HandState::State::SingleFingerContact:
            {
                // TODO update approach direction
                ROS_INFO("Single contact --> Approach");
                tf2::Vector3 reflex_z_increment = {0, 0, step_size};
                moveAlongVector(reflex_z_increment);
                break;
            }
            case HandState::State::MultipleFingerContact:
            {
                // do spherical grasp
                ROS_INFO("Multi contact --> Grasping");
                sph_close_client.call(trigger);
                grasped = true;
                ros::Duration(0.5).sleep();
                break;
            }
            }
        }
        else
        {
            // we grasped, move to goal pose
            ROS_INFO("Grasped object --> Moving to goal pose");
            sendTransform(goal_wrist_pose);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::Rate rate(40);
    ROS_INFO("Launched %s.", node_name.c_str());

    tf2::Transform init_wrist_pose;
    bool simulation_only;
    string desired_param = "simulation_only";

    // wait for simulation_only on parameter server
    while (ros::ok())
    {
        if (nh.hasParam(desired_param))
        {
            nh.getParam(desired_param, simulation_only);
            ROS_INFO("Obtained %s: '%s' from parameter server.", desired_param.c_str(), simulation_only ? "1" : "0");
            break;
        }
        else
        {
            ROS_WARN("Could not find parameter '%s' on parameter server.", desired_param.c_str());
            ros::Duration(1.0).sleep();
        }
    }

    if (simulation_only == true)
    {
        // calculate initial wrist pose s.t. hand is in good grasping pose
        init_wrist_pose = calcInitWristPose(&nh);
    }
    else
    {
        // obtain initial pose from robot arm or from computer vision system
        // TODO: wait until we receive the initial pose (e.g. through a rosservice)
        ROS_WARN("Not implemented.");
    }

    // goal pose: goal_position and facing downwards.
    tf2::Vector3 goal_position = tf2::Vector3{0.3, 0, 0.3};
    tf2::Quaternion goal_rotation;
    goal_rotation.setRPY(M_PI, 0, 0);
    tf2::Transform goal_wrist_pose = tf2::Transform(goal_rotation, goal_position);

    BaselineCommander bc = BaselineCommander(&nh, init_wrist_pose, goal_wrist_pose, simulation_only);

    ros::Duration(1.0).sleep();
    ROS_INFO("Starting autonomous control.");

    while (ros::ok())
    {
        bc.timeStep();
        ros::spinOnce();
        rate.sleep();
    }
}