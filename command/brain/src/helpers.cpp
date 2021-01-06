#include <tf2/LinearMath/Quaternion.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetLinkState.h>

#include "helpers.hpp"

using namespace std;

tf2::Vector3 getModelPositionSim(ros::NodeHandle *nh, string frame_name, bool verbose)
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

tf2::Vector3 getReflexPositionSim(ros::NodeHandle *nh, bool verbose)
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
                                 float pos_error[3],
                                 float polar,
                                 float azimuth,
                                 float offset)
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

    // introduce error
    t += tf2::Vector3{pos_error[0], pos_error[1], pos_error[2]};
    tf2::Transform translate_to_object(tf2::Quaternion{0, 0, 0, 1}, t);
    ROS_INFO_STREAM("Introduced positional error " << pos_error << " to model position.");

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

tf2::Transform calcWristWaypointPose(ros::NodeHandle *nh, float clearance)
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

template <typename T>
void getParam(ros::NodeHandle *nh, T *param, const string param_name)
{
    // wait for simulation_only on parameter server
    while (ros::ok())
    {
        if (nh->getParam(param_name, *param))
        {
            ROS_INFO_STREAM("Obtained " << param_name << ": " << *param << " from parameter server.");
            return;
        }
        else
        {
            ROS_WARN("Could not find parameter '%s' on parameter server.", param_name.c_str());
            ros::Duration(1.0).sleep();
        }
    }
}