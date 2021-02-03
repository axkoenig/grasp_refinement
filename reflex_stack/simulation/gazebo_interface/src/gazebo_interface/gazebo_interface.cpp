#include <iostream>
#include <fstream>
#include <ctime>
#include <sstream>

#include <tf2/LinearMath/Quaternion.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/ContactsState.h>

#include "gazebo_interface/gazebo_interface.hpp"

using namespace std;


bool setModelPoseSim(ros::NodeHandle *nh, string model_name, tf2::Transform pose, bool verbose)
{
    tf2::Vector3 t = pose.getOrigin();
    tf2::Quaternion r = pose.getRotation();

    // setup service client
    string service_name = "/gazebo/set_model_state";
    ros::service::waitForService(service_name);
    ros::ServiceClient client = nh->serviceClient<gazebo_msgs::SetModelState>(service_name);

    // setup service message
    gazebo_msgs::SetModelState srv;
    srv.request.model_state.model_name = model_name;
    srv.request.model_state.reference_frame = "world";
    srv.request.model_state.pose.position.x = t[0];
    srv.request.model_state.pose.position.y = t[1];
    srv.request.model_state.pose.position.z = t[2];
    srv.request.model_state.pose.orientation.x = r[0];
    srv.request.model_state.pose.orientation.y = r[1];
    srv.request.model_state.pose.orientation.z = r[2];
    srv.request.model_state.pose.orientation.w = r[3];

    client.call(srv);

    if (verbose == true)
    {
        ROS_INFO_STREAM("Set model position of " << model_name << " to: x=" << t[0] << ", y=" << t[1] << ", z=" << t[2]);
        ROS_INFO_STREAM("Set model orientation of " << model_name << " to: x=" << r[0] << ", y=" << r[1] << ", z=" << r[2] << ", w=" << r[3]);
    }

    return srv.response.success;
}

bool objectTouchesGround()
{
    std::string topic = "/gazebo/object_sensor_bumper";
    float time_out = 0.1;
    gazebo_msgs::ContactsState cs;
    gazebo_msgs::ContactsStateConstPtr msg = ros::topic::waitForMessage<gazebo_msgs::ContactsState>(topic, ros::Duration(time_out));

    if (!msg)
    {
        ROS_INFO_STREAM("No message on '" << topic << "' received! Assuming that object touches ground.");
        return true;
    }

    cs = *msg;

    // num_states is number of pairs in contact
    int num_states = cs.states.size();
    const std::string collision_name = "ground_plane::link::collision";

    // iterate over pairs in contact and look for collisions with ground plane
    for (int i = 0; i < num_states; i++)
    {
        if (cs.states[i].collision1_name == collision_name ||
            cs.states[i].collision2_name == collision_name)
        {
            ROS_INFO("Object touches ground.");
            return true;
        }
    }
    ROS_INFO("Object does not touch ground.");
    return false;
}

tf2::Transform getModelPoseSim(ros::NodeHandle *nh, string model_name, string relative_entity_name, bool verbose)
{
    // setup service client
    string service_name = "/gazebo/get_model_state";
    ros::service::waitForService(service_name);
    ros::ServiceClient client = nh->serviceClient<gazebo_msgs::GetModelState>(service_name);

    // setup service message
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = model_name;
    srv.request.relative_entity_name = relative_entity_name;

    client.call(srv);

    tf2::Vector3 t = {srv.response.pose.position.x,
                      srv.response.pose.position.y,
                      srv.response.pose.position.z};
    tf2::Quaternion r = {srv.response.pose.orientation.x,
                         srv.response.pose.orientation.y,
                         srv.response.pose.orientation.z,
                         srv.response.pose.orientation.w};

    if (verbose == true)
    {
        ROS_INFO("Obtained object pose in world coordinates.");
        ROS_INFO_STREAM("Model position of " << model_name << " is: x=" << t[0] << ", y=" << t[1] << ", z=" << t[2]);
        ROS_INFO_STREAM("Model orientation of " << model_name << " is: x=" << r[0] << ", y=" << r[1] << ", z=" << r[2] << ", w=" << r[3]);
    }

    return tf2::Transform(r, t);
}

tf2::Transform getLinkPoseSim(ros::NodeHandle *nh, string link_name, string reference_frame, bool verbose)
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
    srv.request.link_name = link_name;
    srv.request.reference_frame = reference_frame;

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