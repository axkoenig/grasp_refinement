#include <iostream>
#include <fstream>
#include <ctime>
#include <sstream>

#include <tf2/LinearMath/Quaternion.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/ContactsState.h>

#include "helpers.hpp"

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
        ROS_INFO_STREAM("No message on '" << topic << "' received! Returning false.");
        return false;
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

tf2::Transform getModelPoseSim(ros::NodeHandle *nh, string model_name, bool verbose)
{
    // setup service client
    string service_name = "/gazebo/get_model_state";
    ros::service::waitForService(service_name);
    ros::ServiceClient client = nh->serviceClient<gazebo_msgs::GetModelState>(service_name);

    // setup service message
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = model_name;
    srv.request.relative_entity_name = "world";

    // obtain position of object (we don't care about its orientation for now)
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

tf2::Transform getLinkPoseSim(ros::NodeHandle *nh, string link_name, bool verbose)
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
    srv.request.reference_frame = "world";

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
                                 float azimuthal,
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
    getParam(nh, &object_name, "object_name");
    tf2::Vector3 t = getModelPoseSim(nh, object_name).getOrigin();

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
    q.setRPY(0, polar, azimuthal);
    tf2::Transform rotate_spherical;
    rotate_spherical.setIdentity();
    rotate_spherical.setRotation(q);

    // translation to tool center point (TCP) along negative z axis
    tf2::Vector3 tcp_to_object_offset = tf2::Vector3{0, 0, -offset};
    tf2::Transform translate_to_tcp = tf2::Transform();
    translate_to_tcp.setIdentity();
    translate_to_tcp.setOrigin(tcp_to_object_offset);

    tf2::Transform init_wrist_pose = translate_to_object * rotate_spherical * translate_to_tcp * getTcpToWristFrame();
    printPose(init_wrist_pose, "wrist");

    return init_wrist_pose;
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

void logExperiment(ros::NodeHandle *nh,
                   int final_state,
                   float duration,
                   float pos_error[3],
                   float polar,
                   float azimuthal,
                   float offset)
{
    // get relevant variables from parameter server
    std::string log_name;
    std::string object_name;
    getParam(nh, &log_name, "log_name");
    getParam(nh, &object_name, "object_name");

    // to create time and date info
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    fstream fout;
    fout.open(log_name, ios::out | ios::app);

    if (!fout.good())
    {
        std::ostringstream header;
        header << "time_stamp,"
               << "object_name,"
               << "final_state,"
               << "duration,"
               << "pos_error_x,"
               << "pos_error_y,"
               << "pos_error_z,"
               << "polar,"
               << "azimuthal,"
               << "offset"
               << "\n";

        ROS_INFO_STREAM("Creating file '" << log_name << "' with header '" << header.str() << "'.");
        fout << header.str();
    }
    else
    {
        ROS_INFO_STREAM("Adding results to existing file'" << log_name << "'.");
    }

    ROS_INFO_STREAM("Writing experiment data to file.");
    fout << std::put_time(&tm, "%d-%m-%Y %H-%M-%S") << ", "
         << object_name << ","
         << final_state << ","
         << duration << ","
         << pos_error[0] << ","
         << pos_error[1] << ","
         << pos_error[2] << ","
         << polar << ","
         << azimuthal << ","
         << offset << ","
         << "\n";

    fout.close();
}
