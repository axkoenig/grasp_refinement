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
#include "gazebo_interface/gazebo_interface.hpp"

using namespace std;

tf2::Transform getTcpToWristFrame()
{
    // translate from TCP (i.e. the palm center) to wrist frame
    // z_offset: distance from Reflex origin to palm surface (taken from CAD drawing)
    // x_offset: x coordinate of "palm center" (palm center is defined as the origin of the circle that
    //           intersects origin of swivel_1, swivel_2 and what would be swivel_3 (swivel_3 doesn't
    //           exist in SDF because it is part of base_link.stl))
    float z_offset = -0.09228;
    float x_offset = -0.02;

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
                                 float obj_error[3],
                                 float reflex_error[3],
                                 float polar,
                                 float azimuthal,
                                 float offset,
                                 float z_rot)
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
    t += tf2::Vector3{obj_error[0], obj_error[1], obj_error[2]};
    tf2::Transform translate_to_object(tf2::Quaternion{0, 0, 0, 1}, t);
    ROS_INFO_STREAM("Introduced positional error " << obj_error << " to model position.");

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

    // translation to tool center point (TCP) along negative z axis and introduce positional error
    tf2::Vector3 tcp_to_object_offset = tf2::Vector3{0, 0, -offset};
    tf2::Transform translate_to_tcp = tf2::Transform();
    translate_to_tcp.setIdentity();
    translate_to_tcp.setOrigin(tcp_to_object_offset);

    // rotate along reflex z
    q.setRPY(0, 0, z_rot);
    tf2::Transform rotate_around_z;
    rotate_around_z.setIdentity();
    rotate_around_z.setRotation(q);

    // introduce position error in final wrist frame (corresponds to inaccuracies introduced by the robot arm)
    tf2::Vector3 error_origin = tf2::Vector3{reflex_error[0], reflex_error[1], reflex_error[2]};
    tf2::Transform translate_error;
    translate_error.setIdentity();
    translate_error.setOrigin(error_origin);

    tf2::Transform init_wrist_pose = translate_to_object * rotate_spherical * translate_to_tcp * rotate_around_z * getTcpToWristFrame() * translate_error;
    printPose(init_wrist_pose, "wrist");

    return init_wrist_pose;
}

void logExperiment(ros::NodeHandle *nh,
                   int final_state,
                   float duration,
                   float obj_error[3],
                   float reflex_error[3],
                   float polar,
                   float azimuthal,
                   float offset,
                   std::string status_msg)
{
    // get relevant variables from parameter server
    std::string log_name, object_name;
    float object_mass;
    getParam(nh, &log_name, "log_name");
    getParam(nh, &object_name, "object_name");
    getParam(nh, &object_mass, "object_mass");

    // to create time and date info
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    fstream fout;
    fout.open(log_name, ios::out | ios::app);

    if (fout.fail())
    {
        ROS_INFO("Log file does not exist yet.");
        std::ostringstream header;
        header << "time_stamp,"
               << "object_name,"
               << "object_mass,"
               << "final_state,"
               << "duration,"
               << "obj_error_x,"
               << "obj_error_y,"
               << "obj_error_z,"
               << "reflex_error_x,"
               << "reflex_error_y,"
               << "reflex_error_z,"
               << "polar,"
               << "azimuthal,"
               << "offset,"
               << "status_msg\n";

        ROS_INFO_STREAM("Creating file '" << log_name << "' with header '" << header.str() << "'.");
        fout << header.str();
    }
    else
    {
        ROS_INFO_STREAM("Adding results to existing file '" << log_name << "'.");
    }

    ROS_INFO_STREAM("Writing experiment data to file.");
    fout << std::put_time(&tm, "%d-%m-%Y %H-%M-%S") << ","
         << object_name << ","
         << object_mass << ","
         << final_state << ","
         << duration << ","
         << obj_error[0] << ","
         << obj_error[1] << ","
         << obj_error[2] << ","
         << reflex_error[0] << ","
         << reflex_error[1] << ","
         << reflex_error[2] << ","
         << polar << ","
         << azimuthal << ","
         << offset << ","
         << status_msg << "\n";

    fout.close();
}

void moveObjectOutOfWay(ros::NodeHandle *nh, std::string &object_name, tf2::Transform &old_pose)
{
    ROS_INFO("Moving object out of the way.");

    // it doesn't matter exactly where we move the object, as long as it's far enough away from
    // wrist, s.t. the two won't be able to collide. Here we simply place it 10m away from old_pose
    float x_offset = 10;
    tf2::Transform new_object_pose = old_pose;
    tf2::Vector3 t = new_object_pose.getOrigin();
    t[0] += x_offset;
    new_object_pose.setOrigin(t);
    setModelPoseSim(nh, object_name, new_object_pose);
}