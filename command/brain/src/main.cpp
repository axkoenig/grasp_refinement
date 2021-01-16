#include <math.h>

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "hand_state.cpp"
#include "baseline_controller.cpp"
#include "helpers.cpp"

using namespace std;

string node_name = "baseline_commander_node";

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::Rate rate(40);
    ROS_INFO("Launched %s.", node_name.c_str());

    tf2::Transform init_wrist_pose;
    bool simulation_only;
    float polar, azimuthal, offset, time_out;
    float pos_error[3];
    getParam(&nh, &simulation_only, "simulation_only");
    getParam(&nh, &time_out, "time_out");

    if (simulation_only == true)
    {
        getParam(&nh, &polar, "polar");
        getParam(&nh, &azimuthal, "azimuthal");
        getParam(&nh, &offset, "offset");
        getParam(&nh, &pos_error[0], "pos_error_x");
        getParam(&nh, &pos_error[1], "pos_error_y");
        getParam(&nh, &pos_error[2], "pos_error_z");

        // caluclate pre-grasp pose from spherical coordinates
        init_wrist_pose = calcInitWristPose(&nh, pos_error, polar, azimuthal, offset);
    }
    else
    {
        // obtain initial pose from robot arm or from computer vision system
        // TODO: wait until we receive the initial pose (e.g. through a rosservice)
        ROS_WARN("Not implemented.");
    }

    // goal pose: goal_position and facing downwards.
    tf2::Vector3 goal_position = tf2::Vector3{0.3, 0, 0.45};
    tf2::Quaternion goal_rotation;
    goal_rotation.setRPY(M_PI, 0, 0);
    tf2::Transform goal_wrist_pose = tf2::Transform(goal_rotation, goal_position);

    BaselineController bc = BaselineController(&nh, init_wrist_pose, goal_wrist_pose, simulation_only, time_out);

    ROS_INFO("Starting autonomous control.");

    while (ros::ok())
    {
        bc.timeStep();
        ros::spinOnce();
        rate.sleep();

        if (bc.isFinished())
        {
            ros::Duration duration = ros::Time::now() - bc.getStartTime();
            logExperiment(&nh, bc.getState(), duration.toSec(), pos_error, polar, azimuthal, offset, bc.getStatusMsg());
            break;
        }
    }
}