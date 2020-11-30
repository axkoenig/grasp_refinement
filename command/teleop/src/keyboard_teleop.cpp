#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>

#include <iostream>
#include <map>
#include <termios.h>

#include "reflex_commander.h"

std::string node_name = "teleop_node";
std::string source_frame = "world";
std::string target_frame = "reflex";

float trans_scaling = 0.01;
float rot_scaling = 0.2;
float finger_scaling = 0.1;

// format: {x, y, z, r, p ,y} in "reflex" frame
std::array<float, 6> init_pose = {0, 0, 0.1, -M_PI / 2, 0, 0};

// keys for wrist teleoperation (note this is for my german keyboard)
std::map<char, std::vector<float>> wrist_bindings{

    // format: {x, y, z, r, p ,y}
    {'u', {1, 0, 0, 0, 0, 0}},
    {'i', {0, 1, 0, 0, 0, 0}},
    {'o', {0, 0, 1, 0, 0, 0}},
    {'j', {0, 0, 0, 1, 0, 0}},
    {'k', {0, 0, 0, 0, 1, 0}},
    {'l', {0, 0, 0, 0, 0, 1}},
    {'U', {-1, 0, 0, 0, 0, 0}},
    {'I', {0, -1, 0, 0, 0, 0}},
    {'O', {0, 0, -1, 0, 0, 0}},
    {'J', {0, 0, 0, -1, 0, 0}},
    {'K', {0, 0, 0, 0, -1, 0}},
    {'L', {0, 0, 0, 0, 0, -1}},
};

// keys for wrist teleoperation (note this is for my german keyboard)
std::map<char, std::vector<float>> finger_bindings{

    // format: {f1, f2, f3, preshape}
    {'q', {1, 0, 0, 0}},
    {'w', {0, 1, 0, 0}},
    {'e', {0, 0, 1, 0}},
    {'r', {0, 0, 0, 1}},
    {'Q', {-1, 0, 0, 0}},
    {'W', {0, -1, 0, 0}},
    {'E', {0, 0, -1, 0}},
    {'R', {0, 0, 0, -1}},
};

// BEGIN CODE FROM https://github.com/methylDragon/teleop_twist_keyboard_cpp/blob/master/src/teleop_twist_keyboard.cpp

// for non-blocking keyboard inputs
int getch(void)
{
    int ch;
    struct termios oldt;
    struct termios newt;

    // store old settings, and copy to new settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // make required changes and apply the settings
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    // get the current character
    ch = getchar();

    // reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

tf2::Transform calcReflexInWorld(std::array<float, 6> reflex_pose)
{
    tf2::Vector3 t = {reflex_pose[0], reflex_pose[1], reflex_pose[2]};
    tf2::Quaternion q;
    q.setRPY(reflex_pose[3], reflex_pose[4], reflex_pose[5]);
    tf2::Transform transform(q, t);

    return transform.inverse();
}

// END CODE FROM https://github.com/methylDragon/teleop_twist_keyboard_cpp/blob/master/src/teleop_twist_keyboard.cpp

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::Rate rate(100);
    ReflexCommander rc = ReflexCommander(&nh);
    ROS_INFO("Launched %s node.", node_name.c_str());

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped ts;
    char key(' ');

    std::array<float, 6> cur_pose = init_pose;
    tf2::Transform transform;
    transform = calcReflexInWorld(init_pose);

    // populate and send initial wrist transform
    ts.header.frame_id = source_frame;
    ts.child_frame_id = target_frame;
    ts.header.stamp = ros::Time::now();
    ts.transform = tf2::toMsg(transform);

    // TODO find another, more elegant solution for this
    // wait before publishing first transform to fix warning from wrist_control_node 
    // ""reflex" passed to lookupTransform argument target_frame does not exist."
    ros::Duration(1).sleep();

    ts.header.stamp = ros::Time::now();
    br.sendTransform(ts);

    ROS_INFO("Listening to keyboard input...");

    while (ros::ok())
    {
        key = getch();

        // WRIST CONTROL --------------------------------------------------------
        if (wrist_bindings.count(key) == 1)
        {
            // x, y, z
            cur_pose[0] += trans_scaling * wrist_bindings[key][0];
            cur_pose[1] += trans_scaling * wrist_bindings[key][1];
            cur_pose[2] += trans_scaling * wrist_bindings[key][2];

            // r, p, y
            cur_pose[3] += trans_scaling * wrist_bindings[key][3];
            cur_pose[4] += trans_scaling * wrist_bindings[key][4];
            cur_pose[5] += trans_scaling * wrist_bindings[key][5];

            ROS_INFO("Wrist transform updated.");
        }
        else if (key == 'm')
        {
            // reset wrist to init frame
            cur_pose = init_pose;
            ROS_INFO("Wrist transform reset.");
        }

        transform = calcReflexInWorld(cur_pose);
        ts.header.stamp = ros::Time::now();
        ts.transform = tf2::toMsg(transform);
        br.sendTransform(ts);

        // FINGER CONTROL (TELEOP) ----------------------------------------------
        if (finger_bindings.count(key) == 1)
        {
            float increment[4] = {finger_scaling * finger_bindings[key][0],
                                  finger_scaling * finger_bindings[key][1],
                                  finger_scaling * finger_bindings[key][2],
                                  finger_scaling * finger_bindings[key][3]};
            rc.updatePosIncrement(increment);
            ROS_INFO("Reflex finger positions updated.");
        }
        rc.sendCommands();

        // FINGER CONTROL (PRIMITIVES) ------------------------------------------
        switch (key)
        {
        case 'c':
        {
            rc.executePrimitive(rc.Primitive::Close);
            ROS_INFO("Sent Reflex close command.");
            break;
        }
        case 'x':
        {
            rc.executePrimitive(rc.Primitive::Open);
            ROS_INFO("Sent Reflex open command.");
            break;
        }
        case 'y':
        {
            rc.executePrimitive(rc.Primitive::Pinch);
            ROS_INFO("Sent Reflex pinch command.");
            break;
        }
        }

        // EXIT ------------------------------------------
        if (key == '\x03')
        {
            ROS_INFO("Pressed Ctrl-C. Shutting down...");
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}