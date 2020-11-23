#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <iostream>
#include <map>
#include <termios.h>

#include "reflex_commander.h"

std::string node_name = "teleop_node";
std::string source_frame = "world";
std::string target_frame = "reflex";

float trans_scaling = 0.01;
float rot_scaling = 0.05;
float finger_scaling = 0.1;

// keys for wrist teleoperation (note this is for my german keyboard)
std::map<char, std::vector<float>> wrist_bindings{

    // format: {x, y, z, r, p ,y}
    {'u', {-1, 0, 0, 0, 0, 0}},
    {'i', {0, -1, 0, 0, 0, 0}},
    {'o', {0, 0, -1, 0, 0, 0}},
    {'j', {0, 0, 0, -1, 0, 0}},
    {'k', {0, 0, 0, 0, -1, 0}},
    {'l', {0, 0, 0, 0, 0, -1}},
    {'U', {1, 0, 0, 0, 0, 0}},
    {'I', {0, 1, 0, 0, 0, 0}},
    {'O', {0, 0, 1, 0, 0, 0}},
    {'J', {0, 0, 0, 1, 0, 0}},
    {'K', {0, 0, 0, 0, 1, 0}},
    {'L', {0, 0, 0, 0, 0, 1}},
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
    tf2::Quaternion q;

    ts.header.frame_id = source_frame;
    ts.child_frame_id = target_frame;

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    char key(' ');

    ROS_INFO("Listening to keyboard input...");

    while (ros::ok())
    {
        key = getch();

        // WRIST CONTROL --------------------------------------------------------
        if (wrist_bindings.count(key) == 1)
        {
            ts.transform.translation.x += trans_scaling * wrist_bindings[key][0];
            ts.transform.translation.y += trans_scaling * wrist_bindings[key][1];
            ts.transform.translation.z += trans_scaling * wrist_bindings[key][2];

            roll += rot_scaling * wrist_bindings[key][3];
            pitch += rot_scaling * wrist_bindings[key][4];
            yaw += rot_scaling * wrist_bindings[key][5];

            // convert to quaternions
            q.setRPY(roll, pitch, yaw);
            ts.transform.rotation.x = q.x();
            ts.transform.rotation.y = q.y();
            ts.transform.rotation.z = q.z();
            ts.transform.rotation.w = q.w();
            ROS_INFO("Wrist transform updated.");
        }
        else if (key == 'm')
        {
            // reset wrist to world frame
            ts.transform.translation.x = 0;
            ts.transform.translation.y = 0;
            ts.transform.translation.z = 0;
            ts.transform.rotation.x = 0;
            ts.transform.rotation.y = 0;
            ts.transform.rotation.z = 0;
            ts.transform.rotation.w = 1;
        }
        ts.header.stamp = ros::Time::now();
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
        case 'y':
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