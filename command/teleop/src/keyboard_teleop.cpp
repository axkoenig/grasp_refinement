#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <reflex_msgs/PoseCommand.h>

#include <iostream>
#include <map>
#include <termios.h>

std::string node_name = "keyboard_teleop";
std::string source_frame = "world";
std::string target_frame = "reflex";
std::string pos_cmd_topic = "reflex/pos_cmd";
std::string vel_cmd_topic = "reflex/vel_cmd";
std::string torque_cmd_topic = "reflex/torque_cmd";
std::string open_srv_name = "reflex/open_hand";
std::string close_srv_name = "reflex/close_hand";
double trans_scaling = 0.01;
double rot_scaling = 0.05;

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

// Class to send predefined, high-level motion primitives (e.g. open/close) to Reflex
class ReflexPrimitives
{
private:
    ros::Publisher pub;
    ros::ServiceServer open_service;
    ros::ServiceServer close_service;
    reflex_msgs::PoseCommand pos_cmd;
    float positions[4];

public:
    ReflexPrimitives(ros::NodeHandle *nh)
    {
        pub = nh->advertise<reflex_msgs::PoseCommand>(pos_cmd_topic, 1);
        open_service = nh->advertiseService(open_srv_name, &ReflexPrimitives::callbackOpen, this);
        close_service = nh->advertiseService(close_srv_name, &ReflexPrimitives::callbackClose, this);
    }

    enum Primitive
    {
        Open,
        Close
    };

    std::string arrayToString(float array[], int numEntries)
    {
        std::string str;
        for (int i = 0; i < numEntries; i++)
        {
            str += std::to_string(i) + " ";
        }
        return str;
    }

    std::string executePrimitive(Primitive primitive)
    {
        switch (primitive)
        {
        case Open:
        {
            positions[0] = 0.0;
            positions[1] = 0.0;
            positions[2] = 0.0;
            positions[3] = 0.0;
            break;
        }
        case Close:
        {
            positions[0] = 2.5;
            positions[1] = 2.5;
            positions[2] = 2.5;
            positions[3] = 0.0;
            break;
        }
        }
        ReflexPrimitives::sendCommands(positions);
        return "Sent commands " + arrayToString(positions, 4) + " to " + pos_cmd_topic;
    }

    bool callbackOpen(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        if (req.data)
        {
            std::string msg = ReflexPrimitives::executePrimitive(Open);
            res.success = true;
            res.message = msg;
        }
        else
        {
            res.success = false;
            res.message = "Not sending commands.";
        }
        return true;
    }

    bool callbackClose(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        if (req.data)
        {
            std::string msg = ReflexPrimitives::executePrimitive(Close);
            res.success = true;
            res.message = msg;
        }
        else
        {
            res.success = false;
            res.message = "Not sending commands.";
        }
        return true;
    }

    void sendCommands(float positions[4])
    {
        pos_cmd.f1 = positions[0];
        pos_cmd.f2 = positions[1];
        pos_cmd.f3 = positions[2];
        pos_cmd.preshape = positions[3];

        pub.publish(pos_cmd);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::Rate rate(100);
    ReflexPrimitives rc = ReflexPrimitives(&nh);
    ROS_INFO("Launched %s node.", node_name.c_str());

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped ts;
    tf2::Quaternion q;

    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = source_frame;
    ts.child_frame_id = target_frame;
    ts.transform.translation.x = 0.0;
    ts.transform.translation.y = 0.0;
    ts.transform.translation.z = 0.0;
    ts.transform.rotation.x = 0.0;
    ts.transform.rotation.y = 0.0;
    ts.transform.rotation.z = 0.0;
    ts.transform.rotation.w = 1.0;
    br.sendTransform(ts);

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    char key(' ');

    ROS_INFO("Listening to keyboard input...");

    while (ros::ok())
    {
        key = getch();

        // teleop of wrist
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
        ts.header.stamp = ros::Time::now();
        br.sendTransform(ts);

        // teleop of individual fingers

        // primitive movement of reflex
        switch (key)
        {
        case 'w':
        {
            ROS_INFO("Sent Reflex close command.");
            rc.executePrimitive(rc.Primitive::Close);
            break;
        }

        case 'e':
        {
            ROS_INFO("Sent Reflex open command.");
            rc.executePrimitive(rc.Primitive::Open);
            break;
        }
        }

        // terminate upon ctrl-C
        if (key == '\x03')
        {
            ROS_INFO("Shutting down...");
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}