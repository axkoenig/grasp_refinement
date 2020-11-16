#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <map>
#include <termios.h>

// map for keys (note this is for my german keyboard)
std::map<char, std::vector<float>> bindings{

    // format is {x, y, z, r, p ,y}
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

// For non-blocking keyboard inputs
int getch(void)
{
    int ch;
    struct termios oldt;
    struct termios newt;

    // Store old settings, and copy to new settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Make required changes and apply the settings
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    // Get the current character
    ch = getchar();

    // Reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_teleop");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped ts;
    tf2::Quaternion q;

    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = "world";
    ts.child_frame_id = "reflex";
    ts.transform.translation.x = 0.0;
    ts.transform.translation.y = 0.0;
    ts.transform.translation.z = 0.0;
    ts.transform.rotation.x = 0.0;
    ts.transform.rotation.y = 0.0;
    ts.transform.rotation.z = 0.0;
    ts.transform.rotation.w = 1.0;

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    double scaling_factor = 0.01;
    char key(' ');

    while (ros::ok())
    {
        key = getch();

        if (bindings.count(key) == 1)
        {
            ts.transform.translation.x += scaling_factor * bindings[key][0];
            ts.transform.translation.y += scaling_factor * bindings[key][1];
            ts.transform.translation.z += scaling_factor * bindings[key][2];

            roll += scaling_factor * bindings[key][3];
            pitch += scaling_factor * bindings[key][4];
            yaw += scaling_factor * bindings[key][5];

            // convert to quaternions
            q.setRPY(roll, pitch, yaw);
            ts.transform.rotation.x = q.x();
            ts.transform.rotation.y = q.y();
            ts.transform.rotation.z = q.z();
            ts.transform.rotation.w = q.w();
        }

        // terminate upon ctrl-C
        if (key == '\x03')
        {
            break;
        }

        br.sendTransform(ts);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}