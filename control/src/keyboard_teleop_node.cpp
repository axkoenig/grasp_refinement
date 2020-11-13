// first try publishing to gazebo
// then read keyboard and write teleop part

#include "ros/ros.h"
#include "gazebo_msgs/ModelState.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_teleop");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 1);
    ros::Rate loop_rate(1000);

    gazebo_msgs::ModelState msg;
    msg.model_name = "reflex";
    msg.reference_frame = "world";

    while (ros::ok())
    {
        msg.pose.position.x = 0.0;
        msg.pose.position.y = 0.0;
        msg.pose.position.z = 0.2;
        msg.pose.orientation.x = 0.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.2;
        msg.pose.orientation.w = 0.0;

        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}