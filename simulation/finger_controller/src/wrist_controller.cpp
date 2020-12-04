#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>

std::string node_name = "wrist_control";
std::string topic_name = "gazebo/set_model_state";
std::string source_frame = "world";
std::string target_frame = "reflex";

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<gazebo_msgs::ModelState>(topic_name, 1);
    ROS_INFO("Launched %s node.", node_name.c_str());

    gazebo_msgs::ModelState msg;
    msg.reference_frame = source_frame;
    msg.model_name = target_frame;

// todo change the starting pose
    msg.pose.position.x = 0.0;
    msg.pose.position.y = 0.0;
    msg.pose.position.z = 0.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;

    // TODO: attention if I run this in a simulation (e.g. on a HPC) this rate might
    // not be sufficient to keep the hand in the desired pose because the frames are executed
    // much quicker
    ros::Rate rate(1000);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped ts;

    while (ros::ok())
    {
        try
        {
            ts = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        msg.pose.position.x = ts.transform.translation.x;
        msg.pose.position.y = ts.transform.translation.y;
        msg.pose.position.z = ts.transform.translation.z;
        msg.pose.orientation.x = ts.transform.rotation.x;
        msg.pose.orientation.y = ts.transform.rotation.y;
        msg.pose.orientation.z = ts.transform.rotation.z;
        msg.pose.orientation.w = ts.transform.rotation.w;

        pub.publish(msg);
        rate.sleep();
    }

    return 0;
}
