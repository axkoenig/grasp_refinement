#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <string>

std::string node_name = "wrist_control";
std::string ns = "gazebo";
std::string topic_name = "gazebo/set_model_state";
std::string source_frame = "world";
std::string target_frame = "reflex";

class WristAxisController
{
private:
    ros::Publisher pub;
    std_msgs::Float64 msg;

public:
    WristAxisController(ros::NodeHandle *nh, std::string axis)
    {
        std::string topic = ns + "/virtual_" + axis + "_position_controller/command";
        pub = nh->advertise<std_msgs::Float64>(topic, 1);
    }

    void sendCommand(float command)
    {
        msg.data = command;
        pub.publish(msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<gazebo_msgs::ModelState>(topic_name, 1);
    ROS_INFO("Launched %s node.", node_name.c_str());

    ros::Rate rate(1000);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped ts;

    WristAxisController px_controller = WristAxisController(&nh, "px");
    WristAxisController py_controller = WristAxisController(&nh, "py");
    WristAxisController pz_controller = WristAxisController(&nh, "pz");
    WristAxisController rr_controller = WristAxisController(&nh, "rr");
    WristAxisController rp_controller = WristAxisController(&nh, "rp");
    WristAxisController ry_controller = WristAxisController(&nh, "ry");

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

        // convert geometry_msgs::Quaternion to tf2::Quaterion
        tf2::Quaternion q = tf2::Quaternion(ts.transform.rotation.x,
                                            ts.transform.rotation.y,
                                            ts.transform.rotation.z,
                                            ts.transform.rotation.w);

        // form rotation matrix and obtain r, p, y
        double r, p, y;
        tf2::Matrix3x3(q).getRPY(r, p, y);

        // update commands
        px_controller.sendCommand(ts.transform.translation.x);
        py_controller.sendCommand(ts.transform.translation.y);
        pz_controller.sendCommand(ts.transform.translation.z);
        rr_controller.sendCommand(r);
        rp_controller.sendCommand(p);
        ry_controller.sendCommand(y);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
