#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <reflex_msgs/PoseCommand.h>

std::string pos_cmd_topic = "reflex/pos_cmd";
std::string node_name = "finger_controller_node";
std::string proximal_1_cmd_topic = "/gazebo/proximal_j1_position_controller/command";
std::string proximal_2_cmd_topic = "/gazebo/proximal_j2_position_controller/command";
std::string proximal_3_cmd_topic = "/gazebo/proximal_j3_position_controller/command";
std::string preshape_1_cmd_topic = "/gazebo/preshape_j1_position_controller/command";
std::string preshape_2_cmd_topic = "/gazebo/preshape_j2_position_controller/command";

class ReflexPositionController
{
private:
  ros::Subscriber sub;

  ros::Publisher proximal_1_cmd_pub;
  ros::Publisher proximal_2_cmd_pub;
  ros::Publisher proximal_3_cmd_pub;
  ros::Publisher preshape_1_cmd_pub;
  ros::Publisher preshape_2_cmd_pub;

  std_msgs::Float64 proximal_1_cmd;
  std_msgs::Float64 proximal_2_cmd;
  std_msgs::Float64 proximal_3_cmd;
  std_msgs::Float64 preshape_1_cmd;
  std_msgs::Float64 preshape_2_cmd;

public:
  ReflexPositionController(ros::NodeHandle *nh)
  {
    sub = nh->subscribe(pos_cmd_topic, 1, &ReflexPositionController::callback, this);

    proximal_1_cmd_pub = nh->advertise<std_msgs::Float64>(proximal_1_cmd_topic, 1);
    proximal_2_cmd_pub = nh->advertise<std_msgs::Float64>(proximal_2_cmd_topic, 1);
    proximal_3_cmd_pub = nh->advertise<std_msgs::Float64>(proximal_3_cmd_topic, 1);
    preshape_1_cmd_pub = nh->advertise<std_msgs::Float64>(preshape_1_cmd_topic, 1);
    preshape_2_cmd_pub = nh->advertise<std_msgs::Float64>(preshape_2_cmd_topic, 1);
  }

  void callback(const reflex_msgs::PoseCommand &msg)
  {
    proximal_1_cmd.data = msg.f1;
    proximal_2_cmd.data = msg.f2;
    proximal_3_cmd.data = msg.f3;

    // movement of preshape motors is equal and opposite
    preshape_1_cmd.data = msg.preshape/2;
    preshape_2_cmd.data = msg.preshape/2;

    preshape_1_cmd_pub.publish(preshape_1_cmd);
    preshape_2_cmd_pub.publish(preshape_2_cmd);
    proximal_1_cmd_pub.publish(proximal_1_cmd);
    proximal_2_cmd_pub.publish(proximal_2_cmd);
    proximal_3_cmd_pub.publish(proximal_3_cmd);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reflex_controller");
  ros::NodeHandle nh;
  ReflexPositionController rpc = ReflexPositionController(&nh);
  ros::spin();
}