#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <reflex_msgs/PoseCommand.h>

std::string pos_cmd_topic = "reflex_takktile/command_position";
std::string node_name = "finger_controller_node";
std::string ns = "/gazebo";

// flexure executes *scaling_factor of rotation of proximal link
float scaling_factor = 0.2;

class ReflexFingerPositionController
{
private:
  int finger_id;

  // publishers
  ros::Publisher preshape_pub;
  ros::Publisher proximal_pub;
  ros::Publisher proximal_to_flex_pub;
  ros::Publisher flex_to_distal_pub;

  // messages
  std_msgs::Float64 preshape_msg;
  std_msgs::Float64 proximal_msg;
  std_msgs::Float64 proximal_to_flex_msg;
  std_msgs::Float64 flex_to_distal_msg;

public:
  ReflexFingerPositionController(ros::NodeHandle *nh, int finger_id)
  {
    this->finger_id = finger_id;

    std::string proximal_topic = ns + "/finger_" + std::to_string(this->finger_id) + "_proximal_position_controller/command";
    std::string proximal_to_flex_topic = ns + "/finger_" + std::to_string(this->finger_id) + "_proximal_to_flex_position_controller/command";
    std::string flex_to_distal_topic = ns + "/finger_" + std::to_string(this->finger_id) + "_flex_to_distal_position_controller/command";

    proximal_pub = nh->advertise<std_msgs::Float64>(proximal_topic, 1);
    proximal_to_flex_pub = nh->advertise<std_msgs::Float64>(proximal_to_flex_topic, 1);
    flex_to_distal_pub = nh->advertise<std_msgs::Float64>(flex_to_distal_topic, 1);

    if (this->finger_id != 3)
    {
      std::string preshape_topic = ns + "/finger_" + std::to_string(this->finger_id) + "_preshape_position_controller/command";
      preshape_pub = nh->advertise<std_msgs::Float64>(preshape_topic, 1);
    }
  }
  void send_commands(float proximal, float preshape = 0.0)
  {
    proximal_msg.data = proximal;
    proximal_to_flex_msg.data = scaling_factor * proximal;
    flex_to_distal_msg.data = scaling_factor * proximal;

    proximal_pub.publish(proximal_msg);
    proximal_to_flex_pub.publish(proximal_to_flex_msg);
    flex_to_distal_pub.publish(flex_to_distal_msg);

    if (finger_id != 3)
    {
      preshape_msg.data = preshape;
      preshape_pub.publish(preshape_msg);
    }
  }
};

class ReflexHandPositionController
{
private:
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe(pos_cmd_topic, 1, &ReflexHandPositionController::callback, this);
  ReflexFingerPositionController finger_controllers[3] = {ReflexFingerPositionController(&nh, 1),
                                                          ReflexFingerPositionController(&nh, 2),
                                                          ReflexFingerPositionController(&nh, 3)};

  void callback(const reflex_msgs::PoseCommand &msg)
  {
    // preshape rotation is equal and opposite
    finger_controllers[0].send_commands(msg.f1, msg.preshape / 2);
    finger_controllers[1].send_commands(msg.f2, msg.preshape / 2);
    finger_controllers[2].send_commands(msg.f3);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, node_name);
  ReflexHandPositionController rpc = ReflexHandPositionController();

  ROS_INFO("Launched %s node.", node_name.c_str());
  ROS_INFO("Listening to %s ...", pos_cmd_topic.c_str());

  ros::spin();
}