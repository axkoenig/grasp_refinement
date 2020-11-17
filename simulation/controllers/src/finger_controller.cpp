#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_srvs/SetBool.h"

using namespace std_msgs;
using namespace std_srvs;

class ReflexController
{
private:
  ros::Publisher preshape_1_cmd_pub;
  ros::Publisher preshape_2_cmd_pub;
  ros::Publisher proximal_1_cmd_pub;
  ros::Publisher proximal_2_cmd_pub;
  ros::Publisher proximal_3_cmd_pub;

  ros::ServiceServer open_service;
  ros::ServiceServer close_service;

  Float64 preshape_1_cmd;
  Float64 preshape_2_cmd;
  Float64 proximal_1_cmd;
  Float64 proximal_2_cmd;
  Float64 proximal_3_cmd;

public:
  ReflexController(ros::NodeHandle *nh)
  {
    preshape_1_cmd_pub = nh->advertise<Float64>("/gazebo/preshape_j1_effort_controller/command", 1);
    preshape_2_cmd_pub = nh->advertise<Float64>("/gazebo/preshape_j2_effort_controller/command", 1);
    proximal_1_cmd_pub = nh->advertise<Float64>("/gazebo/proximal_j1_effort_controller/command", 1);
    proximal_2_cmd_pub = nh->advertise<Float64>("/gazebo/proximal_j2_effort_controller/command", 1);
    proximal_3_cmd_pub = nh->advertise<Float64>("/gazebo/proximal_j3_effort_controller/command", 1);

    open_service = nh->advertiseService("/reflex/open_hand", &ReflexController::callback_open, this);
    close_service = nh->advertiseService("/reflex/close_hand", &ReflexController::callback_close, this);
  }

  bool callback_open(SetBool::Request &req, SetBool::Response &res)
  {
    if (req.data)
    {
      ReflexController::command_hand(0, 0, -0.5, -0.5, -0.5);
      res.success = true;
      res.message = "Hand was opened";
    }
    else
    {
      res.success = false;
      res.message = "Not sending any commands";
    }

    return true;
  }

  bool callback_close(SetBool::Request &req, SetBool::Response &res)
  {
    if (req.data)
    {
      ReflexController::command_hand(0, 0, 0.5, 0.5, 0.5);
      res.success = true;
      res.message = "Hand was closed";
    }
    else
    {
      res.success = false;
      res.message = "Not sending any commands";
    }

    return true;
  }

  void command_hand(double pre_1, double pre_2, double prox_1, double prox_2, double prox_3)
  {
    preshape_1_cmd.data = pre_1;
    preshape_2_cmd.data = pre_2;
    proximal_1_cmd.data = prox_1;
    proximal_2_cmd.data = prox_2;
    proximal_3_cmd.data = prox_3;

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
  ReflexController rc = ReflexController(&nh);
  ros::spin();
}