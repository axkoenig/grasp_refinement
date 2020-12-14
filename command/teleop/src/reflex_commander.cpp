#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>
#include <reflex_msgs/PoseCommand.h>
#include <math.h>

std::string node_name = "reflex_commander_node";
std::string open_srv_name = "reflex/open";
std::string close_srv_name = "reflex/close";
std::string pinch_srv_name = "reflex/pinch";
std::string sph_open_srv_name = "reflex/spherical_open";
std::string sph_close_srv_name = "reflex/spherical_close";
std::string pos_incr_srv_name = "reflex/pos_incr";
std::string pos_cmd_topic = "reflex/pos_cmd";

// Responsibilities of this class
// - send predefined, high-level motion primitives (e.g. open/close) to Reflex in position control mode
// - store internal state of reflex
// - offer ROS API via ROS services
// - position increment control mode
class ReflexCommander
{
private:
    ros::Publisher pub;
    ros::ServiceServer open_service;
    ros::ServiceServer close_service;
    ros::ServiceServer pinch_service;
    ros::ServiceServer sph_open_service;
    ros::ServiceServer sph_close_service;
    // ros::ServiceServer pos_incr_service;
    reflex_msgs::PoseCommand pos_cmd;

    // format {finger1, finger2, finger3, preshape}
    std::array<float, 4> low_limits = {0, 0, 0, 0};
    std::array<float, 4> high_limits = {M_PI, M_PI, M_PI, M_PI};
    std::array<float, 4> open_pos = {0, 0, 0, 0};
    std::array<float, 4> close_pos = {2.5, 2.5, 2.5, 0};
    std::array<float, 4> pinch_pos = {1.7, 1.7, 0, M_PI};
    std::array<float, 4> sph_open_pos = {0.8, 0.8, 0.8, M_PI / 2};
    std::array<float, 4> sph_close_pos = {2.5, 2.5, 2.5, M_PI / 2};
    std::array<float, 4> cur_pos = open_pos;

public:
    ReflexCommander(ros::NodeHandle *nh)
    {
        pub = nh->advertise<reflex_msgs::PoseCommand>(pos_cmd_topic, 1);
        open_service = nh->advertiseService(open_srv_name, &ReflexCommander::callbackOpen, this);
        close_service = nh->advertiseService(close_srv_name, &ReflexCommander::callbackClose, this);
        pinch_service = nh->advertiseService(pinch_srv_name, &ReflexCommander::callbackPinch, this);
        sph_open_service = nh->advertiseService(sph_open_srv_name, &ReflexCommander::callbackSphOpen, this);
        sph_close_service = nh->advertiseService(sph_close_srv_name, &ReflexCommander::callbackSphClose, this);
        // pos incr
    }

    enum Primitive
    {
        Open,
        Close,
        Pinch,
        SphericalOpen,
        SphericalClose
    };

    std::string getCurPosStr()
    {
        std::string str;
        int size = cur_pos.size();
        for (int i = 0; i < size; i++)
        {
            str += std::to_string(cur_pos[i]);
            if (i != size - 1)
            {
                str += ", ";
            }
        }
        return str;
    }

    std::string executePrimitive(Primitive primitive)
    {
        switch (primitive)
        {
        case Open:
        {
            cur_pos = open_pos;
            break;
        }
        case Close:
        {
            cur_pos = close_pos;
            break;
        }
        case Pinch:
        {
            cur_pos = pinch_pos;
            break;
        }
        case SphericalOpen:
        {
            cur_pos = sph_open_pos;
            break;
        }
        case SphericalClose:
        {
            cur_pos = sph_close_pos;
            break;
        }
        }
        ReflexCommander::sendCommands();
        std::string msg = "Sent [" + ReflexCommander::getCurPosStr() + "] to " + pos_cmd_topic;
        return msg;
    }

    bool callbackOpen(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        std::string msg = ReflexCommander::executePrimitive(Open);
        res.success = true;
        res.message = msg;
        return true;
    }

    bool callbackClose(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        std::string msg = ReflexCommander::executePrimitive(Close);
        res.success = true;
        res.message = msg;
        return true;
    }

    bool callbackPinch(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        std::string msg = ReflexCommander::executePrimitive(Pinch);
        res.success = true;
        res.message = msg;
        return true;
    }

    bool callbackSphOpen(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        std::string msg = ReflexCommander::executePrimitive(SphericalOpen);
        res.success = true;
        res.message = msg;
        return true;
    }

    bool callbackSphClose(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        std::string msg = ReflexCommander::executePrimitive(SphericalClose);
        res.success = true;
        res.message = msg;
        return true;
    }

    // bool callbackPosIncr(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    // {
    //     std::string msg = ReflexCommander::executePrimitive(SphericalClose);
    //     res.success = true;
    //     res.message = msg;
    //     return true;
    // }

    void updatePosIncrement(float increment[4])
    {
        for (int i = 0; i < 4; i++)
        {
            float val = cur_pos[i] + increment[i];
            if (val > low_limits[i] && val < high_limits[i])
            {
                cur_pos[i] = val;
            }
        }
    }

    void sendCommands()
    {
        pos_cmd.f1 = cur_pos[0];
        pos_cmd.f2 = cur_pos[1];
        pos_cmd.f3 = cur_pos[2];
        pos_cmd.preshape = cur_pos[3];

        pub.publish(pos_cmd);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ReflexCommander rc = ReflexCommander(&nh);
    ROS_INFO("Launched %s node.", node_name.c_str());

    ros::spin();
}