#ifndef REFLEX_COMMANDER_H
#define REFLEX_COMMANDER_H

#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <reflex_msgs/PoseCommand.h>
#include <math.h>

std::string open_srv_name = "reflex/open";
std::string close_srv_name = "reflex/close";
std::string pinch_srv_name = "reflex/pinch";
std::string pos_cmd_topic = "reflex/pos_cmd";
std::string vel_cmd_topic = "reflex/vel_cmd";
std::string torque_cmd_topic = "reflex/torque_cmd";

// Class to send predefined, high-level motion primitives (e.g. open/close) to Reflex in position control mode
class ReflexCommander
{
private:
    ros::Publisher pub;
    ros::ServiceServer open_service;
    ros::ServiceServer close_service;
    ros::ServiceServer pinch_service;
    reflex_msgs::PoseCommand pos_cmd;

    // format {finger1, finger2, finger3, preshape}
    std::array<float, 4> open_pos = {0, 0, 0, 0};
    std::array<float, 4> close_pos = {2.5, 2.5, 2.5, 0};
    std::array<float, 4> pinch_pos = {1.7, 1.7, 0, M_PI};
    std::array<float, 4> cur_pos = open_pos;
    std::array<float, 4> low_limits = {0, 0, 0, 0};
    std::array<float, 4> high_limits = {M_PI, M_PI, M_PI, M_PI};

public:
    ReflexCommander(ros::NodeHandle *nh)
    {
        pub = nh->advertise<reflex_msgs::PoseCommand>(pos_cmd_topic, 1);
        open_service = nh->advertiseService(open_srv_name, &ReflexCommander::callbackOpen, this);
        close_service = nh->advertiseService(close_srv_name, &ReflexCommander::callbackClose, this);
        pinch_service = nh->advertiseService(pinch_srv_name, &ReflexCommander::callbackPinch, this);
    }

    enum Primitive
    {
        Open,
        Close,
        Pinch
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
        }
        ReflexCommander::sendCommands();
        return "Sent commands to " + pos_cmd_topic;
    }

    bool callbackOpen(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        if (req.data)
        {
            std::string msg = ReflexCommander::executePrimitive(Open);
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
            std::string msg = ReflexCommander::executePrimitive(Close);
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

    bool callbackPinch(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        if (req.data)
        {
            std::string msg = ReflexCommander::executePrimitive(Pinch);
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

#endif