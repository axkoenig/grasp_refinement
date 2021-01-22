#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>
#include <reflex_msgs/PoseCommand.h>
#include <math.h>

#include "reflex_interface/PosIncrement.h"
#include "reflex_interface/hand_command.hpp"

HandCommand::HandCommand(ros::NodeHandle *nh)
{
    pub = nh->advertise<reflex_msgs::PoseCommand>(pos_cmd_topic, 1);
    open_service = nh->advertiseService(open_srv_name, &HandCommand::callbackOpen, this);
    close_service = nh->advertiseService(close_srv_name, &HandCommand::callbackClose, this);
    pinch_service = nh->advertiseService(pinch_srv_name, &HandCommand::callbackPinch, this);
    sph_open_service = nh->advertiseService(sph_open_srv_name, &HandCommand::callbackSphOpen, this);
    sph_close_service = nh->advertiseService(sph_close_srv_name, &HandCommand::callbackSphClose, this);
    pos_incr_service = nh->advertiseService(pos_incr_srv_name, &HandCommand::callbackPosIncr, this);
}

std::string HandCommand::getStatusMsg()
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
    std::string msg = "Sent [" + str + "] to " + pos_cmd_topic;
    return msg;
}

void HandCommand::executePrimitive(HandCommand::Primitive primitive, bool verbose)
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
    this->sendCommands();
    if (verbose)
    {
        ROS_INFO_STREAM(getStatusMsg());
    }
}

bool HandCommand::callbackOpen(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    this->executePrimitive(Open);
    res.success = true;
    res.message = this->getStatusMsg();
    return true;
}

bool HandCommand::callbackClose(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    this->executePrimitive(Close);
    res.success = true;
    res.message = this->getStatusMsg();
    return true;
}

bool HandCommand::callbackPinch(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    this->executePrimitive(Pinch);
    res.success = true;
    res.message = this->getStatusMsg();
    return true;
}

bool HandCommand::callbackSphOpen(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    this->executePrimitive(SphericalOpen);
    res.success = true;
    res.message = this->getStatusMsg();
    return true;
}

bool HandCommand::callbackSphClose(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    this->executePrimitive(SphericalClose);
    res.success = true;
    res.message = this->getStatusMsg();
    return true;
}

bool HandCommand::callbackPosIncr(reflex_interface::PosIncrement::Request &req, reflex_interface::PosIncrement::Response &res)
{
    float increment[4] = {(float)req.f1,
                          (float)req.f2,
                          (float)req.f3,
                          (float)req.preshape};
    this->executePosIncrement(increment);
    res.success = true;
    res.message = this->getStatusMsg();
    return true;
}

void HandCommand::executePosIncrement(float increment[4])
{
    for (int i = 0; i < 4; i++)
    {
        float val = cur_pos[i] + increment[i];
        if (val > low_limits[i] && val < high_limits[i])
        {
            cur_pos[i] = val;
        }
    }
    this->sendCommands();
}

void HandCommand::sendCommands()
{
    pos_cmd.f1 = cur_pos[0];
    pos_cmd.f2 = cur_pos[1];
    pos_cmd.f3 = cur_pos[2];
    pos_cmd.preshape = cur_pos[3];

    pub.publish(pos_cmd);
}